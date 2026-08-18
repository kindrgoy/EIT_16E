import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox

import numpy as np

try:
    import serial
    import serial.tools.list_ports
except ImportError:
    serial = None

import matplotlib
matplotlib.use("TkAgg")
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from scipy.interpolate import griddata
from scipy.ndimage import gaussian_filter


# ============================================================
# DEFAULT CONFIGURATION
# ============================================================

DEFAULT_PORT = "COM12"
DEFAULT_BAUD = 921600

N_EL = 16
DIST_EXC = 8
MEAS_PER_EXC = 12
FRAME_SIZE = 192

DEFAULT_MESH_H0 = 0.08
DEFAULT_IMAGE_SIZE = 96

DEFAULT_KALMAN_Q = 0.005
DEFAULT_KALMAN_R = 0.05

DEFAULT_BASELINE_FRAMES = 5

DEFAULT_COLOR_LIMIT = 1.0
DEFAULT_GAUSSIAN_SIGMA = 0.7


# ============================================================
# FIRMWARE ORDER
# ============================================================

def build_expected():
    out = []

    for k in range(N_EL):
        hc = k
        lc = (k + DIST_EXC) % N_EL

        for hp in range(N_EL):
            lp = (hp + 1) % N_EL

            if hp == hc or hp == lc or lp == hc or lp == lc:
                continue

            out.append((hc, lc, hp, lp))

    return out


EXPECTED = build_expected()

assert len(EXPECTED) == FRAME_SIZE


# ============================================================
# KALMAN FILTER
# ============================================================

class VectorKalman:
    def __init__(self, size, q, r):
        self.size = size
        self.q = float(q)
        self.r = float(r)
        self.x = np.zeros(size, dtype=float)
        self.p = np.ones(size, dtype=float)
        self.initialized = False

    def reset(self):
        self.x.fill(0.0)
        self.p.fill(1.0)
        self.initialized = False

    def update(self, z):
        z = np.asarray(z, dtype=float).reshape(-1)

        if z.size != self.size:
            raise ValueError("Kalman image size mismatch")

        if not self.initialized:
            self.x[:] = z
            self.initialized = True
            return self.x.copy()

        self.p += self.q

        gain = self.p / (self.p + self.r)

        self.x += gain * (z - self.x)
        self.p *= (1.0 - gain)

        return self.x.copy()


# ============================================================
# PY-EIT RECONSTRUCTION
# ============================================================

class EITReconstructor:
    def __init__(self, h0, image_size):
        self.h0 = float(h0)
        self.image_size = int(image_size)

        self.ready = False
        self.error = None

        self.mesh = None
        self.protocol = None
        self.eit = None

        self.X = None
        self.Y = None
        self.mask = None

        self._build()

    def _build(self):
        try:
            import pyeit.mesh as mesh
            import pyeit.eit.protocol as protocol
            import pyeit.eit.jac as jac

            # Exactly the same protocol as the ESP32 firmware.
            self.protocol = protocol.create(
                n_el=N_EL,
                dist_exc=DIST_EXC,
                step_meas=1,
                parser_meas="std"
            )

            total_meas = self.protocol.n_exc * self.protocol.n_meas

            if total_meas != FRAME_SIZE:
                raise RuntimeError(
                    f"pyEIT generated {total_meas} measurements "
                    f"({self.protocol.n_exc} exc x {self.protocol.n_meas} meas), "
                    f"expected {FRAME_SIZE}"
                )

            self.mesh = mesh.create(
                n_el=N_EL,
                h0=self.h0
            )

            self.eit = jac.JAC(
                self.mesh,
                self.protocol
            )

            self.eit.setup(
                p=0.5,
                lamb=0.05,
                method="kotre",
                jac_normalized=True
            )

            x = np.linspace(-1.0, 1.0, self.image_size)
            y = np.linspace(-1.0, 1.0, self.image_size)

            self.X, self.Y = np.meshgrid(x, y)
            self.mask = self.X**2 + self.Y**2 <= 1.0

            self.ready = True

        except Exception as exc:
            self.error = repr(exc)
            self.ready = False

    def rebuild(self, h0, image_size):
        self.h0 = float(h0)
        self.image_size = int(image_size)
        self._build()

    def reconstruct(self, current, baseline):
        if not self.ready:
            raise RuntimeError(
                "pyEIT belum siap: " + str(self.error)
            )

        v1 = np.asarray(current, dtype=np.complex128)
        v0 = np.asarray(baseline, dtype=np.complex128)

        if v1.size != FRAME_SIZE or v0.size != FRAME_SIZE:
            raise ValueError("Frame harus berisi 192 measurement")

        # pyEIT dynamic JAC:
        # dv = (v1-v0)/abs(v0)
        ds = self.eit.solve(
            v1,
            v0,
            normalize=True
        )

        ds = np.real(ds)

        # pyEIT returns element values.
        centers = self.mesh.elem_centers[:, :2]

        img = griddata(
            centers,
            ds,
            (self.X, self.Y),
            method="linear",
            fill_value=0.0
        )

        img[~self.mask] = np.nan

        return img


# ============================================================
# SERIAL READER
# ============================================================

class SerialReader(threading.Thread):
    def __init__(self, port, baud, q, stop_event):
        super().__init__(daemon=True)

        self.port = port
        self.baud = baud
        self.q = q
        self.stop_event = stop_event

        self.ser = None
        self.write_lock = threading.Lock()

    def run(self):
        if serial is None:
            self.q.put(("error", "pyserial belum terinstall"))
            return

        try:
            self.ser = serial.Serial(
                self.port,
                self.baud,
                timeout=0.05
            )

            # Allow ESP32 USB serial to settle.
            time.sleep(1.0)

            self.q.put(("status", f"CONNECTED {self.port}"))

            frame_rows = []
            in_frame = False

            while not self.stop_event.is_set():
                raw = self.ser.readline()

                if not raw:
                    continue

                line = raw.decode(
                    "utf-8",
                    errors="ignore"
                ).strip()

                if line.startswith("### SWEEP_START_OPPOSITE"):
                    frame_rows = []
                    in_frame = True
                    continue

                if line.startswith("### SWEEP_END_OPPOSITE"):
                    if in_frame and frame_rows:
                        self.q.put(("frame", frame_rows))

                    frame_rows = []
                    in_frame = False
                    continue

                if not in_frame:
                    continue

                if line.startswith("idx,"):
                    continue

                parts = line.split(",")

                if len(parts) != 10:
                    continue

                try:
                    idx = int(parts[0])
                    hc = int(parts[1])
                    lc = int(parts[2])
                    hp = int(parts[3])
                    lp = int(parts[4])

                    re = float(parts[5])
                    im = float(parts[6])
                    mag = float(parts[7])
                    z = float(parts[8])
                    ok = int(parts[9])

                except ValueError:
                    continue

                if ok != 1:
                    continue

                if not (
                    np.isfinite(re)
                    and np.isfinite(im)
                ):
                    continue

                frame_rows.append({
                    "idx": idx,
                    "hc": hc,
                    "lc": lc,
                    "hp": hp,
                    "lp": lp,
                    "re": re,
                    "im": im,
                    "mag": mag,
                    "z": z,
                })

        except Exception as exc:
            self.q.put(("error", repr(exc)))

        finally:
            try:
                if self.ser:
                    self.ser.close()
            except Exception:
                pass

    def send(self, text):
        if self.ser is None:
            return False

        try:
            with self.write_lock:
                self.ser.write(
                    (text.strip() + "\n").encode("ascii")
                )
                self.ser.flush()

            return True

        except Exception as exc:
            self.q.put(("error", repr(exc)))
            return False


# ============================================================
# FRAME CONVERSION
# ============================================================

def frame_to_complex(frame):
    lookup = {}

    for row in frame:
        key = (
            row["hc"],
            row["lc"],
            row["hp"],
            row["lp"]
        )

        lookup[key] = complex(
            row["re"],
            row["im"]
        )

    values = np.full(
        FRAME_SIZE,
        np.nan + 1j * np.nan,
        dtype=np.complex128
    )

    for i, key in enumerate(EXPECTED):
        if key in lookup:
            values[i] = lookup[key]

    return values


# ============================================================
# GUI
# ============================================================

class LiveEIT:
    def __init__(self, root):
        self.root = root
        self.root.title(
            "Live EIT Imaging - ESP32-S3 / AD5933"
        )
        self.root.geometry("1280x820")

        self.q = queue.Queue()
        self.stop_event = threading.Event()
        self.reader = None

        self.reconstructor = None
        self.kalman = None

        self.baseline = None
        self.baseline_buffer = []
        self.baseline_target = 0

        self.total_frames = 0
        self.last_fps_time = time.perf_counter()
        self.fps_counter = 0
        self.fps = 0.0

        self._build_gui()

        self.root.after(20, self.poll_queue)

    # --------------------------------------------------------
    # GUI
    # --------------------------------------------------------

    def _build_gui(self):
        left = ttk.Frame(self.root)
        left.pack(
            side="left",
            fill="both",
            expand=True
        )

        right = ttk.Frame(
            self.root,
            width=320
        )
        right.pack(
            side="right",
            fill="y"
        )
        right.pack_propagate(False)

        fig = Figure(
            figsize=(7.2, 7.2),
            dpi=100
        )

        self.ax = fig.add_subplot(111)

        self.ax.set_aspect("equal")
        self.ax.set_xlim(-1.12, 1.12)
        self.ax.set_ylim(-1.12, 1.12)
        self.ax.set_xlabel("x")
        self.ax.set_ylabel("y")
        self.ax.set_title(
            "LIVE EIT — baseline difference"
        )

        self.image = self.ax.imshow(
            np.zeros((96, 96)),
            extent=(-1, 1, -1, 1),
            origin="lower",
            cmap="RdBu_r",
            interpolation="bilinear",
            vmin=-DEFAULT_COLOR_LIMIT,
            vmax=DEFAULT_COLOR_LIMIT
        )

        self.colorbar = fig.colorbar(
            self.image,
            ax=self.ax,
            fraction=0.046,
            pad=0.04
        )

        theta = np.linspace(
            0,
            2*np.pi,
            300
        )

        self.ax.plot(
            np.cos(theta),
            np.sin(theta),
            linewidth=1
        )

        for i in range(N_EL):
            a = 2*np.pi*i/N_EL

            self.ax.plot(
                np.cos(a),
                np.sin(a),
                "o",
                markersize=5
            )

            self.ax.text(
                1.07*np.cos(a),
                1.07*np.sin(a),
                str(i),
                ha="center",
                va="center",
                fontsize=8
            )

        self.canvas = FigureCanvasTkAgg(
            fig,
            master=left
        )

        self.canvas.get_tk_widget().pack(
            fill="both",
            expand=True
        )

        # ----------------------------------------------------
        # CONTROL
        # ----------------------------------------------------

        ttk.Label(
            right,
            text="LIVE EIT CONTROL",
            font=("Segoe UI", 15, "bold")
        ).pack(pady=(14, 12))

        self.status_var = tk.StringVar(
            value="DISCONNECTED"
        )

        ttk.Label(
            right,
            textvariable=self.status_var
        ).pack(pady=(0, 10))

        ttk.Label(
            right,
            text="Serial Port"
        ).pack(anchor="w", padx=15)

        self.port_var = tk.StringVar(
            value=DEFAULT_PORT
        )

        self.port_box = ttk.Combobox(
            right,
            textvariable=self.port_var
        )

        self.port_box.pack(
            fill="x",
            padx=15,
            pady=4
        )

        self.refresh_ports()

        ttk.Button(
            right,
            text="Refresh Ports",
            command=self.refresh_ports
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        ttk.Label(
            right,
            text="Baudrate"
        ).pack(
            anchor="w",
            padx=15,
            pady=(8, 0)
        )

        self.baud_var = tk.StringVar(
            value=str(DEFAULT_BAUD)
        )

        ttk.Entry(
            right,
            textvariable=self.baud_var
        ).pack(
            fill="x",
            padx=15,
            pady=4
        )

        ttk.Button(
            right,
            text="START",
            command=self.start
        ).pack(
            fill="x",
            padx=15,
            pady=(10, 3)
        )

        ttk.Button(
            right,
            text="STOP",
            command=self.stop
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        ttk.Button(
            right,
            text="CAPTURE BASELINE (5 FRAME)",
            command=self.capture_baseline
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        ttk.Button(
            right,
            text="CLEAR BASELINE",
            command=self.clear_baseline
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        ttk.Button(
            right,
            text="RESET KALMAN",
            command=self.reset_kalman
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        # ----------------------------------------------------
        # SETTINGS
        # ----------------------------------------------------

        ttk.Separator(right).pack(
            fill="x",
            padx=15,
            pady=14
        )

        ttk.Label(
            right,
            text="RECONSTRUCTION SETTINGS",
            font=("Segoe UI", 10, "bold")
        ).pack(
            anchor="w",
            padx=15
        )

        self.mesh_var = tk.StringVar(
            value=str(DEFAULT_MESH_H0)
        )

        ttk.Label(
            right,
            text="Mesh h0"
        ).pack(
            anchor="w",
            padx=15,
            pady=(7, 0)
        )

        ttk.Entry(
            right,
            textvariable=self.mesh_var
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        self.image_var = tk.StringVar(
            value=str(DEFAULT_IMAGE_SIZE)
        )

        ttk.Label(
            right,
            text="Image resolution"
        ).pack(
            anchor="w",
            padx=15,
            pady=(7, 0)
        )

        ttk.Entry(
            right,
            textvariable=self.image_var
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        self.q_var = tk.StringVar(
            value=str(DEFAULT_KALMAN_Q)
        )

        ttk.Label(
            right,
            text="Kalman Q"
        ).pack(
            anchor="w",
            padx=15,
            pady=(7, 0)
        )

        ttk.Entry(
            right,
            textvariable=self.q_var
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        self.r_var = tk.StringVar(
            value=str(DEFAULT_KALMAN_R)
        )

        ttk.Label(
            right,
            text="Kalman R"
        ).pack(
            anchor="w",
            padx=15,
            pady=(7, 0)
        )

        ttk.Entry(
            right,
            textvariable=self.r_var
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        self.clim_var = tk.StringVar(
            value=str(DEFAULT_COLOR_LIMIT)
        )

        ttk.Label(
            right,
            text="Color ±"
        ).pack(
            anchor="w",
            padx=15,
            pady=(7, 0)
        )

        ttk.Entry(
            right,
            textvariable=self.clim_var
        ).pack(
            fill="x",
            padx=15,
            pady=3
        )

        ttk.Button(
            right,
            text="APPLY SETTINGS",
            command=self.apply_settings
        ).pack(
            fill="x",
            padx=15,
            pady=(7, 3)
        )

        # ----------------------------------------------------
        # STATUS
        # ----------------------------------------------------

        ttk.Separator(right).pack(
            fill="x",
            padx=15,
            pady=14
        )

        self.frame_var = tk.StringVar(
            value="Frame: 0"
        )

        self.fps_var = tk.StringVar(
            value="FPS: 0.0"
        )

        self.measure_var = tk.StringVar(
            value="Measurements: 0/192"
        )

        self.baseline_var = tk.StringVar(
            value="Baseline: NOT SET"
        )

        self.recon_var = tk.StringVar(
            value="pyEIT: checking..."
        )

        for var in [
            self.frame_var,
            self.fps_var,
            self.measure_var,
            self.baseline_var,
            self.recon_var
        ]:
            ttk.Label(
                right,
                textvariable=var
            ).pack(
                anchor="w",
                padx=15,
                pady=3
            )

        ttk.Label(
            right,
            text=(
                "Experiment:\n"
                "1. Isi wadah dengan air garam homogen.\n"
                "2. START.\n"
                "3. CAPTURE BASELINE 5 FRAME.\n"
                "4. Masukkan objek/perubahan konduktivitas.\n"
                "5. Amati citra live."
            )
        ).pack(
            anchor="w",
            padx=15,
            pady=(15, 3)
        )

        self._build_reconstructor()

    # --------------------------------------------------------
    # RECONSTRUCTOR
    # --------------------------------------------------------

    def _build_reconstructor(self):
        try:
            h0 = float(self.mesh_var.get())
            size = int(self.image_var.get())

            self.reconstructor = EITReconstructor(
                h0,
                size
            )

            if self.reconstructor.ready:
                self.recon_var.set(
                    f"pyEIT READY | mesh h0={h0}"
                )
            else:
                self.recon_var.set(
                    "pyEIT ERROR"
                )

        except Exception as exc:
            self.recon_var.set(
                "pyEIT ERROR: " + str(exc)
            )

    # --------------------------------------------------------
    # SERIAL
    # --------------------------------------------------------

    def refresh_ports(self):
        if serial is None:
            self.port_box["values"] = []
            return

        ports = [
            p.device
            for p in serial.tools.list_ports.comports()
        ]

        self.port_box["values"] = ports

        if self.port_var.get() not in ports and ports:
            self.port_var.set(ports[0])

    def start(self):
        if self.reader and self.reader.is_alive():
            return

        if serial is None:
            messagebox.showerror(
                "Dependency",
                "pyserial belum terinstall."
            )
            return

        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror(
                "Error",
                "Baudrate tidak valid."
            )
            return

        self.apply_settings()

        self.stop_event.clear()

        self.reader = SerialReader(
            self.port_var.get(),
            baud,
            self.q,
            self.stop_event
        )

        self.reader.start()

        # Give thread time to open COM.
        self.root.after(
            1200,
            self.send_start_command
        )

    def send_start_command(self):
        if self.reader and self.reader.is_alive():
            if self.reader.send("g"):
                self.status_var.set(
                    "RUNNING — sweep started"
                )

    def stop(self):
        if self.reader:
            self.reader.send("x")

        self.stop_event.set()

        self.status_var.set("STOPPED")

    # --------------------------------------------------------
    # BASELINE
    # --------------------------------------------------------

    def capture_baseline(self):
        if not self.reader or not self.reader.is_alive():
            messagebox.showwarning(
                "Not running",
                "Tekan START terlebih dahulu."
            )
            return

        self.baseline_buffer = []

        try:
            self.baseline_target = max(
                1,
                int(DEFAULT_BASELINE_FRAMES)
            )
        except Exception:
            self.baseline_target = 5

        self.baseline = None

        self.baseline_var.set(
            f"Baseline: 0/{self.baseline_target}"
        )

        self.status_var.set(
            "CAPTURING BASELINE — jangan ubah media"
        )

        if self.kalman:
            self.kalman.reset()

    def clear_baseline(self):
        self.baseline = None
        self.baseline_buffer = []
        self.baseline_target = 0

        self.baseline_var.set(
            "Baseline: NOT SET"
        )

        if self.kalman:
            self.kalman.reset()

        self.status_var.set(
            "Baseline cleared"
        )

    # --------------------------------------------------------
    # KALMAN / SETTINGS
    # --------------------------------------------------------

    def reset_kalman(self):
        if self.kalman:
            self.kalman.reset()

        self.status_var.set(
            "Kalman reset"
        )

    def apply_settings(self):
        try:
            h0 = float(self.mesh_var.get())
            image_size = int(self.image_var.get())
            q = float(self.q_var.get())
            r = float(self.r_var.get())
            clim = float(self.clim_var.get())

            if not (0.03 <= h0 <= 0.30):
                raise ValueError(
                    "Mesh h0 sebaiknya 0.03–0.30"
                )

            if not (32 <= image_size <= 192):
                raise ValueError(
                    "Image resolution 32–192"
                )

            if q <= 0 or r <= 0 or clim <= 0:
                raise ValueError(
                    "Q, R, dan Color harus > 0"
                )

            self.reconstructor = EITReconstructor(
                h0,
                image_size
            )

            if not self.reconstructor.ready:
                raise RuntimeError(
                    self.reconstructor.error
                )

            self.kalman = VectorKalman(
                image_size * image_size,
                q,
                r
            )

            self.image.set_data(
                np.zeros(
                    (image_size, image_size)
                )
            )

            self.image.set_clim(
                -clim,
                clim
            )

            self.canvas.draw_idle()

            self.recon_var.set(
                f"pyEIT READY | mesh h0={h0}"
            )

            self.status_var.set(
                "Settings applied"
            )

        except Exception as exc:
            messagebox.showerror(
                "Settings error",
                str(exc)
            )

    # --------------------------------------------------------
    # FRAME PROCESSING
    # --------------------------------------------------------

    def poll_queue(self):
        try:
            while True:
                kind, data = self.q.get_nowait()

                if kind == "status":
                    self.status_var.set(data)

                elif kind == "error":
                    self.status_var.set(
                        "ERROR"
                    )
                    print("[SERIAL ERROR]", data)

                elif kind == "frame":
                    self.process_frame(data)

        except queue.Empty:
            pass

        self.root.after(
            20,
            self.poll_queue
        )

    def process_frame(self, rows):
        v = frame_to_complex(rows)

        valid = np.isfinite(v.real) & np.isfinite(v.imag)

        valid_count = int(valid.sum())

        self.measure_var.set(
            f"Measurements: {valid_count}/{FRAME_SIZE}"
        )

        # Require almost complete frame.
        if valid_count < 180:
            return

        # Repair very rare missing values using channel median.
        if not np.all(valid):
            med_re = np.nanmedian(v.real)
            med_im = np.nanmedian(v.imag)

            v.real[~valid] = med_re
            v.imag[~valid] = med_im

        self.total_frames += 1
        self.fps_counter += 1

        now = time.perf_counter()
        dt = now - self.last_fps_time

        if dt >= 1.0:
            self.fps = self.fps_counter / dt
            self.fps_counter = 0
            self.last_fps_time = now

        self.frame_var.set(
            f"Frame: {self.total_frames}"
        )

        self.fps_var.set(
            f"FPS: {self.fps:.1f}"
        )

        # --------------------------------------------
        # BASELINE CAPTURE MODE
        # --------------------------------------------

        if self.baseline_target > 0:
            self.baseline_buffer.append(v.copy())

            n = len(self.baseline_buffer)

            self.baseline_var.set(
                f"Baseline: {n}/{self.baseline_target}"
            )

            if n >= self.baseline_target:
                stack = np.vstack(
                    self.baseline_buffer
                )

                # Robust baseline:
                # median real and median imaginary separately.
                self.baseline = (
                    np.median(stack.real, axis=0)
                    + 1j * np.median(
                        stack.imag,
                        axis=0
                    )
                )

                self.baseline_buffer = []
                self.baseline_target = 0

                if self.kalman:
                    self.kalman.reset()

                self.baseline_var.set(
                    "Baseline: SET ✓"
                )

                self.status_var.set(
                    "Baseline ready — change the medium/object"
                )

            return

        # --------------------------------------------
        # NO BASELINE = DON'T RECONSTRUCT
        # --------------------------------------------

        if self.baseline is None:
            self.status_var.set(
                "RUNNING — capture baseline first"
            )
            return

        # --------------------------------------------
        # RECONSTRUCTION
        # --------------------------------------------

        try:
            img = self.reconstructor.reconstruct(
                v,
                self.baseline
            )

            sigma = DEFAULT_GAUSSIAN_SIGMA

            if sigma > 0:
                finite = np.isfinite(img)

                temp = np.nan_to_num(
                    img,
                    nan=0.0
                )

                temp = gaussian_filter(
                    temp,
                    sigma=sigma
                )

                img = temp
                img[~finite] = np.nan

            # Kalman.
            flat = np.nan_to_num(
                img,
                nan=0.0
            ).reshape(-1)

            if (
                self.kalman is None
                or self.kalman.size != flat.size
            ):
                self.kalman = VectorKalman(
                    flat.size,
                    float(self.q_var.get()),
                    float(self.r_var.get())
                )

            filtered = self.kalman.update(
                flat
            ).reshape(img.shape)

            # Circular mask.
            size = filtered.shape[0]

            yy, xx = np.indices(
                filtered.shape
            )

            x = (
                2.0 * xx /
                max(size - 1, 1)
                - 1.0
            )

            y = (
                2.0 * yy /
                max(size - 1, 1)
                - 1.0
            )

            filtered[
                x*x + y*y > 1.0
            ] = np.nan

            self.image.set_data(
                filtered
            )

            self.canvas.draw_idle()

            self.status_var.set(
                "LIVE RECONSTRUCTION"
            )

        except Exception as exc:
            self.status_var.set(
                "RECONSTRUCTION ERROR"
            )
            print(
                "[RECON ERROR]",
                repr(exc)
            )

    # --------------------------------------------------------
    # CLOSE
    # --------------------------------------------------------

    def close(self):
        try:
            if self.reader:
                self.reader.send("x")
        except Exception:
            pass

        self.stop_event.set()
        self.root.destroy()


# ============================================================
# MAIN
# ============================================================

def main():
    print("=" * 60)
    print("LIVE EIT V2")
    print("=" * 60)
    print(f"Electrodes       : {N_EL}")
    print(f"Excitation       : opposite / distance {DIST_EXC}")
    print(f"Measurements     : {FRAME_SIZE}/frame")
    print("Baseline         : median of 5 frames")
    print("Kalman           : enabled after reconstruction")
    print("=" * 60)

    root = tk.Tk()

    app = LiveEIT(root)

    root.protocol(
        "WM_DELETE_WINDOW",
        app.close
    )

    root.mainloop()


if __name__ == "__main__":
    main()
