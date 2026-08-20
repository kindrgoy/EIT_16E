import json
import os
import time
import queue
import threading
import tkinter as tk
from tkinter import ttk, messagebox, scrolledtext

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
# SETTINGS FILE (reproducibility)
# ============================================================

SETTINGS_PATH = os.path.join(
    os.path.dirname(os.path.abspath(__file__)),
    "live_eit_settings.json"
)

DEFAULT_SETTINGS = {
    "port": "COM12",
    "baud": 921600,
    "mesh_h0": 0.08,
    "image_size": 96,
    "kalman_q": 0.005,
    "kalman_r": 0.05,
    "color_limit": 1.0,
    "gaussian_sigma": 0.7,
    "baseline_frames": 5,
    "last_cal_resistor": 1000.0,
}


def load_settings():
    if os.path.exists(SETTINGS_PATH):
        try:
            with open(SETTINGS_PATH, "r") as f:
                data = json.load(f)
            merged = dict(DEFAULT_SETTINGS)
            merged.update(data)
            return merged
        except Exception:
            pass
    return dict(DEFAULT_SETTINGS)


def save_settings(d):
    try:
        with open(SETTINGS_PATH, "w") as f:
            json.dump(d, f, indent=2)
    except Exception as exc:
        print("[SETTINGS] gagal simpan:", exc)


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
# PROTOCOL BUILDER — dibangun dari nilai yang DILAPORKAN device,
# bukan hardcode. Ini kunci "penyelarasan" dengan firmware.
# ============================================================

def build_expected_order(n_el, offset):
    """Urutan (hc,lc,hp,lp) sesuai skema opposite-inject/adjacent-sense
    yang dipakai firmware. Kalau firmware berubah skema, fungsi ini yang
    perlu diupdate — tapi n_el/offset sendiri tetap dibaca dari device."""
    out = []
    for k in range(n_el):
        hc = k
        lc = (k + offset) % n_el
        for hp in range(n_el):
            lp = (hp + 1) % n_el
            if hp == hc or hp == lc or lp == hc or lp == lc:
                continue
            out.append((hc, lc, hp, lp))
    return out


# ============================================================
# PY-EIT RECONSTRUCTION — parametrized oleh n_el/offset dari device
# ============================================================

class EITReconstructor:
    def __init__(self, n_el, dist_exc, h0, image_size):
        self.n_el = int(n_el)
        self.dist_exc = int(dist_exc)
        self.h0 = float(h0)
        self.image_size = int(image_size)

        self.ready = False
        self.error = None
        self.frame_size = None

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

            self.protocol = protocol.create(
                n_el=self.n_el,
                dist_exc=self.dist_exc,
                step_meas=1,
                parser_meas="std"
            )

            self.frame_size = self.protocol.n_exc * self.protocol.n_meas

            self.mesh = mesh.create(n_el=self.n_el, h0=self.h0)
            self.eit = jac.JAC(self.mesh, self.protocol)
            self.eit.setup(p=0.5, lamb=0.05, method="kotre", jac_normalized=True)

            x = np.linspace(-1.0, 1.0, self.image_size)
            y = np.linspace(-1.0, 1.0, self.image_size)
            self.X, self.Y = np.meshgrid(x, y)
            self.mask = self.X**2 + self.Y**2 <= 1.0

            self.ready = True
        except Exception as exc:
            self.error = repr(exc)
            self.ready = False

    def reconstruct(self, current, baseline):
        if not self.ready:
            raise RuntimeError("pyEIT belum siap: " + str(self.error))

        v1 = np.asarray(current, dtype=np.complex128)
        v0 = np.asarray(baseline, dtype=np.complex128)

        if v1.size != self.frame_size or v0.size != self.frame_size:
            raise ValueError(f"Frame harus berisi {self.frame_size} measurement")

        ds = self.eit.solve(v1, v0, normalize=True)
        ds = np.real(ds)

        centers = self.mesh.elem_centers[:, :2]
        img = griddata(centers, ds, (self.X, self.Y), method="linear", fill_value=0.0)
        img[~self.mask] = np.nan
        return img


# ============================================================
# SERIAL READER — parse frame blocks DAN generic ### blocks (config,
# calibration, diag, dll) supaya semua respons device bisa ditampilkan.
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
            self.ser = serial.Serial(self.port, self.baud, timeout=0.05)
            time.sleep(1.0)
            self.q.put(("status", f"CONNECTED {self.port}"))

            frame_rows = []
            in_frame = False

            config_lines = []
            in_config = False

            while not self.stop_event.is_set():
                raw = self.ser.readline()
                if not raw:
                    continue

                line = raw.decode("utf-8", errors="ignore").strip()
                if not line:
                    continue

                # ---- FRAME BLOCK ----
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

                if in_frame:
                    if line.startswith("idx,"):
                        continue
                    parts = line.split(",")
                    if len(parts) != 10:
                        continue
                    try:
                        idx = int(parts[0])
                        hc = int(parts[1]); lc = int(parts[2])
                        hp = int(parts[3]); lp = int(parts[4])
                        re = float(parts[5]); im = float(parts[6])
                        mag = float(parts[7]); z = float(parts[8])
                        ok = int(parts[9])
                    except ValueError:
                        continue
                    if ok != 1:
                        continue
                    if not (np.isfinite(re) and np.isfinite(im)):
                        continue
                    frame_rows.append({
                        "idx": idx, "hc": hc, "lc": lc, "hp": hp, "lp": lp,
                        "re": re, "im": im, "mag": mag, "z": z,
                    })
                    continue

                # ---- CONFIG BLOCK (dipakai untuk auto-align) ----
                if line == "### CONFIG":
                    in_config = True
                    config_lines = []
                    continue

                if line == "### CONFIG_END":
                    if in_config:
                        cfg = {}
                        for cl in config_lines:
                            if "=" in cl:
                                k, v = cl.split("=", 1)
                                cfg[k.strip()] = v.strip()
                        self.q.put(("config", cfg))
                    in_config = False
                    continue

                if in_config:
                    config_lines.append(line)
                    self.q.put(("line", line))
                    continue

                # ---- GENERIC ### BLOCKS (calibration, diag, single, dll)
                # Cukup diteruskan mentah ke console log, tidak perlu
                # parsing khusus per jenis — firmware bisa menambah
                # block baru tanpa Python perlu diubah.
                self.q.put(("line", line))

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
                self.ser.write((text.strip() + "\n").encode("ascii"))
                self.ser.flush()
            self.q.put(("sent", text.strip()))
            return True
        except Exception as exc:
            self.q.put(("error", repr(exc)))
            return False


# ============================================================
# FRAME CONVERSION
# ============================================================

def frame_to_complex(frame, expected_order, frame_size):
    lookup = {}
    for row in frame:
        key = (row["hc"], row["lc"], row["hp"], row["lp"])
        lookup[key] = complex(row["re"], row["im"])

    values = np.full(frame_size, np.nan + 1j * np.nan, dtype=np.complex128)
    for i, key in enumerate(expected_order):
        if key in lookup:
            values[i] = lookup[key]
    return values


# ============================================================
# GUI
# ============================================================

class LiveEIT:
    def __init__(self, root):
        self.root = root
        self.root.title("Live EIT Imaging v3 - ESP32-S3 / AD5933 (auto-aligned)")
        self.root.geometry("1420x860")

        self.settings = load_settings()

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

        # Device-reported protocol params — TIDAK di-hardcode.
        self.device_n_el = None
        self.device_offset = None
        self.device_frame_meas = None
        self.expected_order = None
        self.device_config = {}

        self._build_gui()
        self.root.after(20, self.poll_queue)
        self.root.protocol("WM_DELETE_WINDOW", self.on_close)

    # --------------------------------------------------------
    # GUI LAYOUT
    # --------------------------------------------------------

    def _build_gui(self):
        left = ttk.Frame(self.root)
        left.pack(side="left", fill="both", expand=True)

        right = ttk.Frame(self.root, width=360)
        right.pack(side="right", fill="y")
        right.pack_propagate(False)

        # ---- Plot area ----
        fig = Figure(figsize=(7.0, 7.0), dpi=100)
        self.ax = fig.add_subplot(111)
        self.ax.set_aspect("equal")
        self.ax.set_xlim(-1.12, 1.12)
        self.ax.set_ylim(-1.12, 1.12)
        self.ax.set_title("LIVE EIT — baseline difference")

        self.image = self.ax.imshow(
            np.zeros((96, 96)), extent=(-1, 1, -1, 1), origin="lower",
            cmap="RdBu_r", interpolation="bilinear",
            vmin=-self.settings["color_limit"], vmax=self.settings["color_limit"]
        )
        self.colorbar = fig.colorbar(self.image, ax=self.ax, fraction=0.046, pad=0.04)

        theta = np.linspace(0, 2 * np.pi, 300)
        self.ax.plot(np.cos(theta), np.sin(theta), linewidth=1)
        self.electrode_dots, = self.ax.plot([], [], "o", markersize=5, color="black")
        self.electrode_labels = []

        self.canvas = FigureCanvasTkAgg(fig, master=left)
        self.canvas.get_tk_widget().pack(fill="both", expand=True)

        # ---- Console log (bawah plot) ----
        console_frame = ttk.Frame(left)
        console_frame.pack(fill="x", side="bottom")

        ttk.Label(console_frame, text="Console (respons device mentah)").pack(anchor="w", padx=4)
        self.console = scrolledtext.ScrolledText(console_frame, height=8, font=("Consolas", 9))
        self.console.pack(fill="x", padx=4, pady=(0, 4))

        cmd_row = ttk.Frame(console_frame)
        cmd_row.pack(fill="x", padx=4, pady=(0, 6))
        self.cmd_var = tk.StringVar()
        cmd_entry = ttk.Entry(cmd_row, textvariable=self.cmd_var)
        cmd_entry.pack(side="left", fill="x", expand=True)
        cmd_entry.bind("<Return>", lambda e: self.send_raw_command())
        ttk.Button(cmd_row, text="Send", command=self.send_raw_command).pack(side="left", padx=(4, 0))

        # ---- Right panel: notebook with tabs ----
        nb = ttk.Notebook(right)
        nb.pack(fill="both", expand=True, padx=8, pady=8)

        tab_conn = ttk.Frame(nb)
        tab_cal = ttk.Frame(nb)
        tab_recon = ttk.Frame(nb)
        nb.add(tab_conn, text="Connection")
        nb.add(tab_cal, text="Calibration")
        nb.add(tab_recon, text="Reconstruction")

        self._build_connection_tab(tab_conn)
        self._build_calibration_tab(tab_cal)
        self._build_reconstruction_tab(tab_recon)

        # ---- Status footer ----
        footer = ttk.Frame(right)
        footer.pack(fill="x", padx=8, pady=(0, 8))

        self.status_var = tk.StringVar(value="DISCONNECTED")
        self.frame_var = tk.StringVar(value="Frame: 0")
        self.fps_var = tk.StringVar(value="FPS: 0.0")
        self.measure_var = tk.StringVar(value="Measurements: -/-")
        self.baseline_var = tk.StringVar(value="Baseline: NOT SET")
        self.recon_var = tk.StringVar(value="pyEIT: waiting for device config")
        self.protocol_var = tk.StringVar(value="Protocol: -")

        for var in [self.status_var, self.protocol_var, self.frame_var,
                    self.fps_var, self.measure_var, self.baseline_var, self.recon_var]:
            ttk.Label(footer, textvariable=var).pack(anchor="w")

    def _build_connection_tab(self, parent):
        ttk.Label(parent, text="Serial Port").pack(anchor="w", padx=10, pady=(10, 0))
        self.port_var = tk.StringVar(value=self.settings["port"])
        self.port_box = ttk.Combobox(parent, textvariable=self.port_var)
        self.port_box.pack(fill="x", padx=10, pady=4)
        self.refresh_ports()

        ttk.Button(parent, text="Refresh Ports", command=self.refresh_ports).pack(fill="x", padx=10, pady=2)

        ttk.Label(parent, text="Baudrate").pack(anchor="w", padx=10, pady=(8, 0))
        self.baud_var = tk.StringVar(value=str(self.settings["baud"]))
        ttk.Entry(parent, textvariable=self.baud_var).pack(fill="x", padx=10, pady=4)

        ttk.Separator(parent).pack(fill="x", padx=10, pady=10)

        ttk.Button(parent, text="CONNECT + QUERY DEVICE", command=self.start).pack(fill="x", padx=10, pady=3)
        ttk.Button(parent, text="STOP", command=self.stop).pack(fill="x", padx=10, pady=3)

        ttk.Separator(parent).pack(fill="x", padx=10, pady=10)

        ttk.Button(parent, text="Re-query config (p)", command=lambda: self.send_cmd("p")).pack(fill="x", padx=10, pady=2)
        ttk.Button(parent, text="Diagnostic (diag)", command=lambda: self.send_cmd("diag")).pack(fill="x", padx=10, pady=2)
        ttk.Button(parent, text="Help", command=lambda: self.send_cmd("help")).pack(fill="x", padx=10, pady=2)
        ttk.Button(parent, text="I2C Scan", command=lambda: self.send_cmd("scan")).pack(fill="x", padx=10, pady=2)

    def _build_calibration_tab(self, parent):
        ttk.Label(
            parent,
            text=(
                "Kalibrasi multi-titik (polynomial).\n"
                "Pasang R referensi via jumper HC-HP / LC-LP\n"
                "sebelum tiap titik (lihat panduan sebelumnya)."
            ),
            wraplength=320, justify="left"
        ).pack(anchor="w", padx=10, pady=(10, 6))

        ttk.Label(parent, text="R referensi (Ohm)").pack(anchor="w", padx=10)
        self.cal_r_var = tk.StringVar(value=str(self.settings["last_cal_resistor"]))
        ttk.Entry(parent, textvariable=self.cal_r_var).pack(fill="x", padx=10, pady=4)

        ttk.Button(parent, text="Tambah Titik Kalibrasi (c <R>)",
                   command=self.add_cal_point).pack(fill="x", padx=10, pady=3)

        ttk.Separator(parent).pack(fill="x", padx=10, pady=8)

        ttk.Button(parent, text="Fit & Simpan (calfit)",
                   command=lambda: self.send_cmd("calfit")).pack(fill="x", padx=10, pady=2)
        ttk.Button(parent, text="Lihat Status (cal)",
                   command=lambda: self.send_cmd("cal")).pack(fill="x", padx=10, pady=2)
        ttk.Button(parent, text="Hapus Kalibrasi (calclear)",
                   command=self.confirm_calclear).pack(fill="x", padx=10, pady=2)

        ttk.Separator(parent).pack(fill="x", padx=10, pady=8)

        ttk.Label(parent, text="Single-point test (m hc lc hp lp)").pack(anchor="w", padx=10)
        row = ttk.Frame(parent)
        row.pack(fill="x", padx=10, pady=4)
        self.m_hc = tk.StringVar(value="0")
        self.m_lc = tk.StringVar(value="8")
        self.m_hp = tk.StringVar(value="1")
        self.m_lp = tk.StringVar(value="2")
        for label, var in [("HC", self.m_hc), ("LC", self.m_lc), ("HP", self.m_hp), ("LP", self.m_lp)]:
            ttk.Label(row, text=label).pack(side="left")
            ttk.Entry(row, textvariable=var, width=4).pack(side="left", padx=(2, 8))
        ttk.Button(parent, text="Ukur Titik Ini",
                   command=self.send_single_measure).pack(fill="x", padx=10, pady=3)
        ttk.Button(parent, text="Repeatability Test (rtest, 30x)",
                   command=lambda: self.send_cmd("rtest")).pack(fill="x", padx=10, pady=3)

    def _build_reconstruction_tab(self, parent):
        self.mesh_var = tk.StringVar(value=str(self.settings["mesh_h0"]))
        self.image_var = tk.StringVar(value=str(self.settings["image_size"]))
        self.q_var = tk.StringVar(value=str(self.settings["kalman_q"]))
        self.r_var = tk.StringVar(value=str(self.settings["kalman_r"]))
        self.clim_var = tk.StringVar(value=str(self.settings["color_limit"]))
        self.baseline_frames_var = tk.StringVar(value=str(self.settings["baseline_frames"]))

        for label, var in [
            ("Mesh h0", self.mesh_var),
            ("Image resolution", self.image_var),
            ("Kalman Q", self.q_var),
            ("Kalman R", self.r_var),
            ("Color ±", self.clim_var),
            ("Baseline frames", self.baseline_frames_var),
        ]:
            ttk.Label(parent, text=label).pack(anchor="w", padx=10, pady=(8, 0))
            ttk.Entry(parent, textvariable=var).pack(fill="x", padx=10, pady=2)

        ttk.Button(parent, text="APPLY + SAVE SETTINGS",
                   command=self.apply_settings).pack(fill="x", padx=10, pady=(10, 3))

        ttk.Separator(parent).pack(fill="x", padx=10, pady=10)

        ttk.Button(parent, text="CAPTURE BASELINE",
                   command=self.capture_baseline).pack(fill="x", padx=10, pady=3)
        ttk.Button(parent, text="CLEAR BASELINE",
                   command=self.clear_baseline).pack(fill="x", padx=10, pady=3)
        ttk.Button(parent, text="RESET KALMAN",
                   command=self.reset_kalman).pack(fill="x", padx=10, pady=3)
        ttk.Button(parent, text="START SWEEP (g)",
                   command=lambda: self.send_cmd("g")).pack(fill="x", padx=10, pady=(14, 3))
        ttk.Button(parent, text="STOP SWEEP (x)",
                   command=lambda: self.send_cmd("x")).pack(fill="x", padx=10, pady=3)

    # --------------------------------------------------------
    # LOGGING
    # --------------------------------------------------------

    def log(self, text):
        self.console.insert("end", text + "\n")
        self.console.see("end")

    # --------------------------------------------------------
    # SERIAL / COMMANDS
    # --------------------------------------------------------

    def refresh_ports(self):
        if serial is None:
            self.port_box["values"] = []
            return
        ports = [p.device for p in serial.tools.list_ports.comports()]
        self.port_box["values"] = ports
        if self.port_var.get() not in ports and ports:
            self.port_var.set(ports[0])

    def send_cmd(self, text):
        if self.reader and self.reader.is_alive():
            self.reader.send(text)
        else:
            messagebox.showwarning("Not connected", "Connect ke device dulu.")

    def send_raw_command(self):
        text = self.cmd_var.get().strip()
        if not text:
            return
        self.send_cmd(text)
        self.cmd_var.set("")

    def add_cal_point(self):
        try:
            r = float(self.cal_r_var.get())
            if r <= 0:
                raise ValueError
        except ValueError:
            messagebox.showerror("Error", "Nilai R referensi tidak valid.")
            return
        self.settings["last_cal_resistor"] = r
        save_settings(self.settings)
        self.send_cmd(f"c {r}")

    def confirm_calclear(self):
        if messagebox.askyesno("Konfirmasi", "Hapus semua data kalibrasi tersimpan di device?"):
            self.send_cmd("calclear")

    def send_single_measure(self):
        try:
            hc = int(self.m_hc.get()); lc = int(self.m_lc.get())
            hp = int(self.m_hp.get()); lp = int(self.m_lp.get())
        except ValueError:
            messagebox.showerror("Error", "HC/LC/HP/LP harus angka 0-15.")
            return
        self.send_cmd(f"m {hc} {lc} {hp} {lp}")

    def start(self):
        if self.reader and self.reader.is_alive():
            messagebox.showinfo("Info", "Sudah terkoneksi.")
            return
        if serial is None:
            messagebox.showerror("Dependency", "pyserial belum terinstall.")
            return
        try:
            baud = int(self.baud_var.get())
        except ValueError:
            messagebox.showerror("Error", "Baudrate tidak valid.")
            return

        self.settings["port"] = self.port_var.get()
        self.settings["baud"] = baud
        save_settings(self.settings)

        self.stop_event.clear()
        self.reader = SerialReader(self.port_var.get(), baud, self.q, self.stop_event)
        self.reader.start()

        # Setelah serial settle, minta config device — INI YANG
        # MENYELARASKAN Python dengan firmware, bukan hardcode.
        self.root.after(1300, lambda: self.send_cmd("p"))

    def stop(self):
        if self.reader:
            self.reader.send("x")
        self.stop_event.set()
        self.status_var.set("STOPPED")

    def on_close(self):
        try:
            if self.reader:
                self.reader.send("x")
        except Exception:
            pass
        self.stop_event.set()
        save_settings(self.settings)
        self.root.destroy()

    # --------------------------------------------------------
    # DEVICE CONFIG HANDLING — inti penyelarasan
    # --------------------------------------------------------

    def apply_device_config(self, cfg):
        self.device_config = cfg
        self.log(f"[CONFIG] Diterima {len(cfg)} field dari device.")

        protocol_str = cfg.get("PROTOCOL", "UNKNOWN")
        self.protocol_var.set(f"Protocol: {protocol_str}")

        try:
            n_el = int(cfg["N_EL"])
            offset = int(cfg["OFFSET"])
            frame_meas = int(cfg["FRAME_MEAS"])
        except (KeyError, ValueError):
            messagebox.showerror(
                "Config error",
                "Device tidak melaporkan N_EL/OFFSET/FRAME_MEAS dengan benar.\n"
                "Cek firmware — command 'p' harus print block ### CONFIG."
            )
            return

        changed = (
            n_el != self.device_n_el or
            offset != self.device_offset or
            frame_meas != self.device_frame_meas
        )

        self.device_n_el = n_el
        self.device_offset = offset
        self.device_frame_meas = frame_meas
        self.expected_order = build_expected_order(n_el, offset)

        if len(self.expected_order) != frame_meas:
            self.log(
                f"[WARN] Urutan measurement Python ({len(self.expected_order)}) "
                f"!= FRAME_MEAS device ({frame_meas}). Skema opposite/adjacent "
                f"mungkin sudah berubah di firmware — build_expected_order() "
                f"perlu disesuaikan."
            )

        self.status_var.set(f"CONNECTED {self.port_var.get()} | N_EL={n_el} OFFSET={offset}")

        if changed or self.reconstructor is None:
            self._rebuild_reconstructor()

    def _rebuild_reconstructor(self):
        if self.device_n_el is None:
            return
        try:
            h0 = float(self.mesh_var.get())
            size = int(self.image_var.get())
        except ValueError:
            messagebox.showerror("Error", "Mesh h0 / image resolution tidak valid.")
            return

        self.reconstructor = EITReconstructor(
            n_el=self.device_n_el,
            dist_exc=self.device_offset,
            h0=h0,
            image_size=size,
        )

        if self.reconstructor.ready:
            self.recon_var.set(
                f"pyEIT READY | n_el={self.device_n_el} offset={self.device_offset} "
                f"frame_size={self.reconstructor.frame_size}"
            )
            self._draw_electrode_markers()
        else:
            self.recon_var.set("pyEIT ERROR: " + str(self.reconstructor.error))

        self.kalman = None  # rebuild lazily di process_frame

    def _draw_electrode_markers(self):
        n = self.device_n_el
        xs = [np.cos(2 * np.pi * i / n) for i in range(n)]
        ys = [np.sin(2 * np.pi * i / n) for i in range(n)]
        self.electrode_dots.set_data(xs, ys)

        for t in self.electrode_labels:
            t.remove()
        self.electrode_labels = []
        for i in range(n):
            a = 2 * np.pi * i / n
            t = self.ax.text(1.07 * np.cos(a), 1.07 * np.sin(a), str(i),
                              ha="center", va="center", fontsize=8)
            self.electrode_labels.append(t)
        self.canvas.draw_idle()

    # --------------------------------------------------------
    # BASELINE
    # --------------------------------------------------------

    def capture_baseline(self):
        if not self.reader or not self.reader.is_alive():
            messagebox.showwarning("Not running", "Connect dan START dulu.")
            return
        if self.reconstructor is None or not self.reconstructor.ready:
            messagebox.showwarning("Not ready", "pyEIT belum siap (query config dulu).")
            return

        try:
            n_target = max(1, int(self.baseline_frames_var.get()))
        except ValueError:
            n_target = self.settings["baseline_frames"]

        self.baseline_buffer = []
        self.baseline_target = n_target
        self.baseline = None
        self.baseline_var.set(f"Baseline: 0/{self.baseline_target}")
        self.status_var.set("CAPTURING BASELINE — jangan ubah media")
        if self.kalman:
            self.kalman.reset()

    def clear_baseline(self):
        self.baseline = None
        self.baseline_buffer = []
        self.baseline_target = 0
        self.baseline_var.set("Baseline: NOT SET")
        if self.kalman:
            self.kalman.reset()
        self.status_var.set("Baseline cleared")

    def reset_kalman(self):
        if self.kalman:
            self.kalman.reset()
        self.status_var.set("Kalman reset")

    def apply_settings(self):
        try:
            h0 = float(self.mesh_var.get())
            image_size = int(self.image_var.get())
            q = float(self.q_var.get())
            r = float(self.r_var.get())
            clim = float(self.clim_var.get())
            baseline_frames = int(self.baseline_frames_var.get())

            if not (0.03 <= h0 <= 0.30):
                raise ValueError("Mesh h0 sebaiknya 0.03-0.30")
            if not (32 <= image_size <= 192):
                raise ValueError("Image resolution 32-192")
            if q <= 0 or r <= 0 or clim <= 0:
                raise ValueError("Q, R, Color harus > 0")
            if baseline_frames < 1:
                raise ValueError("Baseline frames minimal 1")

            self.settings.update({
                "mesh_h0": h0, "image_size": image_size,
                "kalman_q": q, "kalman_r": r, "color_limit": clim,
                "baseline_frames": baseline_frames,
            })
            save_settings(self.settings)

            self._rebuild_reconstructor()

            self.image.set_data(np.zeros((image_size, image_size)))
            self.image.set_clim(-clim, clim)
            self.canvas.draw_idle()

            self.status_var.set("Settings applied & saved")
        except Exception as exc:
            messagebox.showerror("Settings error", str(exc))

    # --------------------------------------------------------
    # QUEUE / FRAME PROCESSING
    # --------------------------------------------------------

    def poll_queue(self):
        try:
            while True:
                kind, data = self.q.get_nowait()

                if kind == "status":
                    self.status_var.set(data)
                elif kind == "sent":
                    self.log(f">> {data}")
                elif kind == "line":
                    self.log(data)
                elif kind == "config":
                    self.apply_device_config(data)
                elif kind == "error":
                    self.status_var.set("ERROR")
                    self.log("[SERIAL ERROR] " + data)
                elif kind == "frame":
                    self.process_frame(data)
        except queue.Empty:
            pass
        self.root.after(20, self.poll_queue)

    def process_frame(self, rows):
        if self.reconstructor is None or not self.reconstructor.ready or self.expected_order is None:
            return

        frame_size = self.reconstructor.frame_size
        v = frame_to_complex(rows, self.expected_order, frame_size)

        valid = np.isfinite(v.real) & np.isfinite(v.imag)
        valid_count = int(valid.sum())
        self.measure_var.set(f"Measurements: {valid_count}/{frame_size}")

        if valid_count < int(frame_size * 0.9):
            return

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
        self.frame_var.set(f"Frame: {self.total_frames}")
        self.fps_var.set(f"FPS: {self.fps:.1f}")

        if self.baseline_target > 0:
            self.baseline_buffer.append(v.copy())
            n = len(self.baseline_buffer)
            self.baseline_var.set(f"Baseline: {n}/{self.baseline_target}")
            if n >= self.baseline_target:
                stack = np.vstack(self.baseline_buffer)
                self.baseline = (
                    np.median(stack.real, axis=0) + 1j * np.median(stack.imag, axis=0)
                )
                self.baseline_buffer = []
                self.baseline_target = 0
                if self.kalman:
                    self.kalman.reset()
                self.baseline_var.set("Baseline: SET \u2713")
                self.status_var.set("Baseline ready \u2014 ubah medium/objek sekarang")
            return

        if self.baseline is None:
            self.status_var.set("RUNNING \u2014 capture baseline dulu")
            return

        try:
            img = self.reconstructor.reconstruct(v, self.baseline)

            sigma = self.settings.get("gaussian_sigma", 0.7)
            if sigma > 0:
                finite = np.isfinite(img)
                temp = np.nan_to_num(img, nan=0.0)
                temp = gaussian_filter(temp, sigma=sigma)
                img = temp
                img[~finite] = np.nan

            flat = np.nan_to_num(img, nan=0.0).reshape(-1)
            if self.kalman is None or self.kalman.size != flat.size:
                self.kalman = VectorKalman(flat.size, float(self.q_var.get()), float(self.r_var.get()))
            filtered = self.kalman.update(flat).reshape(img.shape)

            size = filtered.shape[0]
            yy, xx = np.indices(filtered.shape)
            x = 2.0 * xx / max(size - 1, 1) - 1.0
            y = 2.0 * yy / max(size - 1, 1) - 1.0
            filtered[x * x + y * y > 1.0] = np.nan

            self.image.set_data(filtered)
            self.canvas.draw_idle()
            self.status_var.set("LIVE RECONSTRUCTION")
        except Exception as exc:
            self.status_var.set("RECONSTRUCTION ERROR")
            self.log("[RECON ERROR] " + repr(exc))


# ============================================================
# MAIN
# ============================================================

def main():
    root = tk.Tk()
    app = LiveEIT(root)
    root.mainloop()


if __name__ == "__main__":
    main()