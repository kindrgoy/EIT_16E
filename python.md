## Python Live EIT

The repository includes a Python-based graphical interface for live EIT acquisition and reconstruction.

The application is provided in:

```text
live.py
```

### Requirements

The Python application requires:

- Python 3
- `pyserial`
- `numpy`
- `matplotlib`
- `scipy`
- `pyEIT`

`tkinter` is also required for the graphical interface.

The application automatically reads the device configuration after connecting by sending the `p` command. This allows the Python application to obtain parameters such as the number of electrodes, excitation offset, and frame size directly from the firmware rather than hardcoding them. :contentReference[oaicite:2]{index=2}

### Install Dependencies

Create a Python environment if desired:

```bash
python -m venv .venv
```

Activate the environment.

Windows:

```bash
.venv\Scripts\activate
```

Linux/macOS:

```bash
source .venv/bin/activate
```

Install the required Python packages:

```bash
pip install pyserial numpy matplotlib scipy pyeit
```

### Run the Application

From the directory containing `live.py`:

```bash
python live.py
```

The application opens a graphical interface for:

- Serial connection
- Device configuration
- Single measurement commands
- Repeatability testing
- Multi-point calibration commands
- Baseline acquisition
- Live EIT reconstruction
- Kalman filtering
- Reconstruction parameter adjustment

The Python application stores its local settings in:

```text
live_eit_settings.json
```

This allows serial port, baudrate, mesh parameters, Kalman parameters, image resolution, and baseline settings to be retained between sessions. :contentReference[oaicite:3]{index=3}

### Connecting to the ESP32

1. Connect the ESP32-S3 to the computer.
2. Upload the EIT firmware.
3. Make sure the firmware is running.
4. Start:

```bash
python live.py
```

5. Select the correct serial port.
6. Set the baudrate to match the firmware.
7. Click **CONNECT + QUERY DEVICE**.

The current firmware uses:

```text
Baudrate = 921600
```

The application will automatically send:

```text
p
```

after connection and read the `### CONFIG ... ### CONFIG_END` block returned by the device. :contentReference[oaicite:4]{index=4}

### Live Reconstruction

After the device configuration has been received, the application builds the EIT protocol and reconstruction model using the parameters reported by the firmware.

For the current system:

```text
N_EL    = 16
OFFSET  = 8
```

The application generates the expected opposite-injection and adjacent-sensing measurement order from these parameters. :contentReference[oaicite:5]{index=5}

The reconstruction uses `pyEIT` and a Jacobian-based reconstruction method with Kotre regularization. :contentReference[oaicite:6]{index=6}

### Baseline Acquisition

Before starting live reconstruction:

1. Connect to the device.
2. Start frame acquisition.
3. Ensure the measurement medium is homogeneous.
4. Click **CAPTURE BASELINE**.
5. Wait until the requested number of baseline frames has been collected.
6. After the baseline is stored, introduce the test object or anomaly.

The application constructs the baseline from multiple frames before starting the difference reconstruction. :contentReference[oaicite:7]{index=7} :contentReference[oaicite:8]{index=8}

### Live EIT Workflow

The typical workflow is:

```text
ESP32-S3 + EIT firmware
        ↓
Connect Python application
        ↓
Query device configuration
        ↓
Start frame acquisition
        ↓
Capture homogeneous baseline
        ↓
Introduce anomaly / object
        ↓
Receive new EIT frames
        ↓
Baseline difference
        ↓
pyEIT reconstruction
        ↓
Gaussian smoothing
        ↓
Kalman filtering
        ↓
Live EIT image
```

The live application also provides controls for mesh resolution, image resolution, Kalman Q/R parameters, color range, Gaussian smoothing, and the number of baseline frames. :contentReference[oaicite:9]{index=9}

### Console Commands

The Python interface includes a console for sending commands directly to the firmware. Commands such as:

```text
m 0 8 1 2
rtest 0 8 1 2
c 470
calfit
cal
calclear
diag
help
g
x
```

can be sent without opening a separate Serial Monitor. :contentReference[oaicite:10]{index=10}

This makes the Python application the primary interface for both measurement control and live EIT visualization.