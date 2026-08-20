# 16-Electrode Electrical Impedance Tomography (EIT)

Research prototype of a 16-electrode Electrical Impedance Tomography (EIT) system for experimental investigation of impedance-based tissue imaging.

> **Research prototype:** This project is intended for research and instrumentation development. It is not a clinically validated medical device and must not be used for clinical diagnosis.

---

## Overview

This project develops a multi-electrode EIT measurement system based on:

- **ESP32-S3** as the main controller
- **AD5933** for AC excitation and synchronous measurement
- **4 × CD74HC4067** analog multiplexers for 16-electrode switching
- **NE5532** for the current-injection stage
- **INA118** for differential voltage sensing
- **TL084** for frequency-selective filtering
- Python-based processing and reconstruction

The current EIT protocol uses:

- **16 electrodes**
- **Opposite current injection**
- **Adjacent voltage sensing**
- **50 kHz excitation**

The project is currently focused on instrument characterization, repeatability, calibration, and phantom-based EIT experiments.

---

## System Configuration

| Parameter | Current Configuration |
|---|---:|
| Electrodes | 16 |
| Injection pattern | Opposite |
| Sensing pattern | Adjacent |
| Excitation frequency | 50 kHz |
| AD5933 output range | 200 mVpp |
| AD5933 PGA | ×1 |
| Current-source resistor (`RS`) | 220 Ω |
| Receiver conversion resistor (`RCONV`) | 20 kΩ |
| AD5933 `RFB` | 20 kΩ |
| INA118 gain | ≈ 1× |
| Main controller | ESP32-S3 |

These values represent the current experimental baseline and may change during further characterization.

---

## Why 50 kHz?

The system currently uses a fixed excitation frequency of **50 kHz**.

A fixed-frequency configuration was selected during the initial development stage so that the analog front-end, filtering stage, injection stage, sensing stage, and AD5933 acquisition could be characterized around a known excitation frequency.

---

## Excitation Range and PGA Selection

The current AD5933 configuration is:

'''cpp
AD_OUTPUT_RANGE = CTRL_RANGE_200MV;
AD_PGA_GAIN     = CTRL_PGA_X1;
'''

The 200 mVpp range and ×1 PGA were selected during hardware characterization to obtain a measurable sensing signal while reducing unnecessary signal amplitude and avoiding excessive amplification in the analog front-end.

A higher excitation range can increase signal amplitude and potentially improve signal-to-noise ratio, but it can also increase the risk of:

- current-source overload
- amplifier saturation
- receiver clipping
- waveform distortion
- excessive input signal amplitude

Therefore, the lowest excitation level that provides adequate signal quality is preferred during the initial characterization stage.

---

## INA118 Gain Selection

The INA118 gain is controlled by the external gain resistor `RG`:

'''
G = 1 + 50 kΩ / RG
'''

Typical configurations are:

| `RG` | Approx. Gain |
|---:|---:|
| No external `RG` | 1× |
| 50 kΩ | 2× |
| 25 kΩ | 3× |
| 12.5 kΩ | 5× |

An approximately 5× configuration was tested during development.

For example, with approximately 520 mVpp differential sensing voltage:

'''
Vout ≈ 520 mVpp × 5
     ≈ 2.6 Vpp
'''

Clipping was observed because the INA118 was operated from a 3.3 V supply.

The present baseline therefore uses approximately **1× gain** to provide greater output headroom.

---

## 50 kHz Filtering

An active filtering stage is being developed to improve frequency selectivity around the excitation frequency.

The current target is a band-pass response centered around 50 kHz.

### High-Pass Section

'''
R = 330 Ω
C = 10 nF (103)
'''

Nominal first-order cutoff:

'''
fc ≈ 48.2 kHz
'''

The cutoff is calculated using:

'''
fc = 1 / (2πRC)
'''

### Low-Pass Section

'''
R = 220 Ω
C = 10 nF (103)
'''

Nominal first-order cutoff:

'''
fc ≈ 72.3 kHz
'''

This places the 50 kHz excitation inside the intended passband.

These values represent nominal first-order RC cutoff frequencies. The actual response depends on the complete TL084 filter topology, component tolerances, loading, and op-amp characteristics and must therefore be verified experimentally.

---

## Measurement Protocol

A representative single measurement uses four electrodes:

'''
HC = 0
LC = 8
HP = 1
LP = 2
'''

where:

- `HC` and `LC` are the current-injection electrodes
- `HP` and `LP` are the sensing electrodes

The firmware prevents the sensing electrodes from overlapping the current electrodes.

The raw AD5933 measurement consists of:

'''
Re
Im
Magnitude
'''

Magnitude is calculated as:

'''
Magnitude = sqrt(Re² + Im²)
'''

The current acquisition firmware can average several complex measurements before calculating the reported magnitude.

---

## Repeatability

Repeatability is one of the primary validation steps before calibration and EIT reconstruction.

The firmware provides:

'''
rtest HC LC HP LP
'''

Example:

'''
rtest 0 8 1 2
'''

This performs 30 measurements using the specified electrode configuration and prints one magnitude value for each measurement.

The resulting dataset can be evaluated using:

- mean
- standard deviation
- coefficient of variation (CV)
- minimum and maximum values

A CV below approximately 5% is currently used as an initial engineering target for fixed-connection repeatability.

This is a development criterion and is not a clinical specification.

---

## Known-Resistor Characterization

Known resistors are used to characterize the complete measurement chain before applying the system to an EIT phantom.

Representative resistor values tested during development include:

'''
150 Ω
218 Ω
327 Ω
468 Ω
674 Ω
1193 Ω
'''

The objective is to determine:

1. whether the system responds consistently to changes in resistance
2. whether the response is repeatable
3. the usable impedance range
4. the transfer function between known resistance and measured magnitude

The calibration model is intentionally not assumed to be linear before experimental characterization.

---

## Hardware Characterization

The measurement chain is checked at several points with an oscilloscope:

- excitation signal
- current-source node
- DUT voltage
- HP-LP sensing voltage
- INA118 output
- VIN_AD

The goal is to verify that:

- the excitation frequency is stable
- the analog waveform is sufficiently clean
- the DUT response changes with impedance
- the sensing signal changes with the DUT
- the INA118 does not clip
- VIN_AD contains a measurable component around 50 kHz

---

## Development Findings

Several important behaviors were identified during development.

### AD5933 Measurement State

After some ESP32 resets, raw magnitude measurements could become almost constant even when the analog signal path remained active.

This led to additional investigation of:

- AD5933 initialization
- measurement state
- serial communication
- acquisition timing

### INA118 Clipping

A high INA118 gain caused output clipping with the 3.3 V supply.

The gain was therefore reduced for the baseline configuration.
---

## License

This project is provided for research and educational purposes.