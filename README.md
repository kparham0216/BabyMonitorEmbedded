# Baby Monitor Embedded Firmware

Zephyr RTOS firmware for the Baby Monitor's nRF52840-based sensor board. Reads pulse oximetry (SpO₂, BPM) and temperature data from onboard sensors and exposes the readings over I2C to a Raspberry Pi master.

## Overview

The firmware operates in a continuous loop:

1. **Collect & Process** — Reads 500 samples from the MAX30102 pulse oximeter at ~50 Hz, performs peak detection for BPM (median-filtered), calculates SpO₂ via ratio-of-ratios, and reads temperature from the MAX30205.
2. **Slave Mode** — Suspends the I2C master bus, registers as an I2C slave (address `0x42`), and waits for the Raspberry Pi to read the processed data. After the read completes, it unregisters and resumes master mode.

### Hardware

| Component | I2C Address | Role |
|-----------|-------------|------|
| MAX30102 | 0x57 | Pulse oximeter (red + IR LEDs) |
| MAX30205 | 0x48 | Body temperature sensor |
| nRF52840 (this device) | 0x42 (slave) | Processes sensor data, serves it to Pi |

### I2C Slave Register Map

| Register | Offset | Description |
|----------|--------|-------------|
| BPM | 0x00–0x02 | Beats per minute (24-bit, big-endian) |
| IR | 0x03–0x05 | Raw IR value (24-bit) |
| Temperature | 0x06–0x09 | Raw temp (32-bit, big-endian) |
| SpO₂ | 0x0A | Blood oxygen percentage (0–100) |

### Pin Configuration

- **I2C0 (master to sensors)**: SCL = P0.04, SDA = P0.26
- **I2C1 (slave to Raspberry Pi)**: SCL = P0.04, SDA = P0.26

## Project Structure

```
BabyMonitorEmbedded/
├── CMakeLists.txt          # Zephyr build system entry point
├── prj.conf                # Kconfig: enables I2C, I2C target, power management
├── child_image/
│   └── mcuboot.conf        # MCUboot child image configuration
└── src/
    └── main.c              # Application code (sensor reading, processing, I2C slave)
```

## Prerequisites

- [nRF Connect SDK](https://docs.nordicsemi.com/bundle/ncs-latest/page/nrf/installation.html) (v2.6+ recommended)
- [nRF Connect for VS Code extension pack](https://marketplace.visualstudio.com/items?itemName=nordic-semiconductor.nrf-connect-extension-pack)
- A debug probe (J-Link, nRF DK as debugger, etc.)
- The custom `Board1` board definition (see `Board/BabyMonitorBoard/Me/Board1`)

## Setup with nRF Connect for VS Code

### 1. Install the nRF Connect Extension Pack

In VS Code, install **nRF Connect for VS Code Extension Pack** from the Extensions marketplace. On first launch it will guide you through installing the nRF Connect SDK and toolchain.

### 2. Open the application

Open the `Embedded/BabyMonitorEmbedded` folder (or the parent workspace) in VS Code.

### 3. Add the custom board root

The firmware targets the custom `Board1` board. You need to tell the build system where to find it:

1. Open the **nRF Connect** side panel
2. Click **Add Build Configuration**
3. Under **Board**, click **Add board root** and browse to:
   ```
   Board/BabyMonitorBoard
   ```
4. Select board `Me/Board1`

Alternatively, add to **Extra CMake arguments**:

```
-DBOARD_ROOT=<absolute-path-to>/Board/BabyMonitorBoard
```

### 4. Create a build configuration

1. In the **nRF Connect** side panel, click **Add Build Configuration**
2. Select board: `Board1` (vendor: Me)
3. Leave other settings at defaults (MCUboot is configured via `child_image/mcuboot.conf`)
4. Click **Build Configuration**

### 5. Build

Click **Build** in the nRF Connect side panel, or from terminal:

```bash
west build -b Board1 Embedded/BabyMonitorEmbedded -- -DBOARD_ROOT=<path-to>/Board/BabyMonitorBoard
```

### 6. Flash

Connect the debug probe to the board, then click **Flash** in the side panel, or:

```bash
west flash
```

## Configuration

Key Kconfig options in `prj.conf`:

| Option | Value | Purpose |
|--------|-------|---------|
| `CONFIG_I2C` | y | Enable I2C driver |
| `CONFIG_I2C_TARGET` | y | Enable I2C slave/target mode |
| `CONFIG_PM_DEVICE` | y | Device power management (suspend/resume I2C master) |
| `CONFIG_MAIN_STACK_SIZE` | 2048 | Main thread stack size |

Logging is disabled by default for production. To enable RTT logging for debugging, uncomment the `CONFIG_LOG` lines in `prj.conf`.

## Debugging

1. Uncomment the logging options in `prj.conf`:
   ```
   CONFIG_LOG=y
   CONFIG_LOG_DEFAULT_LEVEL=3
   CONFIG_LOG_BACKEND_RTT=y
   CONFIG_LOG_MODE_IMMEDIATE=y
   CONFIG_USE_SEGGER_RTT=y
   ```
2. Rebuild and flash
3. In the nRF Connect side panel, use **Connected Devices → RTT Terminal** or launch a SEGGER RTT Viewer to see log output
