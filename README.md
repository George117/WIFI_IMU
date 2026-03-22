# WiFi IMU

Real-time wireless IMU data acquisition system. An ESP8266 reads a GY-91 breakout board (MPU6500 + BMP280) and streams 7-DOF sensor data over UDP at 100 Hz. A Python GUI receives the packets and plots the signals live.

Designed for measuring the dynamic response of a Stewart platform.

## Hardware

| Component | Description |
|-----------|-------------|
| ESP8266 (NodeMCU / D1 Mini) | Microcontroller with WiFi |
| GY-91 (MPU6500 + BMP280) | 6-axis IMU + barometric pressure sensor |

### Wiring

| GY-91 Pin | ESP8266 Pin |
|-----------|-------------|
| SDA | D2 |
| SCL | D1 |
| VCC | 3.3 V |
| GND | GND |

### I2C Addresses

| Device  | Address | Notes |
|---------|---------|-------|
| MPU6500 | 0x69 | AD0 pulled high |
| BMP280  | 0x77 | SDO pulled high |

## Project Structure

```
WIFI_IMU/
  wifi_imu/
    wifi_imu.ino        # ESP8266 Arduino sketch
  gui/
    imu_viewer.py       # Python GUI application
    config.json         # GUI configuration
    requirements.txt    # Python dependencies
  PROTOCOL.md           # Binary packet format specification
  README.md             # This file
```

## ESP8266 Firmware

### Dependencies

Install via Arduino IDE Library Manager:

- **MPU9250_WE** by Wolfgang Ewald (provides `MPU6500_WE.h`)

### Network Configuration

Edit the following constants in `wifi_imu/wifi_imu.ino`:

```cpp
const char* WIFI_SSID     = "<YOUR_SSID>";
const char* WIFI_PASSWORD = "<YOUR_PASSWORD>";

IPAddress staticIP(<ESP8266_IP>);        // ESP8266 fixed IP
IPAddress gateway(<GATEWAY_IP>);
IPAddress subnet(255, 255, 255, 0);
IPAddress dns(8, 8, 8, 8);

IPAddress targetIP(<PC_IP>);             // PC running the GUI
const uint16_t TARGET_PORT = 4210;
```

### Sensor Configuration

| Sensor | Parameter | Value |
|--------|-----------|-------|
| MPU6500 | Accelerometer range | +/-4 g |
| MPU6500 | Gyroscope range | +/-500 deg/s |
| MPU6500 | Gyro DLPF | ~41 Hz bandwidth |
| BMP280 | Pressure oversampling | x4 |
| BMP280 | Temperature oversampling | x1 |
| BMP280 | IIR filter | coefficient 4 |
| BMP280 | Mode | Normal (continuous) |

### Flashing

1. Open `wifi_imu/wifi_imu.ino` in the Arduino IDE
2. Select board: **Generic ESP8266 Module** (or your specific board)
3. Set the WiFi credentials and IP addresses
4. Upload

### Serial Output

On successful boot:

```
[WiFi IMU] Starting...
[MPU6500] OK
[BMP280] OK
[WiFi] Connecting...
[WiFi] Connected – IP: <ESP8266_IP>
[UDP] Sending to <PC_IP>:4210 every 10000 us
```

## UDP Packet Format

43-byte fixed-size, little-endian binary packets. See [PROTOCOL.md](PROTOCOL.md) for the full specification.

| Field | Type | Unit |
|-------|------|------|
| sync | 2 x uint8 | `0xAA 0x55` |
| packet_id | uint32 | — |
| timestamp_ms | uint32 | ms |
| accel_x, accel_y, accel_z | 3 x float32 | g |
| gyro_x, gyro_y, gyro_z | 3 x float32 | deg/s |
| pressure | float32 | Pa |
| temperature | float32 | deg C |
| checksum | uint8 | XOR of bytes 0-41 |

## Python GUI

### Setup

```bash
cd gui
python -m venv venv
source venv/bin/activate    # Linux/macOS
# venv\Scripts\activate     # Windows
pip install -r requirements.txt
```

### Running

```bash
python imu_viewer.py
```

Or with CLI overrides:

```bash
python imu_viewer.py --ip 0.0.0.0 --port 4210
```

### Configuration

All GUI settings are stored in `gui/config.json`. The application reads this file at startup. CLI arguments (`--ip`, `--port`) override the config file values.

```json
{
  "ip": "<BIND_IP>",
  "port": 4210,
  "time_window": 10,
  "axes": {
    "accel_x": "X",
    "accel_y": "Y",
    "accel_z": "Z",
    "gyro_x": "Roll",
    "gyro_y": "Pitch",
    "gyro_z": "Yaw"
  },
  "visible": {
    "accel_x": true,
    "accel_y": true,
    "accel_z": true,
    "gyro_x": true,
    "gyro_y": true,
    "gyro_z": true,
    "pressure": false,
    "temperature": false
  },
  "colors": {
    "accel_x": "#e74c3c",
    "accel_y": "#2ecc71",
    "accel_z": "#3498db",
    "gyro_x": "#e74c3c",
    "gyro_y": "#2ecc71",
    "gyro_z": "#3498db",
    "pressure": "#f1c40f",
    "temperature": "#1abc9c"
  }
}
```

#### Configuration Reference

| Key | Type | Default | Description |
|-----|------|---------|-------------|
| `ip` | string | `"0.0.0.0"` | UDP bind IP address |
| `port` | int | `4210` | UDP listen port |
| `time_window` | float | `10` | Visible time window in seconds. Use a small value (e.g. 2) for fast signals, larger (e.g. 60) for trends |
| `axes` | object | — | Custom display names for each axis. Maps IMU axes to physical DOFs (e.g. `"gyro_x": "Roll"`) |
| `visible` | object | all `true` | Show/hide individual plots. Set to `false` to hide a plot and give more space to the remaining ones |
| `colors` | object | — | Hex color for each plot line (e.g. `"#e74c3c"`) |

#### Axis Keys

The following keys are used across `axes`, `visible`, and `colors`:

| Key | Sensor | Unit | Default Name |
|-----|--------|------|--------------|
| `accel_x` | MPU6500 accelerometer | g | Accel X |
| `accel_y` | MPU6500 accelerometer | g | Accel Y |
| `accel_z` | MPU6500 accelerometer | g | Accel Z |
| `gyro_x` | MPU6500 gyroscope | deg/s | Gyro X |
| `gyro_y` | MPU6500 gyroscope | deg/s | Gyro Y |
| `gyro_z` | MPU6500 gyroscope | deg/s | Gyro Z |
| `pressure` | BMP280 | Pa | Pressure |
| `temperature` | BMP280 | deg C | Temperature |

### GUI Features

- **Real-time plotting** at 30 FPS with OpenGL acceleration
- **3-column grid layout** for plots
- **Connection form** to set bind IP and port at runtime
- **Status bar** showing receive rate (Hz), total packets received, and dropped packet count
- **Configurable time window** for zooming into fast signals or viewing trends
- **Custom axis names** to map IMU axes to physical DOFs (X/Y/Z, Roll/Pitch/Yaw)
- **Per-plot visibility** to focus display area on signals of interest
- **Per-plot colors** for visual customization
- **Dropped packet detection** via `packet_id` gap tracking
- **Auto-reset on ESP reboot** (detects timestamp backward jump)
