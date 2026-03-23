# WiFi IMU – UDP Binary Protocol

## Overview

The ESP8266 streams 7-DOF sensor data (MPU6500 + BMP280) over UDP as fixed-size binary packets at 100 Hz (every 10 ms).

- **Transport:** UDP
- **Default port:** 4269
- **Byte order:** Little-endian (native ESP8266 / x86)
- **Packet size:** 43 bytes (fixed)

## Network Defaults

| Parameter    | Value              |
|-------------|--------------------|
| ESP8266 IP  | Set in sketch      |
| Target IP   | Set in sketch      |
| Target Port | 4269 (default)     |

## Packet Layout

All multi-byte fields are **little-endian**. Floats are IEEE 754 single-precision (32-bit).

| Offset | Size (bytes) | Type     | Name         | Unit   | Description                    |
|--------|-------------|----------|--------------|--------|--------------------------------|
| 0      | 1           | uint8    | sync[0]      | —      | Always `0xAA`                  |
| 1      | 1           | uint8    | sync[1]      | —      | Always `0x55`                  |
| 2      | 4           | uint32   | packet_id    | —      | Rolling packet counter (0…2³²) |
| 6      | 4           | uint32   | timestamp_ms | ms     | ESP `millis()` at send time    |
| 10     | 4           | float32  | accel_x      | g      | Accelerometer X                |
| 14     | 4           | float32  | accel_y      | g      | Accelerometer Y                |
| 18     | 4           | float32  | accel_z      | g      | Accelerometer Z                |
| 22     | 4           | float32  | gyro_x       | deg/s  | Gyroscope X                    |
| 26     | 4           | float32  | gyro_y       | deg/s  | Gyroscope Y                    |
| 30     | 4           | float32  | gyro_z       | deg/s  | Gyroscope Z                    |
| 34     | 4           | float32  | pressure     | Pa     | Barometric pressure            |
| 38     | 4           | float32  | temperature  | °C     | Temperature                    |
| 42     | 1           | uint8    | checksum     | —      | XOR of bytes 0–41              |

**Total: 43 bytes**

## Checksum

The checksum is computed as the XOR of all preceding 42 bytes:

```
checksum = 0
for byte in packet[0:42]:
    checksum ^= byte
```

If the computed checksum does not match `packet[42]`, the packet should be discarded.

## Sync Word

Each packet starts with the two-byte sync word `0xAA 0x55`. Since UDP preserves message boundaries, the sync word serves as a sanity check — if a received datagram does not start with `0xAA 0x55`, discard it.

## Sensor Configuration

### MPU6500
- **Accelerometer range:** ±4 g
- **Gyroscope range:** ±500 deg/s
- **Gyro DLPF:** ~41 Hz bandwidth

### BMP280
- **Temperature oversampling:** x1
- **Pressure oversampling:** x4
- **Mode:** Normal (continuous)
- **Standby time:** 0.5 ms
- **IIR filter:** coefficient 4

## Python Parsing Example

```python
import struct

PACKET_FORMAT = '<2s I I 3f 3f f f B'
PACKET_SIZE = 43

def parse_packet(data: bytes) -> dict | None:
    if len(data) != PACKET_SIZE:
        return None

    # Verify sync word
    if data[0:2] != b'\xaa\x55':
        return None

    # Verify checksum
    computed = 0
    for b in data[:42]:
        computed ^= b
    if computed != data[42]:
        return None

    fields = struct.unpack(PACKET_FORMAT, data)
    return {
        'packet_id':    fields[1],
        'timestamp_ms': fields[2],
        'accel':        (fields[3], fields[4], fields[5]),
        'gyro':         (fields[6], fields[7], fields[8]),
        'pressure':     fields[9],
        'temperature':  fields[10],
    }
```

## Packet Rate and Bandwidth

- **Rate:** 100 packets/s (10 ms interval)
- **Bandwidth:** 43 × 100 = 4,300 bytes/s (~34 kbit/s)
- **Packet loss:** Expected on WiFi under load. Use `packet_id` to detect gaps. No retransmission.

## Dropped Packet Detection

The `packet_id` field increments by 1 for each packet. If the receiver sees a gap (e.g., receives id 100 then 103), packets 101 and 102 were lost. The receiver should handle gaps gracefully (interpolate, skip, or log).

## Timestamp

`timestamp_ms` is the ESP8266 `millis()` value at the moment the packet is built. It wraps around every ~49.7 days. Use it for relative timing between packets, not absolute wall-clock time.

## Command Protocol (GUI → ESP)

The GUI can send commands to the ESP on the same UDP port. Commands are 2-byte packets (distinct from the 43-byte data packets by size).

| Command | Bytes | Description |
|---------|-------|-------------|
| Calibrate | `0xCA 0xFE` | Trigger IMU offset calibration |

When the ESP receives a calibrate command:
1. Data streaming pauses
2. `autoOffsets()` runs for ~1-2 seconds (sensor must be stationary and level)
3. Streaming resumes with updated offsets

Calibration offsets are stored in RAM only and reset on reboot.
