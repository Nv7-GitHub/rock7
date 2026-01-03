# Flight Data Logging System

Simple flight data recording and download system for the Rock V7 flight computer with efficient binary storage.

## Code Structure

**Firmware (Pico):**
- [flash.h](include/flash.h) / [flash.cpp](src/flash.cpp) - All logging logic
- [main.cpp](src/main.cpp) - Just 2 calls: `initFlash()` and `logFlightData()`

**Ground Station:**
- [flight_data_manager.py](flight_data_manager.py) - Download and convert to CSV

## How It Works

### On the Pico
- Logs flight data at 50 Hz (every 20ms) to LittleFS filesystem
- Stores data in **binary format** (24 bytes per record) for maximum efficiency
- Creates sequential flight files: `flight_0.bin`, `flight_1.bin`, etc.
- Each new power-up creates a new flight file (no overwriting)
- Data logged: timestamp, altitude, velocity, accel bias, raw accel, raw baro

### Binary Format
Each record is 24 bytes:
- `time_ms` (4 bytes, uint32)
- `altitude_m` (4 bytes, float)
- `velocity_ms` (4 bytes, float)
- `accel_bias_ms2` (4 bytes, float)
- `raw_accel_ms2` (4 bytes, float)
- `raw_baro_m` (4 bytes, float)

**Storage Efficiency:** Binary is ~60% smaller than CSV (24 bytes vs 50-60 bytes per record)

### CSV Output
Python tool automatically converts binary to CSV with columns:
```
time_ms,altitude_m,velocity_ms,accel_bias_ms2,raw_accel_ms2,raw_baro_m
```

## Usage

### 1. Upload Code
```bash
pio run -t upload
```

### 2. Download Flight Data

**Interactive Mode:**
```bash
python3 flight_data_manager.py
```

**Command Line:**
```bash
# List all flights
python3 flight_data_manager.py list

# Download specific flight
python3 flight_data_manager.py get 0

# Download all flights
python3 flight_data_manager.py download-all
```

### 3. Managing Storage

The system automatically:
- Creates new flight files on each power-up
- Never overwrites existing data
- Shows warning if storage is getting full (via INFO command)

To free space:
1. Download your flights
2. Use the interactive menu to delete old flights
3. Or delete all: send `DELETE X` commands via serial

## Serial Commands

You can manually send these commands via serial monitor:

- `LIST` - List all flight files
- `GET X` - Download flight X
- `DELETE X` - Delete flight X  
- `INFO` - Check storage usage

## Python Requirements

```bash
pip install pyserial
```

## Storage Capacity

With ~15MB LittleFS and binary format:
- At 50 Hz with 24 bytes/record = 1,200 bytes/sec
- **~12,800 seconds = ~3.5 hours per 15MB**
- Each 10 minute flight = ~720 KB
- **Can store ~20 full 10-minute flights**

**60% more efficient than CSV storage!**
