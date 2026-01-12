# Webcam Motion Tracker with Pan/Tilt Servo Control

A Python-based motion tracking system using OpenCV that can control pan/tilt servo motors via Pololu Maestro servo controller.

## Features

- **Multiple Tracking Algorithms**
  - MeanShift/CamShift - Histogram-based tracking (fast, good for color tracking)
  - MOSSE - Ultra-fast correlation filter tracking
  - KCF - Good balance of speed and accuracy
  - CSRT - High accuracy for slower movements
  - MedianFlow - Good for predictable, smooth motion

- **Automatic Target Acquisition**
  - Motion detection with background subtraction
  - Automatic tracking of moving objects
  - Configurable sensitivity thresholds

- **Xbox Controller Support**
  - Crosshair-based target selection
  - Left stick for precise crosshair positioning
  - A button hold/release for selection box
  - Y button for tracking reset
  - Menu button to quit

- **Servo Control Features**
  - Real-time pan/tilt control via Pololu Maestro
  - Configurable speed, range, and center positions
  - Smooth movement with dead zones
  - Wide servo range support (0-180° pan, 0-270° tilt)

- **Visual Feedback**
  - Bounding box tracking display
  - Crosshairs and error indicators
  - Real-time FPS display
  - Fullscreen mode support

## Hardware Requirements

### For Tracking Only (No Servos)
- USB Webcam (tested with Logitech HD 1080p)
- Computer with Python 3.8+

### For Full Pan/Tilt Control
- Pololu Micro Maestro 6-Channel USB Servo Controller
- 2x Servo Motors (pan and tilt)
- Pan/Tilt Servo Bracket
- External 5V Power Supply for servos (2A+ recommended)
- USB Cable for Maestro

### Optional
- Xbox One/360 Controller for manual target selection

## Wiring Diagram

```
                    +5V External Power
                          │
     ┌────────────────────┴────────────────────┐
     │                                          │
     │    ┌─────────────┐   ┌─────────────┐    │
     └────┤ Pan Servo   │   │ Tilt Servo  ├────┘
          │   (VCC)     │   │   (VCC)     │
          │             │   │             │
          │   Signal────┼───┤   Signal────┼──────┐
          │             │   │             │      │
          │   GND───────┤   │   GND───────┤      │
          └─────────────┘   └─────────────┘      │
                │                   │            │
                │                   │            │
           ┌────┴───────────────────┴────┐       │
           │    POLOLU MAESTRO           │       │
           │                             │       │
           │    Channel 0 ───────────────┼───────┘
           │    Channel 1 ───────────────┼───── (Tilt Signal)
           │    GND ─────────────────────┼───── Common Ground
           │    USB ──────────────────────────── To Computer
           └─────────────────────────────┘
```

**IMPORTANT:** Always use external power for servos - do not power from USB!

## Installation

### 1. Install Python Dependencies

It’s recommended to use a local virtual environment (`.venv`). It is intentionally not committed to git/GitHub.

```bash
cd webcam-tracker
python -m venv .venv
./.venv/Scripts/Activate.ps1
pip install -r requirements.txt
```

If PowerShell blocks activation, run `Set-ExecutionPolicy -Scope CurrentUser RemoteSigned` once, or use `./.venv/Scripts/activate` from CMD.

**Dependencies:**
- opencv-python
- opencv-contrib-python (for legacy trackers)
- numpy
- pyserial (for Pololu Maestro communication)
- XInput-Python (for Xbox controller support)

### 2. Configure Pololu Maestro (Optional - for servo control)

1. Connect Pololu Maestro via USB
2. Install Pololu Maestro Control Center software
3. Configure channels 0 (pan) and 1 (tilt)
4. Note the COM port (e.g., "COM3")
5. Ensure the baud rate is set to 9600

### 3. Configure Settings

Edit `config.py` to match your setup:

```python
# Camera settings
CAMERA_INDEX = 1          # Try 0, 1, or 2
USE_DSHOW = True          # DirectShow backend for Windows

# Tracker type
TRACKER_TYPE = "MEANSHIFT"  # Options: MEANSHIFT, CAMSHIFT, MOSSE, KCF, CSRT, MEDIANFLOW

# Auto-detection
AUTO_DETECT = True        # Automatic motion detection
FACE_DETECTION = False    # Face detection (if enabled)

# Servo settings (if using)
ENABLE_SERVO = True       # Set to True when Maestro connected
SERIAL_PORT = "COM3"      # Your Maestro's COM port
PAN_SPEED = 15            # Pan movement speed (0-20)
TILT_SPEED = 15           # Tilt movement speed (0-20)
TILT_CENTER = 45          # Center tilt angle in degrees
```

## Usage

### Run the Tracker

```bash
python main.py
```

### Keyboard Controls

| Key | Action |
|-----|--------|
| `s` | Manual target selection |
| `r` | Reset tracking |
| `f` | Toggle fullscreen |
| `q` | Quit |

### Xbox Controller Controls

| Button/Stick | Action |
|--------------|--------|
| `X` button | Enable crosshair selection mode |
| Left stick | Move crosshair (X and Y) |
| `A` button (hold) | Start selection box at crosshair |
| `A` button (release) | Finalize selection |
| `Y` button | Reset tracking |
| `Menu` button | Quit application |

### Manual Target Selection

**Method 1: Keyboard**
1. Press `s` to enter selection mode
2. Click and drag to draw a box around the object
3. Press `ENTER` or `SPACE` to confirm

**Method 2: Xbox Controller**
1. Press `X` button to enable crosshair
2. Use left stick to position crosshair
3. Hold `A` button to start selection at current position
4. Move left stick to draw selection box
5. Release `A` button to confirm selection

### Automatic Target Acquisition

If `AUTO_DETECT = True` in config:
- The system automatically detects and tracks moving objects
- No manual selection needed
- Great for monitoring applications

## Tracker Comparison

| Tracker | Speed | Accuracy | Best For |
|---------|-------|----------|----------|
| MeanShift | ⚡⚡⚡⚡⚡ | ⭐⭐⭐ | Color-based tracking, very fast |
| CamShift | ⚡⚡⚡⚡⚡ | ⭐⭐⭐ | Adaptive size tracking |
| MOSSE | ⚡⚡⚡⚡⚡ | ⭐⭐ | High-speed tracking, low CPU |
| KCF | ⚡⚡⚡⚡ | ⭐⭐⭐ | General purpose |
| MedianFlow | ⚡⚡⚡⚡ | ⭐⭐⭐ | Predictable, smooth motion |
| CSRT | ⚡⚡ | ⭐⭐⭐⭐⭐ | Precise tracking, slower motion |

## Troubleshooting

### Camera not detected
- Try changing `CAMERA_INDEX` in config.py (0, 1, 2...)
- Set `USE_DSHOW = True` for better USB camera support on Windows
- Make sure no other application is using the camera
- Check camera permissions in Windows settings

### Tracking is jittery
- Increase `DEAD_ZONE` in config.py (try 0.1-0.15)
- Try a different tracker (MeanShift or KCF)
- Reduce `PAN_SPEED` and `TILT_SPEED` (try 10-12)
- Lower the `smoothing_factor` in main.py for more damping

### Pololu Maestro not connecting
- Check Device Manager for COM port number
- Make sure Maestro Control Center is closed
- Verify baud rate is 9600 in both Maestro software and config.py
- Try both COM ports (Command Port vs TTL Port - use Command Port)

### Servos not moving
- Check wiring connections (especially ground)
- Verify external power supply is connected and adequate (5V, 2A+)
- Test servos with Maestro Control Center first
- Ensure channels 0 and 1 are configured correctly
- Check voltage under load (should stay above 4.5V)

### Servo movement inverted
- Adjust pan/tilt direction in servo_controller.py
- Or modify signs in `calculate_servo_adjustment()` in main.py

### Xbox controller not detected
- Install XInput-Python: `pip install XInput-Python`
- Make sure controller is connected before starting application
- Try unplugging and reconnecting the controller
- Check if controller works in other applications

### Motion detection too sensitive
- Increase `BG_THRESHOLD` in config.py (try 30-50)
- Increase `MIN_CONTOUR_AREA` (try 500-1000)
- Adjust lighting to reduce shadows and reflections

## File Structure

```
webcam-tracker/
├── main.py              # Main application
├── config.py            # Configuration settings
├── servo_controller.py  # Pololu Maestro communication
├── requirements.txt     # Python dependencies
├── README.md            # This file
└── arduino_code/        # (Legacy - not used with Maestro)
    └── servo_controller.ino
```

## Configuration Reference

Key settings in `config.py`:

| Setting | Default | Description |
|---------|---------|-------------|
| `CAMERA_INDEX` | 1 | Camera device index |
| `TRACKER_TYPE` | "MEANSHIFT" | Tracking algorithm |
| `AUTO_DETECT` | True | Auto motion detection |
| `ENABLE_SERVO` | True | Enable servo control |
| `SERIAL_PORT` | "COM3" | Maestro COM port |
| `PAN_SPEED` | 15 | Pan movement speed (0-20) |
| `TILT_SPEED` | 15 | Tilt movement speed (0-20) |
| `PAN_CENTER` | 90 | Center pan angle |
| `TILT_CENTER` | 45 | Center tilt angle |
| `PAN_MIN/MAX` | 0/180 | Pan angle range |
| `TILT_MIN/MAX` | 0/270 | Tilt angle range |
| `DEAD_ZONE` | 0.05 | Movement dead zone |
| `BG_THRESHOLD` | 20 | Motion detection sensitivity |

## License

MIT License - Feel free to use and modify!
