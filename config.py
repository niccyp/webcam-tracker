"""
Configuration settings for Webcam Tracker
Adjust these values to match your setup
"""


class Config:
    # ==================== CAMERA SETTINGS ====================
    CAMERA_INDEX = 1          # Camera index (0 = default webcam, 1 = second camera, etc.)
    USE_DSHOW = True          # Use DirectShow backend (Windows) - better USB camera support
    FRAME_WIDTH = 640         # Frame width in pixels
    FRAME_HEIGHT = 480        # Frame height in pixels
    FPS = 30                  # Target frames per second
    MIRROR_MODE = True        # Flip frame horizontally (useful for face tracking)
    START_FULLSCREEN = False  # Start in fullscreen mode
    
    # ==================== TRACKER SETTINqGS ====================
    # Available trackers (in order of speed):
    # - "MEANSHIFT"  : Fastest, histogram-based, fixed window size
    # - "CAMSHIFT"   : Very fast, histogram-based, adapts size/rotation
    # - "MOSSE"      : Very fast, lower accuracy, good for high-speed tracking
    # - "KCF"        : Fast, good accuracy, recommended for most uses
    # - "MEDIANFLOW" : Fast, good for predictable smooth motion
    # - "CSRT"       : Slowest, highest accuracy, good for precise tracking
    TRACKER_TYPE = "MEANSHIFT"
    
    # ==================== AUTO MOTION DETECTION ====================
    AUTO_DETECT = False        # Automatically detect and track moving objects
    MIN_CONTOUR_AREA = 300    # Minimum area (pixels) to consider as motion (more sensitive)
    MAX_CONTOUR_AREA = 80000  # Maximum area to avoid tracking entire frame
    BG_HISTORY = 100          # Background subtractor history frames
    BG_THRESHOLD = 20         # Threshold for foreground detection (lower = more sensitive)
    DETECT_SHADOWS = False    # Detect shadows (slower if True)
    MORPH_KERNEL_SIZE = 5     # Kernel size for noise removal
    REACQUIRE_DELAY = 0.5     # Seconds to wait before reacquiring after lost track
    
    # ==================== FACE DETECTION ====================
    FACE_DETECTION = False    # Enable automatic face detection and tracking
    FACE_SCALE_FACTOR = 1.2   # How much image size is reduced at each scale
    FACE_MIN_NEIGHBORS = 5    # Higher = fewer detections but more reliable
    FACE_MIN_SIZE = (60, 60)  # Minimum face size in pixels
    FACE_REDETECT_INTERVAL = 15  # Frames between face re-detection attempts
    
    # ==================== CAMSHIFT SETTINGS ====================
    # Histogram bins for hue and saturation channels
    CAMSHIFT_HBINS = 16       # Hue bins (lower = faster, less precise)
    CAMSHIFT_SBINS = 8        # Saturation bins
    CAMSHIFT_MIN_SAT = 60     # Minimum saturation to include in histogram
    CAMSHIFT_MIN_VAL = 32     # Minimum value to include in histogram
    CAMSHIFT_MAX_VAL = 255    # Maximum value to include in histogram
    
    # ==================== SERVO CONTROL SETTINGS ====================
    ENABLE_SERVO = True       # Set to True when Maestro is connected
    SERIAL_PORT = "COM3"      # Serial port for Pololu Maestro (Command Port)
    SERIAL_BAUDRATE = 9600    # Maestro default baud rate
    
    # Maestro servo channels
    PAN_CHANNEL = 0           # Maestro channel for pan servo (0-5)
    TILT_CHANNEL = 1          # Maestro channel for tilt servo (0-5)
    
    # Maestro pulse width settings (in quarter-microseconds)
    # Standard servo: 1000-2000µs = 4000-8000 quarter-µs
    # Adjust these if servos go too far or not far enough
    SERVO_MIN_PULSE = 4000    # 1000µs - 0 degrees
    SERVO_MAX_PULSE = 8000    # 2000µs - 180 degrees
    SERVO_CENTER_PULSE = 6000 # 1500µs - 90 degrees
    
    # Movement settings
    PAN_SPEED = 15            # Max pan speed (0-100) - Set to 0 for tilt-only
    TILT_SPEED = 15           # Max tilt speed (0-100) - lower = smoother
    DEAD_ZONE = 0.15          # Ignore movements within this range (0.0-1.0)
    
    # Servo limits (in degrees, centered at 90)
    PAN_MIN = 0               # Full pan range
    PAN_MAX = 180             # Full pan range
    PAN_CENTER = 90
    TILT_MIN = 0              # Maximum upward tilt
    TILT_MAX = 270            # Extended downward tilt range
    TILT_CENTER = 45          # Adjusted down to compensate for mount angle
    
    # ==================== PID CONTROL (Advanced) ====================
    # These values may need tuning for smooth tracking
    USE_PID = False           # Enable PID control for smoother movement
    PID_KP = 0.5              # Proportional gain
    PID_KI = 0.01             # Integral gain
    PID_KD = 0.1              # Derivative gain
