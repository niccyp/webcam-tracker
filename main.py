"""
Webcam Motion Tracking with Pan/Tilt Servo Control
Uses MOSSE or KCF tracker for efficient patch tracking
Xbox Controller support for target selection
"""

import cv2
import time
from config import Config
from servo_controller import ServoController

try:
    import XInput
    XINPUT_AVAILABLE = True
except ImportError:
    XINPUT_AVAILABLE = False
    print("XInput not available. Install with: pip install XInput-Python")


class WebcamTracker:
    def __init__(self):
        self.config = Config()
        self.cap = None
        self.tracker = None
        self.servo = None
        self.frame_center = None
        self.tracking = False
        self.bbox = None
        
        # CamShift specific
        self.roi_hist = None
        self.track_window = None
        self.term_crit = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_COUNT, 10, 1)
        
        # Motion detection
        self.bg_subtractor = None
        self.last_lost_time = 0
        
        # Smoothing for servo control
        self.smoothed_pan = 0
        self.smoothed_tilt = 0
        self.smoothing_factor = 0.3  # Lower = smoother but slower response
        
        # Face detection
        self.face_cascade = None
        self.face_detect_counter = 0
        
        # Display
        self.fullscreen = False
        self.window_name = "Webcam Tracker"
        
        # Xbox controller
        self.controller_available = False
        self.controller_selection_mode = False
        self.crosshair_x = 320
        self.crosshair_y = 240
        self.selection_start = None
        self.a_button_held = False
        self.x_button_was_pressed = False
        self.y_button_was_pressed = False
        self.menu_button_was_pressed = False
        self.lb_button_was_pressed = False
        self.rb_button_was_pressed = False
        self.quit_requested = False
        
        # Tracker cycling
        self.available_trackers = ["MEANSHIFT", "CAMSHIFT", "MOSSE", "KCF", "CSRT", "MEDIANFLOW"]
        try:
            self.current_tracker_index = self.available_trackers.index(self.config.TRACKER_TYPE.upper())
        except ValueError:
            self.current_tracker_index = 0
            self.config.TRACKER_TYPE = self.available_trackers[0]
        
    def initialize_camera(self):
        """Initialize the webcam"""
        def try_open(index: int):
            if self.config.USE_DSHOW:
                # Use DirectShow backend for better USB camera support on Windows
                cap = cv2.VideoCapture(index, cv2.CAP_DSHOW)
            else:
                cap = cv2.VideoCapture(index)
            cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.config.FRAME_WIDTH)
            cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.config.FRAME_HEIGHT)
            cap.set(cv2.CAP_PROP_FPS, self.config.FPS)
            return cap

        self.cap = try_open(self.config.CAMERA_INDEX)

        if not self.cap.isOpened():
            # Fall back to other common indices without forcing a config change.
            self.cap.release()
            self.cap = None
            for fallback_index in [0, 1, 2, 3]:
                if fallback_index == self.config.CAMERA_INDEX:
                    continue
                cap = try_open(fallback_index)
                if cap.isOpened():
                    self.cap = cap
                    print(
                        f"Warning: Could not open CAMERA_INDEX={self.config.CAMERA_INDEX}; "
                        f"using CAMERA_INDEX={fallback_index} instead"
                    )
                    break

        if self.cap is None or not self.cap.isOpened():
            raise Exception("Could not open webcam")
        
        # Calculate frame center for servo control
        self.frame_center = (self.config.FRAME_WIDTH // 2, self.config.FRAME_HEIGHT // 2)
        print(f"Camera initialized: {self.config.FRAME_WIDTH}x{self.config.FRAME_HEIGHT} @ {self.config.FPS}fps")
        
        # Initialize background subtractor for motion detection
        if self.config.AUTO_DETECT:
            self.bg_subtractor = cv2.createBackgroundSubtractorMOG2(
                history=self.config.BG_HISTORY,
                varThreshold=self.config.BG_THRESHOLD,
                detectShadows=self.config.DETECT_SHADOWS
            )
            print("Auto motion detection enabled")
        
        # Initialize face detection
        if self.config.FACE_DETECTION:
            cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
            self.face_cascade = cv2.CascadeClassifier(cascade_path)
            if self.face_cascade.empty():
                print("Warning: Could not load face cascade classifier")
                self.face_cascade = None
            else:
                print("Face detection enabled")
        
    def initialize_servo(self):
        """Initialize servo controller (optional)"""
        if self.config.ENABLE_SERVO:
            try:
                self.servo = ServoController(
                    port=self.config.SERIAL_PORT,
                    baudrate=self.config.SERIAL_BAUDRATE,
                    pan_channel=self.config.PAN_CHANNEL,
                    tilt_channel=self.config.TILT_CHANNEL,
                    min_pulse=self.config.SERVO_MIN_PULSE,
                    max_pulse=self.config.SERVO_MAX_PULSE,
                    pan_min=self.config.PAN_MIN,
                    pan_max=self.config.PAN_MAX,
                    tilt_min=self.config.TILT_MIN,
                    tilt_max=self.config.TILT_MAX,
                    pan_center=self.config.PAN_CENTER,
                    tilt_center=self.config.TILT_CENTER
                )
                print(f"Maestro connected on {self.config.SERIAL_PORT}")
            except Exception as e:
                print(f"Warning: Could not connect to Maestro: {e}")
                print("Running in tracking-only mode (no servo control)")
                self.servo = None
    
    def init_controller(self):
        """Check for Xbox controller"""
        if XINPUT_AVAILABLE:
            try:
                self.controller_available = XInput.get_connected()[0]
                if self.controller_available:
                    print("Xbox controller detected")
                    print("  X button: Enable crosshair")
                    print("  Left stick: Move crosshair")
                    print("  A button: Hold to draw selection box")
                    print("  LB/RB: Cycle tracker algorithms")
                    print("  Y button: Reset tracking")
                    print("  Menu button: Quit")
                    self.crosshair_x = self.config.FRAME_WIDTH // 2
                    self.crosshair_y = self.config.FRAME_HEIGHT // 2
            except:
                self.controller_available = False
    
    def switch_tracker(self, direction):
        """Switch to the next or previous tracker algorithm"""
        old_tracker = self.config.TRACKER_TYPE
        self.current_tracker_index = (self.current_tracker_index + direction) % len(self.available_trackers)
        self.config.TRACKER_TYPE = self.available_trackers[self.current_tracker_index]
        
        if self.tracking:
            # If currently tracking, reset to reinitialize with new tracker
            print(f"Switching tracker from {old_tracker} to {self.config.TRACKER_TYPE} (tracking reset)")
            self.tracking = False
            self.tracker = None
            self.roi_hist = None
            self.track_window = None
        else:
            print(f"Tracker switched to: {self.config.TRACKER_TYPE}")
    
    def handle_controller_input(self, frame):
        """Handle Xbox controller input for target selection and controls"""
        if not self.controller_available or not XINPUT_AVAILABLE:
            return frame
        
        try:
            state = XInput.get_state(0)
            buttons = state.Gamepad.wButtons
            
            # Menu button (0x0080) to quit
            menu_button_pressed = buttons & 0x0080
            if menu_button_pressed and not self.menu_button_was_pressed:
                self.quit_requested = True
                print("Menu button pressed - quitting")
            self.menu_button_was_pressed = menu_button_pressed
            
            # Y button (0x8000) to reset tracking
            y_button_pressed = buttons & 0x8000
            if y_button_pressed and not self.y_button_was_pressed:
                self.tracking = False
                self.tracker = None
                self.bbox = None
                self.roi_hist = None
                self.track_window = None
                if self.servo is not None:
                    self.servo.center()
                print("Y button pressed - tracking reset")
            self.y_button_was_pressed = y_button_pressed
            
            # Left Bumper (0x0100) - Previous tracker
            lb_button_pressed = buttons & 0x0100
            if lb_button_pressed and not self.lb_button_was_pressed:
                self.switch_tracker(-1)
            self.lb_button_was_pressed = lb_button_pressed
            
            # Right Bumper (0x0200) - Next tracker
            rb_button_pressed = buttons & 0x0200
            if rb_button_pressed and not self.rb_button_was_pressed:
                self.switch_tracker(1)
            self.rb_button_was_pressed = rb_button_pressed
            
            # X button (0x4000) to toggle selection mode
            x_button_pressed = buttons & 0x4000
            if x_button_pressed and not self.x_button_was_pressed:
                self.controller_selection_mode = not self.controller_selection_mode
                if self.controller_selection_mode:
                    self.crosshair_x = self.config.FRAME_WIDTH // 2
                    self.crosshair_y = self.config.FRAME_HEIGHT // 2
                    self.selection_start = None
                    self.a_button_held = False
                    print("Crosshair enabled - move left stick, hold A to select")
                else:
                    print("Crosshair disabled")
                    self.selection_start = None
            self.x_button_was_pressed = x_button_pressed
            
            if self.controller_selection_mode:
                # Left stick for crosshair movement
                left_x = state.Gamepad.sThumbLX / 32768.0
                left_y = -state.Gamepad.sThumbLY / 32768.0  # Inverted Y
                
                # Apply deadzone and move crosshair
                if abs(left_x) > 0.15:
                    self.crosshair_x += int(left_x * 8)
                if abs(left_y) > 0.15:
                    self.crosshair_y += int(left_y * 8)
                
                # Clamp to frame bounds
                self.crosshair_x = max(0, min(self.config.FRAME_WIDTH - 1, self.crosshair_x))
                self.crosshair_y = max(0, min(self.config.FRAME_HEIGHT - 1, self.crosshair_y))
                
                # A button (0x1000) for selection
                a_pressed = buttons & 0x1000
                
                if a_pressed and not self.a_button_held:
                    # A button just pressed - start selection
                    self.a_button_held = True
                    self.selection_start = (self.crosshair_x, self.crosshair_y)
                    print(f"Selection started at ({self.crosshair_x}, {self.crosshair_y}) - move stick and release A")
                elif not a_pressed and self.a_button_held:
                    # A button released - complete selection
                    self.a_button_held = False
                    if self.selection_start:
                        x1, y1 = self.selection_start
                        x2, y2 = self.crosshair_x, self.crosshair_y
                        
                        # Create bbox
                        x = min(x1, x2)
                        y = min(y1, y2)
                        w = abs(x2 - x1)
                        h = abs(y2 - y1)
                        
                        if w > 10 and h > 10:
                            self.bbox = (x, y, w, h)
                            if self.config.TRACKER_TYPE.upper() in ["MEANSHIFT", "CAMSHIFT"]:
                                self._init_camshift_histogram(frame, self.bbox)
                            else:
                                self.tracker = self.create_tracker()
                                self.tracker.init(frame, self.bbox)
                            self.tracking = True
                            print(f"Target selected: ({x}, {y}, {w}, {h})")
                            self.controller_selection_mode = False
                        else:
                            print("Selection too small, try again")
                        
                        self.selection_start = None
                
                # Draw crosshair
                cv2.drawMarker(frame, (self.crosshair_x, self.crosshair_y), 
                              (0, 255, 0), cv2.MARKER_CROSS, 25, 2)
                cv2.circle(frame, (self.crosshair_x, self.crosshair_y), 3, (0, 255, 0), -1)
                
                if self.selection_start and self.a_button_held:
                    # Draw selection rectangle
                    x1, y1 = self.selection_start
                    cv2.rectangle(frame, (x1, y1), (self.crosshair_x, self.crosshair_y), 
                                (0, 255, 0), 2)
                
                # Instructions
                cv2.putText(frame, "Left Stick: Move | Hold A: Select | Release A: Confirm | X: Cancel", 
                           (10, frame.shape[0] - 40), cv2.FONT_HERSHEY_SIMPLEX, 
                           0.5, (0, 255, 0), 2)
        
        except Exception as e:
            print(f"Controller error: {e}")
            pass
        
        return frame
    
    def create_tracker(self):
        """Create the tracking algorithm"""
        tracker_type = self.config.TRACKER_TYPE.upper()
        
        if tracker_type == "MEANSHIFT" or tracker_type == "CAMSHIFT":
            # MeanShift/CamShift - Histogram-based, very fast
            return None  # These don't use OpenCV tracker interface
        elif tracker_type == "MOSSE":
            # MOSSE - Fastest, good for real-time
            return cv2.legacy.TrackerMOSSE_create()
        elif tracker_type == "KCF":
            # KCF - Good balance of speed and accuracy
            return cv2.TrackerKCF_create()
        elif tracker_type == "CSRT":
            # CSRT - Most accurate but slower
            return cv2.TrackerCSRT_create()
        elif tracker_type == "MEDIANFLOW":
            # MedianFlow - Good for predictable motion
            return cv2.legacy.TrackerMedianFlow_create()
        else:
            print(f"Unknown tracker type: {tracker_type}, defaulting to MEANSHIFT")
            return None
    
    def detect_motion(self, frame):
        """Detect moving objects using background subtraction"""
        # Apply background subtraction
        fg_mask = self.bg_subtractor.apply(frame)
        
        # Remove noise with morphological operations
        kernel = cv2.getStructuringElement(
            cv2.MORPH_ELLIPSE, 
            (self.config.MORPH_KERNEL_SIZE, self.config.MORPH_KERNEL_SIZE)
        )
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_OPEN, kernel)
        fg_mask = cv2.morphologyEx(fg_mask, cv2.MORPH_CLOSE, kernel)
        
        # Find contours
        contours, _ = cv2.findContours(fg_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Find the largest valid contour
        best_contour = None
        best_area = 0
        
        for contour in contours:
            area = cv2.contourArea(contour)
            if self.config.MIN_CONTOUR_AREA < area < self.config.MAX_CONTOUR_AREA:
                if area > best_area:
                    best_area = area
                    best_contour = contour
        
        if best_contour is not None:
            # Get bounding box of the largest moving object
            x, y, w, h = cv2.boundingRect(best_contour)
            return (x, y, w, h), fg_mask
        
        return None, fg_mask
    
    def detect_faces(self, frame):
        """Detect faces using Haar cascade classifier"""
        if self.face_cascade is None:
            return None
        
        # Convert to grayscale for detection
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        
        # Detect faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=self.config.FACE_SCALE_FACTOR,
            minNeighbors=self.config.FACE_MIN_NEIGHBORS,
            minSize=self.config.FACE_MIN_SIZE,
            flags=cv2.CASCADE_SCALE_IMAGE
        )
        
        if len(faces) > 0:
            # Return the largest face detected
            largest_face = max(faces, key=lambda f: f[2] * f[3])
            return tuple(largest_face)
        
        return None
    
    def auto_init_tracking(self, frame, bbox):
        """Automatically initialize tracking on detected motion"""
        self.bbox = bbox
        
        if self.config.TRACKER_TYPE.upper() in ["MEANSHIFT", "CAMSHIFT"]:
            self._init_camshift_histogram(frame, bbox)
        else:
            self.tracker = self.create_tracker()
            self.tracker.init(frame, bbox)
        
        self.tracking = True
        print(f"Auto-tracking started: {bbox}")
    
    def select_target(self, frame):
        """Allow user to select tracking target"""
        print("\n=== SELECT TARGET ===")
        print("Draw a box around the object to track")
        print("Press ENTER or SPACE to confirm")
        print("Press 'c' to cancel")
        
        bbox = cv2.selectROI("Select Target", frame, fromCenter=False, showCrosshair=True)
        cv2.destroyWindow("Select Target")
        
        if bbox[2] > 0 and bbox[3] > 0:  # Valid selection
            self.bbox = bbox
            
            if self.config.TRACKER_TYPE.upper() in ["MEANSHIFT", "CAMSHIFT"]:
                # Initialize histogram for MeanShift/CamShift
                self._init_camshift_histogram(frame, bbox)
            else:
                # Use standard OpenCV tracker
                self.tracker = self.create_tracker()
                self.tracker.init(frame, bbox)
            
            self.tracking = True
            print(f"Tracking initialized: {bbox}")
            return True
        return False
    
    def _init_camshift_histogram(self, frame, bbox):
        """Initialize histogram for MeanShift/CamShift tracking"""
        x, y, w, h = [int(v) for v in bbox]
        
        # Extract ROI and convert to HSV
        roi = frame[y:y+h, x:x+w]
        hsv_roi = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        
        # Create mask to exclude low saturation/value pixels
        mask = cv2.inRange(hsv_roi, 
                          (0, self.config.CAMSHIFT_MIN_SAT, self.config.CAMSHIFT_MIN_VAL),
                          (180, 255, self.config.CAMSHIFT_MAX_VAL))
        
        # Calculate histogram on H and S channels
        self.roi_hist = cv2.calcHist([hsv_roi], [0, 1], mask,
                                     [self.config.CAMSHIFT_HBINS, self.config.CAMSHIFT_SBINS],
                                     [0, 180, 0, 256])
        
        # Normalize histogram
        cv2.normalize(self.roi_hist, self.roi_hist, 0, 255, cv2.NORM_MINMAX)
        
        # Set initial tracking window
        self.track_window = (x, y, w, h)
        print(f"Histogram initialized with {self.config.CAMSHIFT_HBINS}x{self.config.CAMSHIFT_SBINS} bins")
    
    def _update_meanshift(self, frame):
        """Update tracking using MeanShift algorithm (fixed window size)"""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Calculate back projection
        dst = cv2.calcBackProject([hsv], [0, 1], self.roi_hist,
                                  [0, 180, 0, 256], 1)
        
        # Apply MeanShift
        ret, self.track_window = cv2.meanShift(dst, self.track_window, self.term_crit)
        
        x, y, w, h = self.track_window
        
        # Check if tracking is valid
        if w > 0 and h > 0:
            return True, (x, y, w, h)
        return False, None
    
    def _update_camshift(self, frame):
        """Update tracking using CamShift algorithm"""
        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Calculate back projection
        dst = cv2.calcBackProject([hsv], [0, 1], self.roi_hist,
                                  [0, 180, 0, 256], 1)
        
        # Apply CamShift
        ret, self.track_window = cv2.CamShift(dst, self.track_window, self.term_crit)
        
        # Get bounding box from rotated rect
        pts = cv2.boxPoints(ret)
        x, y, w, h = cv2.boundingRect(pts)
        
        # Check if tracking is valid (non-zero area)
        if w > 0 and h > 0:
            return True, (x, y, w, h)
        return False, None
    
    def calculate_servo_adjustment(self, object_center):
        """Calculate pan/tilt adjustments based on object position"""
        error_x = object_center[0] - self.frame_center[0]
        error_y = object_center[1] - self.frame_center[1]
        
        # Normalize errors to -1.0 to 1.0 range
        norm_error_x = error_x / (self.config.FRAME_WIDTH / 2)
        norm_error_y = error_y / (self.config.FRAME_HEIGHT / 2)
        
        # Apply dead zone (ignore small movements)
        if abs(norm_error_x) < self.config.DEAD_ZONE:
            norm_error_x = 0
        if abs(norm_error_y) < self.config.DEAD_ZONE:
            norm_error_y = 0
        
        # Calculate target servo speed based on error magnitude
        target_pan = -norm_error_x * self.config.PAN_SPEED  # Inverted for correct direction
        target_tilt = norm_error_y * self.config.TILT_SPEED
        
        # Apply exponential smoothing to reduce oscillation
        self.smoothed_pan = (self.smoothing_factor * target_pan + 
                            (1 - self.smoothing_factor) * self.smoothed_pan)
        self.smoothed_tilt = (self.smoothing_factor * target_tilt + 
                             (1 - self.smoothing_factor) * self.smoothed_tilt)
        
        pan_speed = int(self.smoothed_pan)
        tilt_speed = int(self.smoothed_tilt)
        
        return pan_speed, tilt_speed, error_x, error_y
    
    def draw_overlay(self, frame, bbox=None, error_x=0, error_y=0, fps=0):
        """Draw tracking overlay on frame"""
        # Draw center crosshair
        cv2.line(frame, (self.frame_center[0] - 20, self.frame_center[1]),
                 (self.frame_center[0] + 20, self.frame_center[1]), (0, 255, 255), 1)
        cv2.line(frame, (self.frame_center[0], self.frame_center[1] - 20),
                 (self.frame_center[0], self.frame_center[1] + 20), (0, 255, 255), 1)
        
        if bbox is not None:
            x, y, w, h = [int(v) for v in bbox]
            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
            # Draw center point of tracked object
            obj_center = (x + w // 2, y + h // 2)
            cv2.circle(frame, obj_center, 5, (0, 0, 255), -1)
            # Draw line from center to object
            cv2.line(frame, self.frame_center, obj_center, (255, 0, 0), 1)
        
        # Draw status info
        if self.tracking:
            status = "TRACKING FACE" if self.config.FACE_DETECTION else "TRACKING"
        elif self.config.FACE_DETECTION:
            status = "SEARCHING FOR FACE..."
        else:
            status = "IDLE - Press 's' or X button to select target"
        color = (0, 255, 0) if self.tracking else (0, 255, 255)
        cv2.putText(frame, status, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 2)
        cv2.putText(frame, f"FPS: {fps:.1f}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        cv2.putText(frame, f"Tracker: {self.config.TRACKER_TYPE}", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        if self.tracking:
            cv2.putText(frame, f"Error X: {error_x:+.0f}  Y: {error_y:+.0f}", 
                       (10, 95), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
        
        # Draw controls help
        controls = "[s]elect | [f]ullscreen"
        if self.controller_available:
            controls += " | [X]=Crosshair [A]=Select [Y]=Reset [Menu]=Quit"
        else:
            controls += " | [r]eset | [q]uit"
        cv2.putText(frame, f"Controls: {controls}", 
                   (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 200, 200), 1)
        
        return frame
    
    def toggle_fullscreen(self):
        """Toggle between fullscreen and windowed mode"""
        self.fullscreen = not self.fullscreen
        if self.fullscreen:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            print("Fullscreen mode enabled")
        else:
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
            print("Windowed mode enabled")
    
    def run(self):
        """Main tracking loop"""
        self.initialize_camera()
        self.initialize_servo()
        self.init_controller()
        
        print("\n=== WEBCAM TRACKER STARTED ===")
        print("Press 's' to select a target to track")
        if self.controller_available:
            print("Controller: X=Crosshair | A=Select | Y=Reset | Menu=Quit")
        print("Press 'r' to reset tracking")
        print("Press 'f' to toggle fullscreen")
        print("Press 'q' to quit")
        print("=" * 35)
        
        # Create window with fullscreen support
        cv2.namedWindow(self.window_name, cv2.WINDOW_NORMAL)
        if self.config.START_FULLSCREEN:
            self.fullscreen = True
            cv2.setWindowProperty(self.window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        
        fps = 0
        frame_count = 0
        start_time = time.time()
        
        while True:
            # Check if quit was requested by controller
            if self.quit_requested:
                break
                
            ret, frame = self.cap.read()
            if not ret:
                print("Failed to grab frame")
                break
            
            # Flip frame horizontally for mirror effect (optional)
            if self.config.MIRROR_MODE:
                frame = cv2.flip(frame, 1)
            
            # Handle controller input for selection
            frame = self.handle_controller_input(frame)
            
            error_x, error_y = 0, 0
            
            # Auto-detect motion or faces when not tracking
            if not self.tracking and not self.controller_selection_mode:
                # Try motion detection first if enabled
                if self.config.AUTO_DETECT and self.bg_subtractor is not None:
                    motion_bbox, _ = self.detect_motion(frame)
                    if motion_bbox is not None:
                        print(f"Motion detected: {motion_bbox}")
                        self.auto_init_tracking(frame, motion_bbox)
                # Fall back to face detection if no motion and face detection enabled
                elif self.config.FACE_DETECTION and self.face_cascade is not None:
                    self.face_detect_counter += 1
                    # Only run face detection every N frames for performance
                    if self.face_detect_counter >= self.config.FACE_REDETECT_INTERVAL:
                        self.face_detect_counter = 0
                        face_bbox = self.detect_faces(frame)
                        if face_bbox is not None:
                            print(f"Face detected: {face_bbox}")
                            self.auto_init_tracking(frame, face_bbox)
            
            if self.tracking:
                # Update tracker based on type
                tracker_type = self.config.TRACKER_TYPE.upper()
                if tracker_type == "MEANSHIFT":
                    success, bbox = self._update_meanshift(frame)
                elif tracker_type == "CAMSHIFT":
                    success, bbox = self._update_camshift(frame)
                elif self.tracker is not None:
                    success, bbox = self.tracker.update(frame)
                else:
                    success = False
                    bbox = None
                
                if success:
                    self.bbox = bbox
                    x, y, w, h = [int(v) for v in bbox]
                    object_center = (x + w // 2, y + h // 2)
                    
                    # Calculate servo adjustments
                    pan_speed, tilt_speed, error_x, error_y = self.calculate_servo_adjustment(object_center)
                    
                    # Send to servo controller
                    if self.servo is not None:
                        self.servo.move(pan_speed, tilt_speed)
                else:
                    # Tracking failed
                    print("Tracking lost!")
                    self.tracking = False
                    self.bbox = None
                    if self.servo is not None:
                        self.servo.stop()
            
            # Calculate FPS
            frame_count += 1
            elapsed = time.time() - start_time
            if elapsed >= 1.0:
                fps = frame_count / elapsed
                frame_count = 0
                start_time = time.time()
            
            # Draw overlay
            frame = self.draw_overlay(frame, self.bbox if self.tracking else None, 
                                      error_x, error_y, fps)
            
            # Display frame
            cv2.imshow(self.window_name, frame)
            
            # Handle keyboard input
            key = cv2.waitKey(1) & 0xFF
            
            if key == ord('q'):
                break
            elif key == ord('s'):
                # Select new target
                ret, fresh_frame = self.cap.read()
                if ret:
                    if self.config.MIRROR_MODE:
                        fresh_frame = cv2.flip(fresh_frame, 1)
                    self.select_target(fresh_frame)
            elif key == ord('r'):
                # Reset tracking
                self.tracking = False
                self.tracker = None
                self.bbox = None
                self.roi_hist = None
                self.track_window = None
                if self.servo is not None:
                    self.servo.center()
                print("Tracking reset")
            elif key == ord('f'):
                # Toggle fullscreen
                self.toggle_fullscreen()
        
        self.cleanup()
    
    def cleanup(self):
        """Clean up resources"""
        print("\nCleaning up...")
        if self.cap is not None:
            self.cap.release()
        if self.servo is not None:
            self.servo.center()
            self.servo.close()
        cv2.destroyAllWindows()
        print("Goodbye!")


if __name__ == "__main__":
    tracker = WebcamTracker()
    try:
        tracker.run()
    except KeyboardInterrupt:
        print("\nInterrupted by user")
        tracker.cleanup()
    except Exception as e:
        print(f"Error: {e}")
        tracker.cleanup()
        raise
