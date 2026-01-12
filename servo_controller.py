"""
Servo Controller Module
Handles communication with Pololu Micro Maestro for pan/tilt servo control
"""

import serial
import time
import threading


class ServoController:
    def __init__(
        self,
        port="COM3",
        baudrate=9600,
        timeout=1,
        pan_channel=0,
        tilt_channel=1,
        min_pulse=4000,
        max_pulse=8000,
        pan_min=0,
        pan_max=180,
        tilt_min=0,
        tilt_max=180,
        pan_center=90,
        tilt_center=90,
        delta_scale=0.5,
    ):
        """
        Initialize serial connection to Pololu Micro Maestro
        
        Args:
            port: Serial port (e.g., "COM3" on Windows)
            baudrate: Baud rate (Maestro default is 9600)
            timeout: Serial timeout in seconds
            pan_channel: Maestro channel for pan servo (0-5)
            tilt_channel: Maestro channel for tilt servo (0-5)
        """
        self.port = port
        self.baudrate = baudrate
        self.serial = None
        self.connected = False
        self.current_pan = 90
        self.current_tilt = 90
        self.lock = threading.Lock()
        
        # Servo channels on Maestro
        self.pan_channel = pan_channel
        self.tilt_channel = tilt_channel
        
        # Maestro pulse width range (in quarter-microseconds)
        # Standard servo: 1000-2000 µs = 4000-8000 quarter-µs
        self.min_pulse = int(min_pulse)  # 1000 µs (0 degrees)
        self.max_pulse = int(max_pulse)  # 2000 µs (180 degrees)

        # Angle limits (degrees)
        self.pan_min = float(pan_min)
        self.pan_max = float(pan_max)
        self.tilt_min = float(tilt_min)
        self.tilt_max = float(tilt_max)

        # Startup / center positions (degrees)
        self.pan_center = float(pan_center)
        self.tilt_center = float(tilt_center)

        # How much each move() delta changes the servo angle per call
        self.delta_scale = float(delta_scale)
        
        self._connect()
    
    def _connect(self):
        """Establish serial connection to Maestro"""
        try:
            self.serial = serial.Serial(
                port=self.port,
                baudrate=self.baudrate,
                timeout=1
            )
            time.sleep(0.5)  # Brief pause for connection
            self.connected = True
            print(f"Connected to Pololu Maestro on {self.port}")
            
            # Center servos on startup
            self.center()
            
        except serial.SerialException as e:
            self.connected = False
            raise Exception(f"Failed to connect to Maestro: {e}")
    
    def _angle_to_pulse(self, angle):
        """Convert angle (0-180) to Maestro pulse width (quarter-microseconds)"""
        angle = max(0.0, min(180.0, float(angle)))
        pulse = self.min_pulse + (angle / 180.0) * (self.max_pulse - self.min_pulse)
        return int(pulse)

    def _clamp_angles(self):
        self.current_pan = max(self.pan_min, min(self.pan_max, self.current_pan))
        self.current_tilt = max(self.tilt_min, min(self.tilt_max, self.current_tilt))
    
    def _set_target(self, channel, pulse):
        """
        Set servo target position using Maestro Compact Protocol
        
        Command format: 0x84, channel, target_low, target_high
        Target is in quarter-microseconds (e.g., 6000 = 1500 µs = center)
        """
        if not self.connected:
            return
        
        with self.lock:
            try:
                cmd = bytes([
                    0x84,  # Set Target command
                    channel,
                    pulse & 0x7F,  # Lower 7 bits
                    (pulse >> 7) & 0x7F  # Upper 7 bits
                ])
                self.serial.write(cmd)
                self.serial.flush()  # Ensure data is sent immediately
            except serial.SerialException as e:
                print(f"Serial write error: {e}")
                self.connected = False
    
    def move(self, pan_delta, tilt_delta):
        """
        Move servos by specified delta values
        
        Args:
            pan_delta: Change in pan angle (-100 to 100, scaled)
            tilt_delta: Change in tilt angle (-100 to 100, scaled)
        """
        if not self.connected:
            return
        
        # Scale deltas to angle change
        self.current_pan += float(pan_delta) * self.delta_scale
        self.current_tilt += float(tilt_delta) * self.delta_scale

        # Clamp to configured limits
        self._clamp_angles()
        
        # Send to servos
        pan_pulse = self._angle_to_pulse(self.current_pan)
        tilt_pulse = self._angle_to_pulse(self.current_tilt)
        
        self._set_target(self.pan_channel, pan_pulse)
        self._set_target(self.tilt_channel, tilt_pulse)
    
    def set_position(self, pan_angle, tilt_angle):
        """
        Set absolute servo positions
        
        Args:
            pan_angle: Pan servo angle (0-180)
            tilt_angle: Tilt servo angle (0-180)
        """
        if not self.connected:
            return
        
        self.current_pan = float(pan_angle)
        self.current_tilt = float(tilt_angle)
        self._clamp_angles()
        
        pan_pulse = self._angle_to_pulse(self.current_pan)
        tilt_pulse = self._angle_to_pulse(self.current_tilt)
        
        self._set_target(self.pan_channel, pan_pulse)
        self._set_target(self.tilt_channel, tilt_pulse)
    
    def center(self):
        """Center both servos to 90 degrees"""
        self.set_position(self.pan_center, self.tilt_center)
        print("Servos centered")
    
    def stop(self):
        """Stop servo movement (Maestro holds current position)"""
        pass
    
    def close(self):
        """Close serial connection"""
        if self.serial is not None and self.serial.is_open:
            self.center()
            time.sleep(0.3)
            self.serial.close()
            print("Serial connection closed")
        self.connected = False


class DummyServoController:
    """
    Dummy controller for testing without hardware
    """
    def __init__(self, *args, **kwargs):
        print("Using dummy servo controller (no hardware)")
        self.current_pan = 90
        self.current_tilt = 90
    
    def move(self, pan_delta, tilt_delta):
        pass
    
    def set_position(self, pan_angle, tilt_angle):
        self.current_pan = pan_angle
        self.current_tilt = tilt_angle
    
    def center(self):
        self.current_pan = 90
        self.current_tilt = 90
    
    def stop(self):
        pass
    
    def close(self):
        pass


# Simple PID controller for smooth tracking
class PIDController:
    def __init__(self, kp=0.5, ki=0.01, kd=0.1, output_limits=(-100, 100)):
        """
        Simple PID controller
        
        Args:
            kp: Proportional gain
            ki: Integral gain
            kd: Derivative gain
            output_limits: Tuple of (min, max) output values
        """
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.output_min, self.output_max = output_limits
        
        self.reset()
    
    def reset(self):
        """Reset controller state"""
        self.prev_error = 0
        self.integral = 0
        self.last_time = time.time()
    
    def update(self, error):
        """
        Calculate PID output
        
        Args:
            error: Current error value
            
        Returns:
            Control output value
        """
        current_time = time.time()
        dt = current_time - self.last_time
        
        if dt <= 0:
            dt = 0.001
        
        # Proportional term
        p_term = self.kp * error
        
        # Integral term (with anti-windup)
        self.integral += error * dt
        self.integral = max(-50, min(50, self.integral))  # Anti-windup
        i_term = self.ki * self.integral
        
        # Derivative term
        derivative = (error - self.prev_error) / dt
        d_term = self.kd * derivative
        
        # Calculate output
        output = p_term + i_term + d_term
        
        # Clamp output
        output = max(self.output_min, min(self.output_max, output))
        
        # Save state for next iteration
        self.prev_error = error
        self.last_time = current_time
        
        return output
