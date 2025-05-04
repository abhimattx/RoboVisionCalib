"""
Camera handling modules for calibration system.
"""

import cv2
from pypylon import pylon

def initialize_camera():
    """Initialize and configure Basler camera."""
    available_devices = pylon.TlFactory.GetInstance().EnumerateDevices()
    if not available_devices:
        raise Exception("No camera found")
    
    camera = pylon.InstantCamera(pylon.TlFactory.GetInstance().CreateFirstDevice())
    camera.Open()
    # Fix deprecation warning
    camera.PixelFormat.Value = "RGB8"  # Changed from direct assignment
    camera.ExposureAuto.SetValue("Continuous")
    camera.GainAuto.SetValue("Continuous")
    
    print(f"Using camera: {camera.GetDeviceInfo().GetModelName()}")
    return camera

class CameraCapture:
    """Manages camera operations for calibration"""
    
    def __init__(self):
        """Initialize camera capture system"""
        self.camera = None
        self.running = False
        
    def start(self):
        """Start the camera capture system"""
        try:
            self.camera = initialize_camera()
            self.camera.StartGrabbing(pylon.GrabStrategy_LatestImageOnly)
            self.running = True
            print("Camera initialized successfully")
            return True
        except Exception as e:
            print(f"Failed to start camera: {str(e)}")
            return False
    
    def stop(self):
        """Stop the camera capture system and release resources"""
        if self.camera and self.camera.IsGrabbing():
            self.camera.StopGrabbing()
            self.camera.Close()
        self.running = False
        cv2.destroyAllWindows()
    
    def get_frame(self):
        """Capture and return a single frame from the camera"""
        if not self.camera or not self.running:
            print("Camera not initialized")
            return None
            
        try:
            grabResult = self.camera.RetrieveResult(5000, pylon.TimeoutHandling_ThrowException)
            if grabResult.GrabSucceeded():
                img = grabResult.Array
                img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                grabResult.Release()
                return img
            else:
                grabResult.Release()
                return None
        except Exception as e:
            print(f"Frame capture error: {str(e)}")
            return None