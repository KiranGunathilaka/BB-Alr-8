import cv2
import numpy as np
import sys
from controller import Robot, Camera, Motor

class BBAlr8OpenCVController:
    def __init__(self, target_color='red', stop_threshold=0.7):
        """
        Initialize the robot controller with OpenCV color tracking.
        """
        # Initialize the robot
        self.robot = Robot()
        
        # Get the time step of the current world
        self.time_step = int(self.robot.getBasicTimeStep())
        
        # Initialize motors
        self.left_motor = self.robot.getDevice('leftMotor')
        self.right_motor = self.robot.getDevice('rightMotor')
        
        # Initialize camera
        self.camera = self.robot.getDevice('fcam')
        self.camera.enable(self.time_step)
        
        # Get camera dimensions
        self.width = self.camera.getWidth()
        self.height = self.camera.getHeight()
        
        # Set motor properties
        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
        
        # Color tracking parameters
        self.target_color = target_color
        self.stop_threshold = stop_threshold
        
        # Default motor speeds
        self.MAX_SPEED = 7.0
        
        # Color ranges (in HSV)
        self.color_ranges = {
            'red': ([0, 100, 100], [10, 255, 255]),
            'blue': ([100, 100, 100], [140, 255, 255]),
            'green': ([40, 100, 100], [80, 255, 255]),
        }
    
    def detect_color_object(self, frame):
        """
        Detect color object using OpenCV.
        Returns object information or None if no object found.
        """
        # Convert to HSV color space
        img_hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        
        # Get color range for target color
        lower, upper = self.color_ranges.get(self.target_color, 
                                             ([0, 100, 100], [10, 255, 255]))
        
        # Create color mask
        mask = cv2.inRange(img_hsv, np.array(lower), np.array(upper))
        
        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Visualize frame with detection
        frame_with_detection = frame.copy()
        
        # Find largest contour
        if contours:
            largest_contour = max(contours, key=cv2.contourArea)
            
            # Get moments to calculate center
            M = cv2.moments(largest_contour)
            if M["m00"] != 0:
                cx = int(M["m10"] / M["m00"])
                cy = int(M["m01"] / M["m00"])
                
                # Calculate area and vertical position
                area = cv2.contourArea(largest_contour)
                vertical_position = cy / self.height
                
                # Draw contour and label
                cv2.drawContours(frame_with_detection, [largest_contour], -1, (0, 255, 0), 2)
                
                # Add text label
                label = f"{self.target_color.capitalize()} Object"
                cv2.putText(frame_with_detection, label, (cx, cy), 
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)
                
                return {
                    'x': cx,
                    'y': cy,
                    'area': area,
                    'vertical_position': vertical_position,
                    'frame': frame_with_detection
                }
        
        return {'frame': frame_with_detection, 'x': None, 'y': None}
    
    def adjust_movement(self, object_info):
        """
        Adjust robot movement based on object position.
        
        :param object_info: Dictionary with object detection information
        """
        if object_info['x'] is None:
            return
        
        width = self.camera.getWidth()
        center = width // 2
        
        # Calculate error (deviation from center)
        error = object_info['x'] - center
        
        # Proportional control for turning
        turn_rate = error * 0.1
        
        # Set motor speeds
        left_speed = self.MAX_SPEED - turn_rate
        right_speed = self.MAX_SPEED + turn_rate
        
        # Ensure speeds are within limits
        left_speed = max(min(left_speed, self.MAX_SPEED), -self.MAX_SPEED)
        right_speed = max(min(right_speed, self.MAX_SPEED), -self.MAX_SPEED)
        
        self.left_motor.setVelocity(left_speed)
        self.right_motor.setVelocity(right_speed)
    
    def move_forward(self):
        """
        Move the robot forward at a constant speed.
        """
        self.left_motor.setVelocity(self.MAX_SPEED)
        self.right_motor.setVelocity(self.MAX_SPEED)
    
    def stop(self):
        """
        Stop the robot's movement.
        """
        self.left_motor.setVelocity(0)
        self.right_motor.setVelocity(0)
    
    def run(self):
        """
        Main control loop for the robot.
        """
        while self.robot.step(self.time_step) != -1:
            # Capture image from Webots camera
            image_data = self.camera.getImage()
            if not image_data:
                print("ERROR: No image data received!")
                continue
            
            # Convert Webots image to OpenCV format
            frame = np.frombuffer(image_data, np.uint8).reshape((self.height, self.width, 4))
            
            # Convert RGBA to BGR
            frame_bgr = cv2.cvtColor(frame, cv2.COLOR_RGBA2BGR)
            
            # Detect color object
            object_info = self.detect_color_object(frame_bgr)
            
            # Display the frame with detections
            cv2.imshow('Robot Camera - Color Tracking', object_info['frame'])
            
            # Break loop if 'q' is pressed
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
            
            # # Check for detected object
            # if object_info['x'] is not None:
            #     # Check if object is close enough to stop
            #     if object_info['vertical_position'] >= self.stop_threshold:
            #         self.stop()
            #         break
                
            #     # Adjust movement towards object
            #     self.adjust_movement(object_info)
            # else:
            #     # No object, move forward
            #     self.move_forward()
        
        # Clean up
        cv2.destroyAllWindows()

def main():
    # You can change the target color and stop threshold here
    # Options: 'red', 'green', 'blue'
    controller = BBAlr8OpenCVController(target_color='blue', stop_threshold=0.7)
    controller.run()

if __name__ == "__main__":
    main()