import cv2
import numpy as np
import time
import threading
from kobukidriver import Kobuki


class ColorSequenceRobot:
    def __init__(self):
        """Initialize the robot controller with color tracking capabilities."""
        # Initialize robot
        self.robot = Kobuki()

        # Initialize camera
        # List of camera interfaces to try
        camera_ports = [0, 1, 2]
        camera_connected = False
        
        while not camera_connected:
            for port in camera_ports:
                try:
                    print(f"Attempting to connect to camera on port {port}...")
                    self.cap = cv2.VideoCapture(port)
                    
                    # Check if camera opened successfully
                    if self.cap.isOpened():
                        # Verify we can read a frame
                        ret, frame = self.cap.read()
                        if ret:
                            print(f"Successfully connected to camera on port {port}")
                            self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
                            self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
                            camera_connected = True
                            break
                        else:
                            print(f"Camera on port {port} opened but couldn't read a frame")
                            self.cap.release()
                    else:
                        print(f"Failed to open camera on port {port}")
                except Exception as e:
                    print(f"Error trying camera port {port}: {e}")
                    if hasattr(self, 'cap'):
                        self.cap.release()
            
            if not camera_connected:
                print("No working camera found. Retrying in 5 seconds...")
                time.sleep(5)  # Wait before trying again
                
        self.width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))

        # Motor control parameters
        self.max_speed = 150
        self.running = True
        self.direction = "Stop"

        # Color detection parameters (HSV ranges)
        self.color_ranges = {
            "blue": (np.array([90, 50, 50]), np.array([130, 255, 255])),
            "red1": (
                np.array([0, 120, 70]),
                np.array([10, 255, 255]),
            ),  # Lower red range
            "red2": (
                np.array([160, 100, 70]),
                np.array([180, 255, 255]),
            ),  # Upper red range
            "green": (np.array([40, 40, 40]), np.array([80, 255, 255])),
            "yellow": (np.array([20, 100, 100]), np.array([30, 255, 255])),
        }

        # Task and phase management
        self.color_sequence = ["blue", "red", "green", "yellow"]
        self.current_color_index = 0
        self.current_color = self.color_sequence[self.current_color_index]
        self.current_task = "initial_turn_left"
        self.target_mode = "box"  # Either "box" or "goal"

        # Object detection parameters
        self.min_box_area = 500
        self.goal_area_threshold = 20000  # Adjust based on your camera and objects
        self.last_detection_time = time.time()
        self.detection_timeout = 2  # seconds to wait before giving up

        # Navigation parameters
        self.has_box = False
        self.turn_start_time = 0
        self.forward_time = 3000  # milliseconds, adjustable
        self.timer = 0
        self.retreat_time = 5000  # milliseconds

        # Create visualization window
        cv2.namedWindow("Robot View", cv2.WINDOW_NORMAL)

        # Pre-capture frame to initialize camera
        ret, _ = self.cap.read()
        if not ret:
            raise Exception("Could not access camera")

        print(f"Robot initialized. Starting with {self.current_color} phase.")

    def control_robot(self):
        """Thread function to control robot motors based on direction."""
        while self.running:
            if self.direction == "Forward":
                self.robot.move(self.max_speed, self.max_speed, 0)
            elif self.direction == "Left":
                self.robot.move(self.max_speed * 0.3, self.max_speed, 0)
            elif self.direction == "Right":
                self.robot.move(self.max_speed, self.max_speed * 0.3, 0)
            elif self.direction == "Stop":
                self.robot.move(0, 0, 0)
            elif self.direction == "Slow Forward":
                self.robot.move(self.max_speed * 0.5, self.max_speed * 0.5, 0)
            elif self.direction == "Turn Left":
                self.robot.move(0, self.max_speed * 0.8, 1)
            elif self.direction == "Turn Right":
                self.robot.move(self.max_speed * 0.8, 0, 1)
            elif self.direction == "Reverse":
                self.robot.move(-self.max_speed * 0.5, -self.max_speed * 0.5, 1)

            time.sleep(0.05)

    def detect_color_object(self, frame, goal_mode=False):
        """
        Detect colored objects in the frame.

        Args:
            frame: Camera frame to process
            goal_mode: If True, look for larger objects (goals)

        Returns:
            tuple: (detected, x, y, w, h, area, center_x)
        """
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Create mask for the current color
        if self.current_color == "red":
            # For red, combine two ranges due to how hue wraps around
            mask1 = cv2.inRange(
                hsv, self.color_ranges["red1"][0], self.color_ranges["red1"][1]
            )
            mask2 = cv2.inRange(
                hsv, self.color_ranges["red2"][0], self.color_ranges["red2"][1]
            )
            mask = cv2.bitwise_or(mask1, mask2)
        else:
            # For other colors, use the corresponding range
            lower, upper = self.color_ranges[self.current_color]
            mask = cv2.inRange(hsv, lower, upper)

        # Apply morphology operations to clean up the mask
        kernel = np.ones((5, 5), np.uint8)
        mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
        mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

        # Apply the mask
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Area constraints based on mode
        min_area = self.min_box_area
        if goal_mode:
            min_area = self.min_box_area * 3  # Goals are larger

        # Process contours
        max_area = 0
        best_contour = None

        for contour in contours:
            area = cv2.contourArea(contour)
            if area > min_area and area > max_area:
                max_area = area
                best_contour = contour

        # If contour found, draw and return information
        if best_contour is not None:
            x, y, w, h = cv2.boundingRect(best_contour)
            center_x = x + w // 2

            # Draw bounding box
            cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)

            # Draw center line
            cv2.line(frame, (center_x, y), (center_x, y + h), (255, 0, 0), 2)

            # Draw image center for reference
            img_center_x = self.width // 2
            cv2.line(
                frame, (img_center_x, 0), (img_center_x, self.height), (0, 255, 255), 1
            )

            # Add information
            cv2.putText(
                frame,
                f"{self.current_color.capitalize()} {self.target_mode}",
                (x, y - 10),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Area: {max_area:.1f}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Task: {self.current_task}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Update last detection time
            self.last_detection_time = time.time()

            return True, x, y, w, h, max_area, center_x
        else:
            # No object detected
            cv2.putText(
                frame,
                f"Looking for {self.current_color} {self.target_mode}",
                (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )
            cv2.putText(
                frame,
                f"Task: {self.current_task}",
                (10, 60),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.6,
                (255, 255, 255),
                2,
            )

            # Draw image center
            img_center_x = self.width // 2
            cv2.line(
                frame, (img_center_x, 0), (img_center_x, self.height), (0, 255, 255), 1
            )

            # Add small visualization of the mask
            small_mask = cv2.resize(mask, (160, 120))
            frame[10:130, self.width - 170 : self.width - 10] = cv2.cvtColor(
                small_mask, cv2.COLOR_GRAY2BGR
            )

            return False, 0, 0, 0, 0, 0, 0

    def is_box_captured(self, y, h):
        """Check if box is captured (near bottom of frame)."""
        CAPTURE_THERESHOLD = 0.9
        return y > self.height * CAPTURE_THERESHOLD

    def is_at_goal(self, area):
        """Check if robot has reached the goal based on area."""
        return area > self.goal_area_threshold

    def transition_to_next_color(self):
        """Move to the next color in the sequence."""
        self.current_color_index += 1
        if self.current_color_index < len(self.color_sequence):
            self.current_color = self.color_sequence[self.current_color_index]
            self.current_task = "turn_to_find_box"
            self.target_mode = "box"
            self.has_box = False
            self.timer = 0
            print(f"Transitioning to {self.current_color.upper()} phase")
        else:
            self.current_task = "all_completed"
            print("All color phases completed!")

    def run(self):
        """Main control loop for the robot."""
        # Start motor control thread
        robot_thread = threading.Thread(target=self.control_robot)
        robot_thread.daemon = True
        robot_thread.start()

        TURN_TIME = 8000

        # Play start sound
        self.robot.play_on_sound()

        # Initial task timing
        start_time = time.time()
        self.timer = 0

        try:
            while self.running:
                # Capture frame
                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to capture frame, retrying...")
                    time.sleep(0.1)
                    continue

                # Update timer
                current_time = time.time()
                elapsed = current_time - start_time
                self.timer = int(elapsed * 1000)  # Convert to milliseconds

                # For timing display
                cv2.putText(
                    frame,
                    f"Timer: {self.timer} ms",
                    (10, self.height - 10),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (255, 255, 255),
                    2,
                )

                # State machine for robot tasks
                if self.current_task == "initial_turn_left":
                    # First task: Turn left
                    self.direction = "Turn Left"
                    self.target_mode = "none"

                    # Display status
                    cv2.putText(
                        frame,
                        "Initial Turn Left",
                        (self.width // 2 - 80, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                    # After 2 seconds, move to next task
                    if self.timer >= TURN_TIME:
                        self.current_task = "go_forward"
                        start_time = current_time  # Reset timer
                        print("Initial left turn complete, moving forward")

                elif self.current_task == "go_forward":
                    # Second task: Move forward for set time
                    self.direction = "Forward"
                    self.target_mode = "none"

                    # Display progress
                    progress = min(100, int(self.timer / self.forward_time * 100))
                    cv2.putText(
                        frame,
                        f"Moving Forward: {progress}%",
                        (self.width // 2 - 120, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                    # After forward_time milliseconds, turn right
                    if self.timer >= self.forward_time:
                        self.current_task = "initial_turn_right"
                        start_time = current_time  # Reset timer
                        print("Forward movement complete, turning right")

                elif self.current_task == "initial_turn_right":
                    # Third task: Turn right
                    self.direction = "Turn Right"
                    self.target_mode = "none"

                    # Display status
                    cv2.putText(
                        frame,
                        "Initial Turn Right",
                        (self.width // 2 - 80, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                    # After 2 seconds, start looking for the first box
                    if self.timer >= TURN_TIME:
                        self.current_task = "find_box"
                        self.target_mode = "box"
                        start_time = current_time  # Reset timer
                        print(
                            f"Initial right turn complete, searching for {self.current_color} box"
                        )

                elif self.current_task == "find_box":
                    # Search for colored box
                    self.target_mode = "box"
                    detected, x, y, w, h, area, center_x = self.detect_color_object(
                        frame, False
                    )

                    if not detected:
                        # Turn slowly to search
                        self.direction = "Turn Left"

                        # Check for timeout
                        if (
                            current_time - self.last_detection_time
                            > self.detection_timeout
                        ):
                            cv2.putText(
                                frame,
                                "Searching...",
                                (self.width // 2 - 80, self.height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 0, 255),
                                2,
                            )
                    else:
                        # Found box, move to approach
                        self.current_task = "approach_box"
                        print(f"Found {self.current_color} box, approaching")

                elif self.current_task == "approach_box":
                    # Approach the detected box
                    self.target_mode = "box"
                    detected, x, y, w, h, area, center_x = self.detect_color_object(
                        frame, False
                    )

                    if not detected:
                        # Lost the box, go back to search
                        self.current_task = "find_box"
                        self.direction = "Stop"
                        print(f"Lost {self.current_color} box, searching again")
                    elif self.is_box_captured(y, h):
                        # Box is captured
                        self.direction = "Stop"
                        self.has_box = True
                        self.current_task = "turn_to_goal"
                        start_time = current_time  # Reset timer
                        print(
                            f"{self.current_color} box captured, turning to find goal"
                        )
                    else:
                        # Follow the box
                        img_center_x = self.width // 2
                        error = center_x - img_center_x

                        # Adjust direction based on box position
                        if abs(error) < 50:
                            self.direction = "Forward"
                        elif error < 0:
                            self.direction = "Left"
                        else:
                            self.direction = "Right"

                elif self.current_task == "turn_to_goal":
                    # Turn to face the goal area
                    self.direction = "Turn Right"
                    #self.direction = "Turn Left"
                    self.target_mode = "goal"

                    # Display status
                    cv2.putText(
                        frame,
                        "Turning to Find Goal",
                        (self.width // 2 - 100, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                    # After 2 seconds, start looking for the goal
                    if self.timer >= TURN_TIME:
                        self.current_task = "find_goal"
                        start_time = current_time  # Reset timer
                        print(
                            f"Turned to face goal area, searching for {self.current_color} goal"
                        )

                elif self.current_task == "find_goal":
                    # Search for the goal of same color
                    self.target_mode = "goal"
                    detected, x, y, w, h, area, center_x = self.detect_color_object(
                        frame, True
                    )

                    if not detected:
                        # Turn slowly to search
                        self.direction = "Turn Left"

                        # Check for timeout
                        if (
                            current_time - self.last_detection_time
                            > self.detection_timeout
                        ):
                            cv2.putText(
                                frame,
                                "Searching for Goal...",
                                (self.width // 2 - 100, self.height // 2),
                                cv2.FONT_HERSHEY_SIMPLEX,
                                0.8,
                                (0, 0, 255),
                                2,
                            )
                    else:
                        # Found goal, move to approach
                        self.current_task = "approach_goal"
                        print(f"Found {self.current_color} goal, approaching")

                elif self.current_task == "approach_goal":
                    # Approach the detected goal
                    self.target_mode = "goal"
                    detected, x, y, w, h, area, center_x = self.detect_color_object(
                        frame, True
                    )

                    if not detected:
                        # Lost the goal, go back to search
                        self.current_task = "find_goal"
                        self.direction = "Stop" 
                        print(f"Lost {self.current_color} goal, searching again")
                    elif self.is_at_goal(area) : 
                        # Reached the goal
                        self.direction = "Stop"
                        self.current_task = "retreat"
                        start_time = current_time  # Reset timer
                        print(f"Reached {self.current_color} goal, retreating")
                    else:
                        # Follow the goal
                        img_center_x = self.width // 2
                        error = center_x - img_center_x

                        # Adjust direction based on goal position
                        if abs(error) < 50:
                            self.direction = "Slow Forward"
                        elif error < 0:
                            self.direction = "Left"
                        else:
                            self.direction = "Right"

                elif self.current_task == "retreat":
                    # Back up after reaching goal
                    self.direction = "Reverse"

                    # Display progress
                    progress = min(100, int(self.timer / self.retreat_time * 100))
                    cv2.putText(
                        frame,
                        f"Retreating: {progress}%",
                        (self.width // 2 - 80, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                    # After retreat_time, move to next color
                    if self.timer >= self.retreat_time:
                        self.direction = "Stop"
                        self.transition_to_next_color()
                        start_time = current_time  # Reset timer

                elif self.current_task == "turn_to_find_box":
                    # Turn to position for finding next box
                    self.direction = "Turn Left"

                    # Display status
                    cv2.putText(
                        frame,
                        f"Turning to Find {self.current_color.capitalize()} Box",
                        (self.width // 2 - 150, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.8,
                        (0, 0, 255),
                        2,
                    )

                    # After turning period, start looking for the next box
                    if self.timer >= 3000:  # 3 seconds of turning
                        self.current_task = "find_box"
                        start_time = current_time  # Reset timer
                        print(f"Positioned to find {self.current_color} box, searching")

                elif self.current_task == "all_completed":
                    # All colors processed
                    self.direction = "Stop"
                    cv2.putText(
                        frame,
                        "All Tasks Completed!",
                        (self.width // 2 - 120, self.height // 2),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.0,
                        (0, 255, 0),
                        2,
                    )

                # Display the frame
                cv2.imshow("Robot View", frame)

                # Handle keyboard input
                key = cv2.waitKey(1) & 0xFF
                if key == ord("q"):
                    break
                elif key == ord("f"):
                    # Adjust forward time
                    self.forward_time += 500
                    print(f"Forward time increased to {self.forward_time} ms")
                elif key == ord("g"):
                    # Decrease forward time
                    self.forward_time = max(500, self.forward_time - 500)
                    print(f"Forward time decreased to {self.forward_time} ms")
                elif key == ord("r"):
                    # Reset to beginning
                    self.current_color_index = 0
                    self.current_color = self.color_sequence[self.current_color_index]
                    self.current_task = "initial_turn_left"
                    self.has_box = False
                    start_time = current_time
                    print("Reset to beginning of sequence")

        finally:
            # Cleanup
            self.running = False
            robot_thread.join(timeout=1.0)
            self.robot.move(0, 0, 0)
            self.robot.play_off_sound()
            self.cap.release()
            cv2.destroyAllWindows()
            print("Robot stopped and resources released")


def main():
    """Main function to run the robot."""
    print("Starting Color Sequence Robot")

    # Parameters that can be adjusted
    robot = ColorSequenceRobot()

    # Adjust forward time if needed (default is 3000 ms)
    robot.forward_time = 3000

    # Adjust retreat time if needed (default is 3000 ms)
    robot.retreat_time = 5000

    # Adjust goal area threshold based on your objects and camera
    robot.goal_area_threshold = 70000

    print(
        "Robot configured. Press 'q' to quit, 'f'/'g' to adjust forward time, 'r' to reset."
    )

    try:
        robot.run()
    except Exception as e:
        print(f"An error occurred: {e}")


if __name__ == "__main__":
    main()
