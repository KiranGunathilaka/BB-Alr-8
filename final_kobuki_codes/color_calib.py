import cv2
import numpy as np
import time


def create_trackbars():
    """Create trackbars for HSV range adjustment."""
    # Create a window for the trackbars
    cv2.namedWindow("HSV Trackbars")

    # Create trackbars for lower and upper HSV bounds
    cv2.createTrackbar("L-H", "HSV Trackbars", 0, 179, lambda x: None)
    cv2.createTrackbar("L-S", "HSV Trackbars", 0, 255, lambda x: None)
    cv2.createTrackbar("L-V", "HSV Trackbars", 0, 255, lambda x: None)
    cv2.createTrackbar("U-H", "HSV Trackbars", 179, 179, lambda x: None)
    cv2.createTrackbar("U-S", "HSV Trackbars", 255, 255, lambda x: None)
    cv2.createTrackbar("U-V", "HSV Trackbars", 255, 255, lambda x: None)

    # Set initial values for common colors (blue by default)
    cv2.setTrackbarPos("L-H", "HSV Trackbars", 90)
    cv2.setTrackbarPos("L-S", "HSV Trackbars", 50)
    cv2.setTrackbarPos("L-V", "HSV Trackbars", 50)
    cv2.setTrackbarPos("U-H", "HSV Trackbars", 130)
    cv2.setTrackbarPos("U-S", "HSV Trackbars", 255)
    cv2.setTrackbarPos("U-V", "HSV Trackbars", 255)


def get_trackbar_values():
    """Get current values from all trackbars."""
    l_h = cv2.getTrackbarPos("L-H", "HSV Trackbars")
    l_s = cv2.getTrackbarPos("L-S", "HSV Trackbars")
    l_v = cv2.getTrackbarPos("L-V", "HSV Trackbars")
    u_h = cv2.getTrackbarPos("U-H", "HSV Trackbars")
    u_s = cv2.getTrackbarPos("U-S", "HSV Trackbars")
    u_v = cv2.getTrackbarPos("U-V", "HSV Trackbars")

    return np.array([l_h, l_s, l_v]), np.array([u_h, u_s, u_v])


def set_preset_values(preset):
    """Set trackbar positions to preset values for common colors."""
    presets = {
        "blue": ([90, 50, 50], [130, 255, 255]),
        "green": ([40, 40, 40], [80, 255, 255]),
        "red_lower": ([0, 120, 70], [10, 255, 255]),
        "red_upper": ([160, 100, 70], [180, 255, 255]),
        "yellow": ([20, 100, 100], [30, 255, 255]),
    }

    if preset in presets:
        lower, upper = presets[preset]
        cv2.setTrackbarPos("L-H", "HSV Trackbars", lower[0])
        cv2.setTrackbarPos("L-S", "HSV Trackbars", lower[1])
        cv2.setTrackbarPos("L-V", "HSV Trackbars", lower[2])
        cv2.setTrackbarPos("U-H", "HSV Trackbars", upper[0])
        cv2.setTrackbarPos("U-S", "HSV Trackbars", upper[1])
        cv2.setTrackbarPos("U-V", "HSV Trackbars", upper[2])


def connect_camera():
    """Attempt to connect to a camera."""
    camera_ports = [0, 1, 2]

    for port in camera_ports:
        try:
            print(f"Attempting to connect to camera on port {port}...")
            cap = cv2.VideoCapture(port)

            # Check if camera opened successfully
            if cap.isOpened():
                # Verify we can read a frame
                ret, frame = cap.read()
                if ret:
                    print(f"Successfully connected to camera on port {port}")
                    return cap
                else:
                    print(f"Camera on port {port} opened but couldn't read a frame")
                    cap.release()
            else:
                print(f"Failed to open camera on port {port}")
        except Exception as e:
            print(f"Error trying camera port {port}: {e}")
            if "cap" in locals():
                cap.release()

    return None


def main():
    """Main function to run the color calibration tool."""
    print("Starting Color Calibration Tool")

    # Connect to camera
    cap = connect_camera()
    if cap is None:
        print("No working camera found. Exiting.")
        return

    # Create trackbars for HSV adjustment
    create_trackbars()

    # Create windows for displaying images
    cv2.namedWindow("Original", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Mask", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Result", cv2.WINDOW_NORMAL)

    # Add instructions
    print("\nInstructions:")
    print("1. Place the colored box in front of the camera")
    print("2. Adjust the trackbars to isolate the color")
    print("3. Use keyboard shortcuts for preset colors:")
    print("   - 'b': Blue preset")
    print("   - 'g': Green preset")
    print("   - 'r': Red lower range preset")
    print("   - 't': Red upper range preset")
    print("   - 'y': Yellow preset")
    print("4. Press 's' to save the current values")
    print("5. Press 'q' to quit\n")

    current_color = "unnamed"

    while True:
        # Read frame from camera
        ret, frame = cap.read()
        if not ret:
            print("Failed to capture frame, retrying...")
            time.sleep(0.1)
            continue

        # Convert to HSV
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Get current trackbar positions
        lower_hsv, upper_hsv = get_trackbar_values()

        # Create mask and apply it
        mask = cv2.inRange(hsv, lower_hsv, upper_hsv)
        result = cv2.bitwise_and(frame, frame, mask=mask)

        # Find contours
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Draw contours on the original frame
        frame_with_contours = frame.copy()
        cv2.drawContours(frame_with_contours, contours, -1, (0, 255, 0), 2)

        # Draw color name and current HSV range
        cv2.putText(
            frame_with_contours,
            f"Color: {current_color}",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            frame_with_contours,
            f"Lower HSV: [{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}]",
            (10, 60),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        cv2.putText(
            frame_with_contours,
            f"Upper HSV: [{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}]",
            (10, 90),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (0, 255, 255),
            2,
        )

        # Show images
        cv2.imshow("Original", frame_with_contours)
        cv2.imshow("Mask", mask)
        cv2.imshow("Result", result)

        # Handle keyboard input
        key = cv2.waitKey(1) & 0xFF
        if key == ord("q"):
            break
        elif key == ord("b"):
            set_preset_values("blue")
            current_color = "blue"
        elif key == ord("g"):
            set_preset_values("green")
            current_color = "green"
        elif key == ord("r"):
            set_preset_values("red_lower")
            current_color = "red (lower range)"
        elif key == ord("t"):
            set_preset_values("red_upper")
            current_color = "red (upper range)"
        elif key == ord("y"):
            set_preset_values("yellow")
            current_color = "yellow"
        elif key == ord("s"):
            # Save current values
            print(f"\nSaved values for {current_color}:")
            print(f"Lower HSV: [{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}]")
            print(f"Upper HSV: [{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}]")

            # Format for direct use in the robot code
            print(f"\nFor robot code:")
            print(
                f'"{current_color}": (np.array([{lower_hsv[0]}, {lower_hsv[1]}, {lower_hsv[2]}]), np.array([{upper_hsv[0]}, {upper_hsv[1]}, {upper_hsv[2]}])),'
            )

            # Special case for red (which needs two ranges)
            if (
                current_color == "red (lower range)"
                or current_color == "red (upper range)"
            ):
                print(
                    "\nRemember: For red, you need to combine both lower and upper ranges!"
                )

    # Cleanup
    cap.release()
    cv2.destroyAllWindows()
    print("Color Calibration Tool closed")


if __name__ == "__main__":
    main()