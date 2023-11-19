import cv2

image = cv2.VideoCapture(0)

image.set(3, 600)
image.set(4, 500)
image.set(5, 30)  # Set frame
image.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
image.set(cv2.CAP_PROP_BRIGHTNESS, 40)  # Set brightness. Range: -64 to 64
image.set(cv2.CAP_PROP_CONTRAST, 40)  # Set contrast. Range: -64 to 64
image.set(cv2.CAP_PROP_EXPOSURE, 156)  # Set exposure. Range: 1 to 5000

while True:
    ret, frame = image.read()

    if ret:
        # Convert the frame to grayscale
        gray_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # Apply histogram equalization to improve contrast
        equalized_frame = cv2.equalizeHist(gray_frame)

        # Apply a threshold to keep only the darkest pixels
        _, dark_areas = cv2.threshold(equalized_frame, 50, 255, cv2.THRESH_BINARY)

        # Extract specific pixel values (adjust these coordinates accordingly)
        height, width = frame.shape[:2]
        left1Loc = (width // 2 - width // 4, height - 3)
        left2Loc = (width // 2 - width // 4 - width // 8, height - 3)
        right1Loc = (width // 2 + width // 4, height - 3)
        right2Loc = (width // 2 + width // 4 + width // 8, height - 3)

        # Draw circles at extracted pixel locations
        color = (0, 255, 0)
        cv2.circle(frame, left1Loc, 3, color, -1)
        cv2.circle(frame, left2Loc, 3, color, -1)
        cv2.circle(frame, right1Loc, 3, color, -1)
        cv2.circle(frame, right2Loc, 3, color, -1)

        # Display the frame
        cv2.imshow('Frame with Points', frame)

    # Exit the loop when 'q' is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

image.release()
cv2.destroyAllWindows()
