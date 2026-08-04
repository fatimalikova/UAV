import cv2

# Must match the dictionary used in generate_marker.py -- if you generate a
# marker with one dictionary, you MUST detect it with the same dictionary,
# otherwise it simply won't be recognized.
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

# DetectorParameters holds various fine-tuning knobs for the detection
# algorithm (thresholding sensitivity, minimum marker size, etc). Defaults
# work fine for a first test -- we'll only touch these later if detection
# turns out to be unreliable in real lighting conditions.
detector_params = cv2.aruco.DetectorParameters()

# ArucoDetector bundles the dictionary + parameters together. Create it
# ONCE, outside the loop below, and reuse it for every frame -- creating a
# new one every frame would be wasteful and unnecessary.
detector = cv2.aruco.ArucoDetector(ARUCO_DICT, detector_params)


def main():
    # 0 = default camera (usually the built-in laptop webcam).
    # If the wrong camera opens (e.g. you have more than one), try 1, 2...
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: could not open camera. Is another app (Zoom, Teams...) using it?")
        return

    print("Camera opened. Hold the printed marker up to the camera.")
    print("Press 'q' in the video window to quit.")

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Error: could not read a frame from the camera.")
            break

        # ArUco detection works on grayscale images (color isn't needed --
        # the marker is pure black/white, so converting first also makes
        # detection faster).
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

        # corners: pixel coordinates of each detected marker's 4 corners
        # ids: the ID of each detected marker (matches what generate_marker.py printed)
        # rejected: shapes that looked marker-like but didn't decode correctly -- ignored here
        corners, ids, rejected = detector.detectMarkers(gray)

        if ids is not None:
            # Draws a green outline around each detected marker and labels its ID,
            # directly onto the color frame (so you can see it in the window).
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)

            for marker_id, marker_corners in zip(ids.flatten(), corners):
                # marker_corners has shape (1, 4, 2) -- 4 corner points, each an (x, y) pixel
                pts = marker_corners[0]
                center_x = int(pts[:, 0].mean())
                center_y = int(pts[:, 1].mean())
                print(f"Detected marker ID {marker_id} at pixel ({center_x}, {center_y})")

        cv2.imshow("ArUco Detection", frame)

        # waitKey(1) waits 1ms for a keypress and returns -1 if none was
        # pressed; this line checks whether that key was 'q'.
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()


if __name__ == "__main__":
    main()