import cv2
import numpy as np

# ArUco markers come in "dictionaries" -- families of markers with a fixed
# grid size (e.g. 4x4, 6x6 bits) and a fixed number of unique IDs.
# DICT_4X4_50 = 4x4 bit grid, 50 unique marker IDs available (0 to 49).
# Smaller grids (4x4) are easier to detect from farther away / lower
# resolution, but support fewer unique IDs -- good enough for us, since
# we only need ONE marker for the drop zone.
ARUCO_DICT = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)

MARKER_ID = 0        # which marker (0-49) to generate -- pick any, just remember it
MARKER_SIZE_PX = 600  # output image size in pixels (bigger = sharper when printed larger)
PADDING_PX = 60       # white border ("quiet zone") around the marker -- helps detection

def generate_marker():
    # Generate the marker itself, as a black/white square image
    marker_img = cv2.aruco.generateImageMarker(ARUCO_DICT, MARKER_ID, MARKER_SIZE_PX)

    # Put it on a bigger white canvas, so there's a clean margin around it
    # (ArUco detection relies on a clear light border around the black
    # square to know where the marker starts/ends).
    canvas_size = MARKER_SIZE_PX + 2 * PADDING_PX
    canvas = np.full((canvas_size, canvas_size), 255, dtype=np.uint8)
    canvas[PADDING_PX:PADDING_PX + MARKER_SIZE_PX,
           PADDING_PX:PADDING_PX + MARKER_SIZE_PX] = marker_img

    filename = f"aruco_marker_{MARKER_ID}.png"
    cv2.imwrite(filename, canvas)
    print(f"Marker saved as {filename}")
    print(f"Marker ID: {MARKER_ID} (remember this for the detection code later)")

if __name__ == "__main__":
    generate_marker()