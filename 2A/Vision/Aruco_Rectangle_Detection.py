import cv2
import numpy as np
from pyzbar.pyzbar import decode
import cv2
import cv2.aruco as aruco
qr_code_width=100#milimetres
map_width=3000
map_heigh=2000
#width_ratio=qr_code_width/map_width
#heigh_ratio=qr_code_width/map_heigh

def draw_qr_codes(frame):
    # Convert the frame to grayscale (pyzbar works better in grayscale)
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect and decode the QR codes
    qr_codes = decode(gray)

    # Initialize the dictionary to store QR code data and coordinates
    qr_data_dict = {}

    for qr_code in qr_codes:
        # Extract QR code corner points (polygon)
        points = qr_code.polygon

        # Check if we have four points (it should be a quadrilateral)
        if len(points) == 4:
            # Draw a polygon around the QR code (to handle rotation)
            pts = [(point.x, point.y) for point in points]
            pts = np.array(pts, dtype=np.int32)
            cv2.polylines(frame, [pts], isClosed=True, color=(0, 255, 0), thickness=2)

            # Decode the QR code's data
            qr_data = qr_code.data.decode('utf-8')

            # Store the QR code data and coordinates in the dictionary
            qr_data_dict[qr_data] = pts.tolist()  # Store the coordinates as a list of tuples

            # Get the top-left point to place the text (you can change this as needed)
            x, y = pts[0][0], pts[0][1]

            # Write the decoded data near the QR code
            cv2.putText(frame, qr_data, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    return frame, qr_data_dict

'''def detect_rectangles(frame):
    # Convert the frame to grayscale for processing
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Apply GaussianBlur to reduce noise
    blurred = cv2.GaussianBlur(gray, (5, 5), 0)

    # Apply Canny Edge Detection
    edges = cv2.Canny(blurred, 50, 150)

    # Find contours from the edges
    contours, _ = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    for contour in contours:
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # Check if the polygon has 4 sides
        if len(approx) == 4:
            # Calculate the aspect ratio to ensure it's close to a rectangle
            (x, y, w, h) = cv2.boundingRect(approx)
            aspect_ratio = float(w) / h

            # Check if the aspect ratio is reasonable (to filter out non-rectangular shapes)
            if 0.8 <= aspect_ratio <= 1.2:  # Aspect ratio close to 1 for square-like shapes
                # Check if the shape is large enough to be considered a rectangle
                if cv2.contourArea(contour) > 1000:  # Adjust the minimum area threshold if needed
                    # Draw a red rectangle around the detected rectangle
                    cv2.polylines(frame, [approx], isClosed=True, color=(0, 0, 255), thickness=2)

    return frame
'''
def detect_aruco(frame):
    # Load the dictionary that was used to generate the markers.
    aruco_dict = aruco.getPredefinedDictionary(aruco.DICT_4X4_250)

    # Initialize the detector parameters using default values
    parameters = aruco.DetectorParameters()

    # Create the ArUco detector
    detector = aruco.ArucoDetector(aruco_dict, parameters)

    # Convert to grayscale
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # Detect the markers in the image
    corners, ids, rejectedImgPoints = detector.detectMarkers(gray)

    if ids is not None:
        print("detected ")
        # Draw detected markers
        aruco.drawDetectedMarkers(frame, corners, ids)

        # Print the IDs of the detected markers
        for i in range(len(ids)):
            c = corners[i][0]
            # Position to write the ID
            x = int(c[0][0])
            y = int(c[0][1])
            cv2.putText(frame, str(ids[i][0]), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
            print(str(ids[i][0]))

    return frame


def main():
    # Open the default camera
    cap = cv2.VideoCapture(0)

    if not cap.isOpened():
        print("Error: Could not open camera.")
        return

    while True:
        # Capture frame-by-frame
        ret, frame = cap.read()

        if not ret:
            print("Error: Failed to capture image.")
            break
        qr_data_dict = {}
        # Process the frame to detect QR codes and draw rectangles
        frame,qr_data_dict = draw_qr_codes(frame)

        # Process the frame to detect general rectangles and highlight them in red
        #frame = detect_rectangles(frame)

        frame = detect_aruco(frame)

        # Display the resulting frame
        cv2.imshow('QR Code and Rectangle Detector', frame)

        # Press 'q' to quit
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # Release the capture and close windows
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()