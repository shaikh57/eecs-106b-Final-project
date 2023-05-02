import cv2

# Create a VideoCapture object to access the webcam
cap = cv2.VideoCapture(1)

count = 0
while True:
    # Read a frame from the webcam
    ret, frame = cap.read()
    resize_img = cv2.resize(frame  , (640 , 480))

    # Convert the frame to grayscale
    gray = cv2.cvtColor(resize_img, cv2.COLOR_BGR2GRAY)

    # Detect edges in the grayscale frame using the Canny algorithm
    edges = cv2.Canny(gray, 100, 200)

    # Find contours in the edges image
    contours, hierarchy = cv2.findContours(edges, cv2.RETR_TREE, cv2.CHAIN_APPROX_SIMPLE)

    # Loop over all contours and filter for rectangular shapes
    for contour in contours:
        approx = cv2.approxPolyDP(contour, 0.01 * cv2.arcLength(contour, True), True)
        if len(approx) == 4:
            x, y, w, h = cv2.boundingRect(approx)
            aspect_ratio = float(w)/h
            if aspect_ratio >= 0.8 and aspect_ratio <= 1.2 and (w) > 5 and (h) > 5:
                cv2.rectangle(resize_img, (x, y), (x+w, y+h), (0, 255, 0), 2)
                print(x*10/640,y*10/480,(x+w)*10/640,(y+h)*10/480)

    # Show the resulting frame
    cv2.imshow('frame', resize_img)

    # Exit if the 'q' key is pressed
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

# Release the VideoCapture object and close all windows
cap.release()
cv2.destroyAllWindows()
