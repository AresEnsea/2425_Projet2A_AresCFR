import cv2

cap=cv2.VideoCapture(0)

while True:
    ret, frame=cap.read()
    if cv2.waitKey(1)==ord('q'):
        break

    #detecter un visage
    gray_image = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    face_classifier = cv2.CascadeClassifier(
        cv2.data.haarcascades + "haarcascade_frontalface_default.xml"
    )
    face = face_classifier.detectMultiScale(
        gray_image, scaleFactor=1.1, minNeighbors=5, minSize=(40, 40)
    )
    for (x, y, w, h) in face:
        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 4)

    #afficher le frame
    cv2.imshow('video', frame)

cap.release()
cv2.destroyAllWindows()