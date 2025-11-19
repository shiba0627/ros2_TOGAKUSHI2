import cv2

for i in range(100):
    cap = cv2.VideoCapture(i)
    if cap.isOpened():
        print(f"カメラを発見: index={i}")
        ret, img = cap.read()
        if ret:
            cv2.imshow(f"Camera {i}", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
        cap.release()

