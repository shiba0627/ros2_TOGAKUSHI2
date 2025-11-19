import cv2
from config import CAM_ID, CAM_ID_2

def main():
    cap1 = cv2.VideoCapture(CAM_ID)
    cap1.set(cv2.CAP_PROP_BUFFERSIZE, 5)

    cap2 = cv2.VideoCapture(CAM_ID_2)
    cap2.set(cv2.CAP_PROP_BUFFERSIZE, 5)

    if not cap1.isOpened():
        print('カメラ1が開けません')
        return
    if not cap2.isOpened():
        print('カメラ2が開けません')
        return

    while True:
        ret1, frame1 = cap1.read()
        ret2, frame2 = cap2.read()

        if not ret1:
            print("カメラ1のフレーム取得に失敗しました")
            continue
        if not ret2:
            print("カメラ2のフレーム取得に失敗しました")
            continue

        # 2画面を並べて表示（横に並べる）
        combined = cv2.hconcat([frame1, frame2])
        cv2.imshow('Camera 1 & 2', combined)

        # qキーで終了
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap1.release()
    cap2.release()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()
