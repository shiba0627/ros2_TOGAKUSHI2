#カメラ画像を取得し保存するノード
import cv2
import rclpy

CAM_ID = 4

def get_img():#無限ループで動画を表示
    cap = cv2.VideoCapture(CAM_ID)
    try:
        while True:
            ret, frame = cap.read()
            if not ret:
                print('read()失敗')
                break
            cv2.imshow(f'cam{CAM_ID}',frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    except KeyboardInterrupt:
        print('ctrl + C で終了')
    finally:
        cap.release()
        cv2.destroyAllWindows()

def main():
    get_img()
if __name__ == '__main__':
    main()