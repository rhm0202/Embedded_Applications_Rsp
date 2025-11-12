# 2.
import cv2
import numpy as np

IMGS = ["./img/1.jpg", "./img/2.jpg", "./img/3.jpg", "./img/4.jpg"]  # 올려준 파일명

# 노란색 마스크 범위
Y1_LO = np.array([25, 85, 150], np.uint8)
Y1_HI = np.array([40, 255, 255], np.uint8)


def get_yellow_mask(bgr):
    hsv = cv2.cvtColor(bgr, cv2.COLOR_BGR2HSV)
    h, s, v = cv2.split(hsv)
    hsv = cv2.merge((h, s, v))

    mask = cv2.inRange(hsv, Y1_LO, Y1_HI)
    return mask


def process(path):
    img = cv2.imread(path)

    if img is None:
        print(f"[WARN] 읽기 실패: {path}")
        return

    h, w = img.shape[:2]

    # 노란색 라인 마스크 추출
    mask = get_yellow_mask(img)

    lay = img.copy()
    lay[mask > 0] = (155, 50, 155)

    out_over = cv2.addWeighted(img, 0.5, lay, 0.5, 0)

    cv2.imshow("Overlay", out_over)

    cv2.waitKey(0)
    cv2.destroyAllWindows()


def main():
    for p in IMGS:
        process(p)


if __name__ == "__main__":
    main()
