import cv2

image = cv2.imread("./img/1.jpg")
image2 = image.copy()

# 컨투어 찾기 전 이미지 전처리
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
gray = cv2.GaussianBlur(gray, (5, 5), 0)
gray = cv2.bitwise_not(gray) # 객체보다 배경이 밝은 경우 이미지 반전

# canny edge, threshold 등 다양한 전처리 시도 -> 객체와 배경을 가장 잘 분리하는 전처리 사용
edge = cv2.Canny(gray, 100, 100)
_, thresh = cv2.threshold(gray, 127, 255, cv2.THRESH_BINARY)
adaptive_threshold= cv2.adaptiveThreshold(gray, 255, cv2.ADAPTIVE_THRESH_GAUSSIAN_C, cv2.THRESH_BINARY, 11, 2)

thresh = cv2.erode(thresh, None, iterations=2)
thresh = cv2.dilate(thresh, None, iterations=2)

# 컨투어 찾기
contours, hierachy = cv2.findContours(thresh.copy(), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)


# 컨투어 면적이 큰 순으로 정렬
sorted_contours = sorted(contours, key=cv2.contourArea, reverse=True)

for i in range(len(sorted_contours)):
    contour = sorted_contours[i]
    
    # 근사 컨투어 계산을 위한 0.01의 오차 범위 지정 
    epsilon = 0.01 * cv2.arcLength(contour, True)
    approx = cv2.approxPolyDP(contour, epsilon, True)

    cv2.drawContours(image, [contour], -1, (0,255,0), 3)
    cv2.drawContours(image2, [approx], -1, (0,255,0), 3)

    extLeft = tuple(contour[contour[:, :, 0].argmin()][0])
    extRight = tuple(contour[contour[:, :, 0].argmax()][0])
    extTop = tuple(contour[contour[:, :, 1].argmin()][0])
    extBot = tuple(contour[contour[:, :, 1].argmax()][0])

    cv2.circle(image, extLeft, 8, (0, 0, 255), -1)
    cv2.circle(image, extRight, 8, (0, 255, 0), -1)
    cv2.circle(image, extTop, 8, (255, 0, 0), -1)
    cv2.circle(image, extBot, 8, (255, 255, 0), -1)

# 결과 출력
cv2.imshow('contour', image)
cv2.imshow('approx', image2)
cv2.waitKey()
cv2.destroyAllWindows()