import cv2
import numpy as np

# 색상에 따른 HSV 범위 설정 (빨간색 범위 확장)
colors_hsv = {
    'red_lower': [np.array([0, 120, 120]), np.array([10, 255, 255])],
    'red_upper': [np.array([170, 120, 120]), np.array([180, 255, 255])],
    'yellow': [np.array([22, 130, 140]), np.array([38, 255, 255])],  # 노란색 범위 확장
    'blue': [np.array([100, 100, 120]), np.array([130, 255, 255])],
    'purple': [np.array([125, 50, 50]), np.array([150, 255, 255])]
}

# 색상에 따른 BGR 값
colors_bgr = {
    'red': (0, 0, 255),
    'yellow': (0, 255, 255),
    'blue': (255, 0, 0),
    'purple': (255, 0, 255)
}

# 최소 바운딩 박스 크기 설정
min_width = 20  # 너비 최소값
min_height = 20  # 높이 최소값

def process_video():
    # 카메라 캡처 객체 생성
    cap = cv2.VideoCapture(2)

    # 카메라 열기 실패시 종료
    if not cap.isOpened():
        print("Camera open failed!")
        return

    # 비디오 캡처 반복
    while True:
        # 프레임 캡처
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        cv2.putText(frame, "point", (290, 480), cv2.FONT_HERSHEY_COMPLEX, 0.7, colors_bgr['blue'], 2)
        # HSV 색공간으로 변환
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # 각 색상별로 처리
        for color, (lower, upper) in colors_hsv.items():
            # 빨간색의 경우, 두 범위 모두 처리
            if 'red' in color:
                mask1 = cv2.inRange(hsv_image, colors_hsv['red_lower'][0], colors_hsv['red_lower'][1])
                mask2 = cv2.inRange(hsv_image, colors_hsv['red_upper'][0], colors_hsv['red_upper'][1])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_image, lower, upper)
            
            # 윤곽선 찾기
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

            # 윤곽선을 바탕으로 바운딩 박스 및 라벨링
            for contour in contours:
                x, y, w, h = cv2.boundingRect(contour)
                if w > min_width and h > min_height:  # 크기 필터링 조건
                    cv2.rectangle(frame, (x, y), (x+w, y+h), colors_bgr['red' if 'red' in color else color], 2)
                    cv2.putText(frame, 'red' if 'red' in color else color, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, colors_bgr['red' if 'red' in color else color], 2)
                    

        # 결과 이미지 출력
        cv2.imshow('Detected Colors', frame)

        # 'q'를 누르면 종료
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    # 자원 해제
    cap.release()
    cv2.destroyAllWindows()

process_video()
