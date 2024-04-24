import cv2
import torch
from numpy import random
import numpy as np
from pymycobot.mycobot import MyCobot
import time

# mc라는 객체가 로봇 컨트롤을 위한 메서드를 가지고 있다고 가정합니다.
# get_coords는 현재 좌표를 얻어오는 메서드이며,
# sync_send_coords는 주어진 좌표로 로봇을 이동시키는 메서드입니다.

mc = MyCobot("/dev/ttyACM0", 115200)

def approach_target_incrementally(mc, target_coords, increment=1, tolerance=10):
    current_coords = list(mc.get_coords())
    target_coords = list(target_coords)
    
    while True:
        for i in range(len(current_coords)):
            if abs(target_coords[i] - current_coords[i]) > tolerance:
                if current_coords[i] < target_coords[i]:
                    current_coords[i] += increment
                elif current_coords[i] > target_coords[i]:
                    current_coords[i] -= increment
            # 현재 좌표가 목표 좌표와 tolerance 이내로 도달하면 해당 좌표 조정을 중지합니다.
        
        # 좌표를 조정한 후 로봇에게 새로운 좌표로 이동하도록 명령합니다.
        mc.send_coords(tuple(current_coords), 20, 0)
        time.sleep(0.5)  # 로봇이 이동할 시간을 주기 위해 일시 중지
        
        # 현재 좌표를 업데이트하고 목표에 도달했는지 확인합니다.
        print(current_coords)
        current_coords = list(mc.get_coords())
        if all(abs(target_coords[i] - current_coords[i]) <= tolerance for i in range(len(current_coords))):
            break  # 모든 좌표가 목표 좌표 내에 있으면 반복을 종료합니다.

    print("Target coordinates reached.")

# 예제 사용:
# mc는 로봇의 컨트롤러 객체입니다.
# target_coords는 목표 좌표 및 RPY 값입니다. (x, y, z, roll, pitch, yaw)
# approach_target_incrementally(mc, [100, 200, 150, 0, 0, 90])


width, height = 640, 480

# 로봇 암 초기화
#mc.wait(1)
#mc.set_gripper_state(0, 60)
#mc.wait(1)

#mc.sync_send_angles([0, 0, -90, 30, 90, 0], 60, 2)
mc.sync_send_coords([329, -55, 227, -178, -4, -83], 20, 1)
#mc.sync_send_angles([0, 30, -120, 30, 90, 0], 60, 2)
time.sleep(2)
init_coords = list(mc.get_coords())
init_angle = list(mc.get_angles())
print(mc.get_coords())

mc.set_gripper_calibration()
mc.set_gripper_mode(0)
mc.init_eletric_gripper()

cap = cv2.VideoCapture(2)

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

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        #mc.set_gripper_value(0,20,1)


        # 감지된 객체 처리
        for i, det in enumerate(pred):  # detections per image
            if len(det):
                # 좌표 재조정
                det[:, :4] = scale_coords(img.shape[2:], det[:, :4], frame.shape).round()

                for *xyxy, conf, cls in det:
                    # 결과 표시
                    label = f'{model.names[int(cls)]} {conf:.2f}'
                    plot_one_box(xyxy, frame, label=label, color=(255, 0, 0), line_thickness=2)
                    
                    # 'bottle' 감지 시 로봇 팔 조작
                    if model.names[int(cls)] == 'mouse':
                        x_center = (xyxy[0] + xyxy[2]) / 2
                        y_center = (xyxy[1] + xyxy[3]) / 2

                        if x_center < width * 0.48:
                            init_coords[1] += 1  # 왼쪽 이동
                        elif x_center > width * 0.52:
                            init_coords[1] -= 1  # 오른쪽 이동
                        elif width * 0.48 <= x_center <= width * 0.52:
                            if y_center < height * 0.78:
                                init_coords[0] += 1  # 왼쪽 이동
                            elif y_center > height * 0.82:
                                init_coords[0] -= 1  # 오른쪽 이동

                        mc.send_coords(tuple(init_coords), 20, 1)
                        print(mc.get_coords())
                        time.sleep(0.1)  # 로봇 팔이 위치를 조정할 시간을 줌
                        if width * 0.48 <= x_center <= width * 0.52 and height * 0.78 <= y_center <= height * 0.82:     

                            init_coords[0] += 80
                            mc.send_coord(0, init_coords[0], 20)
                            #mc.sync_send_coords()
                            mc.send_coords(tuple(init_coords), 20, 1)
                            time.sleep(2)
                            print("check1: ", mc.get_coords())
                            init_coords[2:5] = [170, -175, 0]
                            mc.send_coords(tuple(init_coords), 20, 0)
                            time.sleep(2)

                            print("check2: ", mc.get_coords())
                            #approach_target_incrementally(mc, init_coords, increment=1, tolerance=10)
                            time.sleep(1)

                            mc.set_eletric_gripper(1)
                            print(int(((xyxy[2] - xyxy[0])*100*20)/640/16))
                            mc.set_gripper_value(0, 20, 1)
                            #mc.set_gripper_value(int(((xyxy[2] - xyxy[0])*100*20)/640/16), 20, 1)

                            time.sleep(2)

                            mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60, 2)



        # 결과 보여주기
        cv2.imshow('YOLOv7-tiny detection', frame)
        if cv2.waitKey(1) == ord('q'):  # q를 누르면 종료
            # 초기 위치로 로봇 팔을 돌려보내기
  
            break
finally:
    mc.sync_send_angles([0, 0, 0, 0, 0, 0], 60, 2)
    mc.set_eletric_gripper(0)
    mc.set_gripper_value(100,20,1)
    cap.release()
    cv2.destroyAllWindows()
