import cv2
import torch
from numpy import random
from models.experimental import attempt_load
from utils.general import non_max_suppression, scale_coords, check_img_size
from utils.plots import plot_one_box
from pymycobot.mycobot import MyCobot
import time

# mc라는 객체가 로봇 컨트롤을 위한 메서드를 가지고 있다고 가정합니다.
# get_coords는 현재 좌표를 얻어오는 메서드이며,
# sync_send_coords는 주어진 좌표로 로봇을 이동시키는 메서드입니다.

mc = MyCobot("/dev/ttyACM1", 115200)

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

# 모델 로드
device = 'cpu'  # cuda 대신 cpu 사용
model = attempt_load('yolov7-tiny.pt', map_location=device)
model.to(device).eval()
# 영상 소스
cap = cv2.VideoCapture(2)

try:
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break
        #mc.set_gripper_value(0,20,1)

        # 이미지 전처리
        img = torch.from_numpy(frame).to(device)
        img = img.float() / 255.0  # 0 - 255 to 0.0 - 1.0
        img = img.permute(2, 0, 1).unsqueeze(0)

        # 추론
        with torch.no_grad():
            pred = model(img)[0]

        # NMS
        pred = non_max_suppression(pred, 0.25, 0.45, classes=[64], agnostic=False)
 
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
