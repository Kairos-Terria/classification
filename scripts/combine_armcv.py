import cv2
import numpy as np
import time
from pymycobot.mycobot import MyCobot

# 색상에 따른 HSV 범위 설정
colors_hsv = {
    'red': [np.array([0, 120, 120]), np.array([10, 255, 255]), np.array([170, 120, 120]), np.array([180, 255, 255])],
    'yellow': [np.array([22, 130, 140]), np.array([38, 255, 255])],
    'blue': [np.array([100, 100, 120]), np.array([130, 255, 255])],
    'purple': [np.array([125, 50, 50]), np.array([150, 255, 255])]
}

# 최소 바운딩 박스 크기 설정
min_width = 3
min_height = 3

def process_video(mc):
    width, height = 640, 480
    mc.sync_send_coords([239, -60, 331, -176, 0, -83], 20, 1)
    time.sleep(2)
    init_coords = list(mc.get_coords())
    print(init_coords)
    
    mc.set_gripper_calibration()
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()

    cap = cv2.VideoCapture(2)
    if not cap.isOpened():
        print("Camera open failed!")
        return

    while True:
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame.")
            break
        
        hsv_image = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        all_contours = []
        largest_area = 0
        block = [0, 0, '']

        # 색상별 마스크 적용 및 윤곽선 추출
        for color, ranges in colors_hsv.items():
            if color == 'red':
                mask1 = cv2.inRange(hsv_image, ranges[0], ranges[1])
                mask2 = cv2.inRange(hsv_image, ranges[2], ranges[3])
                mask = cv2.bitwise_or(mask1, mask2)
            else:
                mask = cv2.inRange(hsv_image, ranges[0], ranges[1])

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            for contour in contours:
                area = cv2.contourArea(contour)
                if area > largest_area:
                    largest_area = area
                    largest_contour = contour
                    block[0:3] = [cv2.boundingRect(largest_contour)[0] + cv2.boundingRect(largest_contour)[2] // 2,
                                  cv2.boundingRect(largest_contour)[1] + cv2.boundingRect(largest_contour)[3],
                                  color]

        # 로봇 조작 로직
        if block[2]:
            x_center, y_center = block[0], block[1]
            print('Block detected:', block[0:2])

            if x_center < width * 0.48:
                init_coords[1] += 1  # 왼쪽 이동
            elif x_center > width * 0.52:
                init_coords[1] -= 1  # 오른쪽 이동
            elif width * 0.48 <= x_center <= width * 0.52:
                if y_center < height * 0.90:
                    init_coords[0] += 1  # 왼쪽 이동
                elif y_center > height * 0.95:
                    init_coords[0] -= 1  # 오른쪽 이동

            mc.send_coord(1, init_coords[0], 20)
            mc.send_coord(2, init_coords[1], 20)
            #mc.send_coords(tuple(init_coords), 20, 1)
            #print(mc.get_coords())
            time.sleep(0.1)

            if width * 0.48 <= x_center <= width * 0.52 and height * 0.90 <= y_center <= height * 0.95:
                init_coords[0] = 5
                mc.send_coord(1, init_coords[0], 20)
            
                init_coords[2] = 300
                mc.send_coord(3, init_coords[2], 20)
                time.sleep(2)
                init_coords[3] = -175
                mc.send_coord(4, init_coords[3], 20)

                mc.set_eletric_gripper(1)
                mc.set_gripper_value(0, 20, 1)
                time.sleep(2)
                if block[2] == "red" or block[2] == "blue":
                    mc.sync_send_coords([207, -234, 304, -164, -8, -123], 20, 1)  # Assuming this resets the position
                    mc.set_eletric_gripper(0)
                    mc.set_gripper_value(100, 20, 1)
                else:
                    mc.sync_send_coords([251, 197, 243, 177, -5, -35], 20, 1)  # Assuming this resets the position
                    mc.set_eletric_gripper(0)
                    mc.set_gripper_value(100, 20, 1)

                mc.sync_send_coords([239, -60, 331, -176, 0, -83], 20, 1)
 
                print("Task completed with:", block[2])
                break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    mc = MyCobot("/dev/ttyACM0", 115200)
    while True:
        process_video(mc)
