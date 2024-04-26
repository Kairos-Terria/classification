from pymycobot.mycobot import MyCobot
import time
import cv2

gripper = False
camera = True
down = False

mc = MyCobot("/dev/ttyACM0", 115200)

mc.sync_send_angles([0, 0, 0, 0, 0, 0], 20)
mc.sync_send_angles([0, -120, 130, -90, 90, 0], 20)

if down:
    init_coords = mc.get_coords()
    print(init_coords)
    init_coords[2] = 140
    mc.send_coords(init_coords, 20, 1)
    print(init_coords)

if gripper:
    mc.set_gripper_calibration()
    mc.set_gripper_mode(0)
    mc.init_eletric_gripper()
    time.sleep(1)

    for count in range(1):
      mc.set_eletric_gripper(0)
      mc.set_gripper_value(0,20,1)
      time.sleep(2)
      mc.set_eletric_gripper(0)
      mc.set_gripper_value(100,20,1)
      time.sleep(2)

if camera:
    cap = cv2.VideoCapture(2)
    while cap.isOpened():
        ret, frame = cap.read()
        if not ret:
            break

        cv2.namedWindow("window", cv2.WINDOW_NORMAL)
        cv2.resizeWindow("window", width=640, height=480)
        cv2.imshow("window", frame)
        cv2.waitKey(1)

