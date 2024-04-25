from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyACM0", 115200)

mc.sync_send_angles([0, 0, 0, 0, 0, 0], 20)
#mc.sync_send_angles([0, -120, 130, -90, 90, 0], 20)

mc.set_gripper_calibration()
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(1)

for count in range(5):
  mc.set_eletric_gripper(0)
  mc.set_gripper_value(0,20,1)
  time.sleep(2)
  mc.set_eletric_gripper(0)
  mc.set_gripper_value(100,20,1)
  time.sleep(2)
