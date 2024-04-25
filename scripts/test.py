from pymycobot.mycobot import MyCobot
import time

mc = MyCobot("/dev/ttyACM0", 115200)

mc.sync_send_angles([0, 0, 0, 0, 0, 0], 20)
#mc.sync_send_angles([0, -120, 130, -90, 90, 0], 20)
mc.set_gripper_mode(0)
mc.init_eletric_gripper()
time.sleep(1)

mc.set_gripper_calibration()
#mc.init_eletric_gripper()

mc.set_gripper_value(20, 20, 1)
print(mc.get_gripper_value())
time.sleep(5)

mc.set_gripper_value(100, 20, 1)
print(mc.get_gripper_value())
