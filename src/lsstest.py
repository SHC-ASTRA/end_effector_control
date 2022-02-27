import lss
import lss_const as lssc
import time
import gpiozero

enable_pin = gpiozero.OutputDevice(pin=4,active_high=True,initial_value=False)
time.sleep(1)
enable_pin.on()
time.sleep(2)

#print(self.servo_one.getStatus())

CST_LSS_PORT = "/dev/ttyAMA0"
CST_LSS_BAUD = lssc.LSS_DefaultBaud
lss.initBus(CST_LSS_PORT, CST_LSS_BAUD)

servo_one = lss.LSS(1)
servo_two = lss.LSS(2)
servo_three = lss.LSS(3)

servo_one.setColorLED(1)
servo_two.setColorLED(2)
servo_three.setColorLED(3)

servo_two.wheel(10)

time.sleep(2)

nowtime = time.time_ns()
pos1 = servo_one.getPosition()
pos2 = servo_two.getPosition()
pos3 = servo_three.getPosition()
print((time.time_ns()-nowtime)/1e6)
print(pos2)

servo_two.wheel(0)