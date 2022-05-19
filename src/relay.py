#!/usr/bin/python

import rospy
import serial
import time
import gpiozero
from arm_relay.msg import JointRateCommand, ActuatorFeedback
from endeffector_controller.msg import EEStatus
from endeffector_controller.srv import EnableServos, EnableServosResponse, Home, HomeResponse, EnableLaser, EnableLaserResponse, ZeroAxis, ZeroAxisResponse
from std_msgs import *
import lss
import lss_const as lssc

class EndEffectorController:
    def __init__(self):
        rospy.loginfo('Setting up node.')
        rospy.init_node('end_effector_control')

        self.yaw_pos = 0
        self.roll_pos = 0
        self.grip_pos = 0

        self.yaw_zero = 0
        self.roll_zero = 0
        self.grip_zero = 0

        self.enable_service = rospy.Service("/enable_servos", EnableServos, self.enable_servos)
        self.enable_pin = gpiozero.OutputDevice(pin=4,active_high=True,initial_value=False)
        time.sleep(1)
        self.enable_pin.on()
        time.sleep(2)

        #print(self.servo_one.getStatus())

        CST_LSS_PORT = "/dev/ttyAMA0"
        CST_LSS_BAUD = lssc.LSS_DefaultBaud
        lss.initBus(CST_LSS_PORT, CST_LSS_BAUD)

        self.servo_one = lss.LSS(1)
        self.servo_two = lss.LSS(2)
        self.servo_three = lss.LSS(3)

        self.servos_enabled = True

        self.servo_one.setColorLED(1)
        self.servo_two.setColorLED(2)
        self.servo_three.setColorLED(3)

        self.servo_one.setMaxSpeed(100)
        self.servo_two.setMaxSpeed(100)
        self.servo_three.setMaxSpeed(100)

        self.pos2off = float(self.servo_two.getPosition())/10.0
        self.pos3off = float(self.servo_three.getPosition())/10.0

        self.axis5 = 0.0
        self.axis6 = 0.0
        self.axis7 = 0.0

        self.status_pub = rospy.Publisher('status', msg.String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', ActuatorFeedback, queue_size=1)
        
        self.cmd_rate_sub = rospy.Subscriber("/joint_rate_command", JointRateCommand, self.process_rate_cmd)

        self.homing_service = rospy.Service("home_end_effector", Home, self.home_end_effector)
        self.homing_in_progress = False
        self.homing_success = False

        self.zero_service = rospy.Service("/zero_actuators", ZeroAxis, self.zero_axis)

    def process_rate_cmd(self, rate_cmd):
        #self.ser.write(str(rate_cmd.axis) + ',' + str(rate_cmd.desiredRate) + "\n")        
        if rate_cmd.axis == 6:   # roll
            self.axis6 = rate_cmd.desiredRate
        elif rate_cmd.axis == 5: # yaw
            self.axis5 = rate_cmd.desiredRate
        elif rate_cmd.axis == 7: # gripper
            self.axis7 = rate_cmd.desiredRate
        else:
            return

        if (self.yaw_pos >= 45 and self.axis5 > 0) or (self.yaw_pos <= -45 and self.axis5 < 0):
            self.axis5 = 0

        # s 2 = yaw - roll
        # s 3 = -yaw - roll
        self.servo_two.wheel(int(1*(-self.axis6*3 + self.axis5)))
        self.servo_three.wheel(int(1*(-self.axis6*3 - self.axis5)))
        self.servo_one.wheel(int(10*self.axis7))

    def zero_axis(self, zero_srv):
        if zero_srv.axis == 5:
            self.yaw_zero = self.yaw_pos + self.yaw_zero
            return ZeroAxisResponse(True)
        elif zero_srv.axis == 6:
            self.roll_zero = self.roll_pos + self.roll_zero
            return ZeroAxisResponse(True)
        elif zero_srv.axis == 7:
            self.grip_zero = self.grip_pos + self.grip_zero
            return ZeroAxisResponse(True)
        else:
            return ZeroAxisResponse(False)

    def home_end_effector(self, home_srv):
        rospy.loginfo('Homing not implemented')
        return HomeResponse(False)

    def process_homing_status(self, status):
        self.homing_in_progress = False
        
        if "false" in status:
            self.homing_success = False
        elif "true" in status:
            self.homing_success = True

    def enable_servos(self,enable_srv):        
        if self.servos_enabled and not enable_srv.enable:
            self.enable_pin.off()
            rospy.loginfo('Disabling Servos')
        elif not self.servos_enabled and enable_srv.enable:
            self.enable_pin.on()
            rospy.loginfo('Enabling Servos')

        self.servos_enabled = self.servo_one.getStatus() == '1'

        return EnableServosResponse(self.servos_enabled)
        
    def process_status(self, data):
        self.status_pub.publish(str(data))
        
    def process_feedback(self, axis, angle, actualrate):        
        actuatorFeedback = ActuatorFeedback()
        
        actuatorFeedback.axis = int(axis)
        actuatorFeedback.angle = float(angle)
        actuatorFeedback.actualRate = float(actualrate)
        
        self.feedback_pub.publish(actuatorFeedback)

    def publish_feedback(self):
        try:
            pos2 = float(self.servo_two.getPosition())/10.0 - self.pos2off
            pos3 = float(self.servo_three.getPosition())/10.0 - self.pos3off

            self.yaw_pos = (pos2/2.0 - pos3/2.0) - self.yaw_zero
            self.roll_pos = (-pos2/2.0 - pos3/2.0) / 3.0 - self.roll_zero
            self.grip_pos = float(self.servo_three.getPosition())/10.0 - self.grip_zero

            #print(pos2,pos3,self.yaw_pos,self.roll_pos)

            self.process_feedback(5,self.yaw_pos,0)
            self.process_feedback(6,self.roll_pos,0)
            self.process_feedback(7,self.grip_pos,0)
            
        except:
            rospy.loginfo("Servo failed to return feedback")
        
    
    def run(self):
        rate = rospy.Rate(30)
        while not rospy.is_shutdown():
            self.publish_feedback()
            rate.sleep()


if __name__ == '__main__':
    relay = EndEffectorController()
    relay.run()
  