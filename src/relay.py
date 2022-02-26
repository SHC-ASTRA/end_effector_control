#!/usr/bin/python

import rospy
import serial
import time
import gpiozero
from arm_relay.msg import JointRateCommand, ActuatorFeedback
from endeffector_controller.srv import EnableServos, EnableServosResponse, Home, HomeResponse
from std_msgs import *
import lss
import lss_const as lssc

class EndEffectorController:
    def __init__(self):
        rospy.loginfo('Setting up node.')
        rospy.init_node('end_effector_control')
        
        self.status_pub = rospy.Publisher('status', msg.String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', ActuatorFeedback, queue_size=1)
        
        self.cmd_rate_sub = rospy.Subscriber("/joint_rate_command", JointRateCommand, self.process_rate_cmd)

        self.enable_service = rospy.Service("/enable_servos", EnableServos, self.enable_servos)
        self.servos_enabled = True
        self.enable_pin = gpiozero.OutputDevice(pin=4,active_high=True,initial_value=False)
        time.sleep(1)
        self.enable_pin.on()
        time.sleep(1)

        #print(self.servo_one.getStatus())

        CST_LSS_PORT = "/dev/ttyAMA0"
        CST_LSS_BAUD = lssc.LSS_DefaultBaud
        lss.initBus(CST_LSS_PORT, CST_LSS_BAUD)

        self.servo_one = lss.LSS(1)
        self.servo_two = lss.LSS(2)
        self.servo_three = lss.LSS(3)

        self.axis5 = 0.0
        self.axis6 = 0.0
        self.axis7 = 0.0

        self.homing_service = rospy.Service("/home_end_effector", Home, self.home_end_effector)
        self.homing_in_progress = False
        self.homing_success = False

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

        # s 2 = yaw + roll
        # s 3 = -yaw + roll
        self.servo_two.wheel(int(50*(self.axis6 + self.axis5)))
        self.servo_three.wheel(int(50*(self.axis6 - self.axis5)))
        self.servo_one.wheel(int(10*self.axis7))

    def home_end_effector(self, home_srv):
        if not self.homing_in_progress and not self.homing_success:
            self.ser.write('h 0\n')
            self.homing_in_progress = True
        
        return HomeArmBaseResponse(self.homing_success)

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
        
    def process_feedback(self, data):        
        actuatorFeedback = ActuatorFeedback()
        
        # parse input data into keys for reference
        args = data.split(',')
        results = {}
        for pair in args:
            name, value = pair.split('=')
            results[name] = value
            
        actuatorFeedback.axis = int(results['x'])
        actuatorFeedback.angle = float(results['a'])
        actuatorFeedback.actualRate = float(results['r'])
        
        self.feedback_pub.publish(actuatorFeedback)

    def publish_feedback(self):
        pass
    
    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            rate.sleep()


if __name__ == '__main__':
    relay = EndEffectorController()
    relay.run()
  