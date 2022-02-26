#!/usr/bin/python

import rospy
import serial
import time
from arm_relay.msg import JointRateCommand, ActuatorFeedback
from arm_relay.srv import HomeArmBase, HomeArmBaseResponse
from std_msgs import *

class ArmRelay:
    def __init__(self):
        rospy.loginfo('Setting up node.')
        rospy.init_node('arm_base_relay')
        
        # Initialize serial port and connect to teensy microcontroller
        self.ser = None
        while (self.ser is None):
            port = rospy.get_param('~arm_teensy_serial_port', '/dev/ttyACM0')
            try:
                self.ser = serial.Serial(port,115200)
            except:
                rospy.loginfo("Unable to locate Teensy (@{0}), please ensure communications.".format(port))
                time.sleep(3)
        rospy.loginfo('Connected to teensy.')
        
        self.status_pub = rospy.Publisher('status', msg.String, queue_size=10)
        self.feedback_pub = rospy.Publisher('feedback', ActuatorFeedback, queue_size=1)
        
        self.cmd_rate_sub = rospy.Subscriber("/joint_rate_command", JointRateCommand, self.process_rate_cmd)
        
        self.homing_service = rospy.Service("/home_arm_base", HomeArmBase, self.home_arm_base)
        self.homing_in_progress = False
        self.homing_success = False

        self.topic_publisher_callback = {
            'status' : self.process_status,
            'feedback' : self.process_feedback,
            'response': self.process_status,
            'error': self.process_status,
            'homing_status' : self.process_homing_status
        }


    def home_arm_base(self, home_srv):
        if not self.homing_in_progress and not self.homing_success:
            self.ser.write('h 0\n')
            self.homing_in_progress = True
        
        return HomeArmBaseResponse(self.homing_success)

    def process_homing_status(self, status):
        self.homing_in_progress = False
        
        if status == "false":
            self.homing_success = False
        elif status == "true":
            self.homing_success = True

    def process_rate_cmd(self, rate_cmd):
        self.ser.write(str(rate_cmd.axis) + ',' + str(rate_cmd.desiredRate) + "\n")
        
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
        
    def process_message(self, message):
        args = message.split(";")
        topic = args[0]
        data = args[1]
        try:
            self.topic_publisher_callback[topic](data)
        except Exception as e:
            print(e)
    
    def run(self):
        rate = rospy.Rate(60)
        while not rospy.is_shutdown():
            while self.ser.in_waiting > 0:
                message = self.ser.read_until('\n')
                print(message)
                self.process_message(message)
            rate.sleep()


if __name__ == '__main__':
    relay = ArmRelay()
    relay.run()
  