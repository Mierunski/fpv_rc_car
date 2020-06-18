#!/usr/bin/env python2
import rospy
import serial
import time
import threading
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Joy

# Author: Andrew Dai
# This ROS Node converts Joystick inputs from the joy node
# into commands for turtlesim

# Receives joystick messages (subscribed to Joy topic)
# then converts the joysick inputs into Twist commands
# axis 1 aka left stick vertical controls linear speed
# axis 0 aka left stick horizonal controls angular speed
class RCCar():
    def callback(self, data):
        angle = int((data.axes[0]-0.8)*200 + 50)
        # print(data.axes[0])
        motor = int(data.axes[2]*50 + 50)
        if angle < 0:
            angle = 0
        elif angle > 100:
            angle = 100
        if motor < 0:
            motor = 0
        elif motor > 100:
            motor = 100
        if data.axes[3] > -0.8:
            motor += 101
        self.msg = b"M{:03d}{:03d}\r".format(motor, angle)

    def flusher(self):
        while self.running:
            try:
                line = self.ser.readline()
                if line != '':
                    print(line)
            except AttributeError:
                pass
            except serial.serialutil.SerialException:
                break


    # Intializes everything
    def start(self):
        self.running = True
        self.msg = "M000000\r"
        # publishing to "turtle1/cmd_vel" to control turtle1
        self.ser = serial.Serial('/dev/ttyACM0', 115200, timeout=0.1)  # open serial port
        # subscribed to joystick inputs on topic "joy"
        rospy.Subscriber("joy", Joy, self.callback)
        # starts the node
        rospy.init_node('rc_car')

        x = threading.Thread(target=self.flusher)
        x.start()
        time.sleep(1)
        print("creating a publisher")
        self.timer = rospy.Timer(rospy.Duration(0.05), self.demo_callback)
        print("timer called")
        rospy.spin()
        self.running = False
        self.ser.close()

    def demo_callback(self, timer):
        print(self.msg, )
        self.ser.write(self.msg)

if __name__ == '__main__':
    r = RCCar()
    r.start()
