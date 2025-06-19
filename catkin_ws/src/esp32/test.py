#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float32

data_arduino1 = 1.0
data_arduino2 = 6.0

def callback_arduino1(msg):
    global data_arduino1
    data_arduino1 = msg.data
    rospy.loginfo("Received from Arduino 1: %f", data_arduino1)

# Callback for Arduino 2
def callback_arduino2(msg):
    global data_arduino2
    data_arduino2 = msg.data
    rospy.loginfo("Received from Arduino 2: %f", data_arduino2)

def publisher():
    rospy.init_node('arduino_comm', anonymous=True)

    pub1 = rospy.Publisher('n1', Float32, queue_size=10)
    pub2 = rospy.Publisher('n2', Float32, queue_size=10)

    rospy.Subscriber('n1', Float32, callback_arduino1)
    rospy.Subscriber('n2', Float32, callback_arduino2)

    rate = rospy.Rate(1)  
    while not rospy.is_shutdown():
        #rospy.loginfo("Publishing data to Arduino 1: %f", data_arduino1)
        #rospy.loginfo("Publishing data to Arduino 2: %f", data_arduino2)

        pub1.publish(data_arduino1)
        pub2.publish(data_arduino2)

        rate.sleep()

if __name__ == '__main__':
    try:
        publisher()
    except rospy.ROSInterruptException:
        pass

