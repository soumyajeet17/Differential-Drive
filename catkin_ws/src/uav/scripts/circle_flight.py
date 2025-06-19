#!/usr/bin/env python3
import rospy
import math
import time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.srv import CommandBool, SetMode
from mavros_msgs.msg import State

current_state = None
def state_cb(msg):
    global current_state
    current_state = msg

def main():
    global current_state

    rospy.init_node("circle_flight_node")

    state_sub = rospy.Subscriber("mavros/state", State, state_cb)
    pose_pub = rospy.Publisher("mavros/setpoint_position/local", PoseStamped, queue_size=10)
    arming_client = rospy.ServiceProxy("mavros/cmd/arming", CommandBool)
    set_mode_client = rospy.ServiceProxy("mavros/set_mode", SetMode)

    rate = rospy.Rate(20)

    # Wait for connection
    while not rospy.is_shutdown() and current_state is None:
        rospy.loginfo_throttle(2, "Waiting for FCU connection...")
        rate.sleep()

    rospy.loginfo("FCU connected.")

    pose = PoseStamped()
    pose.pose.position.z = 2  # takeoff altitude

    # Pre-send setpoints before OFFBOARD
    for _ in range(100):
        pose.header.stamp = rospy.Time.now()
        pose_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Switching to OFFBOARD mode...")
    set_mode_client(base_mode=0, custom_mode="OFFBOARD")
    rospy.loginfo("Arming drone...")
    arming_client(True)

    # Circle parameters
    center_x, center_y = 0.0, 0.0
    radius = 20.0
    height = 2.0
    circle_time = 50  # seconds
    start_time = time.time()

    while not rospy.is_shutdown():
        t = time.time() - start_time
        if t > circle_time:
            break

        angle = (2 * math.pi * t) / circle_time
        x = center_x + radius * math.cos(angle)
        y = center_y + radius * math.sin(angle)

        yaw = math.atan2(center_y - y, center_x - x)

        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = height

        from tf.transformations import quaternion_from_euler
        q = quaternion_from_euler(0, 0, yaw)
        pose.pose.orientation.x = q[0]
        pose.pose.orientation.y = q[1]
        pose.pose.orientation.z = q[2]
        pose.pose.orientation.w = q[3]

        pose.header.stamp = rospy.Time.now()
        pose_pub.publish(pose)
        rate.sleep()

    # Hover for a moment
    rospy.loginfo("Circle complete. Hovering...")
    for _ in range(100):
        pose.header.stamp = rospy.Time.now()
        pose_pub.publish(pose)
        rate.sleep()

    rospy.loginfo("Landing...")
    set_mode_client(base_mode=0, custom_mode="AUTO.LAND")

if __name__ == "__main__":
    main()

