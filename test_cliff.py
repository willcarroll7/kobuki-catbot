import rospy, math
from kobuki_msgs.msg import CliffEvent
from geometry_msgs.msg import Twist

robot_state = 0

def callback(data):
    print(data)
    if data.state == 1:
        print("data.state: ", data.state)
        global robot_state
        robot_state = 1

def listener():
    rospy.init_node("listener", anonymous=True)
    rospy.Subscriber("/mobile_base/events/cliff", CliffEvent, callback)

    rate = rospy.rate(10)
    pub = rospy.Publisher('cmd_vel_mux/input/navi', Twist, queue_size=10)
    twist = Twist()
    while not rospy.is_shutdown():
        if robot_state == 1:
            for each in range(40):
                twist.linear.x = 0
                twist.angular.z = math.pi/4.0
                pub.publish(twist)
                rate.sleep()

        rate.sleep()

if __name__ == "__main__":
    listener()