import math
import numpy as np 
import rospy
from std_msgs.msg import Float32MultiArray

def callback(data):
    rospy.loginfo(rospy.get_name(),data.data)
    angles=np.array([data.data[0], data.data[1], data.data[2], data.data[3]])
    speeds=np.array([data.data[4], data.data[5], data.data[6], data.data[7]])
    dataFlag = True

def listener():
    rospy.init_node('odometry_node',anonymous=True)
    rospy.Subscriber("cmd_vel", Float32MultiArray, callback)

def talker1():
    pub=rospy.Publisher("cmd_vel_prj",Float32MultiArray)
    # rospy.init_node('talker1')
    rate=rospy.Rate(10)
    while not rospy.is_shutdown():
        data=[linearVelocity, angleCenterMass, radius]%rospy.get_time()
        rospy.loginfo(data)
        pub.publish(data)
        rate.sleep()

