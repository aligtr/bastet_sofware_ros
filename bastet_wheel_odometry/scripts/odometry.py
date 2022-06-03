import math
import numpy as np 
import rospy 
from std_msgs.msg import Float32MultiArray
from nav_msgs.msg import Odometry 
from tf.transformations import *

baseWeight=0.43
baseLenght=0.53
dataFlag = False
xStart=0
yStart=0
angleStart=0
dt=-0.1*6.3/2.5*4/3*1.3


posithionLenght=np.array([baseLenght/2, baseLenght/2, -baseLenght/2, -baseLenght/2])
positionWeight=np.array([baseWeight/2, -baseWeight/2, baseWeight/2, -baseWeight/2])





def odometry(angularVelocity, linearVelocity, angleCenterMass, xStart, yStart, angleStart, dt):
    Vxr=linearVelocity*math.cos(angleCenterMass)
    Vyr=linearVelocity*math.sin(angleCenterMass)
    xEnd=xStart+dt*(Vxr*math.cos(angleStart)-Vyr*math.sin(angleStart))
    yEnd=yStart+dt*(Vxr*math.sin(angleStart)+Vyr*math.cos(angleStart))
    angleEnd=angleStart+dt*angularVelocity
    # xStart=xEnd
    # yStart=yEnd
    # angleStart=angleEnd
    writeOdometry=open('/home/user/workspace/src/bastet_wheel_odometry/scripts/Odometry.txt', 'a')
    try:
        # print(f'write to file')
        writeOdometry.write(f'\n{xEnd}; {yEnd}; {angleEnd}')
    finally:
        # print(f'close file')
        writeOdometry.close()
    return [xEnd, yEnd, angleEnd, Vxr, Vyr]


def abFilter(values, xk_1, yk_1):
    alpha=0.355; beta=0.0000005; dt=0.5
    xk=xk_1+(np.array(yk_1)*dt)
    yk=yk_1
    rk=values-xk
    xk=xk+alpha*np.array(rk)
    yk=yk+np.array((beta*np.array(rk)))/dt
    xk_1=xk
    yk_1=yk
    filteredValues=xk_1
    return [filteredValues, xk_1, yk_1]


def ssFilter(sample,k):
    filteredValues=(np.sum(sample, axis=0))/k
    return filteredValues

class Listen:
    dataFlag = False
    angles=[]
    speeds=[]
    def callback(self, data):
        # rospy.loginfo(rospy.get_name(),data.data[1])
        self.angles=np.array([data.data[0], data.data[1], data.data[2], data.data[3]])
        self.speeds=np.array([data.data[4], data.data[5], data.data[6], data.data[7]])/60*3.14*0.254
        self.dataFlag = True
        # print('hallo')

    def listener(self):
        rospy.Subscriber("wheel_info", Float32MultiArray, self.callback)

if __name__ == '__main__':
    try:
        open('/home/user/workspace/src/bastet_wheel_odometry/scripts/Odometry.txt','w').close()
        writeOdometry=open('/home/user/workspace/src/bastet_wheel_odometry/scripts/Odometry.txt', 'a')
        writeOdometry.write(f'0; 0; 0')
        writeOdometry.close()
        sampleAngles=[]
        sampleSpeeds=[]
        xk_1_angles=np.zeros((4))
        yk_1_angles=np.zeros((4))
        xk_1_speeds=np.zeros((4))
        yk_1_speeds=np.zeros((4))
        k=0
        rospy.init_node('odometry_node',anonymous=True)
        listen = Listen()
        listen.listener()
        pub=rospy.Publisher("odometry",Odometry, queue_size=10)
        rate=rospy.Rate(10)
        while not rospy.is_shutdown():
            if (listen.dataFlag == True):
                # print(f'{listen.angles, listen.speeds}')
                k=k+1
                sampleAngles.append(listen.angles)
                sampleSpeeds.append(listen.speeds)
                xRotationAverage=0
                yRotationAverage=0
                angularVelocity1=0

                # if (k>5):
                #     sampleAngles.pop(0)
                #     sampleSpeeds.pop(0)
                #     angles=ssFilter(sampleAngles,5)
                #     speeds=ssFilter(sampleSpeeds,5)
                # else:
                #     angles=ssFilter(sampleAngles,k)
                #     speeds=ssFilter(sampleSpeeds,k)

                # [listen.angles, xk_1_angles, yk_1_angles]=abFilter(listen.angles, xk_1_angles, yk_1_angles)
                # [listen.speeds, xk_1_speeds, yk_1_speeds]=abFilter(listen.speeds, xk_1_speeds, yk_1_speeds)

                for i in range(0,listen.angles.size):
                    if (listen.angles[i]==0):
                        listen.angles[i]=0.00001

                for i in range(0, listen.angles.size-1):
                    for j in range(i+1, listen.angles.size):
                        if ((i==0) and (j==3)) or ((i==1) and (j==2)):
                            continue
                        else:
                            b1=positionWeight[i]-math.tan(listen.angles[i]+math.pi/2)*posithionLenght[i]
                            b2=positionWeight[j]-math.tan(listen.angles[j]+math.pi/2)*posithionLenght[j]
                            if (math.tan(listen.angles[i]+math.pi/2)-math.tan(listen.angles[j]+math.pi/2)==0):
                                xCenterRotation=99999999
                            else:
                                xCenterRotation=(b2-b1)/(math.tan(listen.angles[i]+math.pi/2)-math.tan(listen.angles[j]+math.pi/2)) #x=(b2-b1)/(k1-k2)
                            xRotationAverage=xRotationAverage+xCenterRotation
                            yCenterRotation=math.tan(listen.angles[i]+math.pi/2)*xCenterRotation+b1 #y=k1x+b1
                            yRotationAverage=yRotationAverage+yCenterRotation
                xRotationAverage=xRotationAverage/4
                yRotationAverage=yRotationAverage/4

                for i in range(0,listen.speeds.size):
                    radiusWheels=math.sqrt((baseLenght/2-xRotationAverage)**2+(baseWeight/2-yRotationAverage)**2)
                    angularVelocity=listen.speeds[i]/radiusWheels
                    angularVelocity1=angularVelocity1+angularVelocity
                angularVelocity=angularVelocity1/4

                radius=math.sqrt(xRotationAverage**2+yRotationAverage**2)

                if radius>=9999999:
                    angleCenterMass=listen.angles[0]
                    linearVelocity=listen.speeds[0]
                    angularVelocity=0
                elif radius<1e-6:
                    xRotationAverage=0
                    yRotationAverage=0
                    angleCenterMass=0
                    radius=0
                    linearVelocity=0
                    # radiusFL=math.sqrt((baseLenght/2-xRotationAverage)**2+(baseWeight/2-yRotationAverage)**2)
                    # angularVelocity=listen.speeds[0]/radiusFL
                else:
                    if (xRotationAverage > 0) and (yRotationAverage > 0):
                        angleCenterMass=-math.atan2(xRotationAverage, yRotationAverage)
                    elif (xRotationAverage < 0) and (yRotationAverage > 0):
                        angleCenterMass=math.atan2(-xRotationAverage, yRotationAverage)
                    elif (xRotationAverage < 0) and (yRotationAverage < 0):
                        angleCenterMass=-math.atan2(-xRotationAverage, -yRotationAverage)
                        radius=-radius
                    elif (xRotationAverage > 0) and (yRotationAverage < 0):
                        angleCenterMass=math.atan2(xRotationAverage, -yRotationAverage)
                        radius=-radius
                    # radiusFL=math.sqrt((baseLenght/2-xRotationAverage)**2+(baseWeight/2-yRotationAverage)**2)
                    # angularVelocity=-listen.speeds[0]/radiusFL
                    linearVelocity=angularVelocity*radius
                # print(f'\n{angularVelocity}; {linearVelocity}; {angleCenterMass}')
                [xEnd, yEnd, angleEnd, Vxr, Vyr]=odometry(angularVelocity, linearVelocity, angleCenterMass, xStart, yStart, angleStart, dt)
                
                xStart=xEnd
                yStart=yEnd
                angleStart=angleEnd

                now=rospy.Time.now()
                odom=Odometry()
                odom.header.frame_id='odom'
                odom.header.stamp=now
                odom.child_frame_id='base_link'
                odom.pose.pose.position.x=xEnd
                odom.pose.pose.position.y=yEnd
                odom.pose.pose.position.z=0
                q=quaternion_from_euler(.0,.0,angleEnd)
                odom.pose.pose.orientation.x=q[0]
                odom.pose.pose.orientation.y=q[1]
                odom.pose.pose.orientation.z=q[2]
                odom.pose.pose.orientation.w=q[3]
                odom.pose.covariance=np.diag([1e-2, 1e-2, 1e-2, 1e3, 1e3, 1e-1]).ravel()
                odom.twist.twist.linear.x=Vxr
                odom.twist.twist.linear.y=Vyr
                odom.twist.twist.angular.z=angularVelocity
                odom.twist.covariance=np.diag([1e-2, 1e3, 1e3, 1e3, 1e3, 1e-2]).ravel()
                pub.publish(odom)

                listen.dataFlag  = False
            rate.sleep()
    except rospy.ROSInterruptException:
        pass

    