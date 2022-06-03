#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <std_msgs/Float32MultiArray.h>
#include "Canbus.h"
#include <cmath>

#define ERR_ID 3
#define CMD_ID 4
#define FL 1010
#define FR 1020
#define RL 1030
#define RR 1040

static const float pi = 3.1416;
static const float L = 0.53;
static const float C = 0.43;
float V;
float R0;
float gam;
bool cmdFlag = false;


template<typename T>
T sign(T a)
{
    if (a > 0) return 1;
    else if (a < 0) return -1;
    else return 0;
}

void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg)
{
    float Vx, Vy, W;
    float x,y;
    float R[4], targetAngle[4];  
    float Rm,gm;
    
    ROS_INFO("I heard: [Vx=%f, Vy=%f, W=%f]", msg->linear.x, msg->linear.y, msg->angular.z);
    // TODO: recive cmd information and send to CAN
    Vx = msg->linear.x;
    Vy = msg->linear.y;
    W = msg->angular.z;
    if (Vx == 0 & Vy == 0)
    {
        gam = 0;
        if (W != 0)
        {
            R0 = 0;
            V = W * powf(L/2*L/2 + C/2*C/2, 0.5f);
        }
        else
        {
            R0 = 1600;
            V = 0;
        }
    }
    else
    {
        /*if (Vx == 0 & Vy != 0)
        {
            gam = sign<float>(Vy) * pi/2;
        }
        else
        {
            if (Vx != 0 & Vy == 0)
            {
                gam = 0;
            }
            else
            {
                gam = atan2(Vy, Vx);
            }
        }*/
        gam = atan2(Vy, Vx);
        if (gam > pi/2)
        {
            gam = gam - pi;
        }
        if (gam < -pi/2)
        {
            gam = gam + pi;
        }
        V = powf(Vx*Vx + Vy*Vy,0.5f);
        R0 = V/W;

        x=R0*cos(pi/2+gam);
        y=R0*sin(pi/2+gam);
        if (R>0){
            targetAngle[0]=atan2(L/2-x, y-C/2);
            targetAngle[1]=atan2(L/2-x, y+C/2);
            targetAngle[2]=atan2(-L/2-x, y-C/2);
            targetAngle[3]=atan2(-L/2-x, y+C/2);
        }
        else{
            targetAngle[3]=atan2(L/2+x, -y-C/2);
            targetAngle[2]=atan2(L/2+x, -y+C/2);
            targetAngle[1]=atan2(-L/2+x, -y-C/2);
            targetAngle[0]=atan2(-L/2+x, -y+C/2);
        }

        R[0]=fabs(y-C/2)/fabs(cos(targetAngle[0]));
        R[1]=fabs(y+C/2)/fabs(cos(targetAngle[1]));
        R[2]=fabs(y-C/2)/fabs(cos(targetAngle[2]));
        R[3]=fabs(y+C/2)/fabs(cos(targetAngle[3]));

        if(R[0]>1600) R[0]=1600;
        if(R[1]>1600) R[1]=1600;
        if(R[2]>1600) R[2]=1600;
        if(R[3]>1600) R[3]=1600;
        Rm=R[0];
        if (Rm<R[1]) Rm=R[1];
        if (Rm<R[2]) Rm=R[2];
        if (Rm<R[3]) Rm=R[3];

        V = V*Rm/R0;
    }

    
    cmdFlag = true;
}

int main(int argc, char **argv){
    char sendData[8];
    float tmpMsg[8];
    DWORD   ID[4]={FL, FR, RL, RR}; //FR, RL, 
    uint8_t iID=0;
    Canbus can(0,I7565H2,CANBaud_250K,0);
    int16_t tmpV, tmpR, tmpG;
    ros::init(argc, argv, "mpc");
    ros::NodeHandle nh;
    ros::Publisher feedbackPublisher  = nh.advertise<std_msgs::Float32MultiArray>("wheel_info", 1000);
    ros::Subscriber sub = nh.subscribe("cmd_vel", 1000, cmdCallback);
    ros::Rate loop_rate(100);

    if(can.getErrorState()==0)
    {
        ROS_INFO("Can open");
    }
    else
    {
        ROS_ERROR("Can error %d",can.getErrorState());
        return 0;
    }

    _VCI_CAN_MSG data;

    while (ros::ok()){
        //data = VCI_Get_Library_Version();

        if (!can.read(&data))
        {
            //ROS_WARN("NO AVALIBLE MASSAGE FROM CAN");
        }
        else
        {
            // ROS_INFO("MASSAGE RECIVED FROM CAN");
            if (data.ID == ID[iID])
            {
                tmpMsg[4+iID]=(float)(((int16_t)(data.Data[0] + data.Data[1]*256)))/128;
                tmpMsg[iID]=(float)(((int16_t)(data.Data[2] + data.Data[3]*256)))/256;
                // ROS_INFO("CAN Data: %d; %d; %d; %d", data.Data[0], data.Data[1], data.Data[2], data.Data[3]);
                // ROS_INFO("speed:%f; angle: %f", tmpMsg[4+iID], tmpMsg[iID]);
                if (iID == 3) 
                {
                    tmpMsg[1] = -tmpMsg[1];
                    tmpMsg[2] = -tmpMsg[2];
                    std_msgs::Float32MultiArray msg;
                    for (size_t i = 0; i < 8; i++)
                    {
                        msg.data.push_back(tmpMsg[i]);
                        //ROS_INFO("%f",tmpMsg[i]);
                    }
                    iID = 0;
                    feedbackPublisher.publish(msg);
                    can.canFlush();
                    //ROS_INFO("all recived");
                }
                else iID++;
            }
            else
            {
                if (data.ID == ERR_ID)
                {
                    ROS_WARN("wheel %d return error %d", data.Data[1], data.Data[0]);
                }
            }
        }

        if (cmdFlag == true)
        {
            tmpV = (int16_t)(V*2048);
            sendData[0]=tmpV % 256;
            sendData[1]=(tmpV) / 256;
            tmpG = (int16_t)(gam*2048);
            sendData[2]=tmpG % 256;
            sendData[3]=(tmpG) / 256;
            tmpR = (int16_t)(R0*10);
            sendData[4]=tmpR % 256;
            sendData[5]=(tmpR) / 256;
            // sendData[6]=(tmpR) % 256 / 256 / 256;
            // sendData[7]=(tmpR) / 256 /256 /256;
            
            ROS_INFO("Vel: %f, Gam: %f, R: %f",V, gam, R0);
            ROS_INFO("Vel: %d, %d",sendData[0],sendData[1]);
            ROS_INFO("Gam: %d, %d",sendData[2],sendData[3]);
            ROS_INFO("R: %d, %d, %d, %d",sendData[4],sendData[5],sendData[6],sendData[7]);
            can.sendMsg(sendData, 8, CMD_ID);
            cmdFlag = false;
        }
        

        // std_msgs::Float32MultiArray msg;
        // msg.data.push_back(0.7798);
        // msg.data.push_back(0.6728);
        // msg.data.push_back(0.6053); 
        // msg.data.push_back(0.5086);
        // msg.data.push_back(25.1027);
        // msg.data.push_back(28.3222);
        // msg.data.push_back(21.7059);
        // msg.data.push_back(25.3603);
        // feedbackPublisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }
    return 0;
}