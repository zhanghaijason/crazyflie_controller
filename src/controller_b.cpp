#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ios>
#include <fstream>
#include <iostream>
#include "pid.hpp"
double get(
    const ros::NodeHandle& n,
    const std::string& name) {
    double value;
    n.getParam(name, value);
    return value;
}

class Controller
{
public:
    Controller(                             //constructor  with member initilization
        const std::string& worldFrame,
        const std::string& frame,
        const ros::NodeHandle& n)
        : m_worldFrame(worldFrame)            // in constructor, use m_ to identify data member names
        , m_frame(frame) 
        , m_pubNav()
        , m_listener()
        , m_pidX(
            get(n, "PIDs/X/kp"),
            get(n, "PIDs/X/kd"),
            get(n, "PIDs/X/ki"),
            get(n, "PIDs/X/minOutput"),
            get(n, "PIDs/X/maxOutput"),
            get(n, "PIDs/X/integratorMin"),
            get(n, "PIDs/X/integratorMax"),
            "x")
        , m_pidY(
            get(n, "PIDs/Y/kp"),
            get(n, "PIDs/Y/kd"),
            get(n, "PIDs/Y/ki"),
            get(n, "PIDs/Y/minOutput"),
            get(n, "PIDs/Y/maxOutput"),
            get(n, "PIDs/Y/integratorMin"),
            get(n, "PIDs/Y/integratorMax"),
            "y")
        , m_pidZ(
            get(n, "PIDs/Z/kp"),
            get(n, "PIDs/Z/kd"),
            get(n, "PIDs/Z/ki"),
            get(n, "PIDs/Z/minOutput"),
            get(n, "PIDs/Z/maxOutput"),
            get(n, "PIDs/Z/integratorMin"),
            get(n, "PIDs/Z/integratorMax"),
            "z")
        , m_pidYaw(
            get(n, "PIDs/Yaw/kp"),
            get(n, "PIDs/Yaw/kd"),
            get(n, "PIDs/Yaw/ki"),
            get(n, "PIDs/Yaw/minOutput"),
            get(n, "PIDs/Yaw/maxOutput"),
            get(n, "PIDs/Yaw/integratorMin"),
            get(n, "PIDs/Yaw/integratorMax"),
            "yaw")
        , m_state(Idle)
        , m_goal()
        , m_subscribeGoal()
        , m_serviceTakeoff()
        , m_serviceLand()
        , m_thrust(0)
        , m_startZ(0)                                                   // start altitude of drone
    {
        ros::NodeHandle nh;       // initilize a nodehandle nh
        m_listener.waitForTransform(m_worldFrame, m_frame, ros::Time(0), ros::Duration(10.0)); 
        m_pubNav = nh.advertise<geometry_msgs::Twist>("cmd_vel", 1);    //The second parameter to advertise() is the size of the message queue used for publishing messages.
		                                                                //m_pubNav corresponds to chatter_pub in ''ros::Publisher chatter_pub''
																		//Tell the master that we are going to be publishing a message of type std_msgs/Twist on the topic 'cmd_vel'. 																//This lets the master tell any nodes listening on chatter that we are going to publish data on that topic. 
        m_subscribeGoal = nh.subscribe("goal", 1, &Controller::goalChanged, this);  //subscribe to a topic "goal", and the callback function is Controller::goalChanged, goalchanged function under class controller 
		                                                                            // publish.py advertise to topic "goal" and once "goal" changed, will get the new goal informaiton
        m_serviceTakeoff = nh.advertiseService("takeoff", &Controller::takeoff, this); // service is 1 to 1 and bidirectional 
        m_serviceLand = nh.advertiseService("land", &Controller::land, this);   // 'this'   if the subscriber is inside the class, can use this for &class_name, it is the class method to define a subscribe
    }

    void run(double frequency)
    {
        ros::NodeHandle node;
        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);          // Timer is the callback function, maybe we can use concurrent
        ros::spin();
    }
private:
    void goalChanged(
        const geometry_msgs::PoseStamped::ConstPtr& msg)
    {
        m_goal = *msg;
    }

    bool takeoff(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Takeoff requested!");
        m_state = TakingOff;

        tf::StampedTransform transform;
        m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);   //loopupTransform is used for finding the transform matrix between two frames. The first input is target, the second input is source frame
        m_startZ = transform.getOrigin().z();                                         //drone's starting position

        return true;
    }

    bool land(
        std_srvs::Empty::Request& req,
        std_srvs::Empty::Response& res)
    {
        ROS_INFO("Landing requested!");
        m_state = Landing;

        return true;
    }

    void getTransform(
        const std::string& sourceFrame,
        const std::string& targetFrame,
        tf::StampedTransform& result)
    {
        m_listener.lookupTransform(sourceFrame, targetFrame, ros::Time(0), result);
    }

    void pidReset()
    {
        m_pidX.reset();
        m_pidY.reset();
        m_pidZ.reset();
        m_pidYaw.reset();
    }

    void iteration(const ros::TimerEvent& e)
    {
        float dt = e.current_real.toSec() - e.last_real.toSec();

        switch(m_state)
        {
        case TakingOff:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() > m_startZ + 0.05 || m_thrust > 50000)
                {
                    pidReset();
                    //m_pidZ.setIntegral((m_thrust-10000) / m_pidZ.ki());
                   // ROS_INFO("current thrust: %f",m_thrust);
                    m_state = Automatic;
                    initial_g=m_thrust-4000;
                    m_thrust = 0;
                    takeoff_time =ros::Time::now().toSec();
                    ROS_INFO("take_off time: %f",takeoff_time);
                }
                else
                {
                    m_thrust += 15000 * dt;
                    geometry_msgs::Twist msg;
                    msg.linear.z = m_thrust;
                    m_pubNav.publish(msg);
                }

            }
            break;
        case Landing:
            {
                m_goal.pose.position.z = m_startZ + 0.05;
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);
                if (transform.getOrigin().z() <= m_startZ + 0.05) {
                    m_state = Idle;
                    geometry_msgs::Twist msg;
                    m_pubNav.publish(msg);
                }
            }
            // intentional fall-thru
        case Automatic:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);  //it seems after this, the transform is filled, transform goal into body frame
		tfScalar roll, pitch, yaw;
		tf::Matrix3x3(transform.getRotation()).getRPY( roll, pitch,yaw);
		double x = transform.getOrigin().getX();
                float target_x=3;
                float target_y=0;
                float target_z=0;
                double y = transform.getOrigin().getY();
                double z = transform.getOrigin().getZ();
                int aaaa;
                geometry_msgs::Twist msg;    
                float u_x = m_pidX.update(x,target_x);       //second-first=error   
                float u_y = m_pidY.update(y,target_y);
                float u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
//                u_x=0;u_y=0;
		float m=0.024;               // weight of CF
		float g=initial_g;
		float target_psi=0;
		float target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
		float target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
		float a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
		float a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
		float a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
		float u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                float phi1=target_phi*180/3.14159;
                float theta1=target_theta*180/3.14159;
//                target_theta=10;
//                target_phi=-10;
		ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
		msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput);   // feedback  and  reference
                msg.linear.x =  std::max(std::min(theta1 , x_maxOutput), x_minOutput); // invert it, in server.cpp, it is reversed 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     //current is the boday frame, so the current pose is [0,0,0,0]
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
                last_x=x;
                last_y=y;
                last_z=z;
                ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
               // ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f Radio output phi theta thrust: (%f,%f,%f)", x,y,z,u_x,u_y,u_z,phi1,theta1,u,msg.linear.y, msg.linear.x,msg.linear.z);
                m_pubNav.publish(msg);
               // prepare_time=current_time;
                current_time1=ros::Time::now().toSec();
               // ROS_INFO("current time:%f",current_time);
                if (sqrt(pow(target_x-x,2)+pow(target_y-y,2)+pow(target_z-z,2))<0.1 && current_time1-takeoff_time>10){
                  m_state = Original_position;
                  ROS_INFO("Prepare time: %f",current_time1);
                  prepare_time=current_time1;
                 // ROS_INFO("Prepare time: %f",prepare_time);
                }
            }
            break;
        case Original_position:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);  //it seems after this, the transform is filled, transform goal into body frame
                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY( roll, pitch,yaw);
                ROS_INFO("original stage intiated");
               // current_time=ros::Time::now().toSec();
               // delta_t=current_time-trajectory_time;
                double x = transform.getOrigin().getX();
                float target_x=1.2;//last_x-0.002;
//                if (abs(target_x-1.1)<=0.01){
//                  target_x=1.1;
//                }
                float target_y=0;
                float target_z=0;
                double y = transform.getOrigin().getY();
                double z = transform.getOrigin().getZ();
                current_time2=ros::Time::now().toSec();
                v_x=(x-last_x)/(current_time2-last_time);

                geometry_msgs::Twist msg; 
                float u_x = m_pidX.update(x,target_x);       //second-first=error   
                float u_y = m_pidY.update(y,target_y);
                float u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
//                u_x=0;u_y=0;
                float m=0.024;               // weight of CF
                float g=initial_g;
                float target_psi=0;
                float target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                float target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                float a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                float a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                float a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                float u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                float phi1=target_phi*180/3.14159;
                float theta1=target_theta*180/3.14159;
//                target_theta=10;
//                target_phi=-10;
                ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput);   // feedback  and  reference
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; // invert it, in server.cpp, it is reversed 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     //current is the boday frame, so the current pose is [0,0,0,0]
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
              //   ROS_INFO("current time, v_x, target_x: %f,  %f, %f",current_time,v_x,target_x);
                last_x=x;
                last_y=y;
                last_z=z;
//                constant_speed_time=current_time;
                last_time=current_time2;
                m_pubNav.publish(msg);
                ROS_INFO("current time: %f, Prepare time : %f, v_x: %f",current_time2, prepare_time,v_x);
                if (sqrt(pow(target_x-x,2)+pow(target_y-y,2)+pow(target_z-z,2))<0.25 && current_time2-prepare_time>8){
                  m_state = Initialize;
//                  last_time=current_time;
                  ROS_INFO("Finish time: %f",current_time2);
                  constant_speed_time=current_time2;
                  ROS_INFO("Finish time: %f",constant_speed_time);
             }

            }
            break;
        case Initialize:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);  //it seems after this, the transform is filled, transform goal into body frame
                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY( roll, pitch,yaw);
                ROS_INFO("Acceleration stage intiated");
                //current_time3=ros::Time::now().toSec();
                //delta_t=current_time3-trajectory_time;
                double x = transform.getOrigin().getX();
                float target_x=4;
                float target_y=0;
                float target_z=0;
                double y = transform.getOrigin().getY();
                double z = transform.getOrigin().getZ();
                current_time3=ros::Time::now().toSec();
                v_x=(x-last_x)/(current_time3-last_time);

                geometry_msgs::Twist msg; 
                float u_x = m_pidX.update(x,target_x);       //second-first=error   
                float u_y = m_pidY.update(y,target_y);
                float u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
                float m=0.024;               // weight of CF
                float g=initial_g;
                float target_psi=0;
                float target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                float target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                float a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                float a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                float a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                float u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                float phi1=target_phi*180/3.14159;
                float theta1=target_theta*180/3.14159;
                //ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput); 
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
                 ROS_INFO("current time,position, v_x, target_x: %f,(%f,%f,%f)  %f, %f",current_time3,x,y,z,v_x,target_x);
                last_x=x;
                last_y=y;
                last_z=z;
                last_time=current_time3;
                m_pubNav.publish(msg);
                std::ofstream myfile1;
                myfile1.open("/home/haijie/Desktop/time.txt", std::ios_base::app | std::ios_base::out);
                myfile1 << std::fixed << current_time3 <<"\n";
                myfile1.close();
                std::ofstream myfile2;
                myfile2.open("/home/haijie/Desktop/position.txt", std::ios_base::app | std::ios_base::out);
                myfile2 << x << " "<< y << " " << z << " " << "\n";
                myfile2.close();
                 if (sqrt(pow(x-2,2))<=0.1&&sqrt(pow(v_x-1.5,2))<=0.15){
                  m_state = Tau;
//                  last_time=current_time;
//                  t0=current_time;
                  ROS_INFO("Goal state achieved!, t0: %f",current_time3);
                  t0=current_time3;
                  ROS_INFO("Goal state achieved!, t0: %f",t0);
             }

                if (x>=3.2){
                  m_state=Idle;
                  ROS_INFO("Out of boundary!");
               }
            }
            break;
        case Tau:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);  //it seems after this, the transform is filled, transform goal into body frame
                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY( roll, pitch,yaw);
                ROS_INFO("Tau control stage intiated");
                //current_time4=ros::Time::now().toSec();
                //delta_t=current_time-trajectory_time;
                double x = transform.getOrigin().getX();
                float target_x=3;
                float target_y=0;
                float target_z=0;
                double y = transform.getOrigin().getY();
                double z = transform.getOrigin().getZ();  
                current_time4=ros::Time::now().toSec();
                v_x=(x-last_x)/(current_time4-last_time);
                d_all=5.2;
                d1=4.5;
                ttc=(5.2-(x+last_x)/2)/v_x;
                t=current_time4-t0;
                if ((d_all-d1)<=(d_all-x)){
                 ttc_r=1/(0.0563*t*t*t+0.1336*t*t-0.9678*t+(-1.5)/2);
                 t1=t;
                 ttc0=-ttc;}
                else
                 {ttc_r=current_time-t1+ttc0;}   
                geometry_msgs::Twist msg; 
                ROS_INFO("t: %f, TTC_ref: %f   TTC_feed: %f",t,ttc_r,ttc);
                float u_x = m_pidX.update(-ttc_r/ttc,1);       //second-first=error   
                float u_y = m_pidY.update(y,target_y);
                float u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
                float m=0.024;               // weight of CF
                float g=initial_g;
                float target_psi=0;
                float target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                float target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                float a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                float a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                float a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                float u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                float phi1=target_phi*180/3.14159;
                float theta1=target_theta*180/3.14159;
                //ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput); 
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
                 ROS_INFO("current time,position, v_x: %f, %f,  %f",current_time4,x,v_x);
                last_x=x;
                last_y=y;
                last_z=z;
                last_time=current_time4;
                m_pubNav.publish(msg);
                std::ofstream myfile1;
                myfile1.open("/home/haijie/Desktop/time.txt", std::ios_base::app | std::ios_base::out);
                myfile1 << std::fixed << current_time4 <<"\n";
                myfile1.close();
                std::ofstream myfile2;
                myfile2.open("/home/haijie/Desktop/position.txt", std::ios_base::app | std::ios_base::out);
                myfile2 << x << " "<< y << " " << z << " " << "\n";
                myfile2.close();
                std::ofstream myfile3;
                myfile1.open("/home/haijie/Desktop/ttcref.txt", std::ios_base::app | std::ios_base::out);
                myfile1 << ttc_r <<"\n";
                myfile1.close();
                std::ofstream myfile4;
                myfile2.open("/home/haijie/Desktop/ttc.txt", std::ios_base::app | std::ios_base::out);
                myfile2 << ttc << "\n";
                myfile2.close();
                //if (abs(v_x)<0.02){
                //  m_state = Idle;
                //  last_time=current_time4;
                //  t0=current_time4;
                //  ROS_INFO("Perched!");
               // }
            }
            break;

        case Idle:
            {
                geometry_msgs::Twist msg;
                m_pubNav.publish(msg);
            }
            break;
        }
    }

private:

    enum State
    {
        Idle = 0,
        Automatic = 1,
        TakingOff = 2,
        Landing = 3,
        Original_position=4,
        Initialize=5,
        Tau=6,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;  // class PID, object m_pidX
    PID m_pidY;
    PID m_pidZ;
    PID m_pidYaw;
    State m_state;
    geometry_msgs::PoseStamped m_goal;                //A Pose with reference coordinate frame and timestamp
    ros::Subscriber m_subscribeGoal;
    ros::ServiceServer m_serviceTakeoff;
    ros::ServiceServer m_serviceLand;
    float last_x, last_y, last_z;
    double current_time, last_time, takeoff_time;
    float m_thrust;
    float m_startZ;
    float initial_g;
    double trajectory_time;
    double delta_t;
    double v_x;
    double prepare_time;
    double constant_speed_time;
    double t0;
    double ttc;
    double ttc0;
    double t;
    float d_all;
    double ttc_r;
    double t1;
    float d1;
    float x1;
    double current_time1=0;
    double current_time2=0;
    double current_time3=0;
    double current_time4=0;
    float x_minOutput=-15; float y_minOutput=-15; float z_minOutput=10000; float x_maxOutput=15; float y_maxOutput=15; float z_maxOutput=60000;
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "controller");    // name of the node to be initiated

  // Read parameters
  ros::NodeHandle n("~");                //main access point to communications with the ROS system
  std::string worldFrame;
  n.param<std::string>("worldFrame", worldFrame, "/world");
  std::string frame;
  n.getParam("frame", frame);           //Fetch a value from the parameter server
  double frequency;
  n.param("frequency", frequency, 100.0);  //param() is similar to getParam(), but allows you to specify a default value 

  Controller controller(worldFrame, frame, n);        //define a class of type Controller, named controller
  controller.run(frequency);                          //The function "run" defined in class,  ros::spin() is in this function
 // ros::NodeHandle node;
//        ros::Timer timer = node.createTimer(ros::Duration(1.0/frequency), &Controller::iteration, this);          // Timer is the callback function, maybe we can use concurrent

//  while (hint==0 && ros::ok()) {
//         ros::spin();}
  return 0;
}
