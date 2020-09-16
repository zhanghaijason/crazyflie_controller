#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <std_srvs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <ios>
#include <fstream>
#include <iostream>
#include "pid.hpp"
#include <eigen3/Eigen/Dense>
#include <random>

using namespace std;
using namespace Eigen;
std::default_random_engine generator;
std::normal_distribution<double> distribution(/*mean=*/0.0, /*stddev=*/1.0);
//using Eigen::MatrixXd;
float x_minOutput=-25; float y_minOutput=-25; float z_minOutput=10000; float x_maxOutput=25; float y_maxOutput=25; float z_maxOutput=65000;
double sigma_a=0.05;
double sigma_z=0.0002;
//Eigen::Vector1d R((Eigen::Matrix1d() << sigma_z*sigma_z).finished());;
double aa=sigma_a*sigma_a;
int n = 2; // Number of states
int m = 1; // Number of measurements
int frequency1=50;
double dt = 1.0/frequency1; // Time step
double dt4=dt*dt*dt*dt;
double dt3=dt*dt*dt;
double dt2=dt*dt;
double q1;
double q2;
double q3;
double q4;
Eigen::MatrixXd Fa(n,n);
Eigen::MatrixXd I(n,n);
Eigen::MatrixXd Ba(n,m);
Eigen::MatrixXd Qa(n,n);
Eigen::MatrixXd Ha(m,n);
Eigen::VectorXd R(m);
Eigen::MatrixXd P(n, n);
Eigen::MatrixXd x_hat(n,m);
Eigen::MatrixXd U(m, m);
Eigen::VectorXd Z(m);
void  Kalmanf(const Eigen::VectorXd& Za){
  //define all the internal variables
  Eigen::MatrixXd x_hat_new(n,m);
  Eigen::MatrixXd Pup(n, n);
  Eigen::VectorXd Y_tilt;
  Eigen::VectorXd S;
  Eigen::MatrixXd K(n,m);
  R<<sigma_z*sigma_z;
  double randomNumber = distribution(generator);
  U<<randomNumber;
  U<<0;
  cout<<"U:"<<U<<"\n";
  cout<<"R:"<<R<<"\n";
// update
  x_hat_new = Fa * x_hat+Ba*U;
  Pup = Fa*P*Fa.transpose() + Qa;
  K = Pup*Ha.transpose()*(Ha*Pup*Ha.transpose() + R).inverse();
  x_hat_new += K * (Z - Ha*x_hat_new);
  P = (I - K*Ha)*Pup;
  x_hat = x_hat_new;
}

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
         , m_pidtau(
            get(n, "PIDs/tau/kp"),
            get(n, "PIDs/tau/kd"),
            get(n, "PIDs/tau/ki"),
            get(n, "PIDs/tau/minOutput"),
            get(n, "PIDs/tau/maxOutput"),
            get(n, "PIDs/tau/integratorMin"),
            get(n, "PIDs/tau/integratorMax"),
            "tau")
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
		x = transform.getOrigin().getX();
                target_x=2.5;  //hover set a as 3, perch set as 2
                target_y=0;
                target_z=-0.1;
                y = transform.getOrigin().getY();
                z = transform.getOrigin().getZ();

                geometry_msgs::Twist msg;    
                u_x = m_pidX.update(x,target_x);       //second-first=error   
                u_y = m_pidY.update(y,target_y);
                u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
		g=initial_g;
		target_psi=0;
		target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
	        target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
		a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
		a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
		a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
		u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                phi1=target_phi*180/3.14159;
                theta1=target_theta*180/3.1415926;
		msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput);   // feedback  and  reference
                msg.linear.x =  std::max(std::min(theta1 , x_maxOutput), x_minOutput); // invert it, in server.cpp, it is reversed 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     //current is the boday frame, so the current pose is [0,0,0,0]
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
                last_x=x;
                last_y=y;
                last_z=z;
                //ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
                ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f Radio output phi theta thrust: (%f,%f,%f)", x,y,z,u_x,u_y,u_z,phi1,theta1,u,msg.linear.y, msg.linear.x,msg.linear.z);
                m_pubNav.publish(msg);
                current_time=ros::Time::now().toSec();
               
                if (sqrt(pow(target_x-x,2)+pow(target_y-y,2)+pow(target_z-z,2))<0.2 && current_time-takeoff_time>10){
                  m_state = perch;
                  ROS_INFO("Prepare time: %f",current_time);
                  prepare_time=current_time;
                }
            }
            break;
        case perch:
            {
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);  //it seems after this, the transform is filled, transform goal into body frame
                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY( roll, pitch,yaw);
                ROS_INFO("perch stage intiated");
                x = transform.getOrigin().getX();
                target_x=2.5;
                target_y=0;
                target_z=0;
                y = transform.getOrigin().getY();
                z = transform.getOrigin().getZ();
                current_time=ros::Time::now().toSec();
                v_x=(x-last_x)/(current_time-last_time);

                geometry_msgs::Twist msg; 
                u_x = m_pidX.update(x,target_x);       //second-first=error   
                u_y = m_pidY.update(y,target_y);
                u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
                g=initial_g;
                target_psi=0;
                target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                phi1=target_phi*180/3.14159;
                theta1=target_theta*180/3.14159;
                ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput);   // feedback  and  reference
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; // invert it, in server.cpp, it is reversed 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     //current is the boday frame, so the current pose is [0,0,0,0]
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
              //   ROS_INFO("current time, v_x, target_x: %f,  %f, %f",current_time,v_x,target_x);
                float v0=(x-last_x)/(current_time-last_time);
                last_x=x;
                last_y=y;
                last_z=z;
                last_time=current_time;
                m_pubNav.publish(msg);
                if (sqrt(pow(target_x-x,2)+pow(target_y-y,2)+pow(target_z-z,2))<0.25 && current_time-prepare_time>8){
                  m_state = Idle;
//                  last_time=current_time;
                  ROS_INFO("Finish time: %f",current_time);
                  constant_speed_time=current_time;
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
                x = transform.getOrigin().getX();
                target_x=1.2;
                target_y=0;
                target_z=0;
                y = transform.getOrigin().getY();
                z = transform.getOrigin().getZ();
                current_time=ros::Time::now().toSec();
                v_x=(x-last_x)/(current_time-last_time);

                geometry_msgs::Twist msg; 
                u_x = m_pidX.update(x,target_x);       //second-first=error   
                u_y = m_pidY.update(y,target_y);
                u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
                g=initial_g;
                target_psi=0;
                target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                phi1=target_phi*180/3.14159;
                theta1=target_theta*180/3.14159;
                ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput);   // feedback  and  reference
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; // invert it, in server.cpp, it is reversed 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     //current is the boday frame, so the current pose is [0,0,0,0]
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
              //   ROS_INFO("current time, v_x, target_x: %f,  %f, %f",current_time,v_x,target_x);
                float v0=(x-last_x)/(current_time-last_time);
                last_x=x;
                last_y=y;
                last_z=z;
                last_time=current_time;
                m_pubNav.publish(msg);
                if (sqrt(pow(target_x-x,2)+pow(target_y-y,2)+pow(target_z-z,2))<0.25 && current_time-prepare_time>8){
                  m_state = Initialize;
//                  last_time=current_time;
                  ROS_INFO("Finish time: %f",current_time);
                  constant_speed_time=current_time;
                  x_hat << x,0;
                  P << 0,0,0,0;
                  Fa << 1,dt,0,1;
                  Ba << dt*dt/2,dt;
                  q1=0.25*dt4*aa;
                  q2=0.5*dt3*aa;
                  q3=0.5*dt3*aa;
                  q4=dt2*aa;
                  Qa << q1,q2,q3,q4;
                  I << 1,0,0,1;
                  Ha << 1,0;
                }
            }
            break;
        case Initialize:
            {
/*                cout<<"F:"<<F<<"\n";
                cout<<"X:"<<X<<"\n";
                cout<<"P:"<<P<<"\n";
                cout<<"B:"<<B<<"\n";
                cout<<"Q:"<<Q<<"\n";
                cout<<"I:"<<I<<"\n";
                cout<<"H"<<H<<"\n";*/
                cout<<"P:"<<P<<"\n";
                cout<<"Q:"<<Qa<<"\n";
                cout<<"q1:"<<q1<<"\n";
                tf::StampedTransform transform;
                m_listener.lookupTransform(m_worldFrame, m_frame, ros::Time(0), transform);  //it seems after this, the transform is filled, transform goal into body frame
                tfScalar roll, pitch, yaw;
                tf::Matrix3x3(transform.getRotation()).getRPY( roll, pitch,yaw);
                ROS_INFO("Acceleration stage intiated");
                x = transform.getOrigin().getX();
                target_x=3;
                target_y=0;
                target_z=0;
                y = transform.getOrigin().getY();
                z = transform.getOrigin().getZ();
                current_time=ros::Time::now().toSec();

                geometry_msgs::Twist msg;
                u_x = m_pidX.update(x,target_x);       //second-first=error   
                u_y = m_pidY.update(y,target_y);
                u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
                g=initial_g;
                target_psi=0;
                target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                phi1=target_phi*180/3.14159;
                theta1=target_theta*180/3.14159;
                //ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput); 
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);     
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
              //  ROS_INFO("current time,position, v_x, target_x, theta: %f,(%f,%f,%f)  %f, %f,%f",current_time,x,y,z,v_x,target_x,pitch);
                last_x=x;
                last_y=y;
                last_z=z;
                last_time=current_time;
                m_pubNav.publish(msg);
                // kalman filter
                Z<<x;
                cout<<"Z:"<<Z<<"\n";
                Kalmanf(Z);
                //
                v_x=x_hat(1,0);
                float dd=x_hat(0,0);
                ROS_INFO("current time,position, v_x, target_x, theta: %f,(%f,%f,%f)  %f, %f,%f",current_time,dd,y,z,v_x,target_x,pitch);
                std::ofstream myfile1;
                myfile1.open("/home/haijie/Desktop/time.txt", std::ios_base::app | std::ios_base::out);
                myfile1 << std::fixed << current_time <<"\n";
                myfile1.close();
                std::ofstream myfile2;
                myfile2.open("/home/haijie/Desktop/position.txt", std::ios_base::app | std::ios_base::out);
                myfile2 << dd << " "<< y << " " << z << " " << "\n";
                myfile2.close();
                std::ofstream myfile5;
                myfile5.open("/home/haijie/Desktop/v_x.txt", std::ios_base::app | std::ios_base::out);
                myfile5 << v_x << "\n";
                myfile5.close();
                
                 if (sqrt(pow(x-2,2))<=0.1&&sqrt(pow(v_x-1.5,2))<=0.15){
                    m_state = Tau;
                    last_time=current_time;
                    x0=x;
                    v0=v_x;
                    t0=current_time;
                    ROS_INFO("Goal state achieved!, t0: %f",current_time);
                    t0=current_time;
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
                x = transform.getOrigin().getX();
                target_x=3;
                target_y=0;
                target_z=0;
                y = transform.getOrigin().getY();
                z = transform.getOrigin().getZ();  
                current_time=ros::Time::now().toSec();
               // v_x=(x-last_x)/(current_time4-last_time);
                d_all=5;
                Z<<x;
                cout<<"Z:"<<Z<<"\n";
                Kalmanf(Z);
                //
                v_x=x_hat(1,0);
                float dd=x_hat(0,0);
                ttc=-(d_all-x)/v_x;
                t=current_time-t0;
                float tau_s=-0.5;
                if (ttc<=tau_s){
                // ttc_r=1/(0.1048*t*t*t-0.122*t*t-0.568*t+(-v0)/(d_all-x0));  //optimal tau
               //  ttc_r=0.7737*t+(d_all-x0)/(-v0);         //constant tau dot
                 ttc_r=1/(0.0067*t*t*t-0.1907*t*t-0.7166*t+(-v0)/(d_all-x0));
                 t1=current_time;
                 ttc0=ttc;}
                else
                 {ttc_r=current_time-t1+ttc0;
                  ROS_INFO("Constant velocity:%f \n",t1);
                  if (ttc_r>0) {ttc_r=-0.02;}
                  }   
                geometry_msgs::Twist msg; 
                ROS_INFO("t: %f, TTC_ref: %f   TTC_feed: %f",t,ttc_r,ttc);
                if (abs(ttc_r)>abs(ttc)){
                   float bonus=2.5;
                   u_x = m_pidtau.update(bonus*ttc_r/ttc,1*bonus);
                } 
                else
                {u_x = m_pidtau.update(ttc_r/ttc,1);}       //second-first=error   
                u_y = m_pidY.update(y,target_y);
                u_z=  m_pidZ.update(z,target_z);     //current is the boday frame, so the current pose is [0,0,0,0]
                g=initial_g;
                target_psi=0;
                target_phi=asin((u_x*sin(target_psi)-u_y*cos(target_psi))/sqrt((u_x*u_x+u_y*u_y+(u_z+g)*(u_z+g))));         // roll
                target_theta=atan((u_x*cos(target_psi)+u_y*sin(target_psi))/(g+u_z));   //pitch
                a1=sin(target_theta)*cos(target_psi)*cos(target_phi)+sin(target_psi)*sin(target_phi);             // sign of tau?????
                a2=sin(target_theta)*sin(target_psi)*cos(target_phi)-cos(target_psi)*sin(target_phi);             // sign of tau?????
                a3=cos(target_theta)*cos(target_phi);             // sign of tau?????
                u=(u_x*a1+u_y*a2+(u_z+g)*a3);
                phi1=target_phi*180/3.14159;
                theta1=target_theta*180/3.14159;
                //ROS_INFO("target position:(%f,%f,%f), PID output: %f,%f,%f  calculated angle phi theta thrust: %f, %f, %f", x,y,z,u_x,u_y,u_z,phi1,theta1,u);
                msg.linear.y = std::max(std::min(phi1 , y_maxOutput), y_minOutput); 
                msg.linear.x =  std::max(std::min(theta1 , y_maxOutput), y_minOutput); ; 
                msg.linear.z =  std::max(std::min(u, z_maxOutput), z_minOutput);;     
                msg.angular.z = -m_pidYaw.update(0.0, yaw);
               // ROS_INFO("Radio output phi theta thrust: (%f,%f,%f)",msg.linear.y, msg.linear.x,msg.linear.z);
                 ROS_INFO("current time,position,u_x,theta, v_x: %f, %f, %f,%f, %f",current_time,x,u_x,msg.linear.x,v_x);
                last_x=x;
                last_y=y;
                last_z=z;
                last_time=current_time;
                m_pubNav.publish(msg);
                std::ofstream myfile1;
                myfile1.open("/home/haijie/Desktop/time.txt", std::ios_base::app | std::ios_base::out);
                myfile1 << std::fixed << current_time <<"\n";
                myfile1.close();
                std::ofstream myfile2;
                myfile2.open("/home/haijie/Desktop/position.txt", std::ios_base::app | std::ios_base::out);
                myfile2 << x << " "<< y << " " << z << " " << "\n";
                myfile2.close();
                std::ofstream myfile3;
                myfile3.open("/home/haijie/Desktop/ttcref.txt", std::ios_base::app | std::ios_base::out);
                myfile3 << ttc_r <<"\n";
                myfile3.close();
                std::ofstream myfile4;
                myfile4.open("/home/haijie/Desktop/ttc.txt", std::ios_base::app | std::ios_base::out);
                myfile4 << ttc << "\n";
                myfile4.close();
                std::ofstream myfile5;
                myfile5.open("/home/haijie/Desktop/v_x.txt", std::ios_base::app | std::ios_base::out);
                myfile5 << v_x << "\n";
                if (abs(v_x)<0.02){
                  m_state = Idle;
                  last_time=current_time;
                  t0=current_time;
                  ROS_INFO("Perched!");
                }
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
        perch=7,
    };

private:
    std::string m_worldFrame;
    std::string m_frame;
    ros::Publisher m_pubNav;
    tf::TransformListener m_listener;
    PID m_pidX;  // class PID, object m_pidX
    PID m_pidY;
    PID m_pidtau;
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
    float x0; float v0; float ttc_last;
    // used for controller
    float x, y, z, target_x, target_y, target_z, u_x, u_y, u_z, g, initial_g, target_psi, target_phi, target_theta, a1, a2, a3, u, phi1, theta1;
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
  n.param("frequency", frequency, 50.0);  //param() is similar to getParam(), but allows you to specify a default value 
  // assign value to the filter parameter
/*  F << 1,dt,0,1;
  B << dt*dt/2,dt;
  Q << 1/4*pow(dt,4)*aa,1/2*pow(dt,3)*aa,1/2*pow(dt,3)*aa,pow(dt,2)*aa;
  I << 1,0,0,1;
  H << 1,0;
  cout << F;*/
//  cout << B;
//  cout << Q;
//  cout << I;
//  cout << H;
  cout<<"dt4:"<<dt4<<"\n";
  Controller controller(worldFrame, frame, n);        //define a class of type Controller, named controller
  controller.run(frequency);                          //The function "run" defined in class,  ros::spin() is in this function
  return 0;
}
