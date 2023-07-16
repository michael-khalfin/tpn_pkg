#include <libsbp_ros_msgs/MsgPosLlh.h>
#include <libsbp_ros_msgs/MsgBaselineHeading.h>
#include <ros/ros.h>
#include <ros/console.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <tpn_pkg/Position.h>
#include <std_msgs/Bool.h>
#define NODE_NAME "tpn_node"

class HelloTPN {
public:
    HelloTPN();
    void run();
private:
    tpn_pkg::Position pos;
    void positionCallback(const libsbp_ros_msgs::MsgPosLlh::ConstPtr& msg);
    void headingCallback(const libsbp_ros_msgs::MsgBaselineHeading::ConstPtr& msg);
    void velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg);

    ros::NodeHandle nh;

    ros::Subscriber gps_pos_sub;
    ros::Subscriber gps_heading_sub;
    ros::Subscriber gps_vel_sub;
    ros::Publisher gps_pub = nh.advertise<tpn_pkg::Position>("gps_chatter", 10);
    //ros::Publisher bool_pub = nh.advertise<std_msgs::Bool>("bool_topic", 10);

    //ros::Publisher gps_pub;

    ros::Rate limiter;
};

HelloTPN::HelloTPN()
    : nh{"~"},
      limiter(10)
{
    gps_pos_sub = nh.subscribe<libsbp_ros_msgs::MsgPosLlh>("/reference/actor1/piksi/position_receiver_0/sbp/pos_llh", 4, &HelloTPN::positionCallback, this);
    gps_heading_sub = nh.subscribe<libsbp_ros_msgs::MsgBaselineHeading>("/attitude/actor1/piksi/attitude_receiver_0/sbp/baseline_heading", 4, &HelloTPN::headingCallback, this);
    gps_vel_sub = nh.subscribe<geometry_msgs::TwistStamped>("/actor1/vehicle/twist", 4, &HelloTPN::velocityCallback, this);

}

void HelloTPN::run()
{
    while (ros::ok())
    {
        //ROS_ERROR_STREAM("PUBLISHING");
        gps_pub.publish(this->pos);
        
        // std_msgs::Bool test = std_msgs::Bool();
        // test.data = true;
        // bool_pub.publish(test);
        ros::spinOnce();
        limiter.sleep();
    }
}

void HelloTPN::positionCallback(const libsbp_ros_msgs::MsgPosLlh::ConstPtr& msg)
{
    libsbp_ros_msgs::MsgPosLlh llh = *msg;
    this->pos.lat = llh.lat;
    this->pos.lon = llh.lon;
    std::cout << "positionCallback -- latitude: " << this->pos.lat << " longitude: " << this->pos.lon;
}

void HelloTPN::headingCallback(const libsbp_ros_msgs::MsgBaselineHeading::ConstPtr& msg)
{
    libsbp_ros_msgs::MsgBaselineHeading baseline_heading = *msg;
    this->pos.heading = baseline_heading.heading / 1000; 
    std::cout << "headingCallback -- heading: " << this->pos.heading;
}

void HelloTPN::velocityCallback(const geometry_msgs::TwistStamped::ConstPtr& msg) {
    geometry_msgs::Twist vel = msg->twist;
    this->pos.angZ = vel.angular.z;
    this->pos.velocity = vel.linear.x;
    std::cout << "velocityCallback -- velocity: " << this->pos.velocity;
    std::cout << "velocityCallback -- angularZ: " << this->pos.angZ;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, NODE_NAME);
    HelloTPN hellotpn = HelloTPN();
    hellotpn.run();
}