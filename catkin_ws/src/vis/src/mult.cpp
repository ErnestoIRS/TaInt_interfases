// ros node that gets the point called /centroid and multiplies it by 100

#include "ros/ros.h"
#include "geometry_msgs/Point.h"
#include "geometry_msgs/PointStamped.h"
#include "std_msgs/Float64.h"
#include "multiply.h"



// Callback function for the subscriber
void callback(const geometry_msgs::Point::ConstPtr& msg) {
    ROS_INFO("I heard: [%f, %f, %f]", msg->x, msg->y, msg->z);
    // create a new point stamped message
    geometry_msgs::PointStamped *h_point = new geometry_msgs::PointStamped();
    // set the stamp to the same as the received message
    // multiply each value of poing by 100
    h_point->point.x = multHundred(msg->x);
    h_point->point.y = multHundred(msg->y);
    h_point->point.z = multHundred(msg->z);

    ROS_INFO("Multiplied is: [%f, %f, %f]", h_point->point.x, h_point->point.y, h_point->point.z );

    // set timestamp to current time
    h_point->header.stamp = ros::Time::now();
    // publish the message
    ros::NodeHandle nh;
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("coordinates_multiplied", 1000);
    pub.publish(*h_point);


    
}



int main(int argc, char **argv) {
    // Initialize the ROS system
    ros::init(argc, argv, "mult");
    // Establish this program as a ROS node
    ros::NodeHandle nh;
    // Create a subscriber object
    ros::Subscriber sub = nh.subscribe("green_object_coordinates", 1000, callback);
    // Create a publisher object
    
    ros::Publisher pub = nh.advertise<geometry_msgs::PointStamped>("coordinates_multiplied", 1000);

    // Enter a loop, pumping callbacks
    ros::spin();
    // Return success
    return 0;
}