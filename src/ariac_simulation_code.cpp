#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

std_srvs::Trigger begin_comp;
std_srvs::SetBool my_bool_var;
tf2_ros::Buffer tfBuffer;


std::vector<osrf_gear::Order> order_vector;

// my_bool_var.request.data = true;

void orderCallback(const osrf_gear::Order& msg)
{
    order_vector.push_back(msg);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_simulation_node");
   
    // Init the node
    ros::NodeHandle n;

    // Init variables
    order_vector.clear();
    int service_call_succeeded;

    // Init ServiceClient
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");

    // Init subscriber
    ros::Subscriber order_sub = n.subscribe("/ariac/orders", 1000, orderCallback);

    // Call the Service
    service_call_succeeded = begin_client.call(begin_comp);

    if(service_call_succeeded == 0)
    {
        ROS_ERROR("Competition service call failed!");
    }
    else{
        ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
    }

    // Set the frequency of loop in the node
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}