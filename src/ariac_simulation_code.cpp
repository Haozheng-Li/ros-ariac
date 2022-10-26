#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

#include "osrf_gear/Order.h"
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>

#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"

std_srvs::Trigger begin_comp;
std_srvs::SetBool my_bool_var;

std::vector<osrf_gear::Order> order_vector;

tf2_ros::Buffer tfBuffer;
tf2_ros::TransformListener tfListener(tfBuffer);

osrf_gear::LogicalCameraImage cameramessage;

geometry_msgs::TransformStamped tfStamped;
geometry_msgs::PoseStamped part_pose, goal_pose;


// my_bool_var.request.data = true;

void orderCallback(const osrf_gear::Order::ConstPtr& msg)
{
    order_vector.push_back(*msg);
}

void cameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg)
{
    cameramessage = *msg;
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
    ros::ServiceClient material_location_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    // Init subscriber
    ros::Subscriber order_sub = n.subscribe("/ariac/orders", 1000, orderCallback);
    ros::Subscriber camera_sub = n.subscribe("/ariac/logical_camera", 1, cameraCallback);

    // Service call status
    service_call_succeeded = begin_client.call(begin_comp);
    if(service_call_succeeded == 0)
    {
        ROS_ERROR("Competition service call failed!, Please shut down and restart");
    }
    else{
        if(begin_comp.response.success)
        {
            ROS_INFO("Competition service called successfully: %s", begin_comp.response.message.c_str());
        }
        else
        {
            ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str());
        }
    }

    // Set the frequency of loop in the node
    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        if (order_vector.size() > 0)
        {
            
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}