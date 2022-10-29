#include <string>
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
std::vector<osrf_gear::LogicalCameraImage> logic_camera_bin_vector, logic_camera_agv_vector, quality_control_sensor_vector;

osrf_gear::GetMaterialLocations get_loc_message;
osrf_gear::LogicalCameraImage camera_message;

geometry_msgs::TransformStamped tfStamped;
geometry_msgs::PoseStamped part_pose, goal_pose;

int total_logical_camera_bin_num = 6;
int total_logical_camera_agv_num = 2;
int total_quality_control_sensor_num = 2;
bool show_first_product_msg_once = true;
bool has_shown_frist_order_msg = false;


void orderCallback(const osrf_gear::Order::ConstPtr& msg)
{
    order_vector.push_back(*msg);
}

void binCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int bin_camera_num)
{
    logic_camera_bin_vector[bin_camera_num] = *msg;
}

void agvCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& msg, int agv_camera_num)
{
    logic_camera_agv_vector[agv_camera_num] = *msg;
}

void qualityCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& quality_msg, int quality_camera_num)
{
    quality_control_sensor_vector[quality_camera_num] = *quality_msg;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_simulation");
   
    // Init the node
    ros::NodeHandle n;

    // Init listener and buffer
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);

    // Init variables
    order_vector.clear();
    logic_camera_bin_vector.clear();
    logic_camera_bin_vector.resize(6);
    logic_camera_agv_vector.clear();
    logic_camera_agv_vector.resize(2);
    quality_control_sensor_vector.clear();
    quality_control_sensor_vector.resize(2);

    int service_call_succeeded;
    int get_loc_call_succeeded;

    std::string logical_camera_name, logical_camera_bin_frame;
    ros::Subscriber camera_sub[6];
    

    my_bool_var.request.data = true;

    // Init ServiceClient
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient get_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    // Init subscriber
    ros::Subscriber order_sub = n.subscribe("/ariac/orders", 1000, orderCallback);

    // We have 6 logical camera bin, so we need to create 6 callback
    for(int i=0; i < total_logical_camera_bin_num; i++)
    {
        logical_camera_name = "/ariac/logical_camera_bin" + std::to_string(i);
        camera_sub[i] = n.subscribe<osrf_gear::LogicalCameraImage>(logical_camera_name, 10, boost::bind(binCameraCallback, _1, i));
    }

    ros:: Subscriber agv_camera_sub1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 10, boost::bind(agvCameraCallback, _1, 0));
    ros:: Subscriber agv_camera_sub2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 10, boost::bind(agvCameraCallback, _1, 1));

    ros:: Subscriber quality_camera_sub1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 10, boost::bind(qualityCameraCallback, _1, 0));
    ros:: Subscriber quality_camera_sub2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 10, boost::bind(qualityCameraCallback, _1, 1));

    // Service call status
    service_call_succeeded = begin_client.call(begin_comp);
    if(service_call_succeeded == 0)
    {
        ROS_ERROR("Competition service call failed!, Please check the competion service!!!!");
    }
    else
    {
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

    while (ros::ok() && service_call_succeeded)
    {
        if (order_vector.size() > 0)
        {
            osrf_gear::Order first_order = order_vector[0];
            osrf_gear::Shipment first_shipment = first_order.shipments[0];
            osrf_gear::Product first_product = first_shipment.products[0];
            get_loc_message.request.material_type = first_product.type;
            get_loc_call_succeeded = get_loc_client.call(get_loc_message);

            ROS_INFO_ONCE("Received order successfully! The  first product type is: [%s]", first_product.type.c_str());
            
            if (get_loc_call_succeeded){
                ROS_INFO_ONCE("The storage location of the material type [%s] is: [%s]", first_product.type.c_str(), get_loc_message.response.storage_units[0].unit_id.c_str());
                // Serach all logic camera data
                for (int j=0; j<total_logical_camera_bin_num; j++)
                {
                    camera_message = logic_camera_bin_vector[j];
                    for (int k=0; k<camera_message.models.size(); k++)
                    {
                        osrf_gear::Model product_mode = camera_message.models[k];
                        if (first_product.type == product_mode.type)
                        {
                            ROS_INFO_ONCE("The position of the first product's type is: [x = %f, y = %f, z = %f]",product_mode.pose.position.x,product_mode.pose.position.y,product_mode.pose.position.z);
                            ROS_INFO_ONCE("The orientation of the material type is: [qx = %f, qy = %f, qz = %f, qw = %f]",product_mode.pose.orientation.x, product_mode.pose.orientation.y, product_mode.pose.orientation.z, product_mode.pose.orientation.w);
                            
                            logical_camera_bin_frame = "logical_camera_bin" + std::to_string(j) +"_frame";

                            try {
                                tfStamped = tfBuffer.lookupTransform("arm1_base_link", logical_camera_bin_frame, ros::Time(0.0), ros::Duration(1.0));
                                ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
                            } 
                            catch (tf2::TransformException &ex) {
                                ROS_ERROR("%s", ex.what());
                            }

                            // Transform coordinate 
                            part_pose.pose = product_mode.pose;
                            tf2::doTransform(part_pose, goal_pose, tfStamped);

                            // Fix the position
                            goal_pose.pose.position.z += 0.10; // 10 cm above the part
                            goal_pose.pose.orientation.w = 0.707;
                            goal_pose.pose.orientation.x = 0.0;
                            goal_pose.pose.orientation.y = 0.707;
                            goal_pose.pose.orientation.z = 0.0;
                            ROS_WARN_ONCE("The pose position of transformed location of first product:[x=%f],[y=%f],[z=%f]",goal_pose.pose.position.x, goal_pose.pose.position.y, goal_pose.pose.position.z);
                            break;
                        }
                    }
                }
            }

        }

        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}