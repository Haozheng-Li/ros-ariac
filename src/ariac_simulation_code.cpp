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

#include "ur_kinematics/ur_kin.h"
#include "sensor_msgs/JointState.h"
#include "trajectory_msgs/JointTrajectory.h"

#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

std_srvs::Trigger begin_comp;

std::vector<osrf_gear::Order> order_vector;
std::vector<osrf_gear::LogicalCameraImage> logic_camera_bin_vector, logic_camera_agv_vector, quality_control_sensor_vector;

osrf_gear::GetMaterialLocations get_loc_message;
osrf_gear::LogicalCameraImage camera_message;

geometry_msgs::TransformStamped tfStamped;
geometry_msgs::PoseStamped part_pose, goal_pose;

sensor_msgs::JointState joint_states;

// Init global variables
int total_logical_camera_bin_num = 6;
int total_logical_camera_agv_num = 2;
int total_quality_control_sensor_num = 2;
bool show_first_product_msg_once = true;
bool has_shown_frist_order_msg = false;

double T_pose[4][4];
double q_pose[6];

// Init callback function
void orderCallback(const osrf_gear::Order::ConstPtr& msg)
{
    order_vector.push_back(*msg);
}

// Camera callback
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

// Joint state callback
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& msgs){
  joint_states = *msgs;
}

double * chooseIKSolution(double whole_pos_solution[][6])
{
    double *pos_solution;

    // Code for selecting a best solution
    /*
    int size = sizeof(whole_pos_solution) / sizeof(whole_pos_solution[0]);
    for (int i=0; i < size; i++)
    {
        // Base on the document, if the second joint are in a smaller range would ruled out half solution
        if (whole_pos_solution[i][1] < 3.14)
        {
            ***
        }
    }
    */

    pos_solution = whole_pos_solution[0];   // Now default choose the first solution of 8 solution
    return pos_solution;
}

trajectory_msgs::JointTrajectory generateJointTrajectoryFromPos(geometry_msgs::PoseStamped desired_pos, int header_seq)
{
    trajectory_msgs::JointTrajectory result_joint_trajectory;
    double T_des[4][4];
    double q_des[8][6];

    // Transfer the goal pose to T matrix
    T_des[0][3] = desired_pos.pose.position.x;
    T_des[1][3] = desired_pos.pose.position.y;
    T_des[2][3] = desired_pos.pose.position.z;
    T_des[3][3] = 1.0;

    T_des[0][0] = 0.0; T_des[0][1] = -1.0; T_des[0][2] = 0.0;
    T_des[1][0] = 0.0; T_des[1][1] = 0.0; T_des[1][2] = 1.0;
    T_des[2][0] = -1.0; T_des[2][1] = 0.0; T_des[2][2] = 0.0;
    T_des[3][0] = 0.0; T_des[3][1] = 0.0; T_des[3][2] = 0.0;

    // Find the solution of inverse kinematics of T matrix of goal pose
    int num_sols = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
    
    // Basic attribute of result_joint_trajectory setting 
    result_joint_trajectory.header.seq = header_seq;
    result_joint_trajectory.header.stamp = ros::Time::now();
    result_joint_trajectory.header.frame_id = "/world";

    result_joint_trajectory.joint_names.clear();
    result_joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    result_joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    result_joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    result_joint_trajectory.joint_names.push_back("elbow_joint");
    result_joint_trajectory.joint_names.push_back("wrist_1_joint");
    result_joint_trajectory.joint_names.push_back("wrist_2_joint");
    result_joint_trajectory.joint_names.push_back("wrist_3_joint");

    // Set a start point
    result_joint_trajectory.points.resize(2);
    result_joint_trajectory.points[0].positions.resize(result_joint_trajectory.joint_names.size());
    for(int indy = 0; indy < result_joint_trajectory.joint_names.size(); indy++){
        for(int indz = 0; indz < joint_states.name.size(); indz++) {
            if(result_joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                result_joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    
    // Select the best solution from IK
    double *q_solution = chooseIKSolution(q_des);
    // Set the end point for the movement
    result_joint_trajectory.points[1].positions.resize(result_joint_trajectory.joint_names.size());
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    result_joint_trajectory.points[1].positions[0] = joint_states.position[1];
    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) {
        result_joint_trajectory.points[1].positions[indy + 1] = q_solution[indy];
    }

    // When to start (immediately upon receipt).
    result_joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    // How long to take for the movement.
    result_joint_trajectory.points[1].time_from_start = ros::Duration(1.0);

    return result_joint_trajectory;
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

    int count = 0;
    int service_call_succeeded;
    int get_loc_call_succeeded;
    bool find_product_succeeded = false;

    std::string logical_camera_name, logical_camera_bin_frame;
    ros::Subscriber camera_sub[6];
    trajectory_msgs::JointTrajectoryPoint desired;
    trajectory_msgs::JointTrajectory joint_trajectory;
    
    control_msgs::FollowJointTrajectoryAction joint_trajectory_as;

    // Init ServiceClient
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient get_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction>trajectory_as("ariac/arm/follow_joint_trajectory", true);

    // Init subscriber
    ros::Subscriber order_sub = n.subscribe("/ariac/orders", 10, orderCallback);
    ros::Subscriber joint_states_sub = n.subscribe("ariac/arm1/joint_states", 10, jointStateCallback);

    // Init joint state publisher
    ros::Publisher joint_state_pub = n.advertise<trajectory_msgs::JointTrajectory>("ariac/arm1/arm/command", 1000);


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
    ros::Rate loop_rate(10);    // f=10HZ, period=100ms
    // ros::MultiThreadedSpinner spinner(4);
    // spinner.spin();
    
    while (ros::ok() && service_call_succeeded)
    {
        // lab5
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
                        osrf_gear::Model product_model = camera_message.models[k];
                        if (first_product.type == product_model.type)
                        {
                            find_product_succeeded = true;
                            ROS_INFO_ONCE("The position of the first product is: [x = %f, y = %f, z = %f]",product_model.pose.position.x,product_model.pose.position.y,product_model.pose.position.z);
                            ROS_INFO_ONCE("The orientation of the first product type is: [qx = %f, qy = %f, qz = %f, qw = %f]",product_model.pose.orientation.x, product_model.pose.orientation.y, product_model.pose.orientation.z, product_model.pose.orientation.w);
                            
                            logical_camera_bin_frame = "logical_camera_bin" + std::to_string(j) +"_frame";

                            try {
                                tfStamped = tfBuffer.lookupTransform("arm1_base_link", logical_camera_bin_frame, ros::Time(0.0), ros::Duration(1.0));
                                ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
                            } 
                            catch (tf2::TransformException &ex) {
                                ROS_ERROR("%s", ex.what());
                            }

                            // Transform coordinate 
                            part_pose.pose = product_model.pose;
                            tf2::doTransform(part_pose, goal_pose, tfStamped);

                            // Fix the position
                            goal_pose.pose.position.z += 0.10; // 10 cm above the part
                            goal_pose.pose.orientation.w = 0.707;
                            goal_pose.pose.orientation.x = 0.0;
                            goal_pose.pose.orientation.y = 0.707;
                            goal_pose.pose.orientation.z = 0.0;     // goal pose is the position of the item we want
                            ROS_WARN_ONCE("The goal pose infomation will show below");
                            ROS_WARN_STREAM_ONCE(goal_pose);
                            break;
                        }
                    }
                }
            }
        }

        // Lab6
        if (find_product_succeeded)
        {
            ROS_INFO_THROTTLE(10, "joint_states position status:[%f, %f, %f, %f, %f, %f]", joint_states.position[0], joint_states.position[1], 
            joint_states.position[2], joint_states.position[3], joint_states.position[4], joint_states.position[5]); //100*100ms=10s

            // joint_states.position[0] is the linear_arm_actuator_joint
            q_pose[0] = joint_states.position[1];
            q_pose[1] = joint_states.position[2];
            q_pose[2] = joint_states.position[3];
            q_pose[3] = joint_states.position[4];
            q_pose[4] = joint_states.position[5];
            q_pose[5] = joint_states.position[6];
            ur_kinematics::forward((double *)&q_pose, (double *)&T_pose);

            ROS_INFO("Below will show T_pose");
            ROS_INFO("%f, %f, %f", T_pose[0][3], T_pose[1][3], T_pose[2][3]);

            joint_trajectory = generateJointTrajectoryFromPos(goal_pose, count++);
            joint_state_pub.publish(joint_trajectory);

            // joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
            // actionlib::SimpleClientGoalState state = trajectory_as.sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(10.0), ros::Duration(10.0));
            // ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
