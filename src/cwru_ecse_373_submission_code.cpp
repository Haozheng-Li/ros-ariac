#include <string>
#include <math.h>
#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"

#include "osrf_gear/Order.h"
#include <osrf_gear/LogicalCameraImage.h>
#include <osrf_gear/GetMaterialLocations.h>
#include "osrf_gear/VacuumGripperControl.h"
#include "osrf_gear/VacuumGripperState.h"

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
osrf_gear::VacuumGripperState gripper_state;
control_msgs::FollowJointTrajectoryAction joint_trajectory_as;


geometry_msgs::TransformStamped tfStamped;
geometry_msgs::PoseStamped part_pose, goal_pose;

std::vector<geometry_msgs::PoseStamped> goal_pose_vector;

sensor_msgs::JointState joint_states;
trajectory_msgs::JointTrajectory joint_trajectory;

// Init global variables
int count = 0;
double *q_solution;
int total_logical_camera_bin_num = 6;
int total_logical_camera_agv_num = 2;
int total_quality_control_sensor_num = 2;
bool show_first_product_msg_once = true;
bool has_shown_frist_order_msg = false;

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

void gripperCallBack(const osrf_gear::VacuumGripperState::ConstPtr& msgs){
    gripper_state = *msgs;
}

void transformPosition(geometry_msgs::PoseStamped *ori_pos, geometry_msgs::PoseStamped *transformed_pos, std::string target_frame, std::string source_frame, tf2_ros::Buffer *buffer)
{
    try {
        tfStamped = buffer->lookupTransform(target_frame, source_frame, ros::Time(0.0), ros::Duration(1.0));
        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
    } 
    catch (tf2::TransformException &ex) {
        ROS_ERROR("%s", ex.what());
    }

    // Transform coordinate 
    tf2::doTransform(*ori_pos, *transformed_pos, tfStamped);

    // Fix the position
    transformed_pos->pose.position.z += 0.10;
    transformed_pos->pose.orientation.w = 0.707;
    transformed_pos->pose.orientation.x = 0.0;
    transformed_pos->pose.orientation.y = 0.707;
    transformed_pos->pose.orientation.z = 0.0;   
}


void chooseIKSolution(double pos_solutions[][6], int solution_num, double *pos_solution)
{
    int best_solution_index = 0;
	for (int index=0; index < solution_num; index++)
	{
		if (pos_solutions[index][1]<=2*M_PI && pos_solutions[index][1]>=M_PI)   // second joint range bewteen pi and 2pi
		{
            best_solution_index = index;
            break;
		}
	}
    // pos_solution = pos_solutions[best_solution_index];
    pos_solution = pos_solutions[0];   // Now default choose the first solution of 8 solution
}

void generateJointTrajectoryFromPos(geometry_msgs::PoseStamped desired_pos, double base_movement=0)
{
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
    int solution_num = ur_kinematics::inverse((double *)&T_des, (double *)&q_des);
    
    // Basic attribute of joint_trajectory setting 
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";

    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");
    // Set a start point
    joint_trajectory.points.resize(2);
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
    for(int indy = 0; indy < joint_trajectory.joint_names.size(); indy++){
        for(int indz = 0; indz < joint_states.name.size(); indz++) {
            if(joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    // Select the best solution from IK
    // chooseIKSolution(q_des, solution_num, q_solution);
    double *q_solution=q_des[0];
    // Set the end point for the movement
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
    // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.
    joint_trajectory.points[1].positions[0] = joint_states.position[1];
    // The actuators are commanded in an odd order, enter the joint positions in the correct positions
    for (int indy = 0; indy < 6; indy++) {
        joint_trajectory.points[1].positions[indy + 1] = q_solution[indy];
    }
    joint_trajectory.points[1].positions[0] = base_movement;

    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    // How long to take for the movement.
    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);
}

void moveBaseToPos(double base_position)
{
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now();
    joint_trajectory.header.frame_id = "/world";

    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");

	joint_trajectory.points.resize(2);
	joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
	joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());
	for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++)
	{
		for (int indz = 0; indz < joint_states.name.size(); indz++)
		{
			if (joint_trajectory.joint_names[indy] == joint_states.name[indz])
			{
				joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
				joint_trajectory.points[1].positions[indy] = joint_states.position[indz];
				break;
			}
        }
    }
    joint_trajectory.points[1].positions[0] = base_position;
    // When to start (immediately upon receipt).
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);
    // How long to take for the movement.
    joint_trajectory.points[1].time_from_start = ros::Duration(4.0);
}


void doTarjectoryAction(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *action_client)
{
    joint_trajectory_as.action_goal.goal.trajectory = joint_trajectory;
    actionlib::SimpleClientGoalState state = action_client->sendGoalAndWait(joint_trajectory_as.action_goal.goal, ros::Duration(30.0), ros::Duration(30.0));
    ROS_INFO("Action Server returned with status: [%i] %s", state.state_, state.toString().c_str());
}

void moveToPos(actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *action_client, geometry_msgs::PoseStamped desired_pos, double base_movement=0)
{
    // if (base_position)
    // {
    //     moveBaseToPos(base_position);
    //     doTarjectoryAction(action_client);
    // }

    generateJointTrajectoryFromPos(desired_pos, base_movement);
    doTarjectoryAction(action_client);
}


void gripperControl(ros::ServiceClient *gripper_client, bool do_attach)
{
    osrf_gear::VacuumGripperControl gripper_control;

    gripper_control.request.enable = do_attach;

    int service_call_succeeded = gripper_client->call(gripper_control);

    if(gripper_control.response.success){
        ros::Time start_wait = ros::Time::now();
        ROS_INFO("gripper service call succeeded, now state=%d", do_attach);
        while((ros::Time::now().toSec() - start_wait.toSec() < 2)){
      }
    }
}

void gripItem(ros::ServiceClient *gripper_client, geometry_msgs::PoseStamped item_position, ros::Rate stop_rate, actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> *action_client, int realease)
{
    geometry_msgs::PoseStamped desired_pos; 
    desired_pos = item_position;
    moveToPos(action_client, desired_pos);

    stop_rate.sleep();
    stop_rate.sleep();

    desired_pos.pose.position.z = item_position.pose.position.z - 0.08;
    moveToPos(action_client, desired_pos);

    stop_rate.sleep();
    stop_rate.sleep();

    if (realease == 0)
    {
        gripperControl(gripper_client, true);
    }
    else{
        gripperControl(gripper_client, false);
    }
    

    stop_rate.sleep();
    stop_rate.sleep();

    desired_pos = item_position;
    moveToPos(action_client, desired_pos);

    stop_rate.sleep();
    stop_rate.sleep();
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "cwru_ecse_373_submission");
   
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

    int goal_material_index = 0;
    int service_call_succeeded;
    int get_loc_call_succeeded;
    bool find_product_succeeded = false;

    std::string logical_camera_name, logical_camera_bin_frame;
    ros::Subscriber camera_sub[6];
    trajectory_msgs::JointTrajectoryPoint desired;

    // Init ServiceClient
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition");
    ros::ServiceClient get_loc_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");
    ros::ServiceClient gripper_client = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");
    
    actionlib::SimpleActionClient<control_msgs::FollowJointTrajectoryAction> trajectory_as("ariac/arm1/arm/follow_joint_trajectory", true);
    trajectory_as.waitForServer();

    // Init subscriber
    ros::Subscriber order_sub = n.subscribe("/ariac/orders", 10, orderCallback);
    ros::Subscriber joint_states_sub = n.subscribe("ariac/arm1/joint_states", 10, jointStateCallback);
    ros::Subscriber gripper_subscriber = n.subscribe("/ariac/arm1/gripper/state", 10, gripperCallBack);

    // We have 6 logical camera bin, so we need to create 6 callback
    for(int i=0; i < total_logical_camera_bin_num; i++)
    {
        logical_camera_name = "/ariac/logical_camera_bin" + std::to_string(i);
        camera_sub[i] = n.subscribe<osrf_gear::LogicalCameraImage>(logical_camera_name, 10, boost::bind(binCameraCallback, _1, i));
    }

    ros::Subscriber agv_camera_sub1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv1", 10, boost::bind(agvCameraCallback, _1, 0));
    ros::Subscriber agv_camera_sub2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/logical_camera_agv2", 10, boost::bind(agvCameraCallback, _1, 1));

    ros::Subscriber quality_camera_sub1 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_1", 10, boost::bind(qualityCameraCallback, _1, 0));
    ros::Subscriber quality_camera_sub2 = n.subscribe<osrf_gear::LogicalCameraImage>("/ariac/quality_control_sensor_2", 10, boost::bind(qualityCameraCallback, _1, 1));


    // Set the frequency of loop in the node
    ros::Rate loop_rate(1);    // f=10HZ, period=100ms

    gripperControl(&gripper_client, false);

    bool gripper_flag = true;

    bool once = true;

    while (ros::ok())
    {
        // Service call status
        service_call_succeeded = begin_client.call(begin_comp);
        if(service_call_succeeded == 0){
            ROS_ERROR_THROTTLE(10, "Competition service call failed!, Please check the competion service!!!!");
        }
        else{
            if(begin_comp.response.success)
            {
                ROS_INFO_THROTTLE(10, "Competition service called successfully: %s", begin_comp.response.message.c_str());
            }
            else
            {
                ROS_WARN_THROTTLE(10, "Competition service returned failure: %s", begin_comp.response.message.c_str());
            }
        }
        
        // Find the order info
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

                            // Transform coordinate 
                            part_pose.pose = product_model.pose;
                            logical_camera_bin_frame = "logical_camera_bin" + std::to_string(j) +"_frame";
                            transformPosition(&part_pose, &goal_pose, "arm1_base_link", logical_camera_bin_frame, &tfBuffer);

                            ROS_WARN_ONCE("The goal pose infomation will show below");
                            ROS_WARN_STREAM_ONCE(goal_pose);

                            if (goal_pose_vector.size() < camera_message.models.size())
                            {
                                goal_pose_vector.push_back(goal_pose);
                            }
                        }
                    }
                }
            }
        }

        if (find_product_succeeded)
        {
            ROS_INFO_THROTTLE(10, "joint_states position status:[%f, %f, %f, %f, %f, %f]", joint_states.position[0], joint_states.position[1], 
            joint_states.position[2], joint_states.position[3], joint_states.position[4], joint_states.position[5]); 

            if (once)
                {
                geometry_msgs::PoseStamped testPos = goal_pose_vector[0];

                moveToPos(&trajectory_as, testPos);
                once = false;
            }

        }
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
