#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"                       //def of service type ""trigger. contains a request & response (?)
#include "std_srvs/SetBool.h"                       //"bool success" indicate successful run of triggered service, "string message" to inform "error messages", etc.

#include "osrf_gear/Order.h"                        //contains shipment, product information
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/GetMaterialLocations.h"         //indicate which bin the product is & position of product
#include "osrf_gear/LogicalCameraImage.h"           //logical cameras' output

#include "tf2_ros/transform_listener.h"             //def: This class provides an easy way to request and receive coordinate frame transform information. 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"    //def: This package allows to convert ros messages to tf2 messages and to retrieve data from ros messages.
#include "geometry_msgs/TransformStamped.h"         //"this message type specifically represents a transformation between two coordinate frames which include information about the transformation, 
                                                    //,translation and rotation as well as the timestamps and frame IDs for both the parent and child coordinate frames"
#include "geometry_msgs/PoseStamped.h"              //3d orientation&rotation, includes timestamp and frame id info.
#include "sensor_msgs/JointState.h"                  //"holds data to describe the state of a set of torque controlled joints". str name - flo position,flo velocity,flo effort 
#include "ik_service/PoseIK.h"
#include "geometry_msgs/Pose.h"

#include "trajectory_msgs/JointTrajectory.h"
#include "actionlib/client/simple_action_client.h"
#include "actionlib/client/terminal_state.h"
#include "control_msgs/FollowJointTrajectoryAction.h"

#include <sstream>
#include <vector>
#include <iostream>

int order_count, shipment_count, product_count, count = 0; //to get the function to be in a loop later, the values of order_count and shipment_count can be used to find
                                                        //the name of every product in a shipment in an order, i think..
std::vector <osrf_gear::Order> order_queue;  //queue for order (needed)
    
void orderCallback(const osrf_gear::Order::ConstPtr& queue_order){      //order callback
   ROS_INFO("order count %d", ++order_count);                           //test order number after each order is announced
   order_queue.push_back(*queue_order);                             

    shipment_count += queue_order->shipments.size();
            ROS_INFO("shipment count %d", shipment_count);
        for (const auto& shipment : queue_order->shipments) {
            product_count += shipment.products.size();
            ROS_INFO("product count %d", product_count);
        }
    }

osrf_gear::LogicalCameraImage camera [6];                               //"camera" array with six elements to store six instances of LogicalCameraImage(s)
void cam1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera1){  //6 camera callbacks. the quality control cameras and agv(?) cameras are not used
    camera[0] = *camera1;
    }
void cam2Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera2){
    camera[1] = *camera2;
    }
void cam3Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera3){
    camera[2] = *camera3;
    }
void cam4Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera4){
    camera[3] = *camera4;
    }
void cam5Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera5){
    camera[4] = *camera5;
    }
void cam6Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera6){
    camera[5] = *camera6;
    }

void getAllOrders(){                            //loop to get all products in one shipment, works
    for (const auto& order: order_queue) {
        ROS_INFO("Order: %s", order.order_id.c_str());

        for (const auto& shipment: order.shipments) {
            ROS_INFO("Shipment: %s", shipment.shipment_type.c_str());

            for (const auto& product: shipment.products) {

                ROS_INFO("Product: %s", product.type.c_str());

            }
        }
    }
    order_queue.clear();
}

std::string getFirstProduct(){
        std::string first_product = order_queue[0].shipments[0].products[0].type;
        ROS_INFO("First product: %s", first_product.c_str());
        return first_product;    
}

std::string getFirstBin(std::string first_product, ros::ServiceClient material_client){
    std::string unit;
    osrf_gear::GetMaterialLocations materialLocations;
    materialLocations.request.material_type = first_product;
    if (material_client.call(materialLocations)) {
        for (const auto& storage_unit : materialLocations.response.storage_units) {
            if (storage_unit.unit_id == "bin") {
                unit = storage_unit.unit_id;
                ROS_INFO("The first product is in bin %s", unit.c_str());
                return unit;
            }
        }
    }
}

geometry_msgs::Pose pose;

geometry_msgs::Pose productPose (std::string first_product, std::string unit){
    std::vector<osrf_gear::Model> products;
    if (unit == "bin4"){
        products = camera[3].models;
    }
    else if (unit == "bin5"){
        products = camera[4].models;
    }
    else if (unit == "bin6"){
        products = camera[5].models;
    }

    geometry_msgs::Pose pose;
    for (const osrf_gear::Model& product : products){
        if (product.type == first_product){
            product.pose == pose;
            ROS_INFO("Product pose (cam frame): POS: X=%f, Y=%f, Z=%f, ORI: X=%f, Y=%f, Z=%f, W=%f",pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    }
    return pose;
}

void ikSolutionsCallback(ros::ServiceClient ik_client, geometry_msgs::PoseStamped goal_pose){
    
}

sensor_msgs::JointState joint_states;        //declare a variable for storing the current state of joints of the robot
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){  //simple callback that stores joint_states to a global for use inside main node loop
    joint_states = *joint_msg;
}

double T_pose[4][4], T_des[4][4];   // Instantiate variables for use with the kinematic system.
double q_pose[6], q_des[8][6];

trajectory_msgs::JointTrajectory desired;

trajectory_msgs::JointTrajectory jointTrajectories(trajectory_msgs::JointTrajectory joint_trajectory){
    // Fill out the joint trajectory header.
    // Each joint trajectory should have an non-monotonically increasing sequence number.
    joint_trajectory.header.seq = count++;
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
    joint_trajectory.joint_names.clear();
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");
    
    joint_trajectory.points.resize(2);      // Set a start and end point.
    return joint_trajectory;
}

void jointMovementCallback(geometry_msgs::PoseStamped movement){
    trajectory_msgs::JointTrajectory joint_trajectory;
    joint_trajectory = jointTrajectories(joint_trajectory);
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());
        for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
            for (int indz = 0; indz < joint_states.name.size(); indz++) {
                if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                        joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                    break;
                    }
                }
            }
            
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);    // When to start (immediately upon receipt).

    int q_des_indx = 0;                                                 // Must select which of the num_sols solutions to use. Just start with the first.

    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());   // Set the end point for the movement

    joint_trajectory.points[1].positions[0] = joint_states.position[1]; // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.

    //get num of solutions

    for (int indy = 0; indy < 6; indy++) {              // The actuators are commanded in an odd order, enter the joint positions in the correct positions
        joint_trajectory.points[1].positions[indy + 1] = q_sols[q_sols_indx][indy];
    }

    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);    // How long to take for the movement.
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_competition_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber order_subscriber = n.subscribe("/ariac/orders", 100, orderCallback);
    order_queue.clear();
    ros::Subscriber joint_subscriber = n.subscribe("/ariac/arm1/joint_states", 10, jointStateCallback); //sub to ariac/arms/joint_states for remapped positions.
 
    std_srvs::Trigger begin_comp;                                      //based on lab, request to begin comp
    std_srvs::SetBool my_bool_var;                                     //declare service message type of my_bool_var (request,response?)
    my_bool_var.request.data = true;                                   //set true for service msg, based on lab
    int service_call_succeeded;                                        //declare variable to capture service call success, based on lab
    ros::ServiceClient begin_client = n.serviceClient<std_srvs::Trigger>("/ariac/start_competition"); //start competition
    service_call_succeeded = begin_client.call(begin_comp);            //call service ariac/start_comp. Use "begin_client" to call service specified by "begin_comp". "service_call_succ.." logs result
    if (service_call_succeeded)                                        //if service_call_request captured any info
    {
        if (begin_comp.response.success)                               //if "begin_comp"'s "success" is true
        {
            ROS_INFO("Competition started successfully: %s", begin_comp.response.message.c_str());
        }
        else
        {
            ROS_WARN("Competition service returned failure: %s", begin_comp.response.message.c_str()); //return failture if competition already started and node was ran
        }
    }
    else
    {
        ROS_ERROR("Competition service call failed! Goodness Gracious!!"); //return if service did not pop up yet
    } 

    ros::Subscriber cam1 = n.subscribe("/ariac/logical_camera_bin1", 10, cam1Callback);
    ros::Subscriber cam2 = n.subscribe("/ariac/logical_camera_bin2", 10, cam2Callback);
    ros::Subscriber cam3 = n.subscribe("/ariac/logical_camera_bin3", 10, cam3Callback);    
    ros::Subscriber cam4 = n.subscribe("/ariac/logical_camera_bin4", 10, cam4Callback);
    ros::Subscriber cam5 = n.subscribe("/ariac/logical_camera_bin5", 10, cam5Callback);
    ros::Subscriber cam6 = n.subscribe("/ariac/logical_camera_bin6", 10, cam6Callback); 

    tf2_ros::Buffer tfBuffer;                                          //declare transformation buffer
    tf2_ros::TransformListener tfListener(tfBuffer);                   //listener that listens to the tf and tf_static topics and to update the buffer
    
    osrf_gear::GetMaterialLocations materialLocations;
    ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");

    // std::string product_type = order_queue[0].shipments[0].products[0].type;
    //  ROS_INFO("First order is: %s", product_type.c_str());     
    // material_locations.request.material_type = product_type;
    // std::string bin = material_locations.response.storage_units[0].unit_id;
    // ROS_INFO("First order is in %s", bin.c_str());

    ros::ServiceClient ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");
    ros::service::waitForService("/pose_ik", 10);
 
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();              // A spinner makes calling ros::spin() unnecessary.

    while (ros::ok()) {       

            //getAllOrders();
            //getFirstProduct();
            //getFirstBin();

            if (!order_queue.empty()){              // Retrieve the transformation
                geometry_msgs::TransformStamped tfStamped;
                try {                               // tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent","how_long_to_wait_for_transform")
                    tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame",
                    ros::Time(0.0), ros::Duration(1.0));
                    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
                    tfStamped.child_frame_id.c_str()); 
                } 
                catch (tf2::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
                }
            }

            geometry_msgs::PoseStamped part_pose, goal_pose;           //declare 3d pose info for part and goal, "Stamped" means timestamped
            
            part_pose.pose = productPose; //CHANGE & ADD

            tf2::doTransform(part_pose, goal_pose, tfStamped);
            goal_pose.pose.position.z += 0.10;              //avoid collision
            goal_pose.pose.orientation.w = 0.707;
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.707;
            goal_pose.pose.orientation.z = 0.0;

            ROS_INFO_STREAM("Goal Pose Position: x = " << goal_pose.pose.position.x << ", y = " << goal_pose.pose.position.y << ", z = " << goal_pose.pose.position.z);
            ROS_INFO_STREAM("Goal Pose Orientation: x = " << goal_pose.pose.orientation.x << ", y = " << goal_pose.pose.orientation.y << ", z = " << goal_pose.pose.orientation.z << ", w = " << goal_pose.pose.orientation.w);
            ROS_INFO_STREAM(" ");

        ROS_INFO_STREAM_THROTTLE(10, "Current Joint States = " << joint_states.position[0]<< ", " << joint_states.position[1] << ", " << joint_states.position[2] << ", " << joint_states.position[3]<< ", " << joint_states.position[4] << ", " << joint_states.position[5] << ", time=" << ros::Time::now());
        
        loop_rate.sleep();   
    }
    return 0;
}