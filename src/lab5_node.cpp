#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"                       //def of service type ""trigger. contains a request & response (?)
#include "std_srvs/SetBool.h"                       //"bool success" indicate successful run of triggered service, "string message" to inform "error messages", etc.

//header files for all osrf_gears
#include "osrf_gear/Order.h"                        //contains shipment, product information
#include "osrf_gear/Shipment.h"
#include "osrf_gear/Product.h"
#include "osrf_gear/VacuumGripperControl.h"         //used to controll the "open" & "close" states of the gripper
#include "osrf_gear/GetMaterialLocations.h"         //indicate which bin the product is & position of product
#include "osrf_gear/LogicalCameraImage.h"           //logical cameras' output

//header files for transformations
#include "tf2_ros/transform_listener.h"             //def: This class provides an easy way to request and receive coordinate frame transform information. 
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"    //def: This package allows to convert ros messages to tf2 messages and to retrieve data from ros messages.
#include "geometry_msgs/TransformStamped.h"         //"this message type specifically represents a transformation between two coordinate frames which include information about the transformation, 
                                                    //,translation and rotation as well as the timestamps and frame IDs for both the parent and child coordinate frames"
#include "geometry_msgs/PoseStamped.h"              //3d orientation&rotation, includes timestamp and frame id info.
#include "sensor_msgs/JointState.h"                  //"holds data to describe the state of a set of torque controlled joints". str name - flo position,flo velocity,flo effort 
#include "ik_service/PoseIK.h"
#include "geometry_msgs/Pose.h"

//header files for creating joint trajectories
#include "trajectory_msgs/JointTrajectory.h"        //header file used to generate joint trajectories
#include "control_msgs/FollowJointTrajectoryAction.h" //action server message type
#include "actionlib/client/simple_action_client.h"    //action server header 
#include "actionlib/client/terminal_state.h"          //action server header 

//other includes
#include <sstream>
#include <vector>
#include <iostream>

int order_count, shipment_count, product_count, count = 0; //to get the function to be in a loop later, the values of order_count and shipment_count can be used to find
                                                          //the name of every product in a shipment in an order, i think..
std::vector <osrf_gear::Order> order_queue; //data vector, to queue for order (needed)
    
void orderCallback(const osrf_gear::Order::ConstPtr& queue_order){      //order callback,works
   ROS_INFO("order count %d", ++order_count); //test order number after each order is announced
   order_queue.push_back(*queue_order);                             

    shipment_count += queue_order->shipments.size(); //test output numbers to be implemented in future code,works
            ROS_INFO("shipment count %d", shipment_count);
        for (const auto& shipment : queue_order->shipments) {
            product_count += shipment.products.size();
            ROS_INFO("product count %d", product_count);
        }
    }

osrf_gear::LogicalCameraImage camera [6];  //"camera" array with six elements to store six instances of LogicalCameraImage(s)
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

void getAllOrders(){  //void to be called in int main, loop to get all products in one shipment, may not be necessary, works
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

std::string getFirstProduct(){  //new idea, declare function to return first product as string type,works
        std::string first_product = order_queue[0].shipments[0].products[0].type; //access the first product "type" of the first shipment of the first order.
        ROS_INFO("First product: %s", first_product.c_str());
        return first_product;   //return as string 
}

//return where the first product is according to bin
std::string getFirstBin(std::string first_product, ros::ServiceClient material_client){ //function takes two parameters, string first_product & instance of ros::ServiceCliend
    std::string unit;   
    osrf_gear::GetMaterialLocations materialLocations;  //declare object of type GetMaterialLocations, request material locations information
    materialLocations.request.material_type = first_product;    //->first_product was set to be the field of material_type, so first bin can be matchd up to first product (??)
    if (material_client.call(materialLocations)) {
        for (const auto& storage_unit : materialLocations.response.storage_units) {
            if (storage_unit.unit_id == "bin") {    //exclude "belt" since belt was not needed
                unit = storage_unit.unit_id;
                ROS_INFO("The first product is in bin %s", unit.c_str());
                return unit;
            }
        }
    }
}

//get pose in camera's frame of first product
geometry_msgs::Pose productPose (std::string first_product, std::string unit){  //take two paramersrs, product's name & its bin location
    std::vector<osrf_gear::Model> products;
    if (unit == "bin4"){        //if the unit name matches, then the camera vector above that bin will be used
        products = camera[3].models;
    }
    else if (unit == "bin5"){
        products = camera[4].models;
    }
    else if (unit == "bin6"){
        products = camera[5].models;
    }

    geometry_msgs::Pose pose;
    for (const osrf_gear::Model& product : products){   //foreach loop(?), iterate over "products" vector
        if (product.type == first_product){     //if product type matches, get its pose
            product.pose == pose;
            ROS_INFO("Product pose (cam frame): POS: X=%f, Y=%f, Z=%f, ORI: X=%f, Y=%f, Z=%f, W=%f",pose.position.x, pose.position.y, pose.position.z, pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w);
        }
    }
    return pose;
}

//return desired joint solution, hopefully
ik_service::JointSolutions ikSolutionsCallback(ros::ServiceClient ik_client, geometry_msgs::PoseStamped goal_pose){
    ik_service::PoseIK ik_pose; //based on lab6
    ik_pose.request.part_pose = goal_pose.pose; 
    int sols, dsols, joint; //dsols = desired solution
    if (ik_client.call(ik_pose)) {
        sols = ik_pose.response.num_sols;
        ROS_INFO("ik_service returned %i solutions", sols);
        for(dsols = 0; dsols<sols; dsols++){ //since only one solution is usable, loop through each solution to see which is feasible
            
            //i do not know how to setup the limits for the joint angles to single out the one solution, but should go something liek:
            for (joint=0; joint<6; joint ++){
                /*
                  if the responded joint angle of each joint of each 
                  solution does not comply to theangle constraints, the code will look like something below
                
                  if   (someJointConstraint - ik_pose.response.joint_solutions[dsols].joint_angles[joint] = 0)
                  then (it is not the solution i want)

                  if (a solutions is found){
                    ROS_INFO("SOlution %i has joint angles: %f, %f, %f, %f, %f, %f", dsols, ik_pose.response.joint_solutions[dsols].joint_angles[0],
                    ik_pose.response.joint_solutions[dsols].joint_angles[1],
                    ik_pose.response.joint_solutions[dsols].joint_angles[2], 
                    ik_pose.response.joint_solutions[dsols].joint_angles[3],
                    ik_pose.response.joint_solutions[dsols].joint_angles[4],
                    ik_pose.response.joint_solutions[dsols].joint_angles[5]
                    );
                  }

                  return ik_pose.response.joint_solutions[dsols];
                */
            }

        }
    }
}

sensor_msgs::JointState joint_states;        //declare a variable for storing the current state of joints of the robot
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){  //simple callback that stores joint_states to a global for use inside main node loop
    joint_states = *joint_msg;
}

/*
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

    else{
        ROS_ERROR("Failed to find any solutions");
    }
}
*/

//A simple callback to operate the grippeer
void gripperStatus (ros::ServiceClient gripper){
    osrf_gear::VacuumGripperControl gripperstatus;
    gripperstatus.request.enable=true;
    if (gripper.call(gripperstatus)){
        if(gripperstatus.response.success){
            ROS_INFO("Gripper ON");
        }
        else{
            ROS_INFO("Gripper OFF");
        }
    }
}

//callback to move joint 
void jointMovement(ros::Publisher jointTrajectory, ik_service::JointSolutions jointAngles){
    trajectory_msgs::JointTrajectory joint_trajectory;  // Declare a variable for generating and publishing a trajectory.

    int count;    
    joint_trajectory.header.seq = count++;  // Fill out the joint trajectory header, each joint trajectory should have an non-monotonically increasing sequence number.
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
    
    joint_trajectory.joint_names.clear();   // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");

    joint_trajectory.points.resize(2);  // Set a start and end point.    
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());   // Set the start point to the current position of the joints from joint_states.
    
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
        for (int indz = 0; indz < joint_states.name.size(); indz++) {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);    // When to start (immediately upon receipt).
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());   // Set the end point for the movement    
    joint_trajectory.points[1].positions[0] = joint_states.position[1];   // Set the linear_arm_actuator_joint from joint_states as it is not part of the inverse kinematics solution.

    for (int indy = 0; indy < 6; indy++) {   // The actuators are commanded in an odd order, enter the joint positions in the correct positions
        joint_trajectory.points[1].positions[indy + 1] = jointAngles.joint_angles[indy];
    }
    
    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);    // How long to take for the movement.
    jointTrajectory.publish(joint_trajectory);  //publish the joint trajectory (int main)
}

//i was also told that the arm needs to move back to its original position, so the same method can be used as above. Since the jointstate was stored above, sensor_msgs::JointState can be used
void jointReturn(ros::Publisher jointTrajectory, sensor_msgs::JointState storedJoints){
    trajectory_msgs::JointTrajectory joint_trajectory;  // Declare a variable for generating and publishing a trajectory.

    int count;    
    joint_trajectory.header.seq = count++;  // Fill out the joint trajectory header, each joint trajectory should have an non-monotonically increasing sequence number.
    joint_trajectory.header.stamp = ros::Time::now(); // When was this message created.
    joint_trajectory.header.frame_id = "/world"; // Frame in which this is specified.
    
    joint_trajectory.joint_names.clear();   // Set the names of the joints being used. All must be present.
    joint_trajectory.joint_names.push_back("linear_arm_actuator_joint");
    joint_trajectory.joint_names.push_back("shoulder_pan_joint");
    joint_trajectory.joint_names.push_back("shoulder_lift_joint");
    joint_trajectory.joint_names.push_back("elbow_joint");
    joint_trajectory.joint_names.push_back("wrist_1_joint");
    joint_trajectory.joint_names.push_back("wrist_2_joint");
    joint_trajectory.joint_names.push_back("wrist_3_joint");

    joint_trajectory.points.resize(2);  // Set a start and end point.    
    joint_trajectory.points[0].positions.resize(joint_trajectory.joint_names.size());   // Set the start point to the current position of the joints from joint_states.
    
    for (int indy = 0; indy < joint_trajectory.joint_names.size(); indy++) {
        for (int indz = 0; indz < joint_states.name.size(); indz++) {
            if (joint_trajectory.joint_names[indy] == joint_states.name[indz]) {
                joint_trajectory.points[0].positions[indy] = joint_states.position[indz];
                break;
            }
        }
    }
    
    joint_trajectory.points[0].time_from_start = ros::Duration(0.0);    // When to start (immediately upon receipt).
    joint_trajectory.points[1].positions.resize(joint_trajectory.joint_names.size());   // Set the end point for the movement    

    /*
    
    */

    joint_trajectory.points[1].time_from_start = ros::Duration(1.0);    // How long to take for the movement.
    jointTrajectory.publish(joint_trajectory);  //publish the joint trajectory (int main)
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_competition_node");
    ros::NodeHandle n;
    ros::Rate loop_rate(10);

    ros::Subscriber order_subscriber = n.subscribe("/ariac/orders", 100, orderCallback);    
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
    ros::Publisher jointTrajectory = n.advertise<trajectory_msgs::JointTrajectory>("/ariac/arm1/arm/command", 100);
    ros::ServiceClient gripper = n.serviceClient<osrf_gear::VacuumGripperControl>("/ariac/arm1/gripper/control");

    order_queue.clear();

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

            //getAllOrders();   //test to get all orders
            //getFirstProduct();    //test to get FirstProduct
            //getFirstBin();    //test to get bin of first product -- segmentation fault (core dumped) ????

        if (!order_queue.empty()){              
                /*
                // Retrieve the transformation
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
            */
            std::string first_product = getFirstProduct();  //the type of the first product
            std::string unit = getFirstBin(first_product, material_client); //bin id what the first product is in
            geometry_msgs::Pose pose = productPose(first_product, unit);    //pose of the product in the camera's frame

            
            geometry_msgs::TransformStamped tfStamped; // Retrieve the transformation, all copied from lab5
            try {
                tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame",
                ros::Time(0.0), ros::Duration(1.0));
                    ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(),
                tfStamped.child_frame_id.c_str());
            } 
            catch (tf2::TransformException &ex) {
                    ROS_ERROR("%s", ex.what());
            } // tf2_ross::Buffer.lookupTransform("to_frame", "from_frame", "how_recent", "how_long_to_wait_for_transform");

            geometry_msgs::PoseStamped part_pose, goal_pose;    // Create variables
            part_pose.pose = pose;  // Copy pose from the logical camera.
            tf2::doTransform(part_pose, goal_pose, tfStamped);
            
            goal_pose.pose.position.z += 0.10; // Add height to the goal pose 10 cm above the part
            goal_pose.pose.orientation.w = 0.707;   // Tell the end effector to rotate 90 degrees around the y-axis (in quaternions...).
            goal_pose.pose.orientation.x = 0.0;
            goal_pose.pose.orientation.y = 0.707;
            goal_pose.pose.orientation.z = 0.0;
            ROS_INFO("Product Pose in robot frame = POS: x= %f, y=%f, z=%f,ORI: x=%f, y=%f, z=%f, w=%f.", goal_pose.pose.position.x, goal_pose.pose.position.y, \
                     goal_pose.pose.position.z, goal_pose.pose.orientation.x, goal_pose.pose.orientation.y, goal_pose.pose.orientation.z, goal_pose.pose.orientation.w);

            sensor_msgs::JointState storedJoints = joint_states; //the initial joint states are stored for moving the entire arm back to its original position

            /*ROS_INFO_STREAM_THROTTLE(10, "Current Joint States = " << joint_states.position[0]<< ", " << joint_states.position[1] << ", " 
              << joint_states.position[2] << ", " << joint_states.position[3]<< ", " << joint_states.position[4] << ", " << joint_states.position[5] << ", time=" << ros::Time::now());
            */
           ik_service::JointSolutions desiredAngles = ikSolutionsCallback(ik_client, goal_pose);
           jointMovement(desiredAngles, jointTrajectory);
           gripperStatus(gripper);
           jointReturn(jointTrajectory);          
        }
        loop_rate.sleep();     
    }
    return 0;
}