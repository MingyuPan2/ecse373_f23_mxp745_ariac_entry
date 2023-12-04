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

#include <sstream>
#include <vector>
#include <iostream>

int order_count, shipment_count, product_count = 0;
std::vector <osrf_gear::Order> order_queue;  //queue for order (needed?)
ros::ServiceClient material_client;       //since ariac/material_locations is a service, it can be called using the same method as start_competition
void orderCallback(const osrf_gear::Order::ConstPtr& queue_order){
   ROS_INFO("Received order %d", ++order_count);
   order_queue.push_back(*queue_order); 

   shipment_count += queue_order->shipments.size();
    for (const auto& shipment : queue_order->shipments) {
        product_count += shipment.products.size();
    }
}

osrf_gear::LogicalCameraImage camera [6];
void cam1Callback(const osrf_gear::LogicalCameraImage::ConstPtr& camera1){
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

void getOrder(){
    for (const auto& order: order_queue) {
        ROS_INFO("Order: %s", order.order_id.c_str());

        for (const auto& shipment: order.shipments) {
            ROS_INFO("Shipment: %s", shipment.shipment_type.c_str());

            for (const auto& product: shipment.products) {

                osrf_gear::GetMaterialLocations material_locations;
                material_locations.request.material_type = product.type;
                material_client.call(material_locations);
                std::string bin = material_locations.response.storage_units.front().unit_id;

                ROS_INFO("Product: %s", product.type.c_str());
                ROS_INFO("BIn: %s", bin.c_str());

            }
        }
    }
    order_queue.clear();
}

sensor_msgs::JointState joint_states;        //declare a variable for storing the current state of joints of the robot
void jointStateCallback(const sensor_msgs::JointState::ConstPtr& joint_msg){  //simple callback that stores joint_states to a global for use inside main node loop
    joint_states = *joint_msg;
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

    ros::Subscriber cam1 = n.subscribe("/ariac/logical_camera_bin1", 100, cam1Callback);
    ros::Subscriber cam2 = n.subscribe("/ariac/logical_camera_bin2", 100, cam2Callback);
    ros::Subscriber cam3 = n.subscribe("/ariac/logical_camera_bin3", 100, cam3Callback);    
    ros::Subscriber cam4 = n.subscribe("/ariac/logical_camera_bin4", 100, cam4Callback);
    ros::Subscriber cam5 = n.subscribe("/ariac/logical_camera_bin5", 100, cam5Callback);
    ros::Subscriber cam6 = n.subscribe("/ariac/logical_camera_bin6", 100, cam6Callback); 

    osrf_gear::GetMaterialLocations material_locations;
    ros::ServiceClient material_client = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations");



    tf2_ros::Buffer tfBuffer;                                          //declare transformation buffer
    tf2_ros::TransformListener tfListener(tfBuffer);                   //listener that listens to the tf and tf_static topics and to update the buffer

    ros::ServiceClient ik_client = n.serviceClient<ik_service::PoseIK>("/pose_ik");
    ros::service::waitForService("/pose_ik", 10);
 
    ros::AsyncSpinner spinner(1); // Use 1 thread
    spinner.start();              // A spinner makes calling ros::spin() unnecessary.

    while (ros::ok()) {       

        if (!order_queue.empty()) {
            getOrder();
        }

        ROS_INFO_STREAM_THROTTLE(10, "Current Joint States = " << joint_states.position[0]<< ", " << joint_states.position[1] << ", " << joint_states.position[2] << ", " << joint_states.position[3]<< ", " << joint_states.position[4] << ", " << joint_states.position[5] << ", time=" << ros::Time::now());
        
        loop_rate.sleep();   
    }
    return 0;
}