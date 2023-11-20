#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "std_srvs/SetBool.h"
#include "osrf_gear/Order.h"                  // contains shipment, product information
#include "osrf_gear/GetMaterialLocations.h"   //indicate which bin the product is & position of product
#include "osrf_gear/LogicalCameraImage.h"     // logical camera output
#include "tf2_ros/transform_listener.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.h"
#include "geometry_msgs/TransformStamped.h"
#include "geometry_msgs/PoseStamped.h"
#include <sstream>
#include <vector>

ros::ServiceClient material_locations;       //since ariac/material_locations is a service, it can be called using the same method as start_competition
std::vector <osrf_gear::Order> order_queue;  // queue for order (?not sure)

void orderCallback(const osrf_gear::Order::ConstPtr& order)           //void callback function for getting the shipment information.
{
    for (const auto& shipment:order->shipments)                       //loop, iterate all shipments in one order, access "shipments" member of "Order object", ("auto" infer type of shipment!)
    {
        ROS_INFO_STREAM("Shipment Type: " << shipment.shipment_type); //log type of shipment, order_#_shipment_#, output once for one order
        for (const auto& product:shipment.products)                   //"shipment" instance of osrc_gear/Order, shipemtn.product=list of products in shipment, loop each shipment
        { 
            ROS_INFO_STREAM("Product:" << product.type);              //log product's type
            order_queue.push_back(*order);                            //put order in queue as vector
            osrf_gear::GetMaterialLocations location;                 //object "location" to call service ariac/material_locations
            location.request.material_type = product.type;            //assign product.type to location.req.., ask /ariacl/material_location service
            if (material_locations.call(location))                    //request location using ariac/material_locations service for all products
            {
                ROS_INFO("For product (%s) is in", product.type.c_str()); //disp product type
                for (const auto& location: location.response.storage_units) //get response of service, specifically the storage unit
                {
                    ROS_INFO("Storage Unit: %s", location.unit_id.c_str());
                }
            }            
            ROS_INFO_STREAM("Position: x, y, z = " << product.pose.position.x << "," << product.pose.position.y << "," << product.pose.position.z);
            ROS_INFO_STREAM("Quaternion orientation: x, y, z, w = " << product.pose.orientation.x << "," << product.pose.orientation.y << "," << product.pose.orientation.z << "," << product.pose.orientation.w); 
            ROS_INFO_STREAM("  ");      
        }
    }
}

/*This callback function can return the position of all piston rod, gasket and gear parts' location in the camera's frame. Not needed(?)

void logicalCameraCallback(const osrf_gear::LogicalCameraImage::ConstPtr& camera) {
    for (const auto& model: camera->models) {
        ROS_INFO_STREAM("Product: " << model.type << ", Position: x = " << model.pose.position.x << ", y = " << model.pose.position.y << ", z = " << model.pose.position.z);
    }}

*/

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ariac_competition_node");
    ros::NodeHandle n;
    ros::Subscriber ordersub = n.subscribe<osrf_gear::Order>("/ariac/orders", 100, orderCallback);

    std_srvs::Trigger begin_comp;                                      //request to begin comp
    std_srvs::SetBool my_bool_var;                                     //declare service message type of my_bool_var (request,response?)
    tf2_ros::Buffer tfBuffer;                                          //declare transformation buffer
    tf2_ros::TransformListener tfListener(tfBuffer);                   //listener that listens to the tf and tf_static topics and to update the buffer

    my_bool_var.request.data = true;                                   //set true for service msg
    int service_call_succeeded;                                        //declare variable to capture service call success

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

    material_locations = n.serviceClient<osrf_gear::GetMaterialLocations>("/ariac/material_locations"); // ??

    std::vector <ros::Subscriber> camera_subscribers;
    std::vector <std::string> logical_cameras = {"/ariac/logical_camera_agv1", //vector of all logical cameras' topics
                                                "/ariac/logical_camera_agv2",
                                                "/ariac/logical_camera_bin1",
                                                "/ariac/logical_camera_bin2",
                                                "/ariac/logical_camera_bin3",
                                                "/ariac/logical_camera_bin4",
                                                "/ariac/logical_camera_bin5",
                                                "/ariac/logical_camera_bin6",};

    /*
    for (const auto& camera_topics:logical_cameras) {  //subscribe to all logical camera topics
        ros::Subscriber camera_location = n.subscribe<osrf_gear::LogicalCameraImage>(camera_topics, 1, logicalCameraCallback);
        camera_subscribers.push_back(camera_location);
    } */
 
    for (const auto& camera_topics:logical_cameras) {                //loop, set up to subscribe to all topics in the vector of logical_cameras
        ros::Subscriber camera_location = n.subscribe<osrf_gear::LogicalCameraImage>(camera_topics, 1, [&](const osrf_gear::LogicalCameraImage::ConstPtr& camera) { //WHY?????
            geometry_msgs::PoseStamped part_pose, goal_pose;         //declare 3d pose info for part and goal, "Stamped" means timestamped
            geometry_msgs::TransformStamped tfStamped;               //tfStamped of type TransformStamped

            for (const auto& model:camera->models) {                 //loop, loop through the model for all outputs of camera
                if (model.type == "piston_rod_part") {               //if detected type is equal to the desired part
                    part_pose.pose = model.pose;                     //assign pose member of model to pose member of part_pose
                    
                    try {tfStamped = tfBuffer.lookupTransform("arm1_base_link", "logical_camera_bin4_frame", ros::Time(0.0), ros::Duration(1.0)); // to frame, from frame, how recent, how long to wait for transform
                        ROS_DEBUG("Transform to [%s] from [%s]", tfStamped.header.frame_id.c_str(), tfStamped.child_frame_id.c_str());
                    } 
                    catch (tf2::TransformException &ex) {
                        ROS_ERROR("%s", ex.what());
                    }

                    tf2::doTransform(part_pose, goal_pose, tfStamped);  //input part_pose, output goal_pose, using transformation information from tfStamped

                    goal_pose.pose.position.z += 0.10;              //avoid collision
                    goal_pose.pose.orientation.w = 0.707;
                    goal_pose.pose.orientation.x = 0.0;
                    goal_pose.pose.orientation.y = 0.707;
                    goal_pose.pose.orientation.z = 0.0;

                    ROS_INFO_STREAM("Goal Pose Position: x = " << goal_pose.pose.position.x << ", y = " << goal_pose.pose.position.y << ", z = " << goal_pose.pose.position.z);
                    ROS_INFO_STREAM("Goal Pose Orientation: x = " << goal_pose.pose.orientation.x << ", y = " << goal_pose.pose.orientation.y << ", z = " << goal_pose.pose.orientation.z << ", w = " << goal_pose.pose.orientation.w);
                }
            }
        });
        camera_subscribers.push_back(camera_location);
    }
 
    while (ros::ok())
    {
        ros::spinOnce();
    }
    return 0;
}