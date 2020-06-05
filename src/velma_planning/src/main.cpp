#include "ros/ros.h"
#include <iostream>
#include <vector>
#include <costmap_2d/costmap_2d_ros.h>
#include <global_planner/planner_core.h>
#include <dwa_local_planner/dwa_planner_ros.h>
#include <teb_local_planner/teb_local_planner_ros.h>
#include <eband_local_planner/eband_local_planner_ros.h>
#include <tf2_ros/transform_listener.h>
#include "tf2_ros/buffer.h"
#include "velma_planning/make_plan.h"
//#include "simple_service/simple.srv"
//#include <simple_service/make_plan.h>

#include "nav_msgs/Odometry.h"
#include "geometry_msgs/Twist.h"
#include "geometry_msgs/Quaternion.h"
#include <tf2/LinearMath/Quaternion.h>
#include <rotate_recovery/rotate_recovery.h>
#include <cmath>

#define loc_planner teb_local_planner::TebLocalPlannerROS

std::unique_ptr<tf2_ros::Buffer> global_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_global;
std::unique_ptr<costmap_2d::Costmap2DROS> global_costmap;
std::unique_ptr<global_planner::GlobalPlanner> glob_planner;

std::unique_ptr<tf2_ros::Buffer> local_buffer;
std::unique_ptr<tf2_ros::TransformListener> tf_local;
std::unique_ptr<costmap_2d::Costmap2DROS> local_costmap;
//std::unique_ptr<teb_local_planner::TebLocalPlannerROS> local_planner;
//std::unique_ptr<dwa_local_planner::DWAPlannerROS> local_planner;
//std::unique_ptr<eband_local_planner::EBandPlannerROS> local_planner;
std::unique_ptr<loc_planner> local_planner;

//service and publisher
ros::ServiceServer plan_service;
ros::Publisher vel_publisher;


//service callback for the planner
bool planServiceCallback(velma_planning::make_plan::Request &req, velma_planning::make_plan::Response &res);

std::string planToPoint(geometry_msgs::PoseStamped &end_point);


void rotateRecovery();

double thetaFromQuat(geometry_msgs::Quaternion &orientation);




int main(int argc, char* argv[]){
    ros::init(argc, argv, "velma_planner");
    ros::NodeHandle node;

    global_buffer.reset(new tf2_ros::Buffer(ros::Duration(10), true));
    tf_global.reset(new tf2_ros::TransformListener(*global_buffer));
    global_costmap.reset(new costmap_2d::Costmap2DROS("global_costmap", *global_buffer));
    global_costmap->start();
    glob_planner.reset(new global_planner::GlobalPlanner("global_planner", global_costmap->getCostmap(), "map"));

    local_buffer.reset(new tf2_ros::Buffer(ros::Duration(10), true));
    tf_local.reset(new tf2_ros::TransformListener(*local_buffer));
    local_costmap.reset(new costmap_2d::Costmap2DROS("local_costmap", *local_buffer));


//    local_planner.reset(new teb_local_planner::TebLocalPlannerROS());
//    local_planner.reset(new dwa_local_planner::DWAPlannerROS());
//    local_planner.reset(new eband_local_planner::EBandPlannerROS());
    local_planner.reset(new loc_planner());
    local_planner->initialize("local_planner", local_buffer.get(), local_costmap.get());

    plan_service = node.advertiseService("velma_planner/go_to_pose", planServiceCallback);
    vel_publisher = node.advertise<geometry_msgs::Twist>("cmd_vel", 10);

    ROS_INFO("initiated planner");
    ros::spin();

    return 0;
}


bool planServiceCallback(velma_planning::make_plan::Request &req, velma_planning::make_plan::Response &res){
    tf2::Quaternion q;
    q.setRPY(0.0, 0.0, req.pose.theta);

    std::cout << "New setpoint: " << std::endl;
    std::cout << "x = " << req.pose.x << ", y = " << req.pose.y << ", yaw: " << req.pose.theta << std::endl;

    geometry_msgs::PoseStamped stpt;
    stpt.header.frame_id = "map";
    stpt.header.stamp = ros::Time::now();
    stpt.pose.position.x = req.pose.x;
    stpt.pose.position.y = req.pose.y;
    stpt.pose.position.z = 0.0;
    stpt.pose.orientation.x = q.getX();
    stpt.pose.orientation.y = q.getY();
    stpt.pose.orientation.z = q.getZ();
    stpt.pose.orientation.w = q.getW();

    try {
        res.result = planToPoint(stpt);
    } catch(...) {
        res.result = "Exception occured";
        return false;
    }

    return true;
}


std::string planToPoint(geometry_msgs::PoseStamped& setpoint){
    bool correctFinish = false;
    bool noPlanError = false;
    int noPlanCount = 0;
    ros::Rate rosRate(10);

    std::vector<geometry_msgs::PoseStamped> localPath;
    geometry_msgs::Twist twist;
    geometry_msgs::PoseStamped pose;
    geometry_msgs::PoseStamped start;

    pose.header.frame_id = "map";
    pose.header.stamp = ros::Time::now();
    start.header.frame_id = "map";
    start.header.stamp = ros::Time::now();

    if(!global_costmap->getRobotPose(start))
        return "Could not acquire robot position from global costmap";

    if(!glob_planner->makePlan(start, setpoint, localPath))
        return "Could not make global plan";

    if(!local_planner->setPlan(localPath))
        return "Could not pass global plan to local planner";

    glob_planner->publishPlan(localPath);

    while(ros::ok()){
        ROS_INFO("planner loop started");
        if(local_planner->isGoalReached()){
            std::cout << "goal reached" << std::endl;
            correctFinish = true;
            break;
        }

        if(!local_planner->computeVelocityCommands(twist)){
            // recovery situation - could not compute new vel cmd
            std::cout << "Could not compute next velocity message, executing recovery behaviour..." << std::endl;
            ++noPlanCount;
            switch(noPlanCount){
                case 1:
                    std::cout << "Try 1: Clearing local costmap and run simple rotate behaviour" << std::endl;
                    local_costmap->resetLayers();
                    //rotateRecoveryBehavior();
                    ros::spinOnce();
                    continue;
                case 2:
                    std::cout << "Try 2: Resetting global plan" << std::endl;
                    if(!global_costmap->getRobotPose(start))
                        return "Could not acquire robot position from global costmap";

                    if(!glob_planner->makePlan(start, setpoint, localPath))
                        return "Could not make global plan";

                    if(!local_planner->setPlan(localPath))
                        return "Could not pass global plan to local planner";

                    glob_planner->publishPlan(localPath);
                    break;
                default:
                    std::cout << "Failed to recover, abort mission" << std::endl;
                    noPlanError = true;
            };

            if(noPlanError)
                break;
        } else {
            noPlanCount = 0;
        }

        ROS_INFO("twist is %f, %f", twist.linear.x, twist.linear.y);
        vel_publisher.publish(twist);
        ros::spinOnce();
        rosRate.sleep();
    }

    geometry_msgs::Twist stopCommand;
    vel_publisher.publish(stopCommand);

    if(noPlanError){
        return "No local plan could be found";
    }

    if(correctFinish){
        return "Setpoint reached";
    }

    return "OK";
}


void rotateRecoveryBehavior(){
    geometry_msgs::Twist twist;
    ros::Rate rosRate(10);
    double startingTheta;
    geometry_msgs::PoseStamped start;
    start.header.frame_id = "map";
    start.header.stamp = ros::Time::now();

    if(!global_costmap->getRobotPose(start)){
        std::cout << "Rotate recovery behaviour failed: could not get robot position" << std::endl;
        return;
    }

    startingTheta = thetaFromQuat(start.pose.orientation);
    const double recoveryAngularVelocity = 0.2;
    twist.angular.z = recoveryAngularVelocity;
    const unsigned int maxTries = 15;
    for(unsigned n = 0; n < maxTries; ++n){
        vel_publisher.publish(twist);
        ros::spinOnce();
        rosRate.sleep();
    }
    do {
        if(!global_costmap->getRobotPose(start)){
            std::cout << "Rotate recovery behaviour failed: could not get robot position" << std::endl;
            return;
        }
        vel_publisher.publish(twist);
        ros::spinOnce();
        rosRate.sleep();
    }
    while( 0.01 < fabs( startingTheta-thetaFromQuat(start.pose.orientation) ) );
    twist.angular.z = 0.0;
    vel_publisher.publish(twist);
}

double thetaFromQuat(geometry_msgs::Quaternion &orientation)
{
    double roll, pitch, yaw;
    tf2::Quaternion quat( orientation.x, orientation.y, orientation.z, orientation.w );
    tf2::Matrix3x3 matrix(quat);
    matrix.getRPY(roll, pitch, yaw);
    return yaw;
}