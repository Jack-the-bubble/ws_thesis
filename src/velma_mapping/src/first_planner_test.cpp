 //
// Created by silver on 03.05.2020.
//


#include "ros/ros.h"
#include <iostream>
#include "std_srvs/SetBool.h"
#include "nav_core/base_global_planner.h"

bool respond(std_srvs::SetBoolRequest &req, std_srvs::SetBoolResponse &rsp){
    ROS_INFO("received a request");
    rsp.message = true;
    rsp.success = true;
    return true;
}


int main(int argc, char **argv){
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;

    ros::ServiceServer service = n.advertiseService("simple_respond", respond);


    std::cout<<"Hello world"<< std::endl;
    ros::spin();

    return 0;

}