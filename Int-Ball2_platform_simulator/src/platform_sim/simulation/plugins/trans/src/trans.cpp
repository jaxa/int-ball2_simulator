
#include "ros/ros.h"
  
#include "math.h"
#include "gazebo_msgs/ModelStates.h"
#include "tf/transform_broadcaster.h"

#include <string>
#include <iostream>
#include <sstream>


std::string model_name="";
std::string world_frame="";
std::string base_frame="";
int updateFreqHz=20;

void models_callback(const gazebo_msgs::ModelStates& model_msg){
    static tf::TransformBroadcaster br;
	int model_size=model_msg.name.size();
	for(int i=0;i<model_size;i++){
		if(model_msg.name[i]==model_name){
	        tf::Transform transform;
    	    tf::poseMsgToTF(model_msg.pose[i],transform);
        	br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), world_frame, base_frame));
		}
	}
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "trans");
	ros::NodeHandle n;
    ros::NodeHandle pn("~");

    //rosparam
	pn.getParam("model_name",  model_name);
    pn.getParam("world_frame", world_frame);
    pn.getParam("base_frame",  base_frame);
    pn.getParam("updateFreqHz",updateFreqHz);

	ROS_INFO("[trans] model_name:%s, world_frame:%s, base_frame:%s, updateFreqHz:%d", 
		model_name.c_str(), world_frame.c_str(), base_frame.c_str(), updateFreqHz);

	//subscriibe
	ros::Subscriber joy_sub   = n.subscribe("/gazebo/model_states", 10, models_callback);
		
	ros::Rate loop_rate(updateFreqHz); 
	while (ros::ok()){
		ros::spinOnce();
		loop_rate.sleep();
	} 
 	return 0;
}
