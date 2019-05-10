#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <pthread.h>

#include <ros/ros.h>
#include <yeebot_commute/JointInfo.h>
#include <sensor_msgs/JointState.h>

bool transJointValues(yeebot_commute::JointInfo::Request &req,
                        yeebot_commute::JointInfo::Response &res);
void retriveJointValues(const sensor_msgs::JointState& joint_state);

pthread_mutex_t mutex;
std::vector<double> position;
std::vector<double> velocity;
std::vector<double> effort;
std::vector<std::string> name;


//service name:joint_states_srv
//joint states topic name: argv[1], default is "joint_states"
int main(int argc,char **argv){
    ros::init(argc,argv,"get_joint_states");
    ros::NodeHandle nh;
    ros::WallDuration sleep_time(5.0);
    
    std::string joint_states_topic="joint_states";
    
    if(argc>1){
    	joint_states_topic=argv[1];
    }

    ros::Subscriber sub=nh.subscribe(joint_states_topic,100,retriveJointValues);
    //sleep_time.sleep();//wait for roslaunch 
    ros::ServiceServer service=nh.advertiseService("joint_states_srv",transJointValues);
    std::cout<<"joint states service start..."<<std::endl;

    ros::spin();
    ros::shutdown();
    return 0;
}

bool transJointValues(yeebot_commute::JointInfo::Request &req,
                        yeebot_commute::JointInfo::Response &res)
{   
    pthread_mutex_lock(&mutex);
    for(unsigned int i=0;i<req.joint_names.size();i++){
        std::vector<std::string>::iterator iter=std::find(name.begin(),name.end(),req.joint_names[i]);
        if(iter !=name.end()){
        	int pos=std::distance(name.begin(),iter);
        	res.name.push_back(name[pos]);
        	res.position.push_back(position[pos]);
        	res.velocity.push_back(velocity[pos]);
        	res.effort.push_back(effort[pos]);
        }
        else{
        	res.result=false;
        	return false;
        }
    }
    //memcpy(&res.position[0],values,6*sizeof(double));
    pthread_mutex_unlock(&mutex);
    res.result=true;
    return true;

}

void retriveJointValues(const sensor_msgs::JointState& joint_state){
    pthread_mutex_lock(&mutex);
    name=joint_state.name;
    position=joint_state.position;
    velocity=joint_state.velocity;
    effort=joint_state.effort;
    pthread_mutex_unlock(&mutex);
}
