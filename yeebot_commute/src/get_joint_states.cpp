#include <vector>
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
double *values=new double[20];

//service name:joint_states_srv
int main(int argc,char **argv){
    ros::init(argc,argv,"get_joint_states");
    ros::NodeHandle nh;
    ros::WallDuration sleep_time(5.0);

    ros::Subscriber sub=nh.subscribe("joint_states",100,retriveJointValues);
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
    for(unsigned int i=0;i<req.dim;i++){
        res.position.push_back(values[i]);
    }
    //memcpy(&res.position[0],values,6*sizeof(double));
    pthread_mutex_unlock(&mutex);
    res.result=true;
    return true;

}

void retriveJointValues(const sensor_msgs::JointState& joint_state){
    pthread_mutex_lock(&mutex);
    memcpy(values, &joint_state.position[0], joint_state.position.size()*sizeof(double));
    pthread_mutex_unlock(&mutex);
}