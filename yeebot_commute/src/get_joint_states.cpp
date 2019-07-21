#include <vector>
#include <algorithm>
#include <string>
#include <iostream>
#include <assert.h>
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
    bool add_vel=true;//if velocity is empty
    bool add_eff=true;//if all not empty
    assert(name.size()==position.size());
    if(velocity.size()!=position.size()){
        add_vel=false;
        add_eff=false;
    }
    else if(effort.size()!=position.size()){
        add_eff=false;
    }
    pthread_mutex_lock(&mutex);
    for(unsigned int i=0;i<req.joint_names.size();i++){
        std::vector<std::string>::iterator iter=std::find(name.begin(),name.end(),req.joint_names[i]);
        if(iter !=name.end()){
        	int pos=std::distance(name.begin(),iter);
        	res.name.push_back(name[pos]);
        	res.position.push_back(position[pos]);
            if(add_eff){
        	    res.velocity.push_back(velocity[pos]);
        	    res.effort.push_back(effort[pos]);
            }else if(add_vel){
                res.velocity.push_back(velocity[pos]);
            }
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
    //if velocity is empty
    //if effort is empty
    //if dual-arms is not in the same sequence
    bool add_vel=true;//if velocity is empty
    bool add_eff=true;//if all not empty
    assert(joint_state.name.size()==joint_state.position.size());
    if(joint_state.velocity.size()!=joint_state.position.size()){
        add_vel=false;
        add_eff=false;
        //std::cout<<"not v\n";
    }
    else if(joint_state.effort.size()!=joint_state.position.size()){
        add_eff=false;
        //std::cout<<"not e\n";
    }
    pthread_mutex_lock(&mutex);
    //check if the name not added before
    for(unsigned int i=0;i<joint_state.name.size();i++){
        std::vector<std::string>::iterator iter=std::find(name.begin(),name.end(),joint_state.name[i]);
        if(iter !=name.end()){  //is exist before
        	int pos=std::distance(name.begin(),iter);
            //name[pos]=joint_state.name[pos];//not needeed
            position[pos]=joint_state.position[i];
            //std::cout<<"exist:"<<name[pos]<<":"<<position[pos]<<std::endl;
            if(add_eff){
                velocity[pos]=joint_state.velocity[i];
                effort[pos]=joint_state.effort[i];
            }else if(add_vel){
                velocity[pos]=joint_state.velocity[i];
            }
        }//if not exist before
        else{
        	name.push_back(joint_state.name[i]);
            position.push_back(joint_state.position[i]);
            //std::cout<<"not exist:"<<name.back()<<":"<<position.back()<<std::endl;
            if(add_eff){
                velocity.push_back(joint_state.velocity[i]);
                effort.push_back(joint_state.effort[i]);
            }else if(add_vel){
                velocity.push_back(joint_state.velocity[i]);
            }
        }
    }
    pthread_mutex_unlock(&mutex);
}
