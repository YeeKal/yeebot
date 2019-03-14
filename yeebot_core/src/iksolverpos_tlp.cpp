#include "yeebot_core/iksolverpos_tlp.h"
#include <boost/date_time.hpp>
#include <ros/ros.h>
#include <limits>

namespace yeebot{
IkSolverPosTLP::IkSolverPosTLP(const KDL::Chain& _chain,const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, 
    Eigen::VectorXi invalid_axis,double _maxtime, double _eps,
    bool _random_restart, bool _try_jl_wrap)
    //class shoud be initialized first??
    :vel_solver_proj_(_chain,invalid_axis),chain(_chain), q_min(_q_min), q_max(_q_max), vik_solver(_chain), fksolver(_chain), delta_q(_chain.getNrOfJoints()),
    maxtime(_maxtime),eps(_eps),rr(_random_restart),wrap(_try_jl_wrap),
    invalid_axis_(invalid_axis)
{
    assert(chain.getNrOfJoints()==_q_min.data.size());
    assert(chain.getNrOfJoints()==_q_max.data.size());

    reset();

    for (uint i=0; i<chain.segments.size(); i++) {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot")!=std::string::npos) {
        if (_q_max(types.size())>=std::numeric_limits<float>::max() && 
            _q_min(types.size())<=std::numeric_limits<float>::lowest())
        types.push_back(yeebot::BasicJointType::Continuous);
        else types.push_back(yeebot::BasicJointType::RotJoint);
    }
    else if (type.find("Trans")!=std::string::npos)
        types.push_back(yeebot::BasicJointType::TransJoint);
    
    }
    
    assert(types.size()==_q_max.data.size());
}

int IkSolverPosTLP::project(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out, const KDL::Twist _bounds){
    if (aborted) //??
        return -3;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration timediff;
    q_out = q_in;
    bounds = _bounds;
           
    double time_left;

    do {
        fksolver.JntToCart(q_out,f);//Frame f
        delta_twist = diffRelative(m_in, f);

        if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
            delta_twist.vel.x(0);
        
        if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
            delta_twist.vel.y(0);
        
        if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
            delta_twist.vel.z(0);
        
        if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
            delta_twist.rot.x(0);
        
        if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
            delta_twist.rot.y(0);
        
        if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
            delta_twist.rot.z(0);

        // if(KDL::Equal(delta_twist,KDL::Twist::Zero(),eps))
        //     return 1;

        delta_twist = KDL::diff(f, m_in);

        double twist_error=0;
        //get invalid error
        unsigned int invalid_row=0;
        unsigned int small=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis_(i)){
                delta_twist[invalid_row]=delta_twist[i];
                if(fabs(delta_twist[invalid_row])<eps) small++;
                invalid_row++;
            }
        }
        if(invalid_row==small){
            return 1;
        }

        //de
        
        // for(int i=0;i<invalid_row;i++){
        //     twist_error +=delta_twist[i]*delta_twist[i];
        // }
        // std::cout<<"tlp error:"<<twist_error<<std::endl;
//
        //changed here
        //vik_solver.CartToJnt(q_out,delta_twist,delta_q);
        //it's better to use shared_ptr for api later
        vel_solver_proj_.CartToJnt(q_out,delta_twist,delta_q);
        KDL::JntArray q_curr;
        
        KDL::Add(q_out,delta_q,q_curr);

        //revise for joint limit
        for(unsigned int j=0; j<q_min.data.size(); j++) {
            if (types[j]==yeebot::BasicJointType::Continuous)
                continue;
            if(q_curr(j) < q_min(j)) 
                if (!wrap || types[j]==yeebot::BasicJointType::TransJoint)
                    // KDL's default 
                    q_curr(j) = q_min(j);
                else {
                    // Find actual wrapped angle between limit and joint
                    double diffangle = fmod(q_min(j)-q_curr(j),2*M_PI);
                    // Subtract that angle from limit and go into the range by a
                    // revolution
                    double curr_angle = q_min(j) - diffangle + 2*M_PI;
                    if (curr_angle > q_max(j))
                        q_curr(j) = q_min(j);
                    else
                        q_curr(j) = curr_angle;
                }
        }
      
        for(unsigned int j=0; j<q_max.data.size(); j++) {
            if (types[j]==yeebot::BasicJointType::Continuous)
                continue;

            if(q_curr(j) > q_max(j)) 
                if (!wrap || types[j]==yeebot::BasicJointType::TransJoint)
                    // KDL's default 
                    q_curr(j) = q_max(j);
                else {
                    // Find actual wrapped angle between limit and joint
                    double diffangle = fmod(q_curr(j)-q_max(j),2*M_PI);
                    // Add that angle to limit and go into the range by a revolution
                    double curr_angle = q_max(j) + diffangle - 2*M_PI;
                    if (curr_angle < q_min(j))
                    q_curr(j) = q_max(j);
                    else
                    q_curr(j) = curr_angle;
                }
        }
      
        //get joint value diff
        Subtract(q_out,q_curr,q_out);
        //if get stuck local minima
        if (q_out.data.isZero(boost::math::tools::epsilon<float>())) {
            if (rr) {//if random restart
                for (unsigned int j=0; j<q_out.data.size(); j++) 
                    if (types[j]==yeebot::BasicJointType::Continuous)
                        q_curr(j)=fRand(q_curr(j)-2*M_PI,q_curr(j)+2*M_PI);
                    else
                        q_curr(j)=fRand(q_min(j),q_max(j));
            }
        }

        q_out=q_curr;
        timediff=boost::posix_time::microsec_clock::local_time()-start_time;
        time_left = maxtime - timediff.total_nanoseconds()/1000000000.0;
    } while (time_left > 0 && !aborted);
    
    return -3;
}
int IkSolverPosTLP::projectNotlocal(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out, const KDL::Twist _bounds){
    if (aborted) //??
        return -3;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration timediff;
    q_out = q_in;
    bounds = _bounds;
           
    double time_left;

    do {
        fksolver.JntToCart(q_out,f);//Frame f
        delta_twist = diffRelative(m_in, f);

        if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
            delta_twist.vel.x(0);
        
        if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
            delta_twist.vel.y(0);
        
        if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
            delta_twist.vel.z(0);
        
        if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
            delta_twist.rot.x(0);
        
        if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
            delta_twist.rot.y(0);
        
        if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
            delta_twist.rot.z(0);

        // if(KDL::Equal(delta_twist,KDL::Twist::Zero(),eps))
        //     return 1;

        delta_twist = KDL::diff(f, m_in);

        double twist_error=0;
        //get invalid error
        unsigned int invalid_row=0;
        unsigned int small=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis_(i)){
                delta_twist[invalid_row]=delta_twist[i];
                if(fabs(delta_twist[invalid_row])<eps) small++;
                invalid_row++;
            }
        }
        if(invalid_row==small){
            return 1;
        }

        //de
        
        // for(int i=0;i<invalid_row;i++){
        //     twist_error +=delta_twist[i]*delta_twist[i];
        // }
        // std::cout<<"tlp error:"<<twist_error<<std::endl;
//
        //changed here
        //vik_solver.CartToJnt(q_out,delta_twist,delta_q);
        //it's better to use shared_ptr for api later
        vel_solver_proj_.CartToJnt(q_out,delta_twist,delta_q);
        KDL::JntArray q_curr;
        
        KDL::Add(q_out,delta_q,q_curr);

        //revise for joint limit
        for(unsigned int j=0; j<q_min.data.size(); j++) {
            if (types[j]==yeebot::BasicJointType::Continuous)
                continue;
            if(q_curr(j) < q_min(j)) 
                if (!wrap || types[j]==yeebot::BasicJointType::TransJoint)
                    // KDL's default 
                    q_curr(j) = q_min(j);
                else {
                    // Find actual wrapped angle between limit and joint
                    double diffangle = fmod(q_min(j)-q_curr(j),2*M_PI);
                    // Subtract that angle from limit and go into the range by a
                    // revolution
                    double curr_angle = q_min(j) - diffangle + 2*M_PI;
                    if (curr_angle > q_max(j))
                        q_curr(j) = q_min(j);
                    else
                        q_curr(j) = curr_angle;
                }
        }
      
        for(unsigned int j=0; j<q_max.data.size(); j++) {
            if (types[j]==yeebot::BasicJointType::Continuous)
                continue;

            if(q_curr(j) > q_max(j)) 
                if (!wrap || types[j]==yeebot::BasicJointType::TransJoint)
                    // KDL's default 
                    q_curr(j) = q_max(j);
                else {
                    // Find actual wrapped angle between limit and joint
                    double diffangle = fmod(q_curr(j)-q_max(j),2*M_PI);
                    // Add that angle to limit and go into the range by a revolution
                    double curr_angle = q_max(j) + diffangle - 2*M_PI;
                    if (curr_angle < q_min(j))
                    q_curr(j) = q_max(j);
                    else
                    q_curr(j) = curr_angle;
                }
        }
      
        //get joint value diff
        Subtract(q_out,q_curr,q_out);
        //if get stuck local minima
        if (q_out.data.isZero(boost::math::tools::epsilon<float>())) {
            return -3;//return false
        }

        q_out=q_curr;
        timediff=boost::posix_time::microsec_clock::local_time()-start_time;
        time_left = maxtime - timediff.total_nanoseconds()/1000000000.0;
    } while (time_left > 0 && !aborted);
    
    return -3;
}

int IkSolverPosTLP::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist _bounds) {

    if (aborted)
      return -3;

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration timediff;
    q_out = q_init;
    bounds = _bounds;

           
    double time_left;

    do {
      fksolver.JntToCart(q_out,f);
      delta_twist = diffRelative(p_in, f);

      if (std::abs(delta_twist.vel.x()) <= std::abs(bounds.vel.x()))
        delta_twist.vel.x(0);
      
      if (std::abs(delta_twist.vel.y()) <= std::abs(bounds.vel.y()))
        delta_twist.vel.y(0);
      
      if (std::abs(delta_twist.vel.z()) <= std::abs(bounds.vel.z()))
        delta_twist.vel.z(0);
      
      if (std::abs(delta_twist.rot.x()) <= std::abs(bounds.rot.x()))
        delta_twist.rot.x(0);
      
      if (std::abs(delta_twist.rot.y()) <= std::abs(bounds.rot.y()))
        delta_twist.rot.y(0);
      
      if (std::abs(delta_twist.rot.z()) <= std::abs(bounds.rot.z()))
        delta_twist.rot.z(0);

      if(Equal(delta_twist,KDL::Twist::Zero(),eps))
        return 1;

      delta_twist = diff(f, p_in);

      vik_solver.CartToJnt(q_out,delta_twist,delta_q);
      KDL::JntArray q_curr;
      
      KDL::Add(q_out,delta_q,q_curr);
      
      for(unsigned int j=0; j<q_min.data.size(); j++) {
        if (types[j]==yeebot::BasicJointType::Continuous)
          continue;
        if(q_curr(j) < q_min(j)) 
          if (!wrap || types[j]==yeebot::BasicJointType::TransJoint)
            // KDL's default 
            q_curr(j) = q_min(j);
          else {
            // Find actual wrapped angle between limit and joint
            double diffangle = fmod(q_min(j)-q_curr(j),2*M_PI);
            // Subtract that angle from limit and go into the range by a
            // revolution
            double curr_angle = q_min(j) - diffangle + 2*M_PI;
            if (curr_angle > q_max(j))
              q_curr(j) = q_min(j);
            else
              q_curr(j) = curr_angle;
          }
      }
      
      for(unsigned int j=0; j<q_max.data.size(); j++) {
        if (types[j]==yeebot::BasicJointType::Continuous)
          continue;

        if(q_curr(j) > q_max(j)) 
          if (!wrap || types[j]==yeebot::BasicJointType::TransJoint)
            // KDL's default 
            q_curr(j) = q_max(j);
          else {
            // Find actual wrapped angle between limit and joint
            double diffangle = fmod(q_curr(j)-q_max(j),2*M_PI);
            // Add that angle to limit and go into the range by a revolution
            double curr_angle = q_max(j) + diffangle - 2*M_PI;
            if (curr_angle < q_min(j))
              q_curr(j) = q_max(j);
            else
              q_curr(j) = curr_angle;
          }
      }
      
      KDL::Subtract(q_out,q_curr,q_out);
      
      if (q_out.data.isZero(boost::math::tools::epsilon<float>())) {
        if (rr) {
          for (unsigned int j=0; j<q_out.data.size(); j++) 
            if (types[j]==yeebot::BasicJointType::Continuous)
              q_curr(j)=fRand(q_curr(j)-2*M_PI,q_curr(j)+2*M_PI);
            else
              q_curr(j)=fRand(q_min(j),q_max(j));
        }

        // Below would be an optimization to the normal KDL, where when it
        // gets stuck, it returns immediately.  Don't use to compare KDL with
        // random restarts or TRAC-IK to plain KDL.

        // else {
        //   q_out=q_curr;
        //   return -3;
        // }
      }

      q_out=q_curr;
     
      timediff=boost::posix_time::microsec_clock::local_time()-start_time;
      time_left = maxtime - timediff.total_nanoseconds()/1000000000.0;
    } while (time_left > 0 && !aborted);
    
    return -3;
  }

IkSolverPosTLP::~IkSolverPosTLP()
{
}

}//end namesapce yeebot
