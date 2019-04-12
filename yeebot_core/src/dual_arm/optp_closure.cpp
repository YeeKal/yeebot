#include <ros/ros.h>
#include <limits>
#include <boost/date_time.hpp>
#include <cmath>

#include "yeebot_core/dual_arm/optp_closure.h"

namespace yeebot{
double minfuncSumSquaredDualP(const std::vector<double>& x, std::vector<double>& grad, void* data);


OptpClosure::OptpClosure(const std::vector<KDL::Chain> _chains,const std::vector<double> _lb, const std::vector<double> _ub,
                        double _maxtime, double _eps)
:chains(_chains), maxtime(_maxtime), eps(std::abs(_eps)),
lb(_lb),ub(_ub)
{   
    reset();
    dim1=chains[0].getNrOfJoints();
    dim2=chains[1].getNrOfJoints();

    if (dim1< 2 || dim2<2) {
        ROS_WARN_THROTTLE(1.0,"OptpClosure can only be run for chains of length 2 or more");
        return;
    }
    optp = nlopt::opt(nlopt::LD_SLSQP, dim1+dim2);
    std::cout<<"01\n";
    for (uint i=0; i<chains[0].segments.size(); i++) {
        std::string type = chains[0].segments[i].getJoint().getTypeName();
        if (type.find("Rot")!=std::string::npos) {
            if (ub[types.size()]>=std::numeric_limits<float>::max() && 
                lb[types.size()]<=std::numeric_limits<float>::lowest())
                types.push_back(BasicJointType::Continuous);
            else
                types.push_back(BasicJointType::RotJoint);
        }
        else if (type.find("Trans")!=std::string::npos)
            types.push_back(BasicJointType::TransJoint);
    }
    for (uint i=0; i<chains[1].segments.size(); i++) {
        std::string type = chains[1].segments[i].getJoint().getTypeName();
        if (type.find("Rot")!=std::string::npos) {
            if (ub[types.size()]>=std::numeric_limits<float>::max() && 
                lb[types.size()]<=std::numeric_limits<float>::lowest())
                types.push_back(BasicJointType::Continuous);
            else
                types.push_back(BasicJointType::RotJoint);
        }
        else if (type.find("Trans")!=std::string::npos)
            types.push_back(BasicJointType::TransJoint);
    }
    assert(types.size()==lb.size());
    std::vector<double> tolerance(1,boost::math::tools::epsilon<float>());
    std::vector<double> tolerancep(1,eps*eps);
    optp.set_xtol_abs(tolerance[0]);
    //optp.set_xtol_abs(1e-5);

    optp.set_lower_bounds(lb);
    optp.set_upper_bounds(ub);
    optp.set_min_objective(minfuncSumSquaredDualP, this);

    fksolver.push_back(KDL::ChainFkSolverPos_recursive(chains[0]));
    fksolver.push_back(KDL::ChainFkSolverPos_recursive(chains[1]));

   
}

int OptpClosure::project( const KDL::Frame& m_in,const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,const KDL::JntArray& q_in, KDL::JntArray& q_out){

    boost::posix_time::ptime start_time = boost::posix_time::microsec_clock::local_time();
    boost::posix_time::time_duration diff;

    q_out=q_in;
    frame_rto1 = m_in;
    invalid_axis_=invalid_axis;
    
    Eigen2KDL(invalid_axis.head(3),invalid_xyz_);
    Eigen2KDL(invalid_axis.tail(3),invalid_rpy_);

    optp.set_maxtime(maxtime);


    double minf; /* the minimum objective value, upon return */

    std::vector<double> x(dim1+dim2);
    //joint limit
    for (uint i=0; i < x.size(); i++) {
        x[i] = q_in(i);
        
        if (types[i]==BasicJointType::Continuous)
            continue;

        if (types[i]==BasicJointType::TransJoint) {
            x[i] = std::min(x[i],ub[i]);
            x[i] = std::max(x[i],lb[i]);
        }
        else {
            
            // Below is to handle bad seeds outside of limits
            
            if (x[i] > ub[i]) {
            //Find actual angle offset
            double diffangle = fmod(x[i]-ub[i],2*M_PI);
            // Add that to upper bound and go back a full rotation
            x[i] = ub[i] + diffangle - 2*M_PI;
            }
            
            if (x[i] < lb[i]) {
            //Find actual angle offset
            double diffangle = fmod(lb[i]-x[i],2*M_PI);
            // Subtract that from lower bound and go forward a full rotation
            x[i] = lb[i] - diffangle + 2*M_PI;
            }        
            
            if (x[i] > ub[i]) 
            x[i] = (ub[i]+lb[i])/2.0;
        }
    }
    
    best_x=x;
    progress = -3;

    //adjust the joint limits
    std::vector<double> artificial_lower_limits(lb.size());

    for (uint i=0; i< lb.size(); i++)
        if (types[i]==BasicJointType::Continuous) 
            artificial_lower_limits[i] = best_x[i]-2*M_PI;
        else if (types[i]==BasicJointType::TransJoint) 
            artificial_lower_limits[i] = lb[i];
        else
            artificial_lower_limits[i] = std::max(lb[i],best_x[i]-2*M_PI);
    
    optp.set_lower_bounds(artificial_lower_limits);

    std::vector<double> artificial_upper_limits(lb.size());

    for (uint i=0; i< ub.size(); i++)
        if (types[i]==BasicJointType::Continuous) 
            artificial_upper_limits[i] = best_x[i]+2*M_PI;
        else if (types[i]==BasicJointType::TransJoint) 
            artificial_upper_limits[i] = ub[i];
        else
            artificial_upper_limits[i] = std::min(ub[i],best_x[i]+2*M_PI);
    
    optp.set_upper_bounds(artificial_upper_limits);
    
    try {
      optp.optimize(x, minf);
    } catch (...) {
    }
    
    if (progress == -1) // Got NaNs
      progress = -3;
        

    if (!aborted && progress < 0) {

		double time_left;
		diff=boost::posix_time::microsec_clock::local_time()-start_time;
		time_left = maxtime - diff.total_nanoseconds()/1000000000.0;

		while (time_left > 0 && !aborted && progress < 0) {
			
			for (uint i=0; i< x.size(); i++){
				x[i]=fRand(artificial_lower_limits[i], artificial_upper_limits[i]);
			}

			optp.set_maxtime(time_left);

			try 
			{
				optp.optimize(x, minf);
			} 
			catch (...) {}

			if (progress == -1) // Got NaNs
			progress = -3;

			diff=boost::posix_time::microsec_clock::local_time()-start_time;
			time_left = maxtime - diff.total_nanoseconds()/1000000000.0;
		}
    }
           

    for (uint i=0; i < x.size(); i++) {
      q_out(i) = best_x[i];
    }
    normalize_seed(q_in,q_out);//adjust jnt value
    //progress: solve state
    //aborted: termination control
    return progress;
}
void OptpClosure::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution) {
    // Make sure rotational joint values are within 1 revolution of seed; then
    // ensure joint limits are met.

    bool improved = false;

    for (uint i=0; i<lb.size(); i++) {

        if (types[i]==BasicJointType::TransJoint)
            continue;

        double target = seed(i);
        double val = solution(i);

        normalizeAngle( val, target );

        if (types[i]==BasicJointType::Continuous) {
            solution(i) = val;
            continue;
        }

        normalizeAngle( val, lb[i], ub[i] );

        solution(i) = val;
    }
}
void OptpClosure::normalizeAngle(double& val, const double& min, const double& max)
{
    if (val > max) {
        //Find actual angle offset
        double diffangle = fmod(val-max,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = max + diffangle - 2*M_PI;
    }

    if (val < min) {
        //Find actual angle offset
        double diffangle = fmod(min-val,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = min - diffangle + 2*M_PI;
    }
}

void OptpClosure::normalizeAngle(double& val, const double& target)
{
    if (val > target+M_PI) {
        //Find actual angle offset
        double diffangle = fmod(val-target,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = target + diffangle - 2*M_PI;
    }

    if (val < target-M_PI) {
        //Find actual angle offset
        double diffangle = fmod(target-val,2*M_PI);
        // Add that to upper bound and go back a full rotation
        val = target - diffangle + 2*M_PI;
    }
}

void OptpClosure::cartSumSquaredErrorP(const std::vector<double>& x, double error[]){
    if (aborted || progress != -3) {
        optp.force_stop();
        return;
    }
    KDL::JntArray q1(dim1),q2(dim2);
    for (uint i=0; i<dim1; i++)
        q1(i)=x[i];
    for(uint i=0;i<dim2;i++)
        q2(i)=x[dim1+i];

    int rc=0;
    rc = fksolver[0].JntToCart(q1,frame1);
    if (rc < 0)
        ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q1.data);
    rc = fksolver[1].JntToCart(q2,frame2);
    if (rc < 0)
        ROS_FATAL_STREAM("KDL FKSolver is failing: "<<q2.data);

    frame_ref=frame1*frame_rto1;
    KDL::Twist delta_twist;
    delta_twist=KDL::Twist(KDL::diff(frame_ref.p,frame_ref.M*eleMulti(frame_ref.M.Inverse()*frame2.p,invalid_xyz_)),
                                   frame_ref.M*eleMulti(diffLocal(frame_ref.M,frame2.M),invalid_rpy_));
    error[0]=0;
	unsigned int invalid_eps=0;
    for(int i=0;i<6;i++){
        error[0] +=delta_twist[i]*delta_twist[i];
    }
    //chen if we have reached there when error is small enough
    if(KDL::Equal(delta_twist,KDL::Twist::Zero(),eps)){
        progress=1;
        best_x=x;
        return;
    }

}
KDL::Vector OptpClosure::diffLocal(const KDL::Rotation& R_a_b1,const KDL::Rotation& R_a_b2,double dt)const {
    KDL::Rotation R_b1_b2(R_a_b1.Inverse()*R_a_b2);
    return R_b1_b2.GetRot() / dt;
}

KDL::Vector OptpClosure::eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const{
    return KDL::Vector(vec1.data[0]*vec2.data[0],vec1.data[1]*vec2.data[1],vec1.data[2]*vec2.data[2]);
}


double minfuncSumSquaredDualP(const std::vector<double>& x, std::vector<double>& grad, void* data) {

    OptpClosure *c = (OptpClosure *) data;

    std::vector<double> vals(x);

	

    double jump=boost::math::tools::epsilon<float>();
    double result[1]; 
    c->cartSumSquaredErrorP(vals, result);

	//std::cout<<"grad j:";
    if (!grad.empty()) {
        double v1[1];
        for (uint i=0; i<x.size(); i++) {
          double original=vals[i];

          vals[i]=original+jump;
          c->cartSumSquaredErrorP(vals, v1);

          vals[i]=original;
          grad[i]=(v1[0]-result[0])/(2.0*jump);
        }
    }

    return result[0];
}

}//end namespace yeebots