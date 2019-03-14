#include <boost/date_time.hpp>
#include <limits>
#include <kdl_parser/kdl_parser.hpp>
#include <urdf/model.h>

#include "yeebot_core/iksolverpos_trackp.h"

namespace yeebot{
IkSolverPosTrackP::IkSolverPosTrackP(const std::string& base_link, const std::string& tip_link,Eigen::VectorXi invalid_axis, const std::string& URDF_param, 
                    double _maxtime, double _eps, SolveType _type)
    :initialized(false),
    eps(_eps),
    maxtime(_maxtime),
    solvetype(_type),
    invalid_axis_(invalid_axis)
{   
    ros::NodeHandle node_handle("~");

    urdf::Model robot_model;
    std::string xml_string;

    std::string urdf_xml,full_urdf_xml;
    node_handle.param("urdf_xml",urdf_xml,URDF_param);
    node_handle.searchParam(urdf_xml,full_urdf_xml);

    ROS_DEBUG_NAMED("trac_ik", "Reading xml file from parameter server");
    if (!node_handle.getParam(full_urdf_xml, xml_string))
      {
        ROS_FATAL_NAMED("trac_ik", "Could not load the xml from parameter server: %s", urdf_xml.c_str());
        return;
      }

    node_handle.param(full_urdf_xml,xml_string,std::string());
    robot_model.initString(xml_string);

    ROS_DEBUG_STREAM_NAMED("trac_ik", "Reading joints and links from URDF");

    KDL::Tree tree;

    if (!kdl_parser::treeFromUrdfModel(robot_model, tree))
      ROS_FATAL("Failed to extract kdl tree from xml robot description");

    if(!tree.getChain(base_link, tip_link, chain))
      ROS_FATAL("Couldn't find chain %s to %s",base_link.c_str(),tip_link.c_str());

    std::vector<KDL::Segment> chain_segs = chain.segments;

    urdf::JointConstSharedPtr joint;

    std::vector<double> l_bounds, u_bounds;

    lb.resize(chain.getNrOfJoints());
    ub.resize(chain.getNrOfJoints());

    uint joint_num=0;
    for(unsigned int i = 0; i < chain_segs.size(); ++i) {
      joint = robot_model.getJoint(chain_segs[i].getJoint().getName());
      if (joint->type != urdf::Joint::UNKNOWN && joint->type != urdf::Joint::FIXED) {
        joint_num++;
        float lower, upper;
        int hasLimits;
        if ( joint->type != urdf::Joint::CONTINUOUS ) {
          if(joint->safety) {
            lower = std::max(joint->limits->lower, joint->safety->soft_lower_limit);
            upper = std::min(joint->limits->upper, joint->safety->soft_upper_limit);
          } else {
            lower = joint->limits->lower;
            upper = joint->limits->upper;
          }
          hasLimits = 1;
        }
        else {
          hasLimits = 0;
        }
        if(hasLimits) {
          lb(joint_num-1)=lower;
          ub(joint_num-1)=upper;
        }
        else {
          lb(joint_num-1)=std::numeric_limits<float>::lowest();
          ub(joint_num-1)=std::numeric_limits<float>::max();
        }
        ROS_DEBUG_STREAM_NAMED("trac_ik", "IK Using joint "<<joint->name<<" "<<lb(joint_num-1)<<" "<<ub(joint_num-1));
      }
    }

    initialize();
}

IkSolverPosTrackP::IkSolverPosTrackP(const KDL::Chain& _chain,const KDL::JntArray& _q_min, const KDL::JntArray& _q_max, 
                    Eigen::VectorXi invalid_axis,double _maxtime, double _eps,
                    SolveType _type)
    :initialized(false),
    chain(_chain),
    lb(_q_min),
    ub(_q_max),
    eps(_eps),
    maxtime(_maxtime),
    solvetype(_type),
    invalid_axis_(invalid_axis)
{
    initialize();
        
}

void IkSolverPosTrackP::initialize() {

    assert(chain.getNrOfJoints()==lb.data.size());
    assert(chain.getNrOfJoints()==ub.data.size());

    jacsolver.reset(new KDL::ChainJntToJacSolver(chain));
    // nl_solver.reset(new NLOPT_IK::NLOPT_IK(chain,lb,ub,maxtime,eps,NLOPT_IK::SumSq));
    // iksolver.reset(new KDL::ChainIkSolverPos_TL(chain,lb,ub,maxtime,eps,true,true));
    solver_optp.reset(new yeebot::IkSolverPosOPTP(chain,lb,ub,invalid_axis_,maxtime,eps,SumSq));
    solver_tlp.reset(new yeebot::IkSolverPosTLP(chain,lb,ub,invalid_axis_,maxtime,eps,true,true));


    for (uint i=0; i<chain.segments.size(); i++) {
    std::string type = chain.segments[i].getJoint().getTypeName();
    if (type.find("Rot")!=std::string::npos) {
        if (ub(types.size())>=std::numeric_limits<float>::max() &&
            lb(types.size())<=std::numeric_limits<float>::lowest())
        types.push_back(BasicJointType::Continuous);
        else
        types.push_back(BasicJointType::RotJoint);
    }
    else if (type.find("Trans")!=std::string::npos)
        types.push_back(BasicJointType::TransJoint);
    }

    assert(types.size()==lb.data.size());

    initialized = true;
  }

//rewrite CartToJnt
int IkSolverPosTrackP::project(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out,
            const KDL::Twist _bounds)
{
    if (!initialized) {
        ROS_ERROR("TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
        return -1;
    }


    start_time = boost::posix_time::microsec_clock::local_time();

    solver_optp->reset();//clear aborted signal
    solver_tlp->reset();

    solutions.clear();
    errors.clear();

    bounds=_bounds;

    ptask1 = std::thread(&IkSolverPosTrackP::runKDLP, this, q_in, m_in);
    ptask2 = std::thread(&IkSolverPosTrackP::runOPTP, this, q_in, m_in);

    ptask1.join();
    ptask2.join();
    
    if (solutions.empty()) {
        q_out=q_in;
        return -3;
    }

    switch (solvetype) {
        case Manip1:
        case Manip2:
            std::sort(errors.rbegin(),errors.rend()); // rbegin/rend to sort by max
        break;
        default:
            std::sort(errors.begin(),errors.end());
        break;
    }

    q_out = solutions[errors[0].second];

    return solutions.size();
}

//rewrite runSolver
//two threads running
template<typename T1, typename T2>
bool IkSolverPosTrackP::runSolverP(T1& solver, T2& other_solver,
                        const KDL::JntArray &q_in,
                        const KDL::Frame &m_in)
{
    KDL::JntArray q_out;

    double fulltime=maxtime;
    KDL::JntArray seed = q_in;

    boost::posix_time::time_duration timediff;
    double time_left;

    while (true) {
        timediff=boost::posix_time::microsec_clock::local_time()-start_time;
        time_left = fulltime - timediff.total_nanoseconds()/1000000000.0;

        if (time_left <= 0)
            break;

        solver.setMaxtime(time_left);

        int RC = solver.project(seed,m_in,q_out,bounds);
        if (RC >=0) {
            switch (solvetype) {
                case Manip1:
                case Manip2:
                    normalize_limits(q_in, q_out);
                break;
                default:
                    normalize_seed(q_in, q_out);//corresponding artifical limits
                break;
            }
            mtx_.lock();
            //joint in solutions have the same value 
            if (unique_solution(q_out)) {
                solutions.push_back(q_out);
                uint curr_size=solutions.size();
                errors.resize(curr_size);
                mtx_.unlock();
                double err, penalty;
                switch (solvetype) {
                case Manip1:
                    penalty = manipPenalty(q_out);
                    err = penalty*IkSolverPosTrackP::ManipValue1(q_out);
                    break;
                case Manip2:
                    penalty = manipPenalty(q_out);
                    err = penalty*IkSolverPosTrackP::ManipValue2(q_out);
                    break;
                default:
                    err = IkSolverPosTrackP::JointErr(q_in,q_out);//squre and sum
                    break;
                }
                mtx_.lock();
                //std::vector<std::pair<double,uint> >  errors;
                //record the solution error
                errors[curr_size-1] = std::make_pair(err,curr_size-1);
                }
                mtx_.unlock();
        }
        //if find solution jump the loop
        if (!solutions.empty() && solvetype == Speed)
            break;
        //if not solution, 
        //get random values and restart
        for (unsigned int j=0; j<seed.data.size(); j++)
            if (types[j]==BasicJointType::Continuous)
                seed(j)=fRand(q_in(j)-2*M_PI, q_in(j)+2*M_PI);
            else
                seed(j)=fRand(lb(j), ub(j));
    }
    other_solver.abort();

    solver.setMaxtime(fulltime);
    return true;

}

bool IkSolverPosTrackP::unique_solution(const KDL::JntArray& sol) {

    for (uint i=0; i< solutions.size(); i++)
    if (myEqual(sol,solutions[i]))
        return false;
    return true;

}

inline void normalizeAngle(double& val, const double& min, const double& max)
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

inline void normalizeAngle(double& val, const double& target)
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


template<typename T1, typename T2>
bool IkSolverPosTrackP::runSolver(T1& solver, T2& other_solver,
                        const KDL::JntArray &q_init,
                        const KDL::Frame &p_in)
{
    KDL::JntArray q_out;

    double fulltime = maxtime;
    KDL::JntArray seed = q_init;

    boost::posix_time::time_duration timediff;
    double time_left;

    while (true) {
    timediff=boost::posix_time::microsec_clock::local_time()-start_time;
    time_left = fulltime - timediff.total_nanoseconds()/1000000000.0;

    if (time_left <= 0)
        break;

    solver.setMaxtime(time_left);

    int RC = solver.CartToJnt(seed,p_in,q_out,bounds);
    if (RC >=0) {
        switch (solvetype) {
        case Manip1:
        case Manip2:
        normalize_limits(q_init, q_out);
        break;
        default:
        normalize_seed(q_init, q_out);
        break;
        }
        mtx_.lock();
        if (unique_solution(q_out)) {
        solutions.push_back(q_out);
        uint curr_size=solutions.size();
        errors.resize(curr_size);
        mtx_.unlock();
        double err, penalty;
        switch (solvetype) {
        case Manip1:
            penalty = manipPenalty(q_out);
            err = penalty*IkSolverPosTrackP::ManipValue1(q_out);
            break;
        case Manip2:
            penalty = manipPenalty(q_out);
            err = penalty*IkSolverPosTrackP::ManipValue2(q_out);
            break;
        default:
            err = IkSolverPosTrackP::JointErr(q_init,q_out);
            break;
        }
        mtx_.lock();
        errors[curr_size-1] = std::make_pair(err,curr_size-1);
        }
        mtx_.unlock();
    }

    if (!solutions.empty() && solvetype == Speed)
        break;

    for (unsigned int j=0; j<seed.data.size(); j++)
        if (types[j]==BasicJointType::Continuous)
            seed(j)=fRand(q_init(j)-2*M_PI, q_init(j)+2*M_PI);
        else
            seed(j)=fRand(lb(j), ub(j));
    }
    other_solver.abort();

    solver.setMaxtime(fulltime);

    return true;
}


void IkSolverPosTrackP::normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution) {
    // Make sure rotational joint values are within 1 revolution of seed; then
    // ensure joint limits are met.

    bool improved = false;

    for (uint i=0; i<lb.data.size(); i++) {

    if (types[i]==BasicJointType::TransJoint)
        continue;

    double target = seed(i);
    double val = solution(i);

    normalizeAngle( val, target );

    if (types[i]==BasicJointType::Continuous) {
        solution(i) = val;
        continue;
    }

    normalizeAngle( val, lb(i), ub(i) );

    solution(i) = val;
    }
}

void IkSolverPosTrackP::normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution) {
    // Make sure rotational joint values are within 1 revolution of middle of
    // limits; then ensure joint limits are met.

    bool improved = false;

    for (uint i=0; i<lb.data.size(); i++) {

    if (types[i] == BasicJointType::TransJoint)
        continue;

    double target = seed(i);

    if (types[i] == BasicJointType::RotJoint && types[i]!=BasicJointType::Continuous)
        target = (ub(i)+lb(i))/2.0;

    double val = solution(i);

    normalizeAngle( val, target );

    if (types[i]==BasicJointType::Continuous) {
        solution(i) = val;
        continue;
    }

    normalizeAngle( val, lb(i), ub(i) );

    solution(i) = val;
    }

}


double IkSolverPosTrackP::manipPenalty(const KDL::JntArray& arr) {
    double penalty = 1.0;
    for (uint i=0; i< arr.data.size(); i++) {
    if (types[i] == BasicJointType::Continuous)
        continue;
    double range = ub(i)-lb(i);
    penalty *= ((arr(i)-lb(i))*(ub(i)-arr(i))/(range*range));
    }
    return std::max(0.0,1.0 - exp(-1*penalty));
}


double IkSolverPosTrackP::ManipValue1(const KDL::JntArray& arr) {
    KDL::Jacobian jac(arr.data.size());

    jacsolver->JntToJac(arr,jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();

    double error = 1.0;
    for(unsigned int i=0; i < singular_values.rows(); ++i)
    error *= singular_values(i,0);
    return error;
}

double IkSolverPosTrackP::ManipValue2(const KDL::JntArray& arr) {
    KDL::Jacobian jac(arr.data.size());

    jacsolver->JntToJac(arr,jac);

    Eigen::JacobiSVD<Eigen::MatrixXd> svdsolver(jac.data);
    Eigen::MatrixXd singular_values = svdsolver.singularValues();

    return singular_values.minCoeff()/singular_values.maxCoeff();
}


int IkSolverPosTrackP::CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& _bounds) {

    if (!initialized) {
    ROS_ERROR("TRAC-IK was not properly initialized with a valid chain or limits.  IK cannot proceed");
    return -1;
    }


    start_time = boost::posix_time::microsec_clock::local_time();

    solver_optp->reset();//clear aborted signal
    solver_tlp->reset();

    solutions.clear();
    errors.clear();

    bounds=_bounds;

    task1 = std::thread(&IkSolverPosTrackP::runKDL, this, q_init, p_in);
    task2 = std::thread(&IkSolverPosTrackP::runNLOPT, this, q_init, p_in);

    task1.join();
    task2.join();
    
    if (solutions.empty()) {
    q_out=q_init;
    return -3;
    }

    switch (solvetype) {
    case Manip1:
    case Manip2:
    std::sort(errors.rbegin(),errors.rend()); // rbegin/rend to sort by max
    break;
    default:
    std::sort(errors.begin(),errors.end());
    break;
    }

    q_out = solutions[errors[0].second];

    return solutions.size();
}


IkSolverPosTrackP::~IkSolverPosTrackP(){
    if (task1.joinable())
        task1.join();
    if (task2.joinable())
        task2.join();
    if (ptask1.joinable())
        ptask1.join();
    if (ptask2.joinable())
        ptask2.join();
}


}//end namespace yeebot