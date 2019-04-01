#ifndef YEEBOT_IKSOLVERPOS_OPTP_H
#define YEEBOT_IKSOLVERPOS_OPTP_H

#include <nlopt.hpp>
#include "yeebot_core/iksolverpos_tlp.h" //for kdl include

#include "yeebot_core/utils.h"

namespace yeebot{
    class IkSolverPosTrackP;
    enum OptType { Joint, DualQuat, SumSq, L2 };
class IkSolverPosOPTP   {
    friend class IkSolverPosTrackP;
public:

    //only support sumsq(dq,sum)
    IkSolverPosOPTP(const KDL::Chain& chain,const KDL::JntArray& q_min, const KDL::JntArray& q_max, 
                    Eigen::VectorXi invalid_axis,double maxtime=0.005, double eps=1e-3,
                    OptType type=SumSq);
    ~IkSolverPosOPTP(){};
    /**
     * @brief project joint values according to 6 axis: x y z r p y
     */ 
    int project(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out,
                 const KDL::Twist bounds=KDL::Twist::Zero(),
                 const KDL::JntArray& q_desired=KDL::JntArray());
    
    void cartSumSquaredErrorP(const std::vector<double>& x, double error[]);
//before
    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds=KDL::Twist::Zero(), const KDL::JntArray& q_desired=KDL::JntArray());

    double minJoints(const std::vector<double>& x, std::vector<double>& grad);
    //  void cartFourPointError(const std::vector<double>& x, double error[]);
    void cartSumSquaredError(const std::vector<double>& x, double error[]);
    void cartDQError(const std::vector<double>& x, double error[]);
    void cartL2NormError(const std::vector<double>& x, double error[]);
    int projectNotlocal(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out,
                const KDL::Twist _bounds,const KDL::JntArray& q_desired=KDL::JntArray());

    inline void setMaxtime(double t) { maxtime = t; }

//private:
    Eigen::VectorXi invalid_axis_;
    nlopt::opt optp;//a new opt for project
    unsigned int invalid_dim_;

    //before
    inline void abort() {
      aborted = true;
    }

    inline void reset() {
      aborted = false;
    }
    

    std::vector<double> lb;
    std::vector<double> ub;

    const KDL::Chain chain;
    std::vector<double> des;


    KDL::ChainFkSolverPos_recursive fksolver;

    double maxtime;
    double eps;
    int iter_counter; 
    OptType TYPE;

    KDL::Frame targetPose;
    KDL::Frame z_up ;
    KDL::Frame x_out;
    KDL::Frame y_out;
    KDL::Frame z_target;
    KDL::Frame x_target;
    KDL::Frame y_target;
    
    std::vector<BasicJointType> types;

    nlopt::opt opt;

    KDL::Frame currentPose;

    std::vector<double> best_x;
    int progress;
    bool aborted;

    KDL::Twist bounds;

    inline static double fRand(double min, double max)
    {
      double f = (double)rand() / RAND_MAX;
      return min + f * (max - min);
    }


};//class KineKdl
typedef std::shared_ptr<IkSolverPosOPTP> IkSolverPosOPTPPtr;
typedef std::shared_ptr<const IkSolverPosOPTP> IkSolverPosOPTPConstPtr;
}//namesapce yeebot

#endif //YEEBOT_KINE_KDL_H