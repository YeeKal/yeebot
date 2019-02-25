#ifndef YEEBOT_IKSOLVERPOS_TRACKP_H
#define YEEBOT_IKSOLVERPOS_TRACKP_H

#include <nlopt.hpp>
#include <kdl/chainjnttojacsolver.hpp>
#include <thread>
#include <mutex>
#include <memory>
#include <boost/date_time.hpp>

#include "yeebot_core/utils.h"
#include "yeebot_core/iksolverpos_optp.h"
#include "yeebot_core/iksolverpos_tlp.h"

namespace yeebot{
enum SolveType { Speed, Distance, Manip1, Manip2 };

class IkSolverPosTrackP  {
public:

    //only support sumsq(dq,sum)
    IkSolverPosTrackP(const KDL::Chain& chain,const KDL::JntArray& q_min, const KDL::JntArray& q_max, 
                    Eigen::VectorXi invalid_axis,double maxtime=0.005, double eps=1e-3,
                    SolveType _type=Speed);
    IkSolverPosTrackP(const std::string& base_link, const std::string& tip_link,Eigen::VectorXi invalid_axis, const std::string& URDF_param="/robot_description", 
                    double _maxtime=0.005, double _eps=1e-5, SolveType _type=Speed);
    ~IkSolverPosTrackP();
    /**
     * @brief project joint values according to 6 axis: x y z r p y
     */ 
    int project(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out,
                 const KDL::Twist bounds=KDL::Twist::Zero());
    bool runKDLP(const KDL::JntArray &q_init, const KDL::Frame &p_in);
    bool runOPTP(const KDL::JntArray &q_init, const KDL::Frame &p_in);

    bool getKDLChain(KDL::Chain& chain_) {
        chain_=chain;
        return initialized;
    }

    bool getKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_) {
        lb_=lb;
        ub_=ub;
        return initialized;
    }

    // Requires a previous call to CartToJnt()
    bool getSolutions(std::vector<KDL::JntArray>& solutions_) {
        solutions_=solutions;
        return initialized && !solutions.empty();
    }

    bool getSolutions(std::vector<KDL::JntArray>& solutions_, std::vector<std::pair<double,uint> >& errors_) {
        errors_=errors;
        return getSolutions(solutions);
    }

    bool setKDLLimits(KDL::JntArray& lb_, KDL::JntArray& ub_) {
        lb=lb_;
        ub=ub_;

        solver_optp.reset(new yeebot::IkSolverPosOPTP(chain,lb,ub,invalid_axis_,maxtime,eps,SumSq));
        solver_tlp.reset(new yeebot::IkSolverPosTLP(chain,lb,ub,invalid_axis_,maxtime,eps,true,true));
        return true;
    }

    static double JointErr(const KDL::JntArray& arr1, const KDL::JntArray& arr2) {
        double err = 0;
        for (uint i=0; i<arr1.data.size(); i++) {
        err += pow(arr1(i) - arr2(i),2);
        }

        return err;
    }

    int CartToJnt(const KDL::JntArray &q_init, const KDL::Frame &p_in, KDL::JntArray &q_out, const KDL::Twist& bounds=KDL::Twist::Zero());

    inline void SetSolveType(SolveType _type) {
        solvetype = _type;
    }

//private:
    // std::unique_ptr<NLOPT_IK::NLOPT_IK> nl_solver;
    // std::unique_ptr<KDL::ChainIkSolverPos_TL> iksolver;
    std::unique_ptr<yeebot::IkSolverPosTLP> solver_tlp;
    std::unique_ptr<yeebot::IkSolverPosOPTP> solver_optp;
    Eigen::VectorXi invalid_axis_;

    template<typename T1, typename T2>
    bool runSolverP(T1& solver, T2& other_solver,
                   const KDL::JntArray &q_init,
                   const KDL::Frame &m_in);
    //before
    bool initialized;
    KDL::Chain chain;
    KDL::JntArray lb, ub;
    std::unique_ptr<KDL::ChainJntToJacSolver> jacsolver;
    double eps;
    double maxtime;
    SolveType solvetype;

    // std::unique_ptr<NLOPT_IK::NLOPT_IK> nl_solver;
    // std::unique_ptr<KDL::ChainIkSolverPos_TL> iksolver;

    boost::posix_time::ptime start_time;

    template<typename T1, typename T2>
    bool runSolver(T1& solver, T2& other_solver,
                   const KDL::JntArray &q_init,
                   const KDL::Frame &p_in);

    bool runKDL(const KDL::JntArray &q_init, const KDL::Frame &p_in);
    bool runNLOPT(const KDL::JntArray &q_init, const KDL::Frame &p_in);

    void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
    void normalize_limits(const KDL::JntArray& seed, KDL::JntArray& solution);

    std::vector<BasicJointType> types;

    std::mutex mtx_;
    std::vector<KDL::JntArray> solutions;
    std::vector<std::pair<double,uint> >  errors;

    std::thread task1,task2,ptask1,ptask2;
    KDL::Twist bounds;

    bool unique_solution(const KDL::JntArray& sol);

    inline static double fRand(double min, double max)
    {
      double f = (double)rand() / RAND_MAX;
      return min + f * (max - min);
    }

    /* @brief Manipulation metrics and penalties taken from "Workspace
    Geometric Characterization and Manipulability of Industrial Robots",
    Ming-June, Tsia, PhD Thesis, Ohio State University, 1986.
    https://etd.ohiolink.edu/!etd.send_file?accession=osu1260297835
    */
    double manipPenalty(const KDL::JntArray&);
    double ManipValue1(const KDL::JntArray&);
    double ManipValue2(const KDL::JntArray&);

    inline bool myEqual(const KDL::JntArray& a, const KDL::JntArray& b) {
      return (a.data-b.data).isZero(1e-4);
    }

    void initialize();

};//class KineKdl
inline bool IkSolverPosTrackP::runKDLP(const KDL::JntArray &q_in, const KDL::Frame &m_in){
    return runSolverP(*solver_tlp.get(),*solver_optp.get(),q_in,m_in);
}

inline bool IkSolverPosTrackP::runOPTP(const KDL::JntArray &q_in, const KDL::Frame &m_in){
    return runSolverP(*solver_optp.get(),*solver_tlp.get(),q_in,m_in);
}

inline bool IkSolverPosTrackP::runKDL(const KDL::JntArray &q_in, const KDL::Frame &m_in)
{
    return runSolver(*solver_tlp.get(),*solver_optp.get(),q_in,m_in);
}

inline bool IkSolverPosTrackP::runNLOPT(const KDL::JntArray &q_in, const KDL::Frame &m_in)
{
    return runSolver(*solver_optp.get(),*solver_tlp.get(),q_in,m_in);
}

typedef std::shared_ptr<IkSolverPosTrackP> IkSolverPosTrackPPtr;
typedef std::shared_ptr<const IkSolverPosTrackP> IkSolverPosTrackPConstPtr;
}//namesapce yeebot

#endif //YEEBOT_KINE_KDL_H