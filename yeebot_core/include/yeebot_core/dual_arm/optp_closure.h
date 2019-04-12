#ifndef YEEBOT_OPTP_CLOSURE_H
#define YEEBOT_OPTP_CLOSURE_H

#include <nlopt.hpp>
#include "yeebot_core/iksolverpos_tlp.h" //for kdl include

#include "yeebot_core/utils.h"

namespace yeebot{

class OptpClosure   {
public:

	//only support sumsq(dq,sum)
	OptpClosure(const std::vector<KDL::Chain> _chains,const std::vector<double> _lb, const std::vector<double> _ub,double _maxtime, double _eps);
	~OptpClosure(){};
	/**
		* @brief project joint values according to 6 axis: x y z r p y
		*/ 
	int project( const KDL::Frame& m_in,const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,const KDL::JntArray& q_in, KDL::JntArray& q_out);

	void cartSumSquaredErrorP(const std::vector<double>& x, double error[]);

	inline void setMaxtime(double t) { maxtime = t; }
	KDL::Vector diffLocal(const KDL::Rotation& R_a_b1,const KDL::Rotation& R_a_b2,double dt=1)const;

	KDL::Vector eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const;
	void normalize_seed(const KDL::JntArray& seed, KDL::JntArray& solution);
	void normalizeAngle(double& val, const double& min, const double& max);
	void normalizeAngle(double& val, const double& target);

//private:
	Eigen::VectorXi invalid_axis_;
	KDL::Vector invalid_xyz_,invalid_rpy_;
	nlopt::opt optp;//a new opt for project

	//before
	inline void abort() {
		aborted = true;
	}

	inline void reset() {
		aborted = false;
	}
	

	std::vector<double> lb;
	std::vector<double> ub;

	const std::vector<KDL::Chain> chains;
	std::vector<double> des;


	std::vector<KDL::ChainFkSolverPos_recursive> fksolver;

	double maxtime;
	double eps;
	int iter_counter; 
	unsigned int dim1,dim2;

	KDL::Frame frame1,frame2,frame_rto1,frame_ref;
	
	std::vector<BasicJointType> types;

	nlopt::opt opt;

	std::vector<double> best_x;
	int progress;
	bool aborted;

	inline static double fRand(double min, double max)
	{
		double f = (double)rand() / RAND_MAX;
		return min + f * (max - min);
	}

};//class KineKdl
typedef std::shared_ptr<OptpClosure> OptpClosurePtr;
typedef std::shared_ptr<const OptpClosure> OptpClosureConstPtr;
}//namesapce yeebot

#endif //YEEBOT_OPTP_CLOSURE_H