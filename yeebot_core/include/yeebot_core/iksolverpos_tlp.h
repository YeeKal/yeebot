/********************************************************************************
Copyright (c) 2015, TRACLabs, Inc.
All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
 are permitted provided that the following conditions are met:

    1. Redistributions of source code must retain the above copyright notice, 
       this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright notice,
       this list of conditions and the following disclaimer in the documentation 
       and/or other materials provided with the distribution.

    3. Neither the name of the copyright holder nor the names of its contributors
       may be used to endorse or promote products derived from this software 
       without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED 
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE 
OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************/

#ifndef YEEBOT_IKSOLVERPOS_TLP_H
#define YEEBOT_IKSOLVERPOS_TLP_H

#include <kdl/chainfksolverpos_recursive.hpp>
#include <kdl/chainiksolvervel_pinv.hpp>

#include "yeebot_core/utils.h"
#include "yeebot_core/iksolvervel_project.h"

namespace yeebot{
class IkSolverPosTrackP;
enum BasicJointType { RotJoint, TransJoint, Continuous };

class IkSolverPosTLP{
    friend class IkSolverPosTrackP;
public:
    IkSolverPosTLP(const KDL::Chain& chain,const KDL::JntArray& q_min, const KDL::JntArray& q_max,
                    Eigen::VectorXi invalid_axis,double maxtime=0.005, double eps=1e-3, 
                    bool random_restart=false,bool try_jl_wrap=false);
    ~IkSolverPosTLP();
    /**
     * TODO: use bool project
     */ 
    int CartToJnt(const KDL::JntArray& q_init, const KDL::Frame& p_in, KDL::JntArray& q_out, const KDL::Twist bounds=KDL::Twist::Zero());
    int project(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out, const KDL::Twist bounds=KDL::Twist::Zero());
    int projectNotlocal(const KDL::JntArray& q_in, const KDL::Frame& m_in, KDL::JntArray& q_out, const KDL::Twist _bounds=KDL::Twist::Zero());

    inline void setMaxtime(double t) { maxtime = t; }
    //added
    Eigen::VectorXi invalid_axis_;
    yeebot::IkSolverVel_project vel_solver_proj_;//improved
    //before
    const KDL::Chain chain;
    KDL::JntArray q_min;
    KDL::JntArray q_max;

    KDL::Twist bounds;

    KDL::ChainIkSolverVel_pinv vik_solver;
    KDL::ChainFkSolverPos_recursive fksolver;
    KDL::JntArray delta_q;
    double maxtime;

    double eps;
    
    bool rr;
    bool wrap;

    std::vector<BasicJointType> types;

    inline void abort() {
      aborted = true;
    }

    inline void reset() {
      aborted = false;
    }

    bool aborted;

    KDL::Frame f;
    KDL::Twist delta_twist;

    inline static double fRand(double min, double max)
    {
      double f = (double)rand() / RAND_MAX;
      return min + f * (max - min);
    }
private:

};//class IkSolverPosTLP

IMETHOD KDL::Twist diffRelative(const KDL::Frame & F_a_b1, const KDL::Frame & F_a_b2, double dt = 1)
{
    return KDL::Twist(F_a_b1.M.Inverse() * diff(F_a_b1.p, F_a_b2.p, dt),
                F_a_b1.M.Inverse() * diff(F_a_b1.M, F_a_b2.M, dt));
}

typedef std::shared_ptr<IkSolverPosTLP> IkSolverPosTLPPtr;
typedef std::shared_ptr<const IkSolverPosTLP> IkSolverPosTLPConstPtr;
}//namesapce yeebot

#endif //YEEBOT_IKSOLVERPOS_TLP_H