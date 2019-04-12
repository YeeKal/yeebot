
#include "yeebot_core/dual_arm/kine_dual.h"



namespace yeebot{
    /**
     * chain_ and urdf_model are enough to initialize the kine
     **/ 
    //
    KineDual::KineDual(std::vector<KDL::Chain> chains,urdf::Model &urdf_model,int max_iter,Eigen::VectorXi invalid_axis,double project_error,double eps)
    :KineBase(),max_iter_(max_iter),invalid_axis_(invalid_axis),
    project_error_(project_error),eps_(eps)
    {   
        
        kines_.push_back(std::make_shared<KineKdl>(chains[0],urdf_model,max_iter_,invalid_axis_,project_error_,eps_));
        kines_.push_back(std::make_shared<KineKdl>(chains[1],urdf_model,max_iter_,invalid_axis_,project_error_,eps_));
        std::vector<double> lb,ub;
        for(unsigned int k=0;k<2;k++)
            for(unsigned int i=0;i<kines_[k]->getJointsNum();i++){
                lb.push_back(kines_[k]->joint_limits_(i,0));
                ub.push_back(kines_[k]->joint_limits_(i,1));
            }
        dim_=kines_[0]->getJointsNum()+kines_[1]->getJointsNum();
        optp_solver_.reset(new OptpClosure(chains,lb,ub,0.005,project_error_));
    }
    //random project
    //arm1 is fixed and change arm2 by iteration
    bool KineDual::optpProject(const Eigen::Isometry3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        KDL::Frame frame_ref;
        KDL::JntArray kdl_jnt_in(kines_[0]->joint_num_+kines_[1]->joint_num_),kdl_jnt_out(kines_[0]->joint_num_+kines_[1]->joint_num_);
        Eigen2KDL(ref_pose,frame_ref);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        int error_status=optp_solver_->project(frame_ref,invalid_axis,kdl_jnt_in,kdl_jnt_out);
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
    bool KineDual::optpProjectLocal(const Eigen::Isometry3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        KDL::Frame frame_ref;
        KDL::JntArray kdl_jnt_in(kines_[0]->joint_num_+kines_[1]->joint_num_),kdl_jnt_out(kines_[0]->joint_num_+kines_[1]->joint_num_);
        Eigen2KDL(ref_pose,frame_ref);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        int error_status=optp_solver_->projectLocal(frame_ref,invalid_axis,kdl_jnt_in,kdl_jnt_out);
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
    //2.add local minima checking: random project
    bool KineDual::axisProject(const Eigen::Isometry3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        //convert eigen to kdl
        KDL::Frame kdl_rto1;//reference to frame1
        KDL::JntArray kdl_jnt1(kines_[0]->joint_num_),kdl_jnt2(kines_[1]->joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_rto1);
        Eigen2KDL(jnt_in.head(kines_[0]->joint_num_),kdl_jnt1);
        Eigen2KDL(jnt_in.tail(kines_[1]->joint_num_),kdl_jnt2);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        jnt_out=jnt_in;
        //std::cout<<kdl_jnt1(0)<<"  "<<kdl_jnt1(6)<<" "<<kdl_jnt2(0)<<"  "<<kdl_jnt2(6)<<std::endl;
        //pre-declaration frequently used variables 
        KDL::Frame frame1,frame2,frame_ref;
        KDL::Twist delta_twist;
        KDL::JntArray delta_jnt(kines_[1]->joint_num_);//for arm2

        int error_status;
        double twist_error;
        //get frame1
        if(error_status=kines_[0]->fksolver_->JntToCart(kdl_jnt1,frame1)<KDL::SolverI::E_NOERROR){
            std::cout<<"impossible\n";
            return getError(error_status);
        }
        frame_ref=frame1*kdl_rto1;
        for(unsigned int i=0;i<max_iter_;i++){
            //get frame2
            if(error_status=kines_[1]->fksolver_->JntToCart(kdl_jnt2,frame2)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::Twist(KDL::diff(frame_ref.p,frame_ref.M*eleMulti(frame_ref.M.Inverse()*frame2.p,invalid_xyz)),
                                   frame_ref.M*eleMulti(diffLocal(frame_ref.M,frame2.M),invalid_rpy));
            //std::cout<<"delta twist:"<<delta_twist(0)<<" "<<delta_twist(1)<<" "<<delta_twist(2)<<" "<<delta_twist(3)<<" "<<delta_twist(4)<<" "<<delta_twist(5)<<std::endl;
            //calc error  
            // twist_error=0;
            // for(int i=0;i<6;i++){
            //     twist_error +=delta_twist[i]*delta_twist[i];
            // }
            //std::cout<<"error:"<<twist_error<<std::endl;
            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),project_error_)){
                KDL2Eigen(kdl_jnt2,jnt_out.tail(kines_[1]->joint_num_));
                //std::cout<<"jnt out:\n"<<jnt_out<<std::endl;
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }

            //presudoinverse of jacobian to get delta joint values 
            if(error_status=kines_[1]->iksolver_vel_->CartToJnt(kdl_jnt2,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //std::cout<<"delta jnt:"<<delta_jnt.data.transpose()<<std::endl;

            if(delta_jnt.data.isZero(boost::math::tools::epsilon<float>())){
                kdl_jnt2=randomJnt(1);//for arm 2
            }

            KDL::Subtract(kdl_jnt2,delta_jnt,kdl_jnt2);
            for(unsigned int k=0;k<kines_[1]->joint_num_;k++){
                if(kdl_jnt2(k)<kines_[1]->joint_limits_(k,0) || kdl_jnt2(k)>kines_[1]->joint_limits_(k,1) ){
                    kdl_jnt2(k)=(kines_[1]->joint_limits_(k,1)-kines_[1]->joint_limits_(k,0))*(1000+rand()%(8000+1))/1e4+kines_[1]->joint_limits_(k,0);//[0.1-0.9]
                }
            }
    
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
        
    }
    //2.add local minima checking: nearest project
    bool KineDual::axisProjectLocal(const Eigen::Isometry3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
          //convert eigen to kdl
        KDL::Frame kdl_rto1;//reference to frame1
        KDL::JntArray kdl_jnt1(kines_[0]->joint_num_),kdl_jnt2(kines_[1]->joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_rto1);
        Eigen2KDL(jnt_in.head(kines_[0]->joint_num_),kdl_jnt1);
        Eigen2KDL(jnt_in.tail(kines_[1]->joint_num_),kdl_jnt2);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        jnt_out=jnt_in;
        //std::cout<<kdl_jnt1(0)<<"  "<<kdl_jnt1(6)<<" "<<kdl_jnt2(0)<<"  "<<kdl_jnt2(6)<<std::endl;
        //pre-declaration frequently used variables 
        KDL::Frame frame1,frame2,frame_ref;
        KDL::Twist delta_twist;
        KDL::JntArray delta_jnt(kines_[1]->joint_num_);//for arm2

        int error_status;
        double twist_error;
        //get frame1
        if(error_status=kines_[0]->fksolver_->JntToCart(kdl_jnt1,frame1)<KDL::SolverI::E_NOERROR){
            std::cout<<"impossible\n";
            return getError(error_status);
        }
        frame_ref=frame1*kdl_rto1;
        for(unsigned int i=0;i<max_iter_;i++){
            //get frame2
            if(error_status=kines_[1]->fksolver_->JntToCart(kdl_jnt2,frame2)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::Twist(KDL::diff(frame_ref.p,frame_ref.M*eleMulti(frame_ref.M.Inverse()*frame2.p,invalid_xyz)),
                                   frame_ref.M*eleMulti(diffLocal(frame_ref.M,frame2.M),invalid_rpy));
            //std::cout<<"delta twist:"<<delta_twist(0)<<" "<<delta_twist(1)<<" "<<delta_twist(2)<<" "<<delta_twist(3)<<" "<<delta_twist(4)<<" "<<delta_twist(5)<<std::endl;
            //calc error  
            // twist_error=0;
            // for(int i=0;i<6;i++){
            //     twist_error +=delta_twist[i]*delta_twist[i];
            // }
            //std::cout<<"error:"<<twist_error<<std::endl;
            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),project_error_)){
                KDL2Eigen(kdl_jnt2,jnt_out.tail(kines_[1]->joint_num_));
                //std::cout<<"jnt out:\n"<<jnt_out<<std::endl;
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }

            //presudoinverse of jacobian to get delta joint values 
            if(error_status=kines_[1]->iksolver_vel_->CartToJnt(kdl_jnt2,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //std::cout<<"delta jnt:"<<delta_jnt.data.transpose()<<std::endl;

            if(delta_jnt.data.isZero(boost::math::tools::epsilon<float>())){
                return getError(-3);//for arm 2
            }

            KDL::Subtract(kdl_jnt2,delta_jnt,kdl_jnt2);


            for(unsigned int k=0;k<kines_[1]->joint_num_;k++){
                if(kdl_jnt2(k)<kines_[1]->joint_limits_(k,0) || kdl_jnt2(k)>kines_[1]->joint_limits_(k,1) ){
                    return getError(-3);//[0.1-0.9]
                }
            }
    
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
    }
    //initial cbirrt 
    bool KineDual::plainProject(const Eigen::Isometry3d& ref_pose, 
                    const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                    const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                    Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        //convert eigen to kdl
        KDL::Frame kdl_rto1;//reference to frame1
        KDL::JntArray kdl_jnt1(kines_[0]->joint_num_),kdl_jnt2(kines_[1]->joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_rto1);
        Eigen2KDL(jnt_in.head(kines_[0]->joint_num_),kdl_jnt1);
        Eigen2KDL(jnt_in.tail(kines_[1]->joint_num_),kdl_jnt2);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        jnt_out=jnt_in;
        //std::cout<<kdl_jnt1(0)<<"  "<<kdl_jnt1(6)<<" "<<kdl_jnt2(0)<<"  "<<kdl_jnt2(6)<<std::endl;
        //pre-declaration frequently used variables 
        KDL::Frame frame1,frame2,frame_ref;
        KDL::Twist delta_twist;
        KDL::JntArray delta_jnt(kines_[1]->joint_num_);//for arm2

        int error_status;
        double twist_error;
        //get frame1
        if(error_status=kines_[0]->fksolver_->JntToCart(kdl_jnt1,frame1)<KDL::SolverI::E_NOERROR){
            std::cout<<"impossible\n";
            return getError(error_status);
        }
        frame_ref=frame1*kdl_rto1;
        for(unsigned int i=0;i<max_iter_;i++){
            //get frame2
            if(error_status=kines_[1]->fksolver_->JntToCart(kdl_jnt2,frame2)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::Twist(KDL::diff(frame_ref.p,frame_ref.M*eleMulti(frame_ref.M.Inverse()*frame2.p,invalid_xyz)),
                                   frame_ref.M*eleMulti(diffLocal(frame_ref.M,frame2.M),invalid_rpy));
            //std::cout<<"delta twist:"<<delta_twist(0)<<" "<<delta_twist(1)<<" "<<delta_twist(2)<<" "<<delta_twist(3)<<" "<<delta_twist(4)<<" "<<delta_twist(5)<<std::endl;
            //calc error  
            twist_error=0;
            for(int i=0;i<6;i++){
                twist_error +=delta_twist[i]*delta_twist[i];
            }
            //std::cout<<"error:"<<twist_error<<std::endl;
            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),project_error_)){
                KDL2Eigen(kdl_jnt2,jnt_out.tail(kines_[1]->joint_num_));
                //std::cout<<"jnt out:\n"<<jnt_out<<std::endl;
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }

            //presudoinverse of jacobian to get delta joint values 
            if(error_status=kines_[1]->iksolver_vel_->CartToJnt(kdl_jnt2,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            KDL::Subtract(kdl_jnt2,delta_jnt,kdl_jnt2);
            for(unsigned int k=0;k<kines_[1]->joint_num_;k++){
                if(kdl_jnt2(k)<kines_[1]->joint_limits_(k,0) || kdl_jnt2(k)>kines_[1]->joint_limits_(k,1) ){
                    return getError(-3);//[0.1-0.9]
                }
            }
    
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
    }

    KDL::JntArray KineDual::randomJnt(unsigned int chain)const{
        srand((unsigned int) time(0));
        KDL::JntArray random_jnt(kines_[chain]->joint_num_);
        for(int i=0;i<kines_[chain]->joint_num_;i++){
            int random_int=1000+rand()%(8000+1);
            random_jnt(i)=(kines_[chain]->joint_limits_(i,1)-kines_[chain]->joint_limits_(i,0))*(random_int)/(double)1e4+kines_[chain]->joint_limits_(i,0); //[0.1-0.9]
        }
        return random_jnt;

    }
    
    KDL::Vector KineDual::diffLocal(const KDL::Rotation& R_a_b1,const KDL::Rotation& R_a_b2,double dt)const {
        KDL::Rotation R_b1_b2(R_a_b1.Inverse()*R_a_b2);
	    return R_b1_b2.GetRot() / dt;
    }

    KDL::Vector KineDual::eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const{
        return KDL::Vector(vec1.data[0]*vec2.data[0],vec1.data[1]*vec2.data[1],vec1.data[2]*vec2.data[2]);
    }
    bool KineDual::solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const{
        return true;
    }

    bool KineDual::solveFK(Eigen::Isometry3d & pose, const std::string& link_name, const Eigen::Ref<const Eigen::VectorXd> &joint_values) const{
        return true;
    }

    bool KineDual::solveIK( const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &joint_values,const Eigen::Isometry3d &pose) const{
        return true;
    }

    bool KineDual::calcJac(Eigen::Ref<Eigen::MatrixXd> &jacobian, const Eigen::Ref<const Eigen::VectorXd> &joint_values ) const{
        return true;
    }
    
    bool KineDual::getError(const int error_status)const{
        //TODO:process error
        // if(error_status==KDL::SolverI::E_NOT_UP_TO_DATE){
        //     std::cout<<"Internal data structures not up to date with chain."<<std::endl;
        // }
        if(error_status<0){
            //std::cout<<"error:kin"<<error_status<<std::endl;
            return false;
        }
        else if(error_status>=0){
            //std::cout<<"warning:"<<std::endl;
            return true;
        }
        return true;
    }

}//end namesapce yeebot