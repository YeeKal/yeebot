#include "yeebot_core/kine_kdl.h"
#include <kdl/solveri.hpp>



namespace yeebot{
    /**
     * chain_ and urdf_model are enough to initialize the kine
     **/ 
    KineKdl::KineKdl(KDL::Chain chain,urdf::Model &urdf_model,int max_iter,Eigen::VectorXi invalid_axis,double project_error,double eps)
    :KineBase(),chain_(chain),max_iter_(max_iter),invalid_axis_(invalid_axis),
    project_error_(project_error),eps_(eps)
    {
        initialize(urdf_model);

    }
    //base_name_/tip_name/trees
    KineKdl::KineKdl(const std::string& urdf_param,const std::string& base_name,const std::string& tip_name,Eigen::VectorXi invalid_axis,double project_error,double eps)
    :KineBase(),invalid_axis_(invalid_axis),
    max_iter_(100),eps_(eps),project_error_(project_error)
    {       
        urdf::Model urdf_model;
        KDL::Tree tree;
        urdf_model.initParam(urdf_param);
        if(!kdl_parser::treeFromUrdfModel(urdf_model,tree)){
            std::cout<<"error!failed to initialize kdl tree from urdf model."<<std::endl;
        }
        tree.getChain(base_name,tip_name,chain_);
        initialize(urdf_model);
    }
    //initialize with chain_
    void KineKdl::initialize(urdf::Model &urdf_model){
        link_num_=chain_.getNrOfSegments();
        joint_num_=chain_.getNrOfJoints();
        link_names_.resize(link_num_);
        joint_names_.resize(joint_num_);
        joint_limits_.resize(joint_num_,2);

        for(int i=0,j=0;i<link_num_;i++){
            const KDL::Segment &seg=chain_.getSegment(i);
            const KDL::Joint& jnt = seg.getJoint();
            link_names_[i]=seg.getName();
            if(jnt.getType()==KDL::Joint::None) 
                continue;

            joint_names_[j]=jnt.getName();
            urdf::JointConstSharedPtr joint = urdf_model.getJoint(jnt.getName());
            joint_limits_(j,0)=joint->limits->lower;
            joint_limits_(j,1)=joint->limits->upper;
            j++;
        }
        
        KDL::JntArray joints_min,joints_max;
        Eigen2KDL(joint_limits_.col(0),joints_min);
        Eigen2KDL(joint_limits_.col(1),joints_max);

        fksolver_.reset(new KDL::ChainFkSolverPos_recursive(chain_));
        jac_solver_.reset(new KDL::ChainJntToJacSolver(chain_));
        iksolver_vel_.reset(new KDL::ChainIkSolverVel_pinv(chain_));
        iksolver_nr_jl_.reset(new KDL::ChainIkSolverPos_NR_JL(chain_,joints_min,joints_max,*fksolver_,*iksolver_vel_,max_iter_,eps_));
        iksolver_trackp_.reset(new yeebot::IkSolverPosTrackP(chain_,joints_min,joints_max,invalid_axis_,0.005,project_error_,Speed));
    }
    bool KineKdl::solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values)const{
        return solveFK(pose,joint_values,-1);
    }
    bool KineKdl::solveFK(Eigen::Isometry3d & pose, const std::string& link_name, const Eigen::Ref<const Eigen::VectorXd> &joint_values)const{
        // if(link_name==base_name_){
        //     pose.setIdentity();
        //     return true;
        // }
        //find the link name index
        for(unsigned int i=0;i<link_num_;i++){
            if(link_name==link_names_[i]){
                return solveFK(pose,joint_values,i+1);
            }
        }
    }

    bool KineKdl::solveFK(Eigen::Isometry3d & pose, const Eigen::Ref<const Eigen::VectorXd> &joint_values,int link_num)const{
        KDL::JntArray kdl_joints;
        Eigen2KDL(joint_values,kdl_joints);
        KDL::Frame kdl_pose;

        int error_status=fksolver_->JntToCart(kdl_joints,kdl_pose,link_num);
        if(getError(error_status)){
            KDL2Eigen(kdl_pose,pose);
            return true;
        }
        return false;
    }
/**
 * @solve ik on ur5-10000:
 * kdl:fail-4115  time-11.15
 * trac:fail-301 time-8.085
 * @solve ik on sda-10000:
 * kdl:fail-8854 time 27.17
 * trac:fail-4047 time-29.096
 */ 

    bool KineKdl::trackSolveIk(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &jnt_out,const Eigen::Isometry3d &pose)
    {
        KDL::JntArray kdl_joints_in,kdl_joints_values;//ik initial joint values
        Eigen2KDL(joint_in,kdl_joints_in);
        KDL::Frame kdl_pose;
        Eigen2KDL(pose,kdl_pose);
        iksolver_trackp_->eps=1e-5;
        iksolver_trackp_->solver_tlp->eps=1e-5;
        iksolver_trackp_->solver_optp->eps=1e-5;
        int error_status=iksolver_trackp_->CartToJnt(kdl_joints_in,kdl_pose,kdl_joints_values);
        iksolver_trackp_->eps=project_error_;
        iksolver_trackp_->solver_tlp->eps=project_error_;
        iksolver_trackp_->solver_optp->eps=project_error_;
        KDL2Eigen(kdl_joints_values,jnt_out);
        return getError(error_status);
    }
/**
 * @about joint limits-on ur5
 * ik:5000 iteration
 * 1. equal to 0: fail-2050  time-5.577
 * 2. equal to bounds: fail-4238  time-8.227
 * 3. equal to  : fail-871  time 3.923
 * but method 1 has a better result because the joint value tends to be more close to 0
 * 
 * @project: 5000 iteration
 * 1. equal to 0: fail-1  time-1.418
 * 2. equal to bounds: ##
 * 3. equal to  : fail-1  time 1.56
 * 
**/
    
    bool KineKdl::solveIK(const Eigen::Ref<const Eigen::VectorXd> &joint_in,Eigen::VectorXd &joint_values,const Eigen::Isometry3d &pose)const{
        KDL::JntArray kdl_joints_in;//ik initial joint values
        Eigen2KDL(joint_in,kdl_joints_in);
        KDL::Frame kdl_pose;
        Eigen2KDL(pose,kdl_pose);

        KDL::JntArray kdl_jnt_out(joint_num_);//ik result
        KDL::JntArray delta_jnt(joint_num_);
        KDL::Frame kdl_new_pose;
        KDL::Twist delta_twist;

        int error_status;
        //error_status=iksolver_nr_jl_->CartToJnt(kdl_joints_in,kdl_pose,kdl_jnt_out);
        //ik by myself
        //not support joint number update
        if(joint_num_!=kdl_joints_in.rows() || joint_num_!=kdl_jnt_out.rows()){
            getError(-10);
            return false;
        }
        kdl_jnt_out=kdl_joints_in;
        for(unsigned int i=0;i<max_iter_;i++){

            // for(unsigned int k=0;k<joint_num_;k++){
            //     std::cout<<kdl_jnt_out(k)<<"  ";
            // }
            //std::cout<<std::endl;
            //get new pose
            if(error_status=fksolver_->JntToCart(kdl_jnt_out,kdl_new_pose)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::diff(kdl_pose,kdl_new_pose);
            //std::cout<<"delta twist:"<<delta_twist(0)<<" "<<delta_twist(1)<<" "<<delta_twist(2)<<" "<<delta_twist(3)<<" "<<delta_twist(4)<<" "<<delta_twist(5)<<std::endl;

            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),eps_)){
                KDL2Eigen(kdl_jnt_out,joint_values);
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }

            //presudoinverse of jacobian to get delta joint values 
            if(error_status=iksolver_vel_->CartToJnt(kdl_jnt_out,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }

            KDL::Subtract(kdl_jnt_out,delta_jnt,kdl_jnt_out);
            //joint limits
            //min_jointlimits<0  max_jointlimits>0
            // for(unsigned int k=0;k<joint_num_;k++){
            //     if(kdl_jnt_out(k)<joint_limits_(k,0)){
            //         kdl_jnt_out(k)=std::fmod(joint_limits_(k,0)-M_PI,2*M_PI)+M_PI;
            //     }else if(kdl_jnt_out(k)>joint_limits_(k,1)){
            //         kdl_jnt_out(k)=std::fmod(joint_limits_(k,1)+M_PI,2*M_PI)-M_PI;
                   
            //     }
            //     if(kdl_jnt_out(k)<joint_limits_(k,0) || kdl_jnt_out(k)>joint_limits_(k,1) ){
            //         kdl_jnt_out(k)=0;//0 is valid as default
            //     }
            // }//end joint limits
            for(unsigned int k=0;k<joint_num_;k++){
                if(kdl_jnt_out(k)<joint_limits_(k,0) || kdl_jnt_out(k)>joint_limits_(k,1) ){
                    kdl_jnt_out(k)=0;//0 is valid as default
                }
            }
            //std::cout<<std::endl;           
        }
        return getError(-5);
        //end ik
        // if(getError(error_status)){
        //     KDL2Eigen(kdl_joints,joint_values);
        //     return true;
        // }
        // return false;
    }

    bool KineKdl::calcJac(Eigen::Ref<Eigen::MatrixXd> &jacobian, const Eigen::Ref<const Eigen::VectorXd> &joint_values )const{
        KDL::JntArray kdl_joints;
        Eigen2KDL(joint_values,kdl_joints);
        KDL::Jacobian kdl_jac(joint_num_);

        int error_status=jac_solver_->JntToJac(kdl_joints,kdl_jac);
        if(getError(error_status)){
            KDL2Eigen(kdl_jac,jacobian);
            return true;
        }
        return false;
    }
   bool KineKdl::trackProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out, IkSolverPosTrackPPtr &ik_track)const 
    {   
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_),kdl_jnt_out(joint_num_);;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);

        int error_status=ik_track->project(kdl_jnt_in,kdl_ref_pose,kdl_jnt_out,KDL::Twist::Zero());
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
    bool KineKdl::optpProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out)
    {   
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_),kdl_jnt_out(joint_num_);;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        iksolver_trackp_->solver_optp->reset();
        int error_status=iksolver_trackp_->solver_optp->project(kdl_jnt_in,kdl_ref_pose,kdl_jnt_out,KDL::Twist::Zero());
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
    bool KineKdl::tlpProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out)
    {   
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_),kdl_jnt_out(joint_num_);;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        iksolver_trackp_->solver_tlp->reset();
        int error_status=iksolver_trackp_->solver_tlp->project(kdl_jnt_in,kdl_ref_pose,kdl_jnt_out,KDL::Twist::Zero());
        
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
     bool KineKdl::optpProjectNotlocal(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out)
    {   
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_),kdl_jnt_out(joint_num_);;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        iksolver_trackp_->solver_optp->reset();
        int error_status=iksolver_trackp_->solver_optp->projectNotlocal(kdl_jnt_in,kdl_ref_pose,kdl_jnt_out,KDL::Twist::Zero());
        if(error_status<0)
            return getError(-3);
        //iksolver_trackp_->normalize_seed(kdl_jnt_in,kdl_jnt_out);
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
    bool KineKdl::tlpProjectNotlocal(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out)
    {   
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_),kdl_jnt_out(joint_num_);;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        iksolver_trackp_->solver_tlp->reset();
        int error_status=iksolver_trackp_->solver_tlp->projectNotlocal(kdl_jnt_in,kdl_ref_pose,kdl_jnt_out,KDL::Twist::Zero());
        
        KDL2Eigen(kdl_jnt_out,jnt_out);
        return getError(error_status);
    }
    //kdl project with joint limits
    bool KineKdl::axisProject(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        //convert eigen to kdl
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        
        //pre-declaration frequently used variables 
        KDL::Frame kdl_new_pose;
        KDL::Twist delta_twist;
        KDL::JntArray delta_jnt(joint_num_);

        KDL::JntArray kdl_jnt_out(joint_num_);
        kdl_jnt_out=kdl_jnt_in;
        int error_status;
        double twist_error;
        for(unsigned int i=0;i<max_iter_;i++){
            
            //fk to get new pose
            if(error_status=fksolver_->JntToCart(kdl_jnt_out,kdl_new_pose)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::Twist(eleMulti(KDL::diff(kdl_ref_pose.p,kdl_new_pose.p),invalid_xyz),  
                                   eleMulti(KDL::diff(kdl_ref_pose.M,kdl_new_pose.M),invalid_rpy));
            //std::cout<<"delta twist:"<<delta_twist(0)<<" "<<delta_twist(1)<<" "<<delta_twist(2)<<" "<<delta_twist(3)<<" "<<delta_twist(4)<<" "<<delta_twist(5)<<std::endl;
            //calc error  
            twist_error=0;
            for(int i=0;i<6;i++){
                twist_error +=delta_twist[i]*delta_twist[i];
            }
            //std::cout<<"error:"<<twist_error<<std::endl;
            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),project_error_)){
                KDL2Eigen(kdl_jnt_out,jnt_out);
                //std::cout<<"jnt out:\n"<<jnt_out<<std::endl;
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }

            //presudoinverse of jacobian to get delta joint values 
            if(error_status=iksolver_vel_->CartToJnt(kdl_jnt_out,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                std::cout<<"vel"<<std::endl;
                return getError(error_status);
            }

            KDL::Subtract(kdl_jnt_out,delta_jnt,kdl_jnt_out);

            for(unsigned int k=0;k<joint_num_;k++){
                if(kdl_jnt_out(k)<joint_limits_(k,0) || kdl_jnt_out(k)>joint_limits_(k,1) ){
                    kdl_jnt_out(k)=0;//0 is valid as default
                }
            }
    
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
        
    }

    //for test
    bool KineKdl::axisProjectLocal(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> jnt_out) const
    {   
        //convert eigen to kdl
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        
        //pre-declaration frequently used variables 
        KDL::Frame kdl_new_pose;
        KDL::Twist delta_twist;
        KDL::JntArray delta_jnt(joint_num_);

        KDL::JntArray kdl_jnt_out(joint_num_);
        kdl_jnt_out=kdl_jnt_in;
        int error_status;
        double twist_error;
        for(unsigned int i=0;i<max_iter_;i++){
            
            //fk to get new pose
            if(error_status=fksolver_->JntToCart(kdl_jnt_out,kdl_new_pose)<KDL::SolverI::E_NOERROR){
                return getError(error_status);
            }
            //differentiate of the frame 
            delta_twist=KDL::Twist(eleMulti(KDL::diff(kdl_ref_pose.p,kdl_new_pose.p),invalid_xyz),  
                                   eleMulti(KDL::diff(kdl_ref_pose.M,kdl_new_pose.M),invalid_rpy));
            //std::cout<<"delta twist:"<<delta_twist(0)<<" "<<delta_twist(1)<<" "<<delta_twist(2)<<" "<<delta_twist(3)<<" "<<delta_twist(4)<<" "<<delta_twist(5)<<std::endl;
            //calc error  
            twist_error=0;
            for(int i=0;i<6;i++){
                twist_error +=delta_twist[i]*delta_twist[i];
            }
            //std::cout<<"error:"<<twist_error<<std::endl;
            if(KDL::Equal(delta_twist,KDL::Twist::Zero(),project_error_)){
                KDL2Eigen(kdl_jnt_out,jnt_out);
                //std::cout<<"jnt out:\n"<<jnt_out<<std::endl;
                return getError((error_status>KDL::SolverI::E_NOERROR ? KDL::SolverI::E_DEGRADED : KDL::SolverI::E_NOERROR));
            }

            //presudoinverse of jacobian to get delta joint values 
            if(error_status=iksolver_vel_->CartToJnt(kdl_jnt_out,delta_twist,delta_jnt)<KDL::SolverI::E_NOERROR){
                std::cout<<"vel"<<std::endl;
                return getError(error_status);
            }

            KDL::Subtract(kdl_jnt_out,delta_jnt,kdl_jnt_out);
            if(delta_jnt.data.isZero(boost::math::tools::epsilon<float>())){
                //return getError(-3);
            }

            for(unsigned int k=0;k<joint_num_;k++){
                if(kdl_jnt_out(k)<joint_limits_(k,0) || kdl_jnt_out(k)>joint_limits_(k,1) ){
                    //kdl_jnt_out(k)=0;//0 is valid as default
                    return getError(-3);
                }
            }
    
        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
        
    }

     //for function in constraint space
    void KineKdl::function(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd> err_out) const{
        //err_out should have the correct size
        //the validity of size won't be verified here
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);

        KDL::Twist delta_twist;
        KDL::Frame kdl_new_pose;

        //fk to get new pose
        fksolver_->JntToCart(kdl_jnt_in,kdl_new_pose);//get new frame

        //differential of the frame 
        delta_twist=KDL::Twist(KDL::diff(kdl_ref_pose.p,kdl_new_pose.p),  
                               KDL::diff(kdl_ref_pose.M,kdl_new_pose.M));

        //get the invalid velocity according to the invalid vector
        int invalid_num=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis(i)>0.5){//TODO:change 0 to epsilon
                err_out(invalid_num)=delta_twist(i);
                invalid_num ++;
            }
        }//end for
    }
    //for jacobian in constraint space
    void KineKdl::jacobian(const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::MatrixXd> jac_out)const {
        Eigen::MatrixXd jac(6,joint_num_);
        //calcJac(jac,jnt_in);
        KDL::JntArray kdl_joints;
        Eigen2KDL(jnt_in,kdl_joints);
        KDL::Jacobian kdl_jac(joint_num_);
        jac_solver_->JntToJac(kdl_joints,kdl_jac);
        KDL2Eigen(kdl_jac,jac);

        int invalid_num=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis(i)>0.5){//TODO:change 0 to epsilon
                jac_out.row(invalid_num)=jac.row(i);
                invalid_num ++;
            }
        }//end for
    }

    /*
    comparation between KDL-SVD and EIgen-SVD.
    kdl is faster(1000,0.4s) but more times to fail.
    eigen is slower(1000,1.6s) but less times to fail.
    */
    bool KineKdl::project(const Eigen::Isometry3d& ref_pose, 
                     const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,
                     const Eigen::Ref<const Eigen::VectorXd> &jnt_in,
                     Eigen::Ref<Eigen::VectorXd>  jnt_out) const{
         //convert eigen to kdl
        KDL::Frame kdl_ref_pose;
        KDL::JntArray kdl_jnt_in(joint_num_);
        KDL::Vector invalid_xyz,invalid_rpy;
        Eigen2KDL(ref_pose,kdl_ref_pose);
        Eigen2KDL(jnt_in,kdl_jnt_in);
        Eigen2KDL(invalid_axis.head(3),invalid_xyz);
        Eigen2KDL(invalid_axis.tail(3),invalid_rpy);
        
        //pre-declaration frequently used variables 
        KDL::Frame kdl_new_pose;
        KDL::Twist delta_twist;
        //KDL::JntArray delta_jnt(joint_num_);
        Eigen::VectorXd delta_jnt(joint_num_);

        KDL::JntArray kdl_jnt_out(joint_num_);
        kdl_jnt_out=kdl_jnt_in;
        jnt_out=jnt_in;

        int invalid_count=0;
        for(int i=0;i<6;i++){
            if(invalid_axis(i)>0.5){
                invalid_count++;
            }
        }
        Eigen::MatrixXd jac(invalid_count,joint_num_);
        Eigen::MatrixXd jac_inv(joint_num_,invalid_count);
        Eigen::VectorXd err_vel(invalid_count);
        KDL::Jacobian kdl_jac(joint_num_);
        int error_status;
        double twist_error;
        for(unsigned int i=0;i<max_iter_;i++){
            function(ref_pose,invalid_axis,jnt_out,err_vel);
            //calc error  
            // twist_error=0;
            // for(int i=0;i<invalid_count;i++){
            //     twist_error +=err_vel[i]*err_vel[i];
            // }
            //std::cout<<"error:"<<twist_error<<std::endl;
            bool complete=true;
            for(int k=0;k<invalid_count;k++){
                if(fabs(err_vel[k])>project_error_){
                    complete=false;
                    break;
                }
            }
            if(complete){
                return getError(KDL::SolverI::E_NOERROR);
            }

            jacobian(invalid_axis,jnt_out,jac);
            //inverse jac
            dampedPInv(jac,jac_inv);
            delta_jnt=jac_inv*err_vel;
            jnt_out=jnt_out-delta_jnt;  

            for(unsigned int k=0;k<joint_num_;k++){
                if(kdl_jnt_out(k)<joint_limits_(k,0) || kdl_jnt_out(k)>joint_limits_(k,1) ){
                    kdl_jnt_out(k)=0;//0 is valid as default
                }
            }

        }//end for
        //return getError(KDL::SolverI::E_MAX_ITERATIONS_EXCEEDED );
        //std::cout<<twist_error<<std::endl;
        return getError(-5);
    }

    KDL::Vector KineKdl::eleMulti(const KDL::Vector &vec1,const KDL::Vector &vec2)const{
        return KDL::Vector(vec1.data[0]*vec2.data[0],vec1.data[1]*vec2.data[1],vec1.data[2]*vec2.data[2]);
    }

    bool KineKdl::getError(const int error_status)const{
        //TODO:process error
        // if(error_status==KDL::SolverI::E_NOT_UP_TO_DATE){
        //     std::cout<<"Internal data structures not up to date with chain."<<std::endl;
        // }
        if(error_status<0){
            //std::cout<<"error:kin"<<error_status<<std::endl;
            return false;
        }
        else if(error_status>0){
            //std::cout<<"warning:"<<std::endl;
            return true;
        }
        return true;
    }

}//end namesapce yeebot