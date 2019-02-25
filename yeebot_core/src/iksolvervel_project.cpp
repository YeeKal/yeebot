#include "yeebot_core/iksolvervel_project.h"
#include <kdl/utilities/svd_eigen_HH.hpp>

namespace yeebot{
    IkSolverVel_project::IkSolverVel_project(const KDL:: Chain& chain,const Eigen::Ref<const Eigen::VectorXi>& invalid_axis,double eps,int maxiter)
    :chain_(chain),
    nj_(chain_.getNrOfJoints()),
    jnt2jac_(chain_),
    jac_(nj_),
    eigen_s_(nj_),
    eigen_tmp_(nj_),
    eigen_v_(nj_,nj_),
    tmp_(nj_),
    eps_(eps),
    maxiter_(maxiter),
    nrZeroSigmas_(0),
    svdResult_(0),
    invalid_axis_(invalid_axis)
    {
        updateInvalidDim();
        
    }

    
    //update all caused by the invalid_axis
    //calculate  invalid number
    void IkSolverVel_project::updateInvalidDim(){
        invalid_dim_=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis_(i)) invalid_dim_ ++;
        }
        eigen_a_.resize(invalid_dim_,nj_);
        eigen_u_.resize(invalid_dim_,nj_);
        
    }

    void IkSolverVel_project::updateInternalDataStructures() {
        //TODO::
        //jnt2jac_.updateInternalDataStructures();
        nj_ = chain_.getNrOfJoints();
        jac_.resize(nj_);
       
        eigen_a_.resize(invalid_dim_,nj_);
        eigen_u_.resize(invalid_dim_,nj_);
        eigen_v_.resize(nj_,nj_);
        eigen_s_.resize(nj_);
        eigen_tmp_.resize(nj_);
        tmp_.resize(nj_);
    }
    //reset invalid axis
    void IkSolverVel_project::setInvalidAxis(const Eigen::Ref<const Eigen::VectorXi>& invalid_axis){
        invalid_axis_=invalid_axis;
        updateInvalidDim();
    }
    /*
    low dim of the matrix will reduce the time of svd
    */
    int IkSolverVel_project::CartToJnt(const KDL::JntArray& q_in, const KDL::Twist& v_in, KDL::JntArray& qdot_out){
        if (nj_ !=chain_.getNrOfJoints())
            return (error=-3);//(error =E_NOT_UP_TO_DATE);
 
        if (nj_ != q_in.rows() || nj_ != qdot_out.rows())
            return (error=-4);//(error =E_SIZE_MISMATCH);
        //cal jac
        error = jnt2jac_.JntToJac(q_in,jac_);
        if (error < E_NOERROR) return error;
        /**
         * jac_temp stores the whole jacobian
         * while jac stores the invalid jacobian rows.
         * Although the jac.rows is fixed to 6, we have change the svd size,
         * so svd will only calculate the first invalid jacobian rows. 
         **/
        
        unsigned int invalid_row=0;
        for(unsigned int i=0;i<6;i++){
            if(invalid_axis_(i)){
                eigen_a_.row(invalid_row)=jac_.data.row(i);
                invalid_row++;
            }
        }

        double sum;
        unsigned int i,j;
        // Initialize near zero singular value counter
        nrZeroSigmas_=0;

        svdResult_ = KDL::svd_eigen_HH(eigen_a_,eigen_u_,eigen_s_,eigen_v_,eigen_tmp_,maxiter_);//eps=1e-300
        if (0 != svdResult_)
        {
            qdot_out.data.setZero();
            return (error=-8);//(error = SolverI::E_SVD_FAILED);
        }
        
        // We have to calculate qdot_out = jac_pinv*v_in
        // Using the svd decomposition this becomes(jac_pinv=V*S_pinv*Ut):
        // qdot_out = V*S_pinv*Ut*v_in

        //first we calculate Ut*v_in
        for (i=0;i<eigen_a_.cols();i++) {
            sum = 0.0;
            for (j=0;j<eigen_a_.rows();j++) {
                //sum+= eigen_u(j,i)*v_in(j);
                sum+=eigen_u_.data()[eigen_u_.rows()*i+j]*v_in(j);
            }
            //If the singular value is too small (<eps), don't invert it but
            //set the inverted singular value to zero (truncated svd)
            if ( fabs(eigen_s_.data()[i])<eps_ ) {
                //  Count number of singular values near zero
                tmp_(i)=0;
                ++nrZeroSigmas_;
            }
            else {
                tmp_(i) = sum/eigen_s_.data()[i];
            }
        }

         //tmp is now: tmp=S_pinv*Ut*v_in, we still have to premultiply
        //it with V to get qdot_out
        for (i=0;i<eigen_a_.cols();i++) {
            sum = 0.0;
            for (j=0;j<eigen_a_.cols();j++) {
                sum+=eigen_v_.data()[eigen_a_.cols()*j+i]*tmp_(j);
            }
            //Put the result in qdot_out
            qdot_out(i)=sum;
        }

         // Note if the solution is singular, i.e. if number of near zero
        // singular values is greater than the full rank of jac
        if ( nrZeroSigmas_ > (eigen_a_.cols()-eigen_a_.rows()) ) {
            return (error = E_CONVERGE_POSEVEL_PROJECT_SINGULAR);   // converged but pinv singular
        } else {
            return (error = E_NOERROR);                 // have converged
        }

        // Initialize near zero singular value counter
        nrZeroSigmas_ = 0 ;
    }
const char* IkSolverVel_project::strError(const int error) const{
    if (E_CONVERGE_POSEVEL_PROJECT_SINGULAR == error) 
        return "Converged put pseudo inverse of jacobian is singular.";
    else return SolverI::strError(error);
}

}//end namespace yeebot