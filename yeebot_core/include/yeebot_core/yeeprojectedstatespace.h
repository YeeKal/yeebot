#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
#include "yeebot_core/pose_constraint.h"
namespace ompl{
    namespace base{
        class YeeProjectedStateSpace: public ProjectedStateSpace{
        public:
            double maxStep_;
            YeeProjectedStateSpace(const StateSpacePtr &ambientSpace, const ConstraintPtr &constraint)
            :ProjectedStateSpace(ambientSpace,constraint)
            {
                setName("Yee" + space_->getName());
                maxStep_=0.05*getMaximumExtent();
            }
            ~YeeProjectedStateSpace() override = default;
            void setMaxStep(double dis){
                maxStep_=dis;
            }
            //in projectedstatespace: No collision checking is performed if interpolate is true.
            //here, it is different
            bool discreteGeodesic(const State *from, const State *to, bool interpolate = false,
            std::vector<State *> *geodesic = nullptr) const override{
                bool reach=true;
                // Save a copy of the from state.
                if (geodesic != nullptr)
                {
                    geodesic->clear();
                    geodesic->push_back(cloneState(from));
                }
            
                const double tolerance = delta_;
            
                // No need to traverse the manifold if we are already there.
                double dist, step, total = 0;
                if ((dist = distance(from, to)) <= tolerance)
                    return true;

                //check if dist is bigger than maxStep
                //if interpolate=false, use maxstep
                double max = dist * lambda_;
                if( !interpolate && dist>maxStep_ ){
                    reach=false;
                    max=maxStep_*lambda_;
                }
                
            
                auto previous = cloneState(from);
                auto scratch = allocState();
            
                auto &&svc = si_->getStateValidityChecker();
            
                do
                {
                    WrapperStateSpace::interpolate(previous, to, delta_ / dist, scratch);
            
                    // Project new state onto constraint manifold
                    if (!dynamic_cast<yeebot::PoseConstraint *>(constraint_.get())->projectNotlocal(scratch)                  // not on manifold
                        || !(svc->isValid(scratch))      // not valid
                        || (step = distance(previous, scratch)) > lambda_ * delta_ || step<0.1*delta_)  // deviated
                        break;
            
                    // Check if we have wandered too far
                    total += step;
                    if (total > max)
                        break;
            
                    // Check if we are no closer than before
                    const double newDist = distance(scratch, to);
                    if (newDist >= dist)
                        break;
            
                    dist = newDist;
                    copyState(previous, scratch);
            
                    // Store the new state
                    if (geodesic != nullptr)
                        geodesic->push_back(cloneState(scratch));
            
                } while (dist >= tolerance);
            
                freeState(scratch);
                freeState(previous);
            
                return dist <= tolerance && reach;
            }
        };
        typedef std::shared_ptr<YeeProjectedStateSpace> YeeProjectedStateSpacePtr;
        typedef std::shared_ptr<const YeeProjectedStateSpace> YeeProjectedStateSpaceConstPtr;
    }
}