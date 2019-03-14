/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2008, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Ioan Sucan */


#include <ompl/base/goals/GoalSampleableRegion.h>
#include <ompl/tools/config/SelfConfig.h>
#include <ompl/base/spaces/constraint/ConstrainedStateSpace.h>
#include <ompl/base/spaces/constraint/ProjectedStateSpace.h>
//#include <ompl/util/String.h>
#include "yeebot_core/cbirrt.h"
#include "yeebot_core/yeeprojectedstatespace.h"

ompl::geometric::CBIRRT::CBIRRT(const base::SpaceInformationPtr &si, bool addIntermediateStates)
: base::Planner(si, addIntermediateStates ? "CBIRRTIntermediate" : "CBIRRT")
{
    specs_.recognizedGoal = base::GOAL_SAMPLEABLE_REGION;
    specs_.directed = true;

    Planner::declareParam<double>("range", this, &CBIRRT::setRange, &CBIRRT::getRange, "0.:1.:10000.");
    Planner::declareParam<bool>("intermediate_states", this, &CBIRRT::setIntermediateStates,
                                &CBIRRT::getIntermediateStates, "0,1");

    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
    addIntermediateStates_ = addIntermediateStates;
}

ompl::geometric::CBIRRT::~CBIRRT()
{
    freeMemory();
}

void ompl::geometric::CBIRRT::setup()
{
    Planner::setup();
    tools::SelfConfig sc(si_, getName());
    sc.configurePlannerRange(maxDistance_);

    if (!tStart_)
        tStart_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    if (!tGoal_)
        tGoal_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    tStart_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
    tGoal_->setDistanceFunction([this](const Motion *a, const Motion *b) { return distanceFunction(a, b); });
}

void ompl::geometric::CBIRRT::freeMemory()
{   
    std::cout<<"free memory start\n";
    std::vector<Motion *> motions;

    if (tStart_)
    {
        tStart_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
    std::cout<<"1\n";

    if (tGoal_)
    {
        tGoal_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state != nullptr)
                si_->freeState(motion->state);
            delete motion;
        }
    }
    std::cout<<"free memory end\n";
}

void ompl::geometric::CBIRRT::clear()
{
    Planner::clear();
    sampler_.reset();
    freeMemory();
    if (tStart_)
        tStart_->clear();
    if (tGoal_)
        tGoal_->clear();
    connectionPoint_ = std::make_pair<base::State *, base::State *>(nullptr, nullptr);
    distanceBetweenTrees_ = std::numeric_limits<double>::infinity();
}
bool ompl::geometric::CBIRRT::shortcutPath(PathGeometric &path,unsigned int maxSteps){
    std::vector<base::State *> &states = path.getStates();
    int count=states.size();
    if (count < 3)
         return false;
 
     if (maxSteps == 0)
         maxSteps = path.getStateCount();
    
    bool result=false;
    int maxN=count-1;
    //std::vector<double > dists(count,0.0);

    //0<=p1<=p2-2<p2<=maxN
    for(unsigned int i=0;i<maxSteps;i++){
        maxN=states.size()-1;//update help variables
        int p1=rng_.uniformInt(0,maxN-2);
        int p2=rng_.uniformInt(p1+2,maxN);
        std::vector<base::State *> geodesic;
        bool validGeo=si_->getStateSpace()->as<base::YeeProjectedStateSpace>()->discreteGeodesic(states[p1],states[p2],true,&geodesic);
        //check if valid motion from state[p1] to state[p2]
        if(! validGeo)
            continue;
        geodesic.push_back(states[p2]);//keep the last
        double lastdis=0;
        double newdis=0;
        for(unsigned int k=p1;k<p2;k++){
            lastdis+=si_->distance(states[k],states[k+1]);
        }
        for(unsigned int j=0;j<geodesic.size()-1;j++){
            newdis +=si_->distance(geodesic[j],geodesic[j+1]);
        }
        if(newdis<lastdis){
            for(int p=p1+1;p<p2;p++){
                si_->freeState(states[p]);
            }
            states.erase(states.begin()+p1+1,states.begin()+p2);
            for(unsigned int j=1;j<geodesic.size()-1;j++){
                states.insert(states.begin()+p1+j,si_->cloneState(geodesic[j]));
            }
        }
        for(unsigned int k=1;k<geodesic.size()-1;k++){
            si_->freeState(geodesic[k]);
        }

    }//edn for
    return true;
}
ompl::geometric::CBIRRT::GrowState ompl::geometric::CBIRRT::growTree(TreeData &tree, TreeGrowingInfo &tgi,
                                                                            Motion *rmotion)
{
    /* find closest state in the tree */
    Motion *nmotion = tree->nearest(rmotion);

    /* assume we can reach the state we go towards */
    bool reach = true;

    /* find state to add */
    base::State *dstate = rmotion->state;


    //store  intermediate states and from state
    //TODO: if distance(from, to) is too long.
    std::vector<base::State *> geodesic;
    bool validGeo=si_->getStateSpace()->as<base::YeeProjectedStateSpace>()->discreteGeodesic(nmotion->state,dstate,false,&geodesic);
    //if reach, add the last one
    if(validGeo)
        geodesic.push_back(si_->cloneState(dstate));
    //check if intermediate states generated
    unsigned int n = geodesic.size();
    if(n<2)
        return TRAPPED;

    //add intermediate states
    for(std::size_t i=1;i<n;i++){
        auto *motion = new Motion;
        motion->state=geodesic[i];
        motion->parent=nmotion;
        motion->root=nmotion->root;
        tree->add(motion);
        nmotion=motion;
    }
    tgi.xmotion = nmotion;//the last added motion
    tgi.xstate=nmotion->state;
    
 
    // const base::State *astate = tgi.start ? nmotion->state : dstate;
    // const base::State *bstate = tgi.start ? dstate : nmotion->state;

    // std::vector<base::State *> states;
    // const unsigned int count = si_->getStateSpace()->validSegmentCount(astate, bstate);



    return validGeo ? REACHED : ADVANCED;
}

ompl::base::PlannerStatus ompl::geometric::CBIRRT::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    auto *goal = dynamic_cast<base::GoalSampleableRegion *>(pdef_->getGoal().get());

    if (goal == nullptr)
    {
        OMPL_ERROR("%s: Unknown type of goal", getName().c_str());
        return base::PlannerStatus::UNRECOGNIZED_GOAL_TYPE;
    }

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(si_);
        si_->copyState(motion->state, st);
        motion->root = motion->state;
        tStart_->add(motion);
    }

    if (tStart_->size() == 0)
    {
        OMPL_ERROR("%s: Motion planning start tree could not be initialized!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!goal->couldSample())
    {
        OMPL_ERROR("%s: Insufficient states in sampleable goal region", getName().c_str());
        return base::PlannerStatus::INVALID_GOAL;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();

    OMPL_INFORM("%s: Starting planning with %d states already in datastructure", getName().c_str(),
                (int)(tStart_->size() + tGoal_->size()));

    TreeGrowingInfo tgi;
    tgi.xstate = si_->allocState();

    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    auto *rmotion = new Motion(si_);
    base::State *rstate = rmotion->state;
    bool startTree = true;
    bool solved = false;

    while (!ptc)
    {
        TreeData &tree = startTree ? tStart_ : tGoal_;
        tgi.start = startTree;
        startTree = !startTree;
        TreeData &otherTree = startTree ? tStart_ : tGoal_;

        if (tGoal_->size() == 0 || pis_.getSampledGoalsCount() < tGoal_->size() / 2)
        {
            const base::State *st = tGoal_->size() == 0 ? pis_.nextGoal(ptc) : pis_.nextGoal();
            if (st != nullptr)
            {
                auto *motion = new Motion(si_);
                si_->copyState(motion->state, st);
                motion->root = motion->state;
                tGoal_->add(motion);
            }

            if (tGoal_->size() == 0)
            {
                OMPL_ERROR("%s: Unable to sample any valid states for goal tree", getName().c_str());
                break;
            }
        }

        /* sample random state */
        sampler_->sampleUniform(rstate);
        bool sampleValid=si_->isValid(rstate);
        while(!sampleValid){
            sampler_->sampleUniform(rstate);
            sampleValid=si_->isValid(rstate);
        }

        GrowState gs = growTree(tree, tgi, rmotion);

        if (gs != TRAPPED)
        {
            /* remember which motion was just added */
            Motion *addedMotion = tgi.xmotion;

            /* attempt to connect trees */

            /* if reached, it means we used rstate directly, no need top copy again */
            if (gs != REACHED)
                si_->copyState(rstate, tgi.xstate);//copy xstate to rstate

            GrowState gsc = ADVANCED;
            tgi.start = startTree;
            while (gsc == ADVANCED)
                gsc = growTree(otherTree, tgi, rmotion);
            //gsc = growTree(otherTree, tgi, rmotion);

            /* update distance between trees */
            const double newDist = tree->getDistanceFunction()(addedMotion, otherTree->nearest(addedMotion));
            if (newDist < distanceBetweenTrees_)
            {
                distanceBetweenTrees_ = newDist;
                // OMPL_INFORM("Estimated distance to go: %f", distanceBetweenTrees_);
            }

            Motion *startMotion = startTree ? tgi.xmotion : addedMotion;
            Motion *goalMotion = startTree ? addedMotion : tgi.xmotion;

            /* if we connected the trees in a valid way (start and goal pair is valid)*/
            if (gsc == REACHED && goal->isStartGoalPairValid(startMotion->root, goalMotion->root))
            {
                // it must be the case that either the start tree or the goal tree has made some progress
                // so one of the parents is not nullptr. We go one step 'back' to avoid having a duplicate state
                // on the solution path
                if (startMotion->parent != nullptr)
                    startMotion = startMotion->parent;
                else
                    goalMotion = goalMotion->parent;

                connectionPoint_ = std::make_pair(startMotion->state, goalMotion->state);

                /* construct the solution path */
                Motion *solution = startMotion;
                std::vector<Motion *> mpath1;
                while (solution != nullptr)
                {
                    mpath1.push_back(solution);
                    solution = solution->parent;
                }

                solution = goalMotion;
                std::vector<Motion *> mpath2;
                while (solution != nullptr)
                {
                    mpath2.push_back(solution);
                    solution = solution->parent;
                }

                auto path(std::make_shared<PathGeometric>(si_));
                path->getStates().reserve(mpath1.size() + mpath2.size());
                for (int i = mpath1.size() - 1; i >= 0; --i)
                    path->append(mpath1[i]->state);
                for (auto &i : mpath2)
                    path->append(i->state);
                shortcutPath(*path,0);
                pdef_->addSolutionPath(path, false, 0.0, getName());
                solved = true;
                break;
            }
            // else
            // {
            //     // We didn't reach the goal, but if we were extending the start
            //     // tree, then we can mark/improve the approximate path so far.
            //     if (!startTree)
            //     {
            //         // We were working from the startTree.
            //         double dist = 0.0;
            //         goal->isSatisfied(tgi.xmotion->state, &dist);
            //         if (dist < approxdif)
            //         {
            //             approxdif = dist;
            //             approxsol = tgi.xmotion;
            //         }
            //     }
            // }
        }
    }

    si_->freeState(tgi.xstate);
    si_->freeState(rstate);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states (%u start + %u goal)", getName().c_str(), tStart_->size() + tGoal_->size(),
                tStart_->size(), tGoal_->size());
    //OMPL_INFORM("shortcut path to %u states",static_cast<PathGeometric &>(*(pdef_->getSolutionPath())).getStateCount());

    // if (approxsol && !solved)
    // {
    //     /* construct the solution path */
    //     std::vector<Motion *> mpath;
    //     while (approxsol != nullptr)
    //     {
    //         mpath.push_back(approxsol);
    //         approxsol = approxsol->parent;
    //     }

    //     auto path(std::make_shared<PathGeometric>(si_));
    //     for (int i = mpath.size() - 1; i >= 0; --i)
    //         path->append(mpath[i]->state);
    //     pdef_->addSolutionPath(path, true, approxdif, getName());
    //     return base::PlannerStatus::APPROXIMATE_SOLUTION;
    // }

    return solved ? base::PlannerStatus::EXACT_SOLUTION : base::PlannerStatus::TIMEOUT;
}

void ompl::geometric::CBIRRT::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    if (tStart_)
        tStart_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addStartVertex(base::PlannerDataVertex(motion->state, 1));
        else
        {
            data.addEdge(base::PlannerDataVertex(motion->parent->state, 1), base::PlannerDataVertex(motion->state, 1));
        }
    }

    motions.clear();
    if (tGoal_)
        tGoal_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->parent == nullptr)
            data.addGoalVertex(base::PlannerDataVertex(motion->state, 2));
        else
        {
            // The edges in the goal tree are reversed to be consistent with start tree
            data.addEdge(base::PlannerDataVertex(motion->state, 2), base::PlannerDataVertex(motion->parent->state, 2));
        }
    }

    // Add the edge connecting the two trees
    data.addEdge(data.vertexIndex(connectionPoint_.first), data.vertexIndex(connectionPoint_.second));

    // Add some info.
    data.properties["approx goal distance REAL"] = std::to_string(distanceBetweenTrees_);
}