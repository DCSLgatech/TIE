/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2015, Rutgers the State University of New Jersey, New Brunswick
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
*   * Neither the name of Rutgers University nor the names of its
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

/* Authors: Zakary Littlefield */

#include "tsst/TSST.h"

#include "ompl/base/goals/GoalState.h"
#include "ompl/base/goals/GoalSampleableRegion.h"
#include "ompl/base/objectives/MinimaxObjective.h"
#include "ompl/base/objectives/MaximizeMinClearanceObjective.h"
#include "ompl/base/objectives/PathLengthOptimizationObjective.h"
#include "ompl/base/objectives/MechanicalWorkOptimizationObjective.h"
#include "ompl/tools/config/SelfConfig.h"
#include "ompl/base/spaces/RealVectorStateSpace.h"
#include "ompl/util/GeometricEquations.h"
#include <limits>
#include <cmath>
#include <fstream>


ompl::control::TSST::TSST(const SpaceInformationPtr &si) : base::Planner(si, "TSST")
{
    specs_.approximateSolutions = true;
    siC_ = si.get();
    prevSolution_.clear();
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();

    Planner::declareParam<double>("goal_bias", this, &TSST::setGoalBias, &TSST::getGoalBias, "0.:.05:1.");
    Planner::declareParam<double>("selection_radius", this, &TSST::setSelectionRadius, &TSST::getSelectionRadius, "0.:.1:"
                                                                                                                "100");
    Planner::declareParam<double>("pruning_radius", this, &TSST::setPruningRadius, &TSST::getPruningRadius, "0.:.1:100");

    addPlannerProgressProperty("iterations INTEGER", [this] { return numIterationsProperty(); });
    addPlannerProgressProperty("Samples INTEGER", [this] { return numSamplesProperty(); });
    addPlannerProgressProperty("motions", [this] { return numMotionsProperty(); });
    addPlannerProgressProperty("best cost REAL", [this] { return bestCostProperty(); });
}

ompl::control::TSST::~TSST()
{
    freeMemory();
}

void ompl::control::TSST::setup()
{
    base::Planner::setup();
    if (!nn_)
        nn_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    nn_->setDistanceFunction([this](const Motion *a, const Motion *b)
                             {
                                 return distanceFunction(a, b);
                             });
    if (!witnesses_)
        witnesses_.reset(tools::SelfConfig::getDefaultNearestNeighbors<Motion *>(this));
    witnesses_->setDistanceFunction([this](const Motion *a, const Motion *b)
                                    {
                                        return distanceFunction(a, b);
                                    });

    if (pdef_)
    {
        if (pdef_->hasOptimizationObjective())
        {
            opt_ = pdef_->getOptimizationObjective();
            if (dynamic_cast<base::MaximizeMinClearanceObjective *>(opt_.get()) ||
                dynamic_cast<base::MinimaxObjective *>(opt_.get()))
                OMPL_WARN("%s: Asymptotic near-optimality has only been proven with Lipschitz continuous cost "
                          "functions w.r.t. state and control. This optimization objective will result in undefined "
                          "behavior",
                          getName().c_str());
        }
        else
        {
            OMPL_WARN("%s: No optimization object set. Using path length", getName().c_str());
            opt_ = std::make_shared<base::PathLengthOptimizationObjective>(si_);
            pdef_->setOptimizationObjective(opt_);
        }
    }

    prevSolutionCost_ = opt_->infiniteCost();

    if((timeInformedEpsilon_>0)||(useTimeInformedRejection_))
    {
      initializeTimeInformedSampling();
    }
}

void ompl::control::TSST::clear()
{
    Planner::clear();
    sampler_.reset();
    controlSampler_.reset();
    freeMemory();
    if (nn_)
        nn_->clear();
    if (witnesses_)
        witnesses_->clear();
    if (opt_)
        prevSolutionCost_ = opt_->infiniteCost();

    iterations_ = 0;
    n_samples_ = 0;
    spaceMeasure = 0;
}

void ompl::control::TSST::freeMemory()
{
    if (nn_)
    {
        std::vector<Motion *> motions;
        nn_->list(motions);
        for (auto &motion : motions)
        {
            if (motion->state_)
                si_->freeState(motion->state_);
            if (motion->control_)
                siC_->freeControl(motion->control_);
            delete motion;
        }
    }
    if (witnesses_)
    {
        std::vector<Motion *> witnesses;
        witnesses_->list(witnesses);
        for (auto &witness : witnesses)
        {
            delete witness;
        }
    }
    for (auto &i : prevSolution_)
    {
        if (i)
            si_->freeState(i);
    }
    prevSolution_.clear();
    for (auto &prevSolutionControl : prevSolutionControls_)
    {
        if (prevSolutionControl)
            siC_->freeControl(prevSolutionControl);
    }
    prevSolutionControls_.clear();
    prevSolutionSteps_.clear();
}

std::vector<Eigen::VectorXd> ompl::control::TSST::getCenterPoints(std::string filePath,const int stateDim)
{
  std::ifstream myfile;
  myfile.open(filePath);
  float a;
  std::vector<Eigen::VectorXd> center_points;
  Eigen::VectorXd xCenter=Eigen::VectorXd(stateDim);
  int i=0;
  while(myfile>>a)
  {
    if(i>=stateDim)
    {
      center_points.push_back(xCenter);
      xCenter=Eigen::VectorXd(stateDim);
      i=0;
    }
    xCenter(i)=a;
    i++;
  }
  myfile.close();
  return center_points;
}

std::vector<Eigen::MatrixXd> ompl::control::TSST::getShapeMats(std::string filePath,const int stateDim)
{
  std::ifstream myfile;
  myfile.open(filePath);
  float a;
  std::vector<Eigen::MatrixXd> shape_mats;
  Eigen::MatrixXd xShape=Eigen::MatrixXd(stateDim,stateDim);
  int i=0;
  int j=0;
  while(myfile>>a)
  {
    if(j>=stateDim)
    {
      j=0;
      i++;
    }
    if(i>=stateDim)
    {
      shape_mats.push_back(xShape);
      // std::cout << xShape << '\n'<<endl;
      xShape=Eigen::MatrixXd(stateDim,stateDim);
      i=0;
      j=0;
    }
    // /std::cout << "i:"<<i<<"j:"<<j << '\n';
    xShape(i,j)=a;
    j++;
  }
  myfile.close();
  return shape_mats;
}

void ompl::control::TSST::initializeTimeInformedSampling()
{
  stateDim=siC_->getStateDimension();
  controlDim=siC_->getControlSpace()->getDimension();
  spaceMeasure=si_->getSpaceMeasure();
  // case1_=0.; case2_=0.; case3_=0.;case4_=0.;
  backcenters=getCenterPoints(backcenterPath,stateDim);
  backshape_mats=getShapeMats(backshapePath,stateDim);
  backshapeL_mats=getShapeMats(backshapeLPath,stateDim);
  for(int i=0;i<backshapeL_mats.size();i++)
  {
    backshape_measure.push_back(abs(backshapeL_mats[i].determinant()));
  }
  std::cout << "backcenters size:" <<backcenters.size();
  std::cout << " backshape size:" <<backshape_mats.size();
  std::cout << " backshapeL size:" <<backshapeL_mats.size()<<"\n";

  fwdcenters=getCenterPoints(fwdcenterPath,stateDim);
  fwdshape_mats=getShapeMats(fwdshapePath,stateDim);
  fwdshapeL_mats=getShapeMats(fwdshapeLPath,stateDim);
  for(int i=0;i<fwdshapeL_mats.size();i++)
  {
    fwdshape_measure.push_back(abs(fwdshapeL_mats[i].determinant()));
  }
  std::cout << "fwdcenters size:" <<fwdcenters.size();
  std::cout << " fwdshape size:" <<fwdshape_mats.size();
  std::cout << " fwdshapeL size:" <<fwdshapeL_mats.size()<<"\n";\

  std::cout << "reach_Tmax:" <<reach_Tmax;
  std::cout << " reach_reso:" <<reach_reso<< '\n';

  xState=Eigen::VectorXd(stateDim);
  xCmS=Eigen::VectorXd(stateDim);
}

void ompl::control::TSST::getTimeInformedSample(base::State *statePtr, double Tcost)
{
  int fwd_ind,back_ind;
  bool succ=false;
  bool check=false;
  double *vec =new double[stateDim];
  t_=rng_.uniformReal(0,std::min(reach_Tmax,Tcost));
  if(rng_.uniform01()<0.5)
  {
    back_ind=(t_/reach_reso);
    fwd_ind=(Tcost-t_)/reach_reso;
    double back_measure=backshape_measure[back_ind];
    double fwd_measure;
    if(Tcost-t_<reach_Tmax)
    {
      fwd_measure=fwdshape_measure[fwd_ind];
      check=true;
    }
    else
    {
      fwd_measure=back_measure+1;
    }
    if(back_measure<fwd_measure)
    {
      xSampleCenter=backcenters[back_ind];
      sampleShapeL=backshapeL_mats[back_ind];
      if(check)
      {
        xCheckCenter=fwdcenters[fwd_ind];
        checkShape=fwdshape_mats[fwd_ind];
      }
    }
    else
    {
      xSampleCenter=fwdcenters[fwd_ind];
      sampleShapeL=fwdshapeL_mats[fwd_ind];
      xCheckCenter=backcenters[back_ind];
      checkShape=backshape_mats[back_ind];
    }
  }
  else
  {
    fwd_ind=(t_/reach_reso);
    back_ind=(Tcost-t_)/reach_reso;
    double fwd_measure=fwdshape_measure[fwd_ind];
    double back_measure;
    if(Tcost-t_<reach_Tmax)
    {
      back_measure=backshape_measure[fwd_ind];
      check=true;
    }
    else
    {
      back_measure=fwd_measure+1;
    }
    if(fwd_measure<back_measure)
    {
      xSampleCenter=fwdcenters[fwd_ind];
      sampleShapeL=fwdshapeL_mats[fwd_ind];
      if(check)
      {
        xCheckCenter=backcenters[back_ind];
        checkShape=backshape_mats[back_ind];
      }
    }
    else
    {
      xSampleCenter=backcenters[back_ind];
      sampleShapeL=backshapeL_mats[back_ind];
      xCheckCenter=fwdcenters[fwd_ind];
      checkShape=fwdshape_mats[fwd_ind];
    }
  }

  for(int att=0;att<10;att++)
  {
    rng_.uniformInBall(1,stateDim,vec);
    xState=sampleShapeL*Eigen::Map<const Eigen::VectorXd>(vec,stateDim)+xSampleCenter;
    if(!check)
    {
      succ=true;
      break;
    }
    else
    {
      xCmS=xState-xCheckCenter;
      if(xCmS.transpose()*checkShape*xCmS<=1)
      {
        succ=true;
        break;
      }
    }
  }
  if(!succ)
  {
    sampler_->sampleUniform(statePtr);
  }
  else
  {
    auto *realstate = static_cast<base::RealVectorStateSpace::StateType *>(statePtr);
    for (unsigned int i = 0; i < stateDim; ++i)
    {
      realstate->values[i] = xState(i);
    }
    si_->enforceBounds(statePtr);
  }
}

bool ompl::control::TSST::vertexInclusion(base::State *state, double t, double Tcost,double steps)
{
  // Probabilistic rejection sampling for vertices
  if(t>Tcost)
  {
    case1_++;
    return false;
  }
  double t2g=Tcost-t;
  if(t2g>reach_Tmax)
  {
    return true;
  }
  auto *realstate = static_cast<base::RealVectorStateSpace::StateType *>(state);
  for(int i=0;i<stateDim;i++)
  {
    xState(i)=realstate->values[i];
  }
  int max_index=(t2g/reach_reso)-1;
  // std::cout << "max_index:" <<max_index<< '\n';
  for(int i=max_index;i>=0;i--)
  {
    xCmS=xState-backcenters[i];
    if(xCmS.transpose()*backshape_mats[i]*xCmS<=1)
    {
      return true;
    }
  }
  return false;
}

ompl::control::TSST::Motion *ompl::control::TSST::selectNode(ompl::control::TSST::Motion *sample)
{
    std::vector<Motion *> ret;
    Motion *selected = nullptr;
    base::Cost bestCost = opt_->infiniteCost();
    nn_->nearestR(sample, selectionRadius_, ret);
    for (auto &i : ret)
    {
        if (!i->inactive_ && opt_->isCostBetterThan(i->accCost_, bestCost))
        {
            bestCost = i->accCost_;
            selected = i;
        }
    }
    if (selected == nullptr)
    {
        int k = 1;
        while (selected == nullptr)
        {
            nn_->nearestK(sample, k, ret);
            for (unsigned int i = 0; i < ret.size() && selected == nullptr; i++)
                if (!ret[i]->inactive_)
                    selected = ret[i];
            k += 5;
        }
    }
    return selected;
}

ompl::control::TSST::Witness *ompl::control::TSST::findClosestWitness(ompl::control::TSST::Motion *node)
{
    if (witnesses_->size() > 0)
    {
        auto *closest = static_cast<Witness *>(witnesses_->nearest(node));
        if (distanceFunction(closest, node) > pruningRadius_)
        {
            closest = new Witness(siC_);
            closest->linkRep(node);
            si_->copyState(closest->state_, node->state_);
            witnesses_->add(closest);
        }
        return closest;
    }
    else
    {
        auto *closest = new Witness(siC_);
        closest->linkRep(node);
        si_->copyState(closest->state_, node->state_);
        witnesses_->add(closest);
        return closest;
    }
}

ompl::base::PlannerStatus ompl::control::TSST::solve(const base::PlannerTerminationCondition &ptc)
{
    checkValidity();
    base::Goal *goal = pdef_->getGoal().get();
    auto *goal_s = dynamic_cast<base::GoalSampleableRegion *>(goal);

    while (const base::State *st = pis_.nextStart())
    {
        auto *motion = new Motion(siC_);
        si_->copyState(motion->state_, st);
        siC_->nullControl(motion->control_);
        nn_->add(motion);
        motion->accCost_ = opt_->identityCost();
        findClosestWitness(motion);
    }

    if (nn_->size() == 0)
    {
        OMPL_ERROR("%s: There are no valid initial states!", getName().c_str());
        return base::PlannerStatus::INVALID_START;
    }

    if (!sampler_)
        sampler_ = si_->allocStateSampler();
    if (!controlSampler_)
        controlSampler_ = siC_->allocControlSampler();

    OMPL_INFORM("%s: Starting planning with %u states already in datastructure", getName().c_str(), nn_->size());
    OMPL_INFORM("using useTimeInformedExploration_:,%f",timeInformedEpsilon_);
    OMPL_INFORM("using useTimeInformedRejection_:%u \n",useTimeInformedRejection_);

    Motion *nmotion;
    Motion *mcmotion;
    Motion *solution = nullptr;
    Motion *approxsol = nullptr;
    double approxdif = std::numeric_limits<double>::infinity();
    bool sufficientlyShort = false;

    auto *rmotion = new Motion(siC_);
    base::State *rstate = rmotion->state_;
    Control *rctrl = rmotion->control_;
    base::State *xstate = si_->allocState();

    iterations_ = 0;
    n_samples_ = 0;
    num_motions=nn_->size();

    while (ptc == false)
    {
        // ompl::time::point start = ompl::time::now();

        /* sample random state (with goal biasing) */
        if (goal_s && rng_.uniform01() < goalBias_ && goal_s->canSample())
        {
            goal_s->sampleGoal(rstate);
        }
        else
        {
          if( (opt_->isFinite(prevSolutionCost_)) && (rng_.uniform01() < timeInformedEpsilon_) )
          {
            getTimeInformedSample(rstate,prevSolutionCost_.value());
          }
          else
          {
            sampler_->sampleUniform(rstate);
          }
        }
        // n_samples_ = ompl::time::seconds(ompl::time::now() - start)*pow(10,6);
        /* find closest state in the tree */
        nmotion = selectNode(rmotion);

        num_motions=nn_->size();

        unsigned int cd;
        if( (opt_->isFinite(prevSolutionCost_)) && (useTimeInformedPropogation_) )
        {
          infDuration=(prevSolutionCost_.value()-nmotion->accCost_.value())/siC_->getPropagationStepSize();
          if(infDuration<=0)
          {
            // iterations_++;
            continue;
          }
          cd = rng_.uniformInt(std::min(unsigned(infDuration), siC_->getMinControlDuration()), std::min(unsigned(infDuration), siC_->getMaxControlDuration()) );
        }
        else
        {
          cd = rng_.uniformInt(siC_->getMinControlDuration(), siC_->getMaxControlDuration() );
        }
        /* sample a random control that attempts to go towards the random state, and also sample a control duration */
        controlSampler_->sample(rctrl);
        unsigned int propCd = siC_->propagateWhileValid(nmotion->state_, rctrl, cd, rstate);

        if (propCd == cd)
        {
            base::Cost incCost(cd*siC_->getPropagationStepSize());
            base::Cost cost = opt_->combineCosts(nmotion->accCost_, incCost);
            if((opt_->isFinite(prevSolutionCost_)) && (useTimeInformedRejection_) )
            {
              acceptMotion=vertexInclusion(rstate,cost.value(),prevSolutionCost_.value(),20);
              if(!acceptMotion)
              {
                // iterations_++;
                continue;
              }
            }
            n_samples_++;
            Witness *closestWitness = findClosestWitness(rmotion);

            if (closestWitness->rep_ == rmotion || opt_->isCostBetterThan(cost, closestWitness->rep_->accCost_))
            {
                Motion *oldRep = closestWitness->rep_;
                /* create a motion */
                auto *motion = new Motion(siC_);
                motion->accCost_ = cost;
                si_->copyState(motion->state_, rmotion->state_);
                siC_->copyControl(motion->control_, rctrl);
                motion->steps_ = cd;
                motion->parent_ = nmotion;
                nmotion->numChildren_++;
                closestWitness->linkRep(motion);

                nn_->add(motion);
                double dist = 0.0;
                bool solv = goal->isSatisfied(motion->state_, &dist);
                if (solv && opt_->isCostBetterThan(motion->accCost_, prevSolutionCost_))
                {
                    approxdif = dist;
                    solution = motion;

                    for (auto &i : prevSolution_)
                        if (i)
                            si_->freeState(i);
                    prevSolution_.clear();
                    for (auto &prevSolutionControl : prevSolutionControls_)
                        if (prevSolutionControl)
                            siC_->freeControl(prevSolutionControl);
                    prevSolutionControls_.clear();
                    prevSolutionSteps_.clear();

                    Motion *solTrav = solution;
                    while (solTrav->parent_ != nullptr)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                        prevSolutionSteps_.push_back(solTrav->steps_);
                        solTrav = solTrav->parent_;
                    }
                    prevSolution_.push_back(si_->cloneState(solTrav->state_));
                    prevSolutionCost_ = solution->accCost_;

                    OMPL_INFORM("Found solution with cost %.2f", solution->accCost_.value());
                    sufficientlyShort = opt_->isSatisfied(solution->accCost_);
                    if (sufficientlyShort)
                        break;
                }
                if (solution == nullptr && dist < approxdif)
                {
                    approxdif = dist;
                    approxsol = motion;

                    for (auto &i : prevSolution_)
                        if (i)
                            si_->freeState(i);
                    prevSolution_.clear();
                    for (auto &prevSolutionControl : prevSolutionControls_)
                        if (prevSolutionControl)
                            siC_->freeControl(prevSolutionControl);
                    prevSolutionControls_.clear();
                    prevSolutionSteps_.clear();

                    Motion *solTrav = approxsol;
                    while (solTrav->parent_ != nullptr)
                    {
                        prevSolution_.push_back(si_->cloneState(solTrav->state_));
                        prevSolutionControls_.push_back(siC_->cloneControl(solTrav->control_));
                        prevSolutionSteps_.push_back(solTrav->steps_);
                        solTrav = solTrav->parent_;
                    }
                    prevSolution_.push_back(si_->cloneState(solTrav->state_));
                }

                if (oldRep != rmotion)
                {
                    while (oldRep->inactive_ && oldRep->numChildren_ == 0)
                    {
                        oldRep->inactive_ = true;
                        nn_->remove(oldRep);

                        if (oldRep->state_)
                            si_->freeState(oldRep->state_);
                        if (oldRep->control_)
                            siC_->freeControl(oldRep->control_);

                        oldRep->state_ = nullptr;
                        oldRep->control_ = nullptr;
                        oldRep->parent_->numChildren_--;
                        Motion *oldRepParent = oldRep->parent_;
                        delete oldRep;
                        oldRep = oldRepParent;
                    }
                }
            }
        }
        iterations_++;
    }

    bool solved = false;
    bool approximate = false;
    if (solution == nullptr)
    {
        solution = approxsol;
        approximate = true;
    }

    if (solution != nullptr)
    {
        /* set the solution path */
        auto path(std::make_shared<PathControl>(si_));
        for (int i = prevSolution_.size() - 1; i >= 1; --i)
            path->append(prevSolution_[i], prevSolutionControls_[i - 1],
                         prevSolutionSteps_[i - 1] * siC_->getPropagationStepSize());
        path->append(prevSolution_[0]);
        solved = true;
        pdef_->addSolutionPath(path, approximate, approxdif, getName());
    }

    si_->freeState(xstate);
    if (rmotion->state_)
        si_->freeState(rmotion->state_);
    if (rmotion->control_)
        siC_->freeControl(rmotion->control_);
    delete rmotion;

    OMPL_INFORM("%s: Created %u states in %u iterations", getName().c_str(), nn_->size(), iterations_);
    OMPL_INFORM("case1: %u, case2: %u, case3: %u, case4: %u ", case1_,case2_,case3_,case4_);

    return {solved, approximate};
}

void ompl::control::TSST::getPlannerData(base::PlannerData &data) const
{
    Planner::getPlannerData(data);

    std::vector<Motion *> motions;
    std::vector<Motion *> allMotions;
    if (nn_)
        nn_->list(motions);

    for (auto &motion : motions)
    {
        if (motion->numChildren_ == 0)
        {
            allMotions.push_back(motion);
        }
    }
    for (unsigned i = 0; i < allMotions.size(); i++)
    {
        if (allMotions[i]->parent_ != nullptr)
        {
            allMotions.push_back(allMotions[i]->parent_);
        }
    }

    double delta = siC_->getPropagationStepSize();

    if (prevSolution_.size() != 0)
        data.addGoalVertex(base::PlannerDataVertex(prevSolution_[0]));

    for (auto m : allMotions)
    {
        if (m->parent_)
        {
            if (data.hasControls())
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_),
                             control::PlannerDataEdgeControl(m->control_, m->steps_ * delta));
            else
                data.addEdge(base::PlannerDataVertex(m->parent_->state_), base::PlannerDataVertex(m->state_));
        }
        else
            data.addStartVertex(base::PlannerDataVertex(m->state_));
    }
}
