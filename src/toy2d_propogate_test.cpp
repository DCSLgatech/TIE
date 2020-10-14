#include <ros/ros.h>
#include <ros/package.h>
#include <iostream>
#include <vector>
#include <fstream>

#include <ompl/control/SpaceInformation.h>
#include <ompl/control/spaces/RealVectorControlSpace.h>
#include <ompl/control/planners/rrt/RRT.h>
#include <ompl/control/planners/sst/SST.h>
#include <ompl/base/goals/GoalState.h>
#include <ompl/base/spaces/SE2StateSpace.h>
#include <ompl/control/ODESolver.h>
#include <ompl/base/StateValidityChecker.h>
#include <ompl/base/OptimizationObjective.h>
#include "utilities/OMPLtools.h"
#include "tsst/TSST.h"
#include <Eigen/Core>

#define DIM_STATE 2
#define DIM_CONTROL 1

namespace ob = ompl::base;
namespace oc = ompl::control;
using namespace std;

double k1=0.;
double k2=.5;
double k3=-0.1;
double k4=0.2;

void TOY2D(const oc::ODESolver::StateType& q, const oc::Control* c, oc::ODESolver::StateType& qdot)
{
    // Retrieve control values.  Velocity is the first entry, steering angle is second.
    const double *u = c->as<ompl::control::RealVectorControlSpace::ControlType>()->values;

    qdot.resize(q.size(), 0);
    qdot[0] = k1*q[0]+k2*q[1];            // x-dot
    qdot[1] = 1*u[0]+ k3*q[0]+k4*q[1];            // y-dot
}

bool isStateValid(oc::SpaceInformationPtr si, const ob::State *state)
{
  std::vector<double> reals;
  si->getStateSpace()->copyToReals(reals, state);
  bool free_state=true;
  if((reals[0]>=-1) && (reals[0]<=0) && (reals[1]>=1) && (reals[1]<=4))
  {
    free_state=false;;
  }
  // if((reals[0]>=-0.5) && (reals[0]<=0.5) && (reals[1]>=-2) && (reals[1]<=0))
  // {
  //   free_state=false;;
  // }
  return si->satisfiesBounds(state)&&(free_state);
}

class TimeOptimalObjective : public ob::OptimizationObjective
{
public:
  TimeOptimalObjective(const oc::SpaceInformationPtr& si):ob::OptimizationObjective(si)
  {
    std::cout << "Time Optimal Motion Planning" << '\n';
  }
  // Define the state and motion cost as these are pure virtual funtions
  // Note that Time Optimal objective (and TSST) does not make use of these
  ob::Cost stateCost(const ob::State* state) const
  {
    return ob::Cost(0);
  }
  ob::Cost motionCost(const ob::State *s1, const ob::State *s2) const
  {
    return ob::Cost(0);
  }
};


int main(int argc, char **argv)
{
	//Init all the variables
	ros::init (argc, argv, "toy2d_propogate_test");
  ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle;
  ROS_INFO("toy2d_propogate_test");

  ob::RealVectorBounds bounds(DIM_STATE);
  int spaceBounds[DIM_STATE][2]={{-5,5},
                                {-5,5}};

  for(int i=0;i<DIM_STATE;i++)
  {
    bounds.setLow(i,spaceBounds[i][0]);
    bounds.setHigh(i,spaceBounds[i][1]);
  }

  ob::RealVectorBounds cbounds(DIM_CONTROL);
  double controlMax=0.5;
  cbounds.setLow(-controlMax);
  cbounds.setHigh(controlMax);

  auto space(std::make_shared<ob::RealVectorStateSpace>(DIM_STATE));
  space->setBounds(bounds);
  // create a control space
  auto cspace(std::make_shared<oc::RealVectorControlSpace>(space,DIM_CONTROL));
  // set the bounds for the control space
  cspace->setBounds(cbounds);

  oc::SpaceInformationPtr si(new oc::SpaceInformation(space,cspace));
  si->setStateValidityChecker([si](const ob::State *state) { return isStateValid(si, state); });

  ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &TOY2D));
  // ompl::control::ODESolverPtr odeSolver (new ompl::control::ODEBasicSolver<> (si, &DoubleIntegrator4D));
  si->setStatePropagator(ompl::control::ODESolver::getStatePropagator(odeSolver));
  si->setPropagationStepSize(0.1);  //step size in sec
  si->setMinMaxControlDuration(1,5); //steps
  si->setup();

  ob::ScopedState<> start(space);
  start->as<ob::RealVectorStateSpace::StateType>()->values[0] =-3;
  start->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;
  ob::ScopedState<> goal(space);
  goal->as<ob::RealVectorStateSpace::StateType>()->values[0] = 3;
  goal->as<ob::RealVectorStateSpace::StateType>()->values[1] = 0;

  ob::ProblemDefinitionPtr pdef(new ob::ProblemDefinition(si));
  pdef->setStartAndGoalStates(start, goal,0.5);
  ompl::base::OptimizationObjectivePtr opt = std::make_shared<TimeOptimalObjective>(si);
  pdef->setOptimizationObjective(opt);

  // oc::RRT *plan_pt=new oc::RRT(si);
  oc::TSST *plan_pt=new oc::TSST(si);
  plan_pt->setGoalBias(0.1);
  // plan_pt->setKNearest(false);
  plan_pt->setProblemDefinition(pdef);

  std::string packagePath = ros::package::getPath("tie");
  string backcenterPath=packagePath+"/src/Data/toy2d_backcenter.txt";
  string backshapePath=packagePath+"/src/Data/toy2d_backshape.txt";
  string backshapeLPath=packagePath+"/src/Data/toy2d_backshapeL.txt";
  string fwdcenterPath=packagePath+"/src/Data/toy2d_fwdcenter.txt";
  string fwdshapePath=packagePath+"/src/Data/toy2d_fwdshape.txt";
  string fwdshapeLPath=packagePath+"/src/Data/toy2d_fwdshapeL.txt";
  plan_pt->setBackReachSetPath(backcenterPath,backshapePath,backshapeLPath,10,0.1);
  plan_pt->setFwdReachSetPath(fwdcenterPath,fwdshapePath,fwdshapeLPath,10,0.1);
  plan_pt->setTimeInformedRejection(true);
  plan_pt->setTimeInformedEpsilon(0.5);
  plan_pt->setup();

  ob::PlannerStatus solved;
  int it=0;
  while(solved!=ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION && it<1)
  {
    it++;
    solved = plan_pt->solve(ob::timedPlannerTerminationCondition(10));
  }
  if(solved==ompl::base::PlannerStatus::StatusType::EXACT_SOLUTION)
  {
    ROS_INFO("found exact solution");
  }
  OMPLtools ompl_tools(si);
  // std::string packagePath = ros::package::getPath("sst_informed");
  string filePath=packagePath+"/src/Data/solution.txt";
  ompl_tools.WriteSolutionToFile(filePath,pdef);

  oc::PlannerData *planner_data =new oc::PlannerData(si);
  plan_pt->getPlannerData(*planner_data);
  filePath=packagePath+"/src/Data/states.txt";
  ompl_tools.WriteGraphToFile(filePath,planner_data);

  return 0;
}
