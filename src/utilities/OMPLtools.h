#ifndef OMPL_TOOLS_
#define OMPL_TOOLS_

#include<iostream>
#include<vector>
#include <fstream>

#include <ompl/base/SpaceInformation.h>
#include <ompl/base/ProblemDefinition.h>
#include <ompl/base/Planner.h>
#include <ompl/base/spaces/RealVectorStateSpace.h>
#include <ompl/geometric/PathGeometric.h>
#include <ompl/control/PathControl.h>
#include <ompl/control/PlannerData.h>

// namespace ob = ompl::base;
// namespace og = ompl::geometric;
using namespace std;

class OMPLtools
{
public:

  OMPLtools(ompl::control::SpaceInformationPtr si_);

  void WriteSolutionToFile(string filePath, ompl::base::ProblemDefinitionPtr pdef);

  void WriteGraphToFile(string filePath, ompl::control::PlannerData *planner_data);
private:
  ompl::control::SpaceInformationPtr si;
  unsigned int dim_state;
};
#endif
