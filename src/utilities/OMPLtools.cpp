#include "OMPLtools.h"

OMPLtools::OMPLtools(ompl::control::SpaceInformationPtr si_)
{
  si=si_;
  dim_state=si->getStateDimension();
}

void OMPLtools::WriteSolutionToFile(string filePath, ompl::base::ProblemDefinitionPtr pdef)
{
  ompl::base::PathPtr path =pdef->getSolutionPath();
  ompl::control::PathControl path_control = static_cast<ompl::control::PathControl &>(*path);
  ofstream myfile;
  myfile.open (filePath);
  path_control.printAsMatrix(std::cout);
  path_control.printAsMatrix(myfile);
  myfile.close();
}

void OMPLtools::WriteGraphToFile(string filePath, ompl::control::PlannerData *planner_data)
{
  ofstream myfile;
  myfile.open (filePath);
  const ompl::base::StateSpace *space(si->getStateSpace().get());
  for(int j=0;j<planner_data->numVertices();j++)
  {
     ompl::base::PlannerDataVertex parent_vertex = planner_data->getVertex(j);
     const ompl::base::State *parent_state=parent_vertex.getState();
     std::vector<double> parent_reals;
     space->copyToReals(parent_reals, parent_state);
     // std::vector<unsigned int> edge_list;
     // planner_data->getEdges(j,edge_list);
     for(int i=0;i<parent_reals.size();i++)
     {
       if(i==parent_reals.size()-1)
       {
         myfile<<parent_reals[i];
       }
       else
       {
         myfile<<parent_reals[i]<<",";
       }
     }
     myfile<<endl;
     // for(int k=0;k<edge_list.size();k++)
     // {
     //   ompl::base::PlannerDataVertex vertex = planner_data->getVertex(edge_list[k]);
     //   const ompl::base::State *state=vertex.getState();
     //   std::vector<double> reals;
     //   space->copyToReals(reals, state);
     //   for(int i=0;i<reals.size();i++)
     //   {
     //     myfile<<reals[i]<<",";
     //   }
     //   for(int i=0;i<parent_reals.size();i++)
     //   {
     //     if(i==parent_reals.size()-1)
     //     {
     //       myfile<<parent_reals[i];
     //     }
     //     else
     //     {
     //       myfile<<parent_reals[i]<<",";
     //     }
     //   }
     //   myfile<<endl;
     // }
  }
  myfile.close();
}
