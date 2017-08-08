#ifndef _reachabilityenergy_h_
#define _reachabilityenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"
#include "reachableEnergyUtils.h"

#include <eigen3/unsupported/Eigen/CXX11/Tensor>

class ReachabilityEnergy: public SearchEnergy
{
  public:
    double energy() const;
    ReachabilityEnergy();

  protected:
    double potentialQualityEnergy() const;
    double contactEnergy() const;
    double potentialQualityScalingFunction(double dist, double cosTheta)const ;
    double reachableQualityEnergy() const;

  private:
    Eigen::VectorXd isReachableFlagMatrix;
    Eigen::VectorXd stepSize;
    Eigen::MatrixXd objectBaseTrans;
    double alpha;
    double contact_coeff;
    double potential_coeff;
    double reachability_coeff;
    double reach_max;
    double reach_min;


    Eigen::VectorXi dims;
    Eigen::VectorXd mins;
    Eigen::VectorXd steps;

   double* reachSpaceArrayFull;
   double* reachSpaceArraySDF;

   Eigen::Tensor<double, 3, Eigen::RowMajor> reachSpaceTensorFull;
   Eigen::Tensor<double, 3, Eigen::RowMajor> reachSpaceTensorSDF;


    double interpolateReachability(Eigen::VectorXd query) const;

};


#endif
