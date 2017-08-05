#ifndef _reachabilityenergy_h_
#define _reachabilityenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"
#include "reachableEnergyUtils.h"

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
    // flann::Index<flann::L2<double> > *poseFlannIndex ;
    Eigen::VectorXd isReachableFlagMatrix;
    Eigen::VectorXd stepSize;
//    std::vector<double> stepSize;
    Eigen::MatrixXd objectBaseTrans;
    double alpha;
    double contact_coeff;
    double potential_coeff;
    double reachability_coeff;


    Eigen::VectorXi dims;
    Eigen::VectorXd mins;
    Eigen::VectorXd steps;
//    double* reachSpaceFull;
//    double* reachSpaceSDF;


    double reachSpaceFull[8][13][8][1][16][32];
    double reachSpaceSDF[8][13][8][1][16][32];


    double interpolateReachability(Eigen::VectorXd query) const;

};


#endif
