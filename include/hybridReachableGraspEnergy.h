#ifndef _hybridreachablegraspenergy_h_
#define _hybridreachablegraspenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"
// #include "reachableEnergyUtils.h"
#include "reachabilityEnergy.h"

class HybridReachableGraspEnergy: public ReachabilityEnergy
{
  public:
    double energy() const;
//    HybridReachableGraspEnergy();
private:
    // double combineEnergies(double contactEnergyVal,
    //                        double potentialEnergy ,
    //                        double reachableEnergy,
    //                        int mix_id
    //                        ) const;

};


#endif
