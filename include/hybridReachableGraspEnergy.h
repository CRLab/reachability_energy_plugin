#ifndef _hybridreachablegraspenergy_h_
#define _hybridreachablegraspenergy_h_

#include "EGPlanner/energy/searchEnergy.h"
#include "EGPlanner/energy/reachableEnergyUtils.h"
#include "EGPlanner/energy/reachabilityEnergy.h"

class HybridReachableGraspEnergy: public ReachabilityEnergy
{
  public:
    double energy() const;
//    HybridReachableGraspEnergy();
private:
    double combineEnergies(double contactEnergyVal,
                           double potentialEnergy ,
                           double reachableEnergy,
                           int mix_id
                           ) const;

};


#endif
