#ifndef _reachablefirsthybridgraspenergy_h_
#define _reachablefirsthybridgraspenergy_h_

#include "graspit/EGPlanner/energy/searchEnergy.h"
#include "reachabilityEnergy.h"

class ReachableFirstHybridGraspEnergy: public ReachabilityEnergy
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
