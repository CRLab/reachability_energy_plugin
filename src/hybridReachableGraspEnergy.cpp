#include "hybridReachableGraspEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/quality/quality.h"
#include "graspit/contact/virtualContact.h"


#include <fstream>
//#define DEBUG


//HybridReachableGraspEnergy::HybridReachableGraspEnergy()
//{
// uses the inherited constructor of the reachableEnergy class
//}

double HybridReachableGraspEnergy::energy() const
{
    mHand->getGrasp()->collectVirtualContacts();

    double potentialEnergy = potentialQualityEnergy();
    double contactEnergyVal = contactEnergy();
    // double reachableEnergy = -reachableQualityEnergy();        //NOTE: THIS IS NEGATIVE OF THE REACHABLE ENERGY i.e. we minimimize the negative of the reachability energy
    double reachableEnergy = reachableQualityEnergy();

    // f(x) = m(x - c)
    // R_c = 1/(1 + e^(f(x)))
    // double R_c = 1/(1 + e^(0.765*(reachableEnergy-(-5.71485739629))));
    // double R_p = 1/(1 + e^(-3.072*(reachableEnergy-(-1.20710678119))))

    double R_p = (reachableEnergy - reach_min)/(reach_max- reach_min);
    double R_c = (reach_max - reachableEnergy)/(reach_max- reach_min);

    #ifdef DEBUG
        cout << "HybridReachableGraspEnergy::energy(): \t" << endl;
        cout << "========================================================================== \t" << endl;
        cout << "contactEnergyVal: \t" << contactEnergyVal << endl;
        cout << "potentialEnergy: \t" << potentialEnergy << endl;
        cout << "reachableEnergy: \t" << reachableEnergy << endl;
    #endif

    if (potentialEnergy > 0.0)
        {
            return contactEnergyVal * 10*R_c;
        }
        return potentialEnergy * 10*R_p;


    // double energyCombined = contact_coeff*contactEnergyVal + potential_coeff*potentialEnergy + reachability_coeff*reachableEnergy;
    // return energyCombined;
}
