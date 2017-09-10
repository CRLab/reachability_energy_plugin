#include "reachableFirstHybridGraspEnergy.h"
#include "graspit/robot.h"
#include "graspit/grasp.h"
#include "graspit/debug.h"
#include "graspit/world.h"
#include "graspit/quality/quality.h"
#include "graspit/contact/virtualContact.h"


#include <fstream>
//#define DEBUG


//ReachableFirstHybridGraspEnergy::ReachableFirstHybridGraspEnergy()
//{
// uses the inherited constructor of the reachableEnergy class
//}

double ReachableFirstHybridGraspEnergy::energy() const
{
    mHand->getGrasp()->collectVirtualContacts();

    double contactEnergyVal;
    double potentialEnergy;
    // double reachableEnergy = -reachableQualityEnergy();        //NOTE: THIS IS NEGATIVE OF THE REACHABLE ENERGY i.e. we minimimize the negative of the reachability energy
    double reachableEnergy = reachableQualityEnergy();

    // f(x) = m(x - c)
    // R_c = 1/(1 + e^(f(x)))
    // double R_c = 1/(1 + e^(0.765*(reachableEnergy-(-5.71485739629))));
    // double R_p = 1/(1 + e^(-3.072*(reachableEnergy-(-1.20710678119))))

    // double R_p = (reachableEnergy - reach_min)/(reach_max- reach_min);
    // double R_c = (reach_max - reachableEnergy)/(reach_max- reach_min);

    #ifdef DEBUG
        contactEnergyVal = contactEnergy();
        potentialEnergy = potentialQualityEnergy();
        cout << "ReachableFirstHybridGraspEnergy::energy(): \t" << endl;
        cout << "========================================================================== \t" << endl;
        cout << "contactEnergyVal: \t" << contactEnergyVal << endl;
        cout << "potentialEnergy: \t" << potentialEnergy << endl;
        cout << "reachableEnergy: \t" << reachableEnergy << endl;
    #endif

    double contactMax = 500.0;

    if (reachableEnergy > 0)
    {
        potentialEnergy = potentialQualityEnergy();
        if (potentialEnergy < 0.0)
        {   // good grasp and reachable
            // potential energy range [-2:0]
            // reachable energy range [0:3]
            // result range [-5:0]
            return potentialEnergy - 0.1 * reachableEnergy;
        }
        else
        {
            // bad grasp but reachable
            // contact energy range [20:200]
            // reachable energy range [0:3]
            // result range [20:350]
            contactEnergyVal = contactEnergy();
            return contactEnergyVal + 1 * (reach_max - reachableEnergy);
        }
    }
    else {
        // not reachable at all
            // contact energy range [20:200]
            // reachable energy range [-3:0]
            // result range [20:350]
        contactEnergyVal = contactEnergy();
        return contactEnergyVal - 10*reachableEnergy;

    }

// if (potentialEnergy > 0.0)
//         {
//             // return contactEnergyVal * 1*R_c;
//             return R_c;
//         }
//         // return potentialEnergy * 1*R_p;
//         return R_p;


    // double energyCombined = contact_coeff*contactEnergyVal + potential_coeff*potentialEnergy + reachability_coeff*reachableEnergy;
    // return energyCombined;
}
