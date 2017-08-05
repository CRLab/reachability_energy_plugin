#include "EGPlanner/energy/hybridReachableGraspEnergy.h"
#include "robot.h"
#include "grasp.h"
#include "debug.h"
#include "world.h"
#include "quality.h"
#include "contact/virtualContact.h"


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
    double reachableEnergy = -reachableQualityEnergy();        //NOTE: THIS IS NEGATIVE OF THE REACHABLE ENERGY

//    if (potentialEnergy > 0.0)
//    {
//        return contactEnergy();
//    }

    //  double alpha = 50;
    //  double alpha = rand() / (RAND_MAX + 1.);
    //  cout << "alpha: " << endl;

    //  if(reachableEnergy > 0)
    //      cout << "reachableEnergy: \t" << reachableEnergy << endl;
    //      cout << "potentialEnergy: \t" << potentialEnergy << endl;

#ifdef DEBUG
    cout << "HybridReachableGraspEnergy::energy(): \t" << endl;
    cout << "========================================================================== \t" << endl;
    cout << "contactEnergyVal: \t" << contactEnergyVal << endl;
    cout << "potentialEnergy: \t" << potentialEnergy << endl;
    cout << "reachableEnergy: \t" << reachableEnergy << endl;
#endif

    std::ofstream outfile;
    outfile.open("/home/aal/wrkSpc/reachability_paper_experiments/reachability_exp/data/annealing_data/cpr_energy.txt", std::ios_base::app);
    outfile << contactEnergyVal << ","<< potentialEnergy << ","<< reachableEnergy << "\n";
    outfile.close();

    //  sleep(30);
    return combineEnergies( contactEnergyVal, potentialEnergy, reachableEnergy, 3) ;

//    return potentialEnergy + alpha * (1 - reachableEnergy);
//      return -reachableEnergy;
}


double HybridReachableGraspEnergy::combineEnergies( double contactEnergyVal,
                                                    double potentialEnergy ,
                                                    double reachableEnergy,
                                                    int mix_id
                                                    ) const
{
    switch(mix_id) {
        case 1:
        cout << "case 1:\t contactEnergyVal: "<< endl;
                return contactEnergyVal;
                break;
    case 2:
        cout << "case 2:\t potentialEnergy: "<< endl;
            return potentialEnergy;
            break;
    case 3:
//        cout << "case 3:\t reachableEnergy: "<< endl;
            return reachableEnergy;
            break;
    case 4:
        cout << "case 4:\t (reachable+contact) : "<< endl;
//        cout << "contactEnergyVal: \t" << contactEnergyVal << endl;
//        cout << "reachableEnergy: \t" << reachableEnergy << endl;
        return contactEnergyVal+10.0*reachableEnergy;
            break;
    case 5:
        cout << "case 5:\t (reachable+contact) guided potential: "<< endl;
//        cout << "contactEnergyVal: \t" << contactEnergyVal << endl;
//        cout << "potentialEnergy: \t" << potentialEnergy << endl;
//        cout << "reachableEnergy: \t" << reachableEnergy << endl;
        if (potentialEnergy > 0.0)
            {
                return contactEnergyVal+100.0*reachableEnergy;
            }
            return potentialEnergy;
            break;
    case 6:
        cout << "case 6:\t (reachable+contact) -> guided (reachable+potential): "<< endl;
        if (potentialEnergy > 0.0)
            {
                return contactEnergyVal+1000.0*reachableEnergy;
            }
            return potentialEnergy+100.0*reachableEnergy;
            break;


        default:
        cout << "default case :\t GuidedPotentialQualityEnergy: "<< endl;
            if (potentialEnergy > 0.0)
                {
                    return contactEnergyVal;
                }
                return potentialEnergy;
                break;
    }

}
