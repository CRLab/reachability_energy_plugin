//######################################################################
//
// GraspIt!
// Copyright (C) 2002-2017  Columbia University in the City of New York.
// All rights reserved.
//
// GraspIt! is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// GraspIt! is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with GraspIt!.  If not, see <http://www.gnu.org/licenses/>.
//
// Author(s):  Iretiayo Akinola
//
// $Id: taskDispatcher.h,v 1.3 2017/08/04 20:15:30 iretiayo Exp $
//
//######################################################################

#include "customEnergyPlugin.h"
#include <iostream>
#include <string>
#include "graspit/EGPlanner/energy/searchEnergyFactory.h"
// #include "contactEnergy.h"
#include "reachabilityEnergy.h"
#include "hybridReachableGraspEnergy.h"


namespace customEnergy {

customEnergyPlugin::customEnergyPlugin()
{
  std::cerr << "Custom energy plugin created\n";
}

customEnergyPlugin::~customEnergyPlugin()
{
  std::cerr << "Custom energy plugin destroyed\n";
}

int customEnergyPlugin::init(int argc, char** argv)
{
    REGISTER_SEARCH_ENERGY_CREATOR("REACHABILITY_ENERGY", ReachabilityEnergy);
    REGISTER_SEARCH_ENERGY_CREATOR("HYBRID_REACHABLE_GRASP_ENERGY", HybridReachableGraspEnergy);
  std::cerr << "Custom energy plugin initialized with args:\n";
  for(int i=0; i < argc; i++)
    {
      std::cerr << argv[i] << std::endl;
    }
  return 0;
}

int customEnergyPlugin::mainLoop()
{
  return 0;
}

}



extern "C" PLUGIN_API Plugin* createPlugin() {
  return new customEnergy::customEnergyPlugin();
}


extern "C" PLUGIN_API std::string getType() {
  return "custom Energy Plugin";
}
