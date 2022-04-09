#include "vehicle/vehicleInfo.h"

#include <iostream>
#include <limits>
#include <random>

namespace SEUTraffic {
    Road *VehicleInfo::getFirstRoad() const {
        return router->getFirstRoad();
    }
}
