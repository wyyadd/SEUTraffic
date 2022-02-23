#include "flow/flow.h"
#include "engine/engine.h"
#include "utility/utility.h"
#include <iostream>

namespace SEUTraffic {
    void Flow::nextStep(double timeInterval) {
        if (!valid) return;
        if (endTime != -1 && currentTime > endTime) return;
        if (currentTime >= startTime) {
            //yzh:当nowTime大于车辆流的产生间隔，产生新的车辆
            while (nowTime >= interval) {
                auto vehicle = new Vehicle(vehicleTemplate, id + "_" + std::to_string(cnt++), engine, this);
                //yzh:确保vehicle的priority唯一
                //yzh:将vehicle放进VehiclePool、VehicleMap、threadVehiclePool
                engine->pushVehicle(vehicle, false);
                //yzh:将vehicle放入FirstRoad的planRouteBuffer中
//                vehicle->getFirstRoad()->addPlanRouteVehicle(vehicle);
                nowTime -= interval;
            }
            nowTime += timeInterval;
        }
        currentTime += timeInterval;
    }

    std::string Flow::getId() const {
        return id;
    }

    void Flow::reset() {
        nowTime = interval;
        currentTime = 0;
        cnt = 0;
    }

}