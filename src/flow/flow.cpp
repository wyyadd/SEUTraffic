

#include "flow/flow.h"
#include "engine/engine.h"
#include "utility/utility.h"
#include <iostream>

namespace SEUTraffic{
    void Flow::nextStep(double timeInterval){
        //todo
        if (isActive) return;
        // if (currentTime > getStartTime() && this->IsActive()) {
        //     return;
        // }
        // if (endTime != -1 && currentTime > endTime)
        //     return;
        if (currentTime >= startTime && startTime < currentTime + timeInterval) {
            // std::cerr <<" flow id = " <<getId() << " start time = " << startTime << " current time = " << currentTime << std::endl;
            if (isActive)
                return;
            Vehicle* vehicle = new Vehicle(id, vehicleTemplate, startTime, engine);
            int priority = vehicle->getPriority();
            while (engine->checkPriority(priority)) priority = engine->rnd();
            vehicle->setPriority(priority);
            engine->pushVehicle(vehicle); //加入车流
            isActive = true;
            return;
        }
        currentTime += timeInterval;
    }

    void Flow::reset(){
        // todo
        nowTime = interval;
        currentTime = 0;
        cnt = 0;
        isActive = false;
    }
}
