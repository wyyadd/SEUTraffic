#ifndef SEUTRAFFIC_FLOW_H
#define SEUTRAFFIC_FLOW_H

#include <iostream>
#include <utility>

#include "vehicle/vehicle.h"
#include "vehicle/router.h"

namespace SEUTraffic {
    class Engine;

    class VehicleInfo;

    class Flow {
    private:
        VehicleInfo vehicleTemplate;

        double interval;
        double nowTime = 0;//yzh：距离产生上一个车辆流的时间
        double currentTime = 0;//yzh:当前时间
        int startTime = 0;//yzh:开始时间
        int endTime = -1;//yzh:结束时间
        int cnt = 0;
        Engine *engine;
        std::string id;
        bool valid = true;

    public:
        Flow(VehicleInfo vehicleTemplate, double timeInterval,
             Engine *engine, int startTime, int endTime, std::string id)
                : vehicleTemplate(std::move(vehicleTemplate)), interval(timeInterval),
                  startTime(startTime), endTime(endTime), engine(engine), id(std::move(id)) {
            assert(timeInterval >= 1 || (startTime == endTime));
            nowTime = interval;
        }

        void nextStep(double timeInterval);

        std::string getId() const;

        bool isValid() const { return this->valid; }

        void setValid(const bool v) {
            if (this->valid && !v)
                std::cerr << "[warning] Invalid route '" << id << "'. Omitted by default." << std::endl;
            this->valid = v;
        }

        void reset();

    };
}

#endif //SEUTRAFFIC_FLOW_H
