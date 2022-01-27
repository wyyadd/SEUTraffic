#ifndef CITYFLOW_FLOW_H
#define CITYFLOW_FLOW_H

#include "vehicle/vehicle.h"
#include <cerrno>
#include <type_traits>

#include <string>
#include <list>
#include <memory>

namespace SEUTraffic {
    class Engine;

    class Flow {

    private:
        VehicleInfo vehicleTemplate;
        std::shared_ptr<const Router> route;
        double interval;
        double nowTime = 0;
        double currentTime = 0;
        int startTime = -1;
        int endTime = -1;
        int cnt = 0;
        bool isActive;
        Engine *engine; //入口指针
        std::string id;

    public:
        Flow(const VehicleInfo &vehicleTemplate, double timeInterval,
        Engine *engine, int startTime, int endTime, const std::string &id)
        : vehicleTemplate(vehicleTemplate), interval(timeInterval),
        startTime(startTime), endTime(endTime), engine(engine), id(id) {
            assert(timeInterval >=1 || (startTime == endTime));
            nowTime = 0; // 初始化
            isActive = false;
        }

        void nextStep(double timeInterval);

        std::string getId() const { return this->id; }

        void reset();

        double getStartTime() const { return this->startTime;}

        bool IsActive() { return this->isActive; }
    };
}

#endif
