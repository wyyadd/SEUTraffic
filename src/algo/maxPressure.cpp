#include "engine/engine.h"

using namespace SEUTraffic;

int getMaxPressurePhase(Intersection *intersection) {
    double maxPressure = -0x3f3f3f;
    double incomings;
    double outcomings;
    int bestPhaseIndex = 0;

    for (LightPhase phase: intersection->getTrafficLight().getPhases()) {
        incomings = 0;
        outcomings = 0;
        std::vector<bool> roadLinkAvailable = phase.getRoadLinkAvailable();
        std::set<Lane *> startLanes;
        std::set<Lane *> endLanes;

        for (RoadLink rlink: intersection->getRoadLinks()) { // 这里的逻辑写错了
            if (roadLinkAvailable[rlink.getIndex()]) {
                for (auto &lanelink: rlink.getLaneLinks()) {
                    Lane *stlane = lanelink.getStartLane();
                    Lane *edlane = lanelink.getEndLane();
                    startLanes.insert(stlane);
                    endLanes.insert(edlane);
                }
            }
        }

        for (auto stLane: startLanes) {
            int incomingCars = stLane->getVehicleCnt();
            incomings += incomingCars;
        }

        for (auto edLane: endLanes) {
            int outcomingcars = edLane->getVehicleCnt();
            outcomings += outcomingcars;
        }
        double pressure = incomings - outcomings;

        if (pressure > maxPressure) {
            maxPressure = pressure;
            bestPhaseIndex = phase.getPhaseIndex();
        }
        startLanes.clear();
        endLanes.clear();
    }
    return bestPhaseIndex;
}

void maxPressure(Engine *engine) {
    std::vector<Intersection> &intersections = engine->getRoadnet().getIntersections();
    for (auto &intersection: intersections) {
        if (intersection.isVirtualIntersection())
            continue;
        //yzh: TSC：maxPressure算法
        if (!intersection.getTrafficLight().changePhase(engine->getInterval())) {
            intersection.getTrafficLight().passTime(engine->getInterval());
        } else {
            int bestPhase = getMaxPressurePhase(&intersection);
            intersection.getTrafficLight().setPhase(bestPhase);
        }
    }
}