#include "vehicle/vehicle.h"
#include "roadnet/roadnet.h"
#include "engine/engine.h"

#include <iostream>
#include <limits>
#include <random>

namespace SEUTraffic
{

    Vehicle::Vehicle (const std::string &id, VehicleInfo &vehicleInfo, int startTime, Engine *engine)
        :id(id), vehicleInfo(vehicleInfo) , startTime(startTime), engine(engine){
        controllerInfo.dis = 0;
        controllerInfo.drivable = vehicleInfo.getRouter().getFirstDrivable(); // 得到第一条Lane
        controllerInfo.running = true;
    };

    Vehicle::Vehicle(const VehicleInfo &vehicleInfo, const std::string &id, Engine *engine, Flow *flow)
        : vehicleInfo(vehicleInfo),id(id), engine(engine),flow(flow){
        controllerInfo.dis = 0;
        controllerInfo.drivable = vehicleInfo.getRouter().getFirstDrivable(); // 得到第一条Lane
        controllerInfo.running = true;

        while (engine->checkPriority(priority = engine->rnd()));
        enterTime = engine->getCurrentTime();
    }

    Vehicle::Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine, Flow *flow)
        : vehicleInfo(vehicle.vehicleInfo), controllerInfo(vehicle.controllerInfo),
         buffer(vehicle.buffer), id(id), engine(engine),flow(flow){
        while (engine->checkPriority(priority = engine->rnd()));
        enterTime = vehicle.enterTime;
    }

    Drivable* Vehicle::getCurDrivable() const
    {
        return controllerInfo.drivable; // 返回当前的drivable
    }

    Drivable* Vehicle::getNextDrivable()
    {
        return vehicleInfo.getRouter().getNextDrivable(1);
    }

    Drivable* Vehicle::getNextDrivable() const
    {
        return vehicleInfo.getRouter().getNextDrivable(1);
    }


    Intersection* Vehicle::getNextIntersection()
    {
        return vehicleInfo.getRouter().getNextInter();
    }

    void Vehicle::setLeader(Vehicle* leaderCar)
    {

        // 设置leader
        controllerInfo.leader = leaderCar;
        if (leaderCar == nullptr) {
            controllerInfo.gap = 0;
        } else {
            // wyy Q: if not equal, laneLink?
            if (leaderCar->getCurDrivable()->getId() == getCurDrivable()->getId())
                controllerInfo.gap = leaderCar->getDistance() - getDistance() - leaderCar->getLen();
            else {
                controllerInfo.gap = leaderCar->getDistance() + getCurDrivable()->getLength() - getDistance() - leaderCar->getLen();
            }
        }
    }

    void Vehicle::update()
    {
        // wyy Q: what does this set mean?
        if (buffer.isEndSet) {
            controllerInfo.end = buffer.end;
            if (buffer.end== true)
                controllerInfo.running = false;
            else controllerInfo.running = true;
            buffer.isEndSet = false;
        }

        if (buffer.isDisSet)
        {
            controllerInfo.dis = buffer.dis;
            buffer.isDisSet = false;
        }

        if (buffer.isDrivableSet)
        {
            controllerInfo.drivable = buffer.drivable;
            buffer.isDrivableSet = false;
            vehicleInfo.getRouter().update(controllerInfo.drivable);
        }
    }

    void Vehicle::updateLeaderAndGap(Vehicle *leader)
    {
        this->setLeader(leader);
    }

    std::map<std::string, std::string> Vehicle::getInfo() const
    {
        std::map<std::string, std::string> info;
        info["running"] = std::to_string(isRunning());
        if (!isRunning())
            return info;

        info["distance"] = std::to_string(getDistance());
        info["speed"] = std::to_string(getSpeed());
        const auto& drivable = getCurDrivable();
        info["drivable"] = drivable->getId();
        const auto &road = drivable->isLane() ? getCurLane()->getBelongRoad() : nullptr;
        if (road) {
            info["road"] = road->getId();
            info["intersection"] = road->getEndIntersection()->getId();
        }
        // add routing info
        std::string route;
        for (const auto r : vehicleInfo.getRouter().getFollowingRoads()) {
            route +=r->getId() + " ";
        }
        info["route"] = route;

        // is available

        //yzh modify: nextDrivable不一定是lane，所以不一定属于road，逻辑需要修改
        //Road* nextRoad = getNextDrivable()->getBelongRoad();
        //RoadLink rlink = road->getEndIntersection()->getRoadLink(road, nextRoad);
        //info["passable"] = rlink.isAvailable() == true ? "1" : "0" ;

        return info;
    }

    Point Vehicle::getPoint() const {
        return controllerInfo.drivable->getPointByDistance(controllerInfo.dis);
        // if (!controllerInfo.drivable->isLane()) {
        // if (fabs(laneChangeInfo.offset) < eps || !controllerInfo.drivable->isLane()) {
        //     return controllerInfo.drivable->getPointByDistance(controllerInfo.dis);
        // } else {
        //     assert(controllerInfo.drivable->isLane());
        //     const Lane *lane = static_cast<const Lane*>(controllerInfo.drivable);
        //     Point origin = lane->getPointByDistance(controllerInfo.dis);
        //     Point next;
        //     double percentage;
        //     std::vector<Lane> lans = lane->getBelongRoad()->getLanes();
        //     if (laneChangeInfo.offset > 0) {
        //         next = lans[lane->getLaneIndex() + 1].getPointByDistance(controllerInfo.dis);
        //         percentage = 2 * laneChangeInfo.offset / (lane->getWidth() +
        //                                                   lans[lane->getLaneIndex() + 1].getWidth());
        //     } else {
        //         next = lans[lane->getLaneIndex() - 1].getPointByDistance(controllerInfo.dis);
        //         percentage = -2 * laneChangeInfo.offset / (lane->getWidth() +
        //                                                    lans[lane->getLaneIndex() - 1].getWidth());
        //     }
        //     Point cur;
        //     cur.x = next.x * percentage + origin.x * (1 - percentage);
        //     cur.y = next.y * percentage + origin.y * (1 - percentage);
        //     return cur;
        // }
        // }
    }
}
