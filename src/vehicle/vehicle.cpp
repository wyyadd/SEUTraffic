#include "vehicle/vehicle.h"
#include "roadnet/roadnet.h"
#include "engine/engine.h"

#include <iostream>
#include <limits>
#include <random>
#include <utility>

namespace SEUTraffic {

    Vehicle::Vehicle(const VehicleInfo &vehicleInfo, std::string id, Engine *engine, Flow *flow)
            : engine(engine), vehicleInfo(vehicleInfo), id(std::move(id)), flow(flow) {
//        controllerInfo.running = true;
        controllerInfo.dis = 0;
        planned = vehicleInfo.getRouter().initRoutePlan();
        controllerInfo.drivable = planned[0]; // 得到第一条Lane
        while (engine->checkPriority(priority = engine->rnd()));
        startTime = engine->getCurrentTime();
    }

    Drivable *Vehicle::getCurDrivable() const {
        return controllerInfo.drivable; // 返回当前的drivable
    }

    Drivable *Vehicle::getNextDrivable() {
        int i = currentDrivableIndex + 1;
        if (i < (int) this->planned.size()) {
            return planned[i];
        } else return nullptr;
    }

    Drivable *Vehicle::getFormerDrivable() {
        if (currentDrivableIndex == 0)
            return nullptr;
        return planned[currentDrivableIndex - 1];
    }

    Intersection *Vehicle::getNextIntersection() {
        return vehicleInfo.getRouter().getNextInter();
    }

    void Vehicle::setLeader(Vehicle *leaderCar) {
        // 设置leader
        controllerInfo.leader = leaderCar;
//        if (leaderCar == nullptr) {
//            controllerInfo.gap = 0;
//        } else {
//            if (leaderCar->getCurDrivable()->getId() == getCurDrivable()->getId())
//                controllerInfo.gap = leaderCar->getDistance() - getDistance() - leaderCar->getLen();
//            else {
//                controllerInfo.gap = leaderCar->getDistance() + getCurDrivable()->getLength() - getDistance() - leaderCar->getLen();
//            }
//        }
    }

    void Vehicle::update() {
        if (buffer.isEndSet) {
            controllerInfo.end = buffer.end;
            controllerInfo.running = !buffer.end;
            buffer.isEndSet = false;
        }

        if (buffer.isDisSet) {
            if (buffer.dis <= controllerInfo.dis)
                controllerInfo.backedDist += controllerInfo.dis - buffer.dis;
            else
                controllerInfo.backedDist = 0;
            controllerInfo.dis = buffer.dis;
            buffer.isDisSet = false;
        }

        if (buffer.isDrivableSet) {
            controllerInfo.drivable = buffer.drivable;
            buffer.isDrivableSet = false;
        }
    }

    void Vehicle::updateLeaderAndGap(Vehicle *leader) {
        this->setLeader(leader);
    }

    std::map<std::string, std::string> Vehicle::getInfo() const {
        std::map<std::string, std::string> info;
        info["running"] = std::to_string(isRunning());
        if (!isRunning())
            return info;

        info["distance"] = std::to_string(getDistance());
        info["speed"] = std::to_string(getSpeed());
        const auto &drivable = getCurDrivable();
        info["drivable"] = drivable->getId();
        const auto &road = drivable->isLane() ? getCurLane()->getBelongRoad() : nullptr;
        if (road) {
            info["road"] = road->getId();
            info["intersection"] = road->getEndIntersection()->getId();
        }
        // add routing info
        std::string route;
        for (const auto r: vehicleInfo.getRouter().getRoute()) {
            route += r->getId() + " ";
        }
        info["route"] = route;

        // is available

        //yzh modify: nextDrivable不一定是lane，所以不一定属于road，逻辑需要修改
        //Road* nextRoad = getNextDrivable()->getBelongRoad();
        //RoadLink rlink = road->getEndIntersection()->getRoadLink(road, nextRoad);
        //info["passable"] = rlink.isAvailable() == true ? "1" : "0" ;

        return info;
    }

    Point Vehicle::getDir() const {
        if (buffer.isDrivableSet)
            return buffer.drivable->getDirectionByDistance(buffer.dis);
        else
            return controllerInfo.drivable->getDirectionByDistance(buffer.isDisSet ? buffer.dis : controllerInfo.dis);
    }

    Point Vehicle::getPoint() const {
        if (buffer.isDrivableSet)
            return buffer.drivable->getPointByDistance(buffer.dis);
        else
            return controllerInfo.drivable->getPointByDistance(buffer.isDisSet ? buffer.dis : controllerInfo.dis);
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

    Lane *Vehicle::getCurLane() const {
        if (getCurDrivable()->isLane()) return (Lane *) getCurDrivable();
        else return nullptr;
    }

    // 参考算法: https://www.csdn.net/tags/MtTaMg4sMjIxODgxLWJsb2cO0O0O.html
    bool Vehicle::ifCrash(Vehicle *v) const {
        if (id == v->getId())
            return false;
        auto point1 = this->getPoint();
        auto point2 = v->getPoint();
        auto dist = calcDist(point1, point2);
        // 宽松了不少， 写得比较糙
        return dist < (this->getWidth() / 2 + v->getWidth() / 2) * 0.5 ;
    }

    void Vehicle::duplicate(Vehicle &v) {
        buffer = v.buffer;
        controllerInfo = v.controllerInfo;
        endTime = v.endTime;
        currentDrivableIndex = v.currentDrivableIndex;
        stopped = v.isStopped();
        priority = v.getPriority();
    }

    void Vehicle::setDrivable(Drivable *drivable) {
        controllerInfo.drivable->popFrontVehicle();
        buffer.drivable = drivable;
        buffer.isDrivableSet = true;
        ++currentDrivableIndex;
    }

    void Vehicle::unsetDrivable() {
        controllerInfo.drivable->pushFrontVehicle(this);
        buffer.drivable->popBackVehicle();
        buffer.drivable = nullptr;
        buffer.isDrivableSet = false;
        --currentDrivableIndex;
    }

    void Vehicle::setEnd(bool end) {
        buffer.end = end;
        buffer.isEndSet = true;
    }
}