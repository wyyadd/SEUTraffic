#ifndef SEUTRAFFIC_ROADNET_H
#define SEUTRAFFIC_ROADNET_H

#include "utility/utility.h"
#include "roadnet/trafficlight.h"
#include "rapidjson/document.h"

#include <vector>
#include <string>
#include <cmath>
#include <list>
#include <map>
#include <queue>
#include <iostream>

namespace SEUTraffic{
    class RoadNet;

    class Intersection;

    class Road;

    class Lane;

    class LaneLink;

    class Vehicle;

    class Intersection {
        friend class RoadNet;

        friend class RoadLink;

        friend class Road;

        friend class TrafficLight;

    private:
        // need reset: roadlinks, trafficlight
        std::string id;
        bool isVirtual;
        double width = 0.0;
        TrafficLight trafficLight;
        std::vector<Road *> roads;
        std::vector<RoadLink> roadLinks; // todo: why not use * here
        std::vector<LaneLink *> laneLinks;
        Point point;

    public:
        std::string getId() const {return this->id;}

        const TrafficLight &getTrafficLight() const { return trafficLight; }

        TrafficLight &getTrafficLight() { return trafficLight; }

        const std::vector<Road *> &getRoads() const { return this->roads; }

        std::vector<Road *> &getRoads() { return this->roads; }

        const std::vector<RoadLink> &getRoadLinks() const { return this->roadLinks; }

        std::vector<RoadLink> &getRoadLinks() { return this->roadLinks; }

        bool isVirtualIntersection() const { return this->isVirtual; }

        const std::vector<LaneLink *> &getLaneLinks();

        void reset();

        RoadLink getRoadLink(Road* lastRoad, Road* curRoad);

        int getMaxpressurePhase(bool isdebug);

        bool isImplicitIntersection() {return trafficLight.getPhases().size() <= 1;}

        // wyy modify: log
        std::vector<Point> getOutline();//yzh:获取Intersection的外轮廓点的坐标

        const Point &getPosition() const { return point; }//yzh:获取Intersection中心点的坐标

    };

    class Road{
        friend class RoadNet;

        friend class Lane;

    private:
        std::string id;
        Intersection *startIntersection = nullptr;
        Intersection *endIntersection = nullptr;
        std::vector<Lane> lanes;
        std::vector<Lane*> lanePointers;
        // wyy modify: add points
        std::vector<Point> points;
        void initLanesPoints();//yzh:根据road坐标计算lane坐标
    public:
        std::string getId() const { return id; }

        Intersection *getStartIntersection() {return this->startIntersection; }

        Intersection *getEndIntersection() { return this->endIntersection; }

        const std::vector<Lane> &getLanes() const { return lanes; }

        void reset();

        double getWidth() const;

        double getLength() const;

        const std::vector<Lane *> & getLanePointers();

        // wyy modify: log
        double averageLength() const;
    };

    enum RoadLinkType{
        go_straight = 3, turn_left = 2, turn_right = 1
    };

    class RoadLink {
        friend class RoadNet;

        friend class LaneLink;

    private:
        Intersection *intersection = nullptr;
        Road *startRoad = nullptr;
        Road *endRoad = nullptr;
        RoadLinkType type;
        std::vector<LaneLink> laneLinks;
        std::vector<LaneLink *> laneLinkPointers;
        int index;
    public:
        const std::vector<LaneLink> &getLaneLinks() const {return this->laneLinks; }

        std::vector<LaneLink> &getLaneLinks() { return this->laneLinks; }

        std::vector<LaneLink *> &getLaneLinkPointers();

        Road *getStartRoad() const {return this->startRoad; }

        Road *getEndRoad() const { return this->endRoad; }

        bool isAvailable() const
        {
            return this->intersection->trafficLight.getCurrentPhase().roadLinkAvailable[this->index];
        }

        bool isTurn() const {
            return this-> type == turn_left || type == turn_right;
        }

        void reset();

        int getIndex() { return index; }

    };

    class Drivable{
        friend class RoadNet;
    public:
        enum DrivableType {
            LANE = 0, LANELINK  = 1
        };

    protected:
        double length;
        double width;
        double maxSpeed;
        std::list<Vehicle *> vehicles;
        DrivableType drivableType;
        // wyy modify: add points
        std::vector<Point> points;

    public:
        virtual ~Drivable() = default;

        const std::list<Vehicle *> &getVehicles() const { return vehicles; }

        std::list<Vehicle *> &getVehicles() {return vehicles;}

        double getLength() const { return length; }

        double getWidth() const { return width; }

        double getMaxSpeed() const { return maxSpeed; }

        size_t getVehicleCnt() const { return vehicles.size(); }

        bool isLane() const { return drivableType == LANE; }

        bool isLaneLink() const { return drivableType == LANELINK;}

        Vehicle *getFirstVehicle() const {
            if (!vehicles.empty()) return vehicles.front();
            return nullptr;
        }

        Vehicle *getLastVehicle() const {
            if (!vehicles.empty()) return vehicles.back();
            return nullptr;
        }

        void pushVehicle(Vehicle *vehicle){
            vehicles.push_back(vehicle);
        }

        void popVehicle(){ vehicles.pop_front(); }

        virtual std::string getId() const = 0;

        DrivableType getDrivableType() const { return drivableType; }//yzh:获得Drivable类型

        // wyy modify: add getPointsByDistance and getDir
        Point getPointByDistance(double dis) const;

        Point getDirectionByDistance(double dis) const;
    };

    class Lane : public Drivable{

        friend class RoadNet;
        friend class Road;

    private:
        int laneIndex;
        std::vector<LaneLink *> laneLinks;
//        std::deque<Vehicle *> waitingBuffer;
        Road *belongRoad = nullptr;//yzh:lane所属road

    public:
        Lane();

        Lane(double width, double maxSpeed, int laneIndex, Road *belongRoad);

        std::string getId() const override{
            return belongRoad->getId() + '_' + std::to_string(getLaneIndex());
        }

        Road *getBelongRoad() const { return this->belongRoad; }

        bool available(const Vehicle *vehicle) const;

        bool canEnter(const Vehicle *vehicle) const;

        size_t getLaneIndex() const {return this-> laneIndex; }

        Intersection *getStartIntersection() const {
            return belongRoad->startIntersection;
        }

        Intersection *getEndIntersection() const {
            return belongRoad->endIntersection;
        }

        std::vector<LaneLink *> getLaneLinksToRoad(const Road *road) const;

        void reset();

        //yzh:add fellow function

        const std::vector<LaneLink *> &getLaneLinks() const { return this->laneLinks; }

        std::vector<LaneLink *> &getLaneLinks() { return this->laneLinks; }

        /* waiting buffer */
//        const std::deque<Vehicle *> &getWaitingBuffer() const { return waitingBuffer; }

//        std::deque<Vehicle *> &getWaitingBuffer() { return waitingBuffer; }

//        void pushWaitingVehicle(Vehicle *vehicle) {
//            waitingBuffer.emplace_back(vehicle);
//        }
    };

    class LaneLink : public Drivable {

        friend class RoadNet;

        friend class Intersection;

    private:
        RoadLink *roadLink = nullptr;
        Lane *startLane = nullptr;
        Lane* endLane = nullptr;

    public:
        std::string getId() const override
        {
            return (startLane ? startLane->getId() : "") + "_TO_" + (endLane ? endLane->getId() : "");
        }

        Lane *getStartLane() const { return startLane; }

        Lane* getEndLane() const { return endLane; }

        // wyy modify: lanelink avaliable
        // yzh：laneLink可用 与 roadLink可用 等价？？？
        bool isAvailable() const { return roadLink->isAvailable(); }

        void reset();

        //yzh:add fellow function  
        LaneLink() {
            width = 4;
            maxSpeed = 10000; //TODO
            drivableType = LANELINK;
        }

        RoadLink *getRoadLink() const { return this->roadLink; }

        RoadLinkType getRoadLinkType() const { return this->roadLink->type; }

        bool isTurn() const { return roadLink->isTurn(); }

    };

    class RoadNet {
    private:
        std::vector<Road> roads;
        std::vector<Intersection> intersections;
        std::map<std::string, Road *> roadMap;
        std::map<std::string, Intersection *> interMap;
        std::map<std::string, Drivable *> drivableMap;

        std::vector<Lane *> lanes;
        std::vector<LaneLink *> laneLinks;
        std::vector<Drivable*> drivables;
        std::vector<std::string> interIds;

        // wyy modify: get Points
        static Point getPoint(const Point &p1, const Point &p2, double a);

    public:
        bool loadFromJson(const std::string& jsonFileName);

        rapidjson::Value convertToJson(rapidjson::Document::AllocatorType &allocator);

        std::vector<Road> &getRoads() { return this->roads; }

        std::vector<Intersection>&getIntersections() { return this-> intersections; }

        const std::vector<Drivable*> &getDrivables() const {
            return drivables;
        }

        const std::vector<Lane *> &getLanes() const
        {
            return lanes;
        }

        const std::vector<LaneLink *> &getLaneLinks() const {
            return laneLinks;
        }

        void reset();

        Road *getRoadById(const std::string &id) const {
            return roadMap.count(id) > 0 ? roadMap.at(id) : nullptr;
        }

        Intersection* getIntersectionById(std::string id)
        {
            return interMap.count(id) > 0 ? interMap.at(id) : nullptr;
        }

        Drivable *getDrivableById(const std::string &id) const {
            return drivableMap.count(id) > 0 ? drivableMap.at(id) : nullptr;
        }

        std::vector<std::string> getInterIds()
        {
            return this->interIds;
        }
    };
}

#endif //SEUTRAFFIC_ROADNET_H
