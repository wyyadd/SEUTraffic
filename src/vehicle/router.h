
#ifndef CITYFLOW_ROUTER_H
#define CITYFLOW_ROUTER_H

#include <vector>
#include <random>
#include <memory>
#include <deque>
#include <list>

namespace SEUTraffic{
    class Road;
    class Drivable;
    class Lane;
    class Intersection;
    class LaneLink;
    class Vehicle;

    class Router{
    private:
        std::vector<Road *> route;
        std::vector<Intersection *> inters;
        std::vector<Drivable*> planned;
        std::vector<Road *> followingRoads;
    public:
        Router(std::vector<Road *> roads, std::vector<Intersection *> inters);

        void initRoutePlan();

        static size_t selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) ;

        static Lane *selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) ;

        static LaneLink *selectLaneLink(const Lane *curLane, const std::vector<LaneLink*> &laneLinks) ;

        Drivable *getFirstDrivable() const;

        Drivable* initNextDrivable(int i, int curRoadIndex); // 初始化路线获得经过的lane

        Drivable* initNextDrivable(const Drivable *curDrivable, int curRoadIndex) const;

        Drivable* getNextDrivable(int i); // 在后续的运行中使用，注意不要和上面的混用

        Road* getFirstRoad() const;

        std::vector<Drivable*> getPlanned() { return planned; }

        Intersection* getNextInter(); //

//        void update(Drivable* drivable);

        std::vector<Road*> getFollowingRoads() const
        {
            return followingRoads;
        }
    };

}
#endif
