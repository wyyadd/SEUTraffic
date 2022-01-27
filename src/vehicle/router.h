
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
        std::vector<Road *> roads;
        std::vector<Intersection *> inters;
        std::vector<Lane*> planned;
        std::vector<Road *> followingRoads;
    public:
        Router(const std::vector<Road *> &roads, const std::vector<Intersection *> &inters);

        void initRoutePlan();

        Lane* getNextLane(int i); // 初始化路线获得经过的lane

        Lane* getNextDrivable(int i); // 在后续的运行中使用，注意不要和上面的混用

        Road* getFirstRoad() const;

        std::vector<Lane*> getPlanned() { return planned; }

        Lane* getFirstDrivable() { return planned[0]; }

        Intersection* getNextInter(); //

        void update(Drivable* drivable);

        std::vector<Road*> getFollowingRoads() const
        {
            return followingRoads;
        }
    };

}
#endif
