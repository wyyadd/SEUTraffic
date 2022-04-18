#ifndef SEUTRAFFIC_ROUTER_H
#define SEUTRAFFIC_ROUTER_H

#include <vector>
#include <random>
#include <memory>
#include <deque>
#include <list>

namespace SEUTraffic {
    class Road;

    class Drivable;

    class Lane;

    class Intersection;

    class LaneLink;

    class Vehicle;

    class Engine;

    class Router {
    private:
        std::vector<Road *> route;

        // TODO 看之后用不用， 不用就删了
        std::vector<Intersection *> inters;

    public:
        Router(std::vector<Road *> roads, std::vector<Intersection *> inters);

        std::vector<Drivable *> initRoutePlan();

        static size_t selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes);

        static Lane *selectLane(const Lane *curLane, const std::vector<Lane *> &lanes);

        static LaneLink *selectLaneLink(const Lane *curLane, const std::vector<LaneLink *> &laneLinks);

        Drivable *getFirstDrivable();

        Drivable *getNextDrivable(const Drivable *curDrivable, int curRoadIndex);

        Road *getFirstRoad() const;

        Intersection *getNextInter(); //

        std::vector<Road *> getRoute() const {
            return route;
        }

    };

}
#endif // SEUTRAFFIC_ROUTER_H
