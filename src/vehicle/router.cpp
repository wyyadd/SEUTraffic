#include "vehicle/router.h"
#include "roadnet/roadnet.h"

#include <queue>
#include <set>
#include <utility>

namespace SEUTraffic{
    Router::Router(std::vector<Road *> roads, std::vector<Intersection *> inters) : route(std::move(roads)), inters(std::move(inters)){
        initRoutePlan();
    }

    void Router::initRoutePlan(){
        for (int i = 0; i < route.size(); i++) {
            followingRoads.push_back(route[i]);
            Lane *drivable = getNextLane(i);
            planned.push_back(drivable); //todo: 添加指针，不知道可不可以这么加入，感觉应该可以
        }
    }

    Lane *Router::getNextLane(int i){ // 初始化路线获得经过的lane
        if (i == route.size() - 1){
            Road *curRoad = route[i];
            Road *lastRoad = route[i - 1];
            Intersection *lastInter = inters[i-1];
            RoadLink lastRoadLink = lastInter->getRoadLink(lastRoad, curRoad);

            int minIndexDif = std::numeric_limits<int>::max();
            Lane *selectedLane = nullptr;
            for (LaneLink laneLink : lastRoadLink.getLaneLinks()){
                Lane *firstLane = laneLink.getstartLane();
                Lane *nextLane = laneLink.getendLane();
                int curLaneDiff = firstLane->getLaneIndex() - nextLane->getLaneIndex();
                if (abs(curLaneDiff) < minIndexDif){
                    minIndexDif = abs(curLaneDiff);
                    selectedLane = nextLane;
                }
            }
            return selectedLane;
        } else {
            Road *curRoad = route[i];
            Road *nextRoad = route[i + 1];
            Intersection* targetInter = inters[i];
            // 大概怀疑一下这里是空的
            RoadLink rlink = targetInter->getRoadLink(curRoad, nextRoad);
            LaneLink laneLink = rlink.getLaneLinks()[0];
            Lane *drivable = laneLink.getstartLane();
            return drivable;
        }
    }

    Lane* Router::getNextDrivable(int i) // 在后续的运行中使用，注意不要和上面的混用
    {
        if (i < this->planned.size()) {
            return planned[i];
        }
        else return nullptr;
    }


    Intersection *Router::getNextInter() //
    {
        return inters[0];
    }

    Road *Router::getFirstRoad() const {
        return route[0];
    }

    void Router::update(Drivable* drivable)
    {
        if (drivable == nullptr) {
            planned.clear();
            inters.clear();
            followingRoads.clear();
            return;
        }
        for (auto it = planned.begin(); it != planned.end(); ) {
            if ( (*it) == drivable) {
                break;
            } else {
                it = planned.erase(it);
            }
        }
        for (auto it = inters.begin(); it != inters.end(); ) {
            if ((*it) == drivable->getBelongRoad()->getEndIntersection()) { //这里出现了严重的bug，那么可能是一些奇怪的东西
                break;
            } else {
                it = inters.erase(it);
            }
        }
        for (auto it = followingRoads.begin(); it != followingRoads.end();) {
            if ((*it) == drivable->getBelongRoad())
                break;
            else {
                it = followingRoads.erase(it);
            }
        }
    }
}
