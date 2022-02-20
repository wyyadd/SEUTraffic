#include "vehicle/router.h"
#include "roadnet/roadnet.h"

#include <queue>
#include <set>
#include <utility>

namespace SEUTraffic{
    Router::Router(std::vector<Road *> roads, std::vector<Intersection *> inters)
                        :route(std::move(roads)), inters(std::move(inters)) {
        initRoutePlan();
    }

    void Router::initRoutePlan(){
        for(auto & i : route)
            followingRoads.push_back(i);
        planned.push_back(getFirstDrivable());
        for (int i = 0; i < 2 * route.size() - 2; i++) {
            auto drivable = initNextDrivable(i, i/2);
            planned.push_back(drivable);
        }
    }

    Drivable *Router::getFirstDrivable() const {
        const std::vector<Lane *> &lanes = route[0]->getLanePointers();
        if (route.size() == 1) {
            return selectLane(nullptr, lanes);
        } else {
            std::vector<Lane *> candidateLanes;
            for (auto lane : lanes) {
                if (!lane->getLaneLinksToRoad(route[1]).empty()) {
                    candidateLanes.push_back(lane);
                }
            }
            assert(!candidateLanes.empty());
            return selectLane(nullptr, candidateLanes);
        }
    }

    Drivable *Router::initNextDrivable(int i, int curRoadIndex){ // 初始化路线获得经过的lane
        Drivable *ret = initNextDrivable(planned[i], curRoadIndex);
        return ret;
    }

    Drivable *Router::initNextDrivable(const Drivable *curDrivable, int curRoadIndex) const {
        if (curDrivable->isLaneLink()) {
            return dynamic_cast<const LaneLink*>(curDrivable)->getEndLane();
        } else {
            const Lane *curLane = dynamic_cast<const Lane *>(curDrivable);
            auto tmpCurRoad = route.begin() + curRoadIndex;
            while ((*tmpCurRoad) != curLane->getBelongRoad() && tmpCurRoad != route.end()) {
                tmpCurRoad++;
            }
            assert(tmpCurRoad != route.end() && curLane->getBelongRoad() == (*tmpCurRoad));
            if (tmpCurRoad == route.end() - 1) {
                return nullptr;
            } else if (tmpCurRoad == route.end() - 2) {
                std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(*(tmpCurRoad+1));
                return selectLaneLink(curLane, laneLinks);
            } else {
                std::vector<LaneLink *> laneLinks = curLane->getLaneLinksToRoad(*(tmpCurRoad+1));
                std::vector<LaneLink *> candidateLaneLinks;
                for (auto laneLink : laneLinks) {
                    Lane *nextLane = laneLink->getEndLane();
                    if (!nextLane->getLaneLinksToRoad(*(tmpCurRoad+2)).empty()) {
                        candidateLaneLinks.push_back(laneLink);
                    }
                }
                return selectLaneLink(curLane, candidateLaneLinks);
            }
        }
    }

    size_t Router::selectLaneIndex(const Lane *curLane, const std::vector<Lane *> &lanes) {
        assert(!lanes.empty());
        time_t now = time(nullptr);
        if (curLane == nullptr) {
            return now % lanes.size();
        }
        int laneDiff = std::numeric_limits<int>::max();
        size_t selected = -1;
        for (size_t i = 0 ; i < lanes.size() ; ++i) {
            auto curLaneDiff = (int)lanes[i]->getLaneIndex() - (int)curLane->getLaneIndex();
            if (abs(curLaneDiff) < laneDiff) {
                laneDiff = abs(curLaneDiff);
                selected = i;
            }
        }
        return selected;
    }

    Lane *Router::selectLane(const Lane *curLane, const std::vector<Lane *> &lanes) {
        if (lanes.empty()) {
            return nullptr;
        }
        return lanes[selectLaneIndex(curLane, lanes)];
    }

    LaneLink *Router::selectLaneLink(const Lane *curLane, const std::vector<LaneLink*> &laneLinks) {
        if (laneLinks.empty()) {
            return nullptr;
        }
        std::vector<Lane *> lanes;
        for (auto laneLink : laneLinks) {
            lanes.push_back(laneLink->getEndLane());
        }
        return laneLinks[selectLaneIndex(curLane, lanes)];
    }

    Drivable* Router::getNextDrivable(int i) // 在后续的运行中使用，注意不要和上面的混用
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

//    void Router::update(Drivable* drivable)
//    {
//        if (drivable == nullptr) {
//            planned.clear();
//            inters.clear();
//            followingRoads.clear();
//            return;
//        }
//        for (auto it = planned.begin(); it != planned.end(); ) {
//            if ( (*it) == drivable) {
//                break;
//            } else {
//                it = planned.erase(it);
//            }
//        }
//        for (auto it = inters.begin(); it != inters.end(); ) {
//            if ((*it) == drivable->getBelongRoad()->getEndIntersection()) { //这里出现了严重的bug，那么可能是一些奇怪的东西
//                break;
//            } else {
//                it = inters.erase(it);
//            }
//        }
//        for (auto it = followingRoads.begin(); it != followingRoads.end();) {
//            if ((*it) == drivable->getBelongRoad())
//                break;
//            else {
//                it = followingRoads.erase(it);
//            }
//        }
    }
