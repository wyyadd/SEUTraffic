#include <algorithm>
#include <ctime>
#include <iostream>
#include <iterator>
#include <cmath>
#include <mutex>
#include <ostream>
#include <string>
#include <utility>
#include "roadnet/roadnet.h"
#include "engine/engine.h"
#include "utility/utility.h"
#include "vehicle/vehicle.h"
#include "vehicle/vehicleInfo.h"

namespace SEUTraffic {
    // wyy modify: updateLog
    void Engine::updateLog() {
        std::string result;
        for (const Vehicle *vehicle: getRunningVehicles()) {
            Point pos = vehicle->getPoint();
            Point direction = vehicle->getCurDrivable()->getDirectionByDistance(vehicle->getDistance());

            // 依次： x y 方位角 ID length width
            result.append(
                    double2string(pos.x) + " " + double2string(pos.y) + " " +
                    double2string(atan2(direction.y, direction.x)) + " "
                    + vehicle->getId() + " 0 " + double2string(vehicle->getLen()) + " "
                    + double2string(vehicle->getWidth()) + ",");
        }
        result.append(";");

        // 什么用处, 我看了Replay.txt, 每条数据的这部分都是一样的
        // format: for every road, check every laneLink is "lightPhase available" in road's endIntersection
        for (Road &road: roadNet.getRoads()) {
            if (road.getEndIntersection()->isVirtualIntersection())
                continue;
            result.append(road.getId());
            for (const Lane &lane: road.getLanes()) {
                if (lane.getEndIntersection()->isImplicitIntersection()) {
                    result.append(" i");
                    continue;
                }

                bool can_go = true;
                for (LaneLink *laneLink: lane.getLaneLinks()) {
                    if (!laneLink->isAvailable()) {
                        can_go = false;
                        break;
                    }
                }
                result.append(can_go ? " g" : " r");
            }
            result.append(",");
        }
        logOut << result << std::endl;
    }

    Engine::Engine(const std::string &configFile, int threadNum) : threadNum(threadNum), startBarrier(threadNum + 1),
                                                                   endBarrier(threadNum + 1) {
        for (int i = 0; i < threadNum; i++) {
            threadVehiclePool.emplace_back();
            threadRoadPool.emplace_back();
            threadIntersectionPool.emplace_back();
            threadDrivablePool.emplace_back();
        }

        bool success = loadConfig(configFile);
        if (!success) {
            std::cerr << "load config failed" << std::endl;
        }

        currentTime = 0;
        for (int i = 0; i < threadNum; i++) {
            threadPool.emplace_back(&Engine::threadController, this,
                                    std::ref(threadVehiclePool[i]),
                                    std::ref(threadRoadPool[i]),
                                    std::ref(threadIntersectionPool[i]),
                                    std::ref(threadDrivablePool[i]));
        }
    }

    Engine::~Engine() {
        // wyy modify: log
        logOut.close();
        finished = true;
        startBarrier.wait();
        for (auto &thread: threadPool) thread.join();
        for (auto &vehiclePair: vehiclePool) {
            delete vehiclePair.second.first;
        }
    }

    // wyy modify: setLogFile
    void Engine::setLogFile(const std::string &jsonFile, const std::string &logFile) {
        if (!writeJsonToFile(jsonFile, jsonRoot)) {
            std::cerr << "write roadNet log file error" << std::endl;
        }
        logOut.open(logFile);
    }

    bool Engine::loadConfig(const std::string &configFile) {
        rapidjson::Document document;
        if (!readJsonFromFile(configFile, document)) {
            std::cerr << "cannot open config file!" << std::endl;
            return false;
        }

        if (!document.IsObject()) {
            std::cerr << "wrong format of config file" << std::endl;
            return false;
        }

        try {
            // interval = getJsonMember<double>("interval", document);
            interval = 1;
            rlTrafficLight = getJsonMember<bool>("rlTrafficLight", document);
            seed = getJsonMember<int>("seed", document);
            rnd.seed(seed);
            dir = getJsonMember<const char *>("dir", document);
            std::string roadnetFile = getJsonMember<const char *>("roadnetFile", document);
            std::string flowFile = getJsonMember<const char *>("flowFile", document);

            if (!loadRoadNet(dir + roadnetFile)) {
                std::cerr << "loading  roadNet file error!" << std::endl;
            }

            if (!loadFlow(dir + flowFile)) {
                std::cerr << "loading flow file error!" << std::endl;
            }
            // wyy modify: save replay
            std::string roadnetLogFile = getJsonMember<const char *>("roadnetLogFile", document);
            std::string replayLogFile = getJsonMember<const char *>("replayLogFile", document);
            setLogFile(dir + roadnetLogFile, dir + replayLogFile);
        } catch (const JsonFormatError &e) {
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool Engine::loadRoadNet(const std::string &jsonFile) {
        bool ans = roadNet.loadFromJson(jsonFile);
        int cnt = 0;

        for (Road &road: roadNet.getRoads()) {
            threadRoadPool[cnt].push_back(&road);
            cnt = (cnt + 1) % threadNum;
        }
        for (Intersection &intersection: roadNet.getIntersections()) {
            threadIntersectionPool[cnt].push_back(&intersection);
            cnt = (cnt + 1) % threadNum;
        }
        for (Drivable *drivable: roadNet.getDrivables()) {
            threadDrivablePool[cnt].push_back(drivable);
            cnt = (cnt + 1) % threadNum;
        }
        // wyy modify: log replay_roadnet json
        jsonRoot.SetObject();
        jsonRoot.AddMember("static", roadNet.convertToJson(jsonRoot.GetAllocator()), jsonRoot.GetAllocator());
        return ans;
    }

    bool Engine::loadFlow(const std::string &jsonFilename) {
        rapidjson::Document root;
        if (!readJsonFromFile(jsonFilename, root)) {
            std::cerr << "cannot open flow file!" << std::endl;
            return false;
        }
        int flowIndex = 0;
        std::list<std::string> path;
        try {
            if (!root.IsArray())
                throw JsonTypeError("flow file", "array");
            for (rapidjson::SizeType i = 0; i < root.Size(); i++) {
                path.emplace_back("flow[" + std::to_string(i) + "]");
                rapidjson::Value &flow = root[i];
                std::vector<Road *> roads;
                std::vector<Intersection *> inters;
                const auto &routes = getJsonMemberArray("route", flow);
                roads.reserve(routes.Size());
                for (auto &route: routes.GetArray()) {
                    path.emplace_back("route[" + std::to_string(roads.size()) + "]");
                    if (!route.IsString()) {
                        throw JsonTypeError("route", "string");
                    }
                    std::string roadName = route.GetString();
                    auto road = roadNet.getRoadById(roadName);
                    if (!road)
                        throw JsonFormatError(
                                "No such road: " + roadName);
                    Intersection *endInter = road->getEndIntersection();
                    inters.push_back(endInter); // todo: which
                    roads.push_back(road);
                    path.pop_back();
                }
                auto router = std::make_shared<const Router>(roads, inters);

                const auto &vehicle = getJsonMemberObject("vehicle", flow);
                auto len = getJsonMember<double>("length", vehicle);
                auto width = getJsonMember<double>("width", vehicle);
                auto startTime = getJsonMember<int>("startTime", flow, 0);
                auto endTime = getJsonMember<int>("endTime", flow, -1);
                auto minGap = getJsonMember<double>("minGap", vehicle);
                VehicleInfo vehicleInfo(len, width, minGap, router);
                Flow newFlow(vehicleInfo,
                             getJsonMember<double>("interval", flow), this, startTime, endTime,
                             "flow_" + std::to_string(i));
                flows.push_back(newFlow);
                path.pop_back();
            }
            assert(path.empty());
        }
        catch (const JsonFormatError &e) {
            std::cerr << "Error occurred when reading flow file" << std::endl;
            for (const auto &node: path) {
                std::cerr << "/" << node;
            }
            std::cerr << " " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    void Engine::vehicleControl(Vehicle &vehicle) {
        //todo
    }

    void Engine::getAction() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::updateAction() {
        startBarrier.wait();
        endBarrier.wait();
//        vehicleRemoveBuffer.clear();// todo： 可以先不管
    }

    // wyy: function_主线程——更新每辆车的dist和变道， 设置结束等信息
    void Engine::updateLocation() {
        updateLocationFlag = false;
        std::sort(pushBuffer.begin(), pushBuffer.end(), vehicleCmp);
        for (auto &vehiclePair: pushBuffer) {
            Vehicle *vehicle = vehiclePair.first;
            Drivable *curDrivable = vehicle->getCurDrivable();
            Drivable *nextDrivable = vehicle->getNextDrivable();
            Vehicle *leader = vehicle->getLeader();
            double maxPossibleDist = vehiclePair.second;
            double currentDist = vehicle->getDistance();
            double currentDrivableLength = curDrivable->getLength();

            // current vehicle is leader in this drivable
            if (leader == nullptr || leader->hasSetEnd() || leader->getChangedDrivable() != nullptr) {
                double remainDist = currentDrivableLength - maxPossibleDist;
                if (nextDrivable == nullptr) {
                    // drive out of current drivable
                    if (remainDist < 0) {
                        vehicle->setEnd(true);
                        vehicle->setStop(false);
                        vehicle->setEndTime(currentTime + remainDist / vehicle->getSpeed());
                        vehicle->setDrivable(nullptr);
                        vehicleActiveCount--;
                        finishedVehicleCnt++;
                    } else { // still on this drivable
                        vehicle->setDis(maxPossibleDist);
                        vehicle->setStop(false);
                    }
                } else { // exist next drivable
                    auto next_leader = nextDrivable->getLastVehicle();
                    bool canNotGo =
                            curDrivable->isLane() && !dynamic_cast<const LaneLink *>(nextDrivable)->isAvailable();
                    // if next drivable has no vehicles
                    if (next_leader == nullptr || next_leader->hasSetEnd() ||
                        next_leader->getChangedDrivable() != nullptr) {
                        if (remainDist >= 0) {
                            vehicle->setDis(maxPossibleDist);
                            vehicle->setStop(false);
                        } else {
                            // if red light then stop
                            if (canNotGo) {
                                vehicle->setDis(currentDrivableLength);
                                vehicle->setStop(true);
                            } else {
                                vehicle->setDis(-remainDist);
                                vehicle->setDrivable(nextDrivable);
                                vehicle->setStop(false);
                            }
                        }
                        continue;
                    }
                    double safe_distance =
                            next_leader->getBufferDist() - next_leader->getMinGap() - next_leader->getLen();
                    if (remainDist >= 0) { // still on this drivable
                        if (safe_distance >= 0 || (remainDist >= -safe_distance)) { // no overlap
                            vehicle->setDis(maxPossibleDist);
                            vehicle->setStop(false);
                        } else { // overlap
                            vehicle->setDis(std::max(currentDrivableLength + safe_distance, currentDist));
                            vehicle->setStop(true);
                        }
                    } else { // less than 0 means will possibly change drivable
                        remainDist = -remainDist;
                        if (remainDist <= safe_distance) { // no overlap
                            // if red light then stop
                            if (canNotGo) {
                                vehicle->setDis(currentDrivableLength);
                                vehicle->setStop(true);
                            } else {
                                vehicle->setDis(remainDist);
                                vehicle->setDrivable(nextDrivable);
                                vehicle->setStop(false);
                            }
                        } else { // over lap
                            if (safe_distance >= 0) { // still can change drivable
                                // if red light then stop
                                if (canNotGo) {
                                    vehicle->setDis(currentDrivableLength);
                                } else {
                                    vehicle->setDis(safe_distance);
                                    vehicle->setDrivable(nextDrivable);
                                }
                            } else { // cannot change drivable
                                vehicle->setDis(std::max(currentDrivableLength + safe_distance, currentDist));
                            }
                            vehicle->setStop(true);
                        }
                    }
                }
            } else { // has leader
                vehicle->setDis(std::max(currentDist,
                                         std::min(maxPossibleDist,
                                                  leader->getBufferDist() - leader->getMinGap() - leader->getLen())));
                vehicle->setStop(maxPossibleDist != vehicle->getBufferDist());
            }
        }
        updateLocationFlag = true;
        startBarrier.wait(); // 等运行完上述的逻辑后，让子线程对包含的vehicle进行删除换道操作
//        endBarrier.wait(); // 等子线程的操作运行完之后，对包含的vehicle进行更新新道路操作push vehicle
        for (auto &vehiclePair: pushBuffer) {
            Vehicle *vehicle = vehiclePair.first;
            Drivable *drivable = vehicle->getChangedDrivable();
            if (drivable != nullptr) {
                drivable->pushVehicle(vehicle);
            }
        }
        pushBuffer.clear();
        //对包含的vehicle进行更新新道路操作push vehicle
        endBarrier.wait(); // 先主线程的操作，后子线程操作, 因为子线程要进行删除操作， 主线程不能push已经删除的vehicle
    }

    void Engine::addCumulativeTravelTime(double endTime, double startTime) {
        cumulativeTravelTime += endTime - startTime;
    }

    void Engine::updateLeaderAndGap() {
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::threadController(std::set<Vehicle *> &vehicles,
                                  std::vector<Road *> &roads,
                                  std::vector<Intersection *> &intersections,
                                  std::vector<Drivable *> &drivables) {
        while (true) {
            threadGetAction(vehicles);
            if (finished)
                break;
            threadUpdateLocation(drivables);
            threadUpdateAction(vehicles);
            threadUpdateLeaderAndGap(drivables);
        }
    }

    void Engine::step(int interval, std::map<std::string, int> &actions) {
        for (auto inter: roadNet.getIntersections()) {
            std::string interId = inter.getId();
            int phase = actions[interId];
            inter.getTrafficLight().setPhase(phase);
        }

        for (int i = 0; i < interval; i++) { // 表示会保持当前的信号调度运行interval的时间
            nextStep();
        }
    }

    void Engine::step2(int interval) {
        for (int i = 0; i < interval; i++) { // 表示会保持当前的信号调度运行interval的时间
            nextStep();
        }
    }

    void Engine::nextStep() {
        for (auto &flow: flows) {
            flow.nextStep(interval);
        }
        getAction();
        updateLocation();
        updateAction();
        updateLeaderAndGap();

        // update traffic light
        std::vector<Intersection> &intersections = roadNet.getIntersections();
        for (auto &intersection: intersections)
            intersection.getTrafficLight().passTime(interval);

        steps += 1;
        currentTime += 1;

        updateLog();
    }

    bool Engine::checkPriority(size_t priority) {
        return vehiclePool.find(priority) != vehiclePool.end();
    }

    //yzh:将vehicle放入vehiclePool、vehicleMap、threadVehiclePool
    void Engine::pushVehicle(Vehicle *const vehicle, bool pushToDrivable) {
        size_t threadIndex = rnd() % threadNum;
        vehiclePool.emplace(vehicle->getPriority(), std::make_pair(vehicle, threadIndex));
        vehicleMap.emplace(vehicle->getId(), vehicle);
        threadVehiclePool[threadIndex].insert(vehicle);

        auto leader = vehicle->getCurDrivable()->getLastVehicle();
        vehicle->getCurDrivable()->pushVehicle(vehicle); // 道路加入车流
        vehicle->setLeader(leader);
        vehicleActiveCount++;
        totalVehicleCnt++;
//        if (pushToDrivable)
//            ((Lane *) vehicle->getCurDrivable())->pushWaitingVehicle(vehicle);
    }

    // wyy: function-计算每个running的车下一秒应该走的距离， 存到buffer中
    void Engine::threadGetAction(std::set<Vehicle *> &vehicles) {
        startBarrier.wait();
        if (finished)
            return;
        std::vector<std::pair<Vehicle *, double>> buffer;
        for (auto vehicle: vehicles) {
            // wyy:这里curlan和belongRoad有什么用
            Drivable *curLane = vehicle->getCurDrivable();
            if (curLane == nullptr) {
                std::cerr << vehicle->getId() << std::endl;
            }
//            Road* belongRoad = curLane->getBelongRoad();

            if (vehicle->isRunning()) {
                double runningTime;
                if (vehicle->getStartTime() >= currentTime &&
                    vehicle->getStartTime() < currentTime + interval) { // start time
                    runningTime = interval - (vehicle->getStartTime() - currentTime); // 这里注意一个问题就是运行时间应该只会是1
                } else {
                    runningTime = interval; // 默认的interval为1s
                }
                double deltaDist = 0;
                deltaDist = runningTime * vehicle->getSpeed();
                deltaDist += vehicle->getDistance();
                buffer.emplace_back(vehicle, deltaDist);
            }
        }
        std::unique_lock<std::mutex> guard(lock);
        pushBuffer.insert(pushBuffer.end(), buffer.begin(), buffer.end());
        guard.unlock();
        endBarrier.wait();
    }

    // wyy: function_对每个drivable删除已变道或已end的车， 更新removeBuffer，map
    void Engine::threadUpdateLocation(const std::vector<Drivable *> &drivables) {
        startBarrier.wait(); //等主线程执行完后进行删除车辆操作
        assert(updateLocationFlag == true);
        endBarrier.wait(); // 等待主线程
        for (Drivable *drivable: drivables) {  //这里引用的vehicle从drivable的vehicles中拿出来，updatelocation中是从pushbuffer里引用，源头确实是threadvehiclepool，所以在后面再次遍历的时候vehicle的状态改变
            auto &vehicles = drivable->getVehicles(); // 为什么要加上&
            auto vehicleIter = vehicles.begin(); // 这是个指针
            while (vehicleIter != vehicles.end()) {
                Vehicle *vehicle = *vehicleIter;
                if ((vehicle->getChangedDrivable() != nullptr && vehicle->getChangedDrivable() != drivable) ||
                    vehicle->hasSetEnd()) {
                    vehicleIter = vehicles.erase(vehicleIter);
                } else {
                    vehicleIter++;
                }

                if (vehicle->hasSetEnd()) { // 在这里就删除车辆，不知道会不会有问题 TODO
                    std::unique_lock<std::mutex> guard(lock);
//                    vehicleRemoveBuffer.insert(vehicle);
                    vehicleMap.erase(vehicle->getId());
                    auto iter = vehiclePool.find(vehicle->getPriority());
                    threadVehiclePool[iter->second.second].erase(vehicle); //在线程的vehicle池里删去了这个vehicle
                    delete vehicle;
                    vehiclePool.erase(iter);
                    guard.unlock();
                }
            }
        }
//        endBarrier.wait(); // 等待子线程做完删除后，主线程在进行所有的换道更新
    }

    // wyy: function_对每辆车更新buffer中的信息
    void Engine::threadUpdateAction(std::set<Vehicle *> &vehicles) {
        startBarrier.wait();
        for (auto vehicle: vehicles) {
            if (vehicle->isRunning()) {
                vehicle->update();
            }
        }
        endBarrier.wait();
    }

    // wyy: function_对每辆车更新leader和gap
    void Engine::threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables) {
        startBarrier.wait();
        for (Drivable *drivable: drivables) {
            Vehicle *leader = nullptr;
            for (Vehicle *vehicle: drivable->getVehicles()) {
                vehicle->updateLeaderAndGap(leader);
                leader = vehicle;
            }
        }
        endBarrier.wait();
    }

    double Engine::getAverageTravelTime() {
        double tt = cumulativeTravelTime;
        // std::cerr<< " cumu time = " << tt << std::endl;
        int n = finishedVehicleCnt;
        for (auto &vehiclePair: vehiclePool) {
            tt += currentTime - vehiclePair.second.first->getStartTime(); // 思考这里有什么问题
            n++;
        }
        // std::cerr<<" n = " << n << " car total time = " << tt<< std::endl;
        if (n == 0)
            return 0;
        else return tt / n;
    }

    void Engine::setTrafficLightPhase(std::string id, int phaseIndex) {
        roadNet.getIntersectionById(std::move(id))->getTrafficLight().setPhase(phaseIndex);
    }

    size_t Engine::getVehicleCount() const {
        return vehicleActiveCount;
    }

    std::vector<const Vehicle *> Engine::getRunningVehicles(bool includeWaiting) const {
        std::vector<const Vehicle *> ret;
        ret.reserve(vehicleActiveCount);
        for (const auto &vehiclePair: vehiclePool) {
            const Vehicle *vehicle = vehiclePair.second.first;
            if ((vehicle->isStopped() && includeWaiting) || vehicle->isRunning()) {
                ret.emplace_back(vehicle);
            }
        }
        return ret;
    }

    std::vector<std::string> Engine::getVehicles(bool includeWaiting) const {
        std::vector<std::string> ret;
        ret.reserve(vehicleActiveCount);
        for (const Vehicle *vehicle: getRunningVehicles(includeWaiting)) {
            ret.emplace_back(vehicle->getId());
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneVehicleCount() const {
        std::map<std::string, int> ret;
        for (const Lane *lane: roadNet.getLanes()) {
            ret.emplace(lane->getId(), lane->getVehicleCnt());
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneWaitingVehicleCount() const {
        std::map<std::string, int> ret;
        for (const Lane *lane: roadNet.getLanes()) {
            int cnt = 0;
            for (Vehicle *vehicle: lane->getVehicles()) {
                if (vehicle->isStopped()) {
                    cnt++;
                }
            }
            ret.emplace(lane->getId(), cnt);
        }
        return ret;
    }

    std::map<std::string, std::vector<std::string>> Engine::getLaneVehicles() {
        std::map<std::string, std::vector<std::string>> ret;
        for (const Lane *lane: roadNet.getLanes()) {
            std::vector<std::string> vehicles;
            for (Vehicle *vehicle: lane->getVehicles()) { // 怎么更新lane上车辆
                vehicles.push_back(vehicle->getId());
            }
            ret.emplace(lane->getId(), vehicles);
        }
        return ret;
    }

    std::map<std::string, std::string> Engine::getVehicleInfo(const std::string &id) const {
        auto iter = vehicleMap.find(id);
        if (iter == vehicleMap.end()) {
            throw std::runtime_error("Vehicle '" + id + "' not found");
        } else {
            Vehicle *vehicle = iter->second;
            return vehicle->getInfo();
        }
    }

    std::map<std::string, double> Engine::getVehicleSpeed() const {
        std::map<std::string, double> ret;
        for (const Vehicle *vehicle: getRunningVehicles(true)) {
            ret.emplace(vehicle->getId(), vehicle->getSpeed());
        }
        return ret;
    }

    std::map<std::string, double> Engine::getVehicleDistance() const {
        std::map<std::string, double> ret;
        for (const Vehicle *vehicle: getRunningVehicles(true)) {
            ret.emplace(vehicle->getId(), vehicle->getDistance());
        }
        return ret;
    }

    std::string Engine::getLeader(const std::string &vehicleId) const {
        auto iter = vehicleMap.find(vehicleId);
        if (iter == vehicleMap.end()) {
            throw std::runtime_error("Vehicle '" + vehicleId + "'not found");
        } else {
            Vehicle *vehicle = iter->second;
            Vehicle *leader = vehicle->getLeader();
            if (leader) {
                return leader->getId();
            } else
                return "";
        }
    }

    void Engine::reset(bool resetRnd) {
        currentTime = 0;

        for (auto &vehiclePair: vehiclePool) {
            delete vehiclePair.second.first;
        }
        for (auto &pool: threadVehiclePool) {
            pool.clear();
        }
        vehiclePool.clear();
        vehicleMap.clear();
        roadNet.reset();

        finishedVehicleCnt = 0;
        cumulativeTravelTime = 0;
        for (auto &flow: flows) {
            flow.reset();
        }
        steps = 0;
        vehicleActiveCount = 0;
        if (resetRnd) {
            rnd.seed(seed);
        }
    }
}
