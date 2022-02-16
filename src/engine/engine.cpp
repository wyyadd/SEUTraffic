#include <algorithm>
#include <ctime>
#include <iostream>
#include <iterator>
#include <cmath>
#include <mutex>
#include <ostream>
#include <string>
#include "roadnet/roadnet.h"
#include "engine/engine.h"
#include "utility/utility.h"
#include "vehicle/vehicle.h"
#include "vehicle/vehicleInfo.h"
namespace SEUTraffic
{
    // wyy modify: updataLog
    void Engine::updateLog() {
        std::string result;
        for (const Vehicle* vehicle: getRunningVehicles()) {
            Point pos = vehicle->getPoint();
            Point dir = vehicle->getCurDrivable()->getDirectionByDistance(vehicle->getDistance());

            // wyy: 不变道不需要
            // int lc = vehicle->lastLaneChangeDirection();
            result.append(
                    double2string(pos.x) + " " + double2string(pos.y) + " " + double2string(atan2(dir.y, dir.x)) + " "
                            + vehicle->getId() + " " + double2string(vehicle->getLen()) + " "
                            + double2string(vehicle->getWidth()) + ",");
        }
        result.append(";");

        for (Road &road : roadnet.getRoads()) {
            if (road.getEndIntersection()->isVirtualIntersection())
                continue;
            result.append(road.getId());
            for (const Lane &lane : road.getLanes()) {
                if (lane.getEndIntersection()->isImplicitIntersection()){
                    result.append(" i");
                    continue;
                }

                bool can_go = true;
                for (LaneLink *laneLink : lane.getLaneLinks()) {
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

    Engine::Engine(const std::string &configFile, int threadNum) :  threadNum(threadNum), startBarrier(threadNum + 1),
                                                                      endBarrier(threadNum +1){
        for (int i = 0; i < threadNum ; i++){
            threadVehiclePool.emplace_back();
            threadRoadPool.emplace_back();
            threadIntersectionPool.emplace_back();
            threadDrivablePool.emplace_back();
        }

        bool success = loadConfig(configFile);
        if (!success){
            std::cerr << "load config failed" << std::endl;
        }

        currentTime = 0;
        for (int i= 0 ; i < threadNum; i++){
            threadPool.emplace_back(&Engine::threadController, this,
                                    std::ref(threadVehiclePool[i]),
                                    std::ref(threadRoadPool[i]),
                                    std::ref(threadIntersectionPool[i]),
                                    std::ref(threadDrivablePool[i]));
        }
    }

    Engine::~Engine()
    {
        // wyy modify: log
        logOut.close();
        finished = true;
        for (auto& thread : threadPool) {
            thread.join();
        }
        for (auto &vehiclePair : vehiclePool) {
            delete vehiclePair.second.first;
        }
    }

    // wyy modify: setLogFile
    void Engine::setLogFile(const std::string &jsonFile, const std::string &logFile) {
        if (!writeJsonToFile(jsonFile, jsonRoot)) {
            std::cerr << "write roadnet log file error" << std::endl;
        }
        logOut.open(logFile);
    }

    bool Engine::loadConfig(const std::string &configFile){
        rapidjson::Document document;
        if (!readJsonFromFile(configFile, document)){
            std::cerr << "cannot open config file!" << std::endl;
            return false;
        }

        if (!document.IsObject()){
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
            std::string roadnetFile =getJsonMember<const char*>("roadnetFile", document);
            std::string flowFile = getJsonMember<const char*>("flowFile", document);

            if(!loadRoadNet(dir + roadnetFile)){
                std::cerr << "loading  roadnet file error!" << std::endl;
            }

            if (!loadFlow(dir + flowFile)){
                std::cerr << "loading flow file error!" << std::endl;
            }
            // wyy modify: save replay
            std::string roadnetLogFile = getJsonMember<const char*>("roadnetLogFile", document);
            std::string replayLogFile = getJsonMember<const char*>("replayLogFile", document);
            setLogFile(dir + roadnetLogFile, dir + replayLogFile);
        } catch (const JsonFormatError &e){
            std::cerr << e.what() << std::endl;
            return false;
        }
        return true;
    }

    bool Engine::loadRoadNet(const std::string &jsonFile){
        bool ans = roadnet.loadFromJson(jsonFile);
        int cnt = 0;

        for (Road &road : roadnet.getRoads()){
            threadRoadPool[cnt].push_back(&road);
            cnt = (cnt +1) % threadNum;
        }
        for (Intersection &intersection : roadnet.getIntersections()){
            threadIntersectionPool[cnt].push_back(&intersection);
            cnt = (cnt + 1) % threadNum;
        }
        for (Drivable *drivable : roadnet.getDrivables()){
            threadDrivablePool[cnt].push_back(drivable);
            cnt = (cnt +1) % threadNum;
        }
        // wyy modify: log replay_roadnet json
        jsonRoot.SetObject();
        jsonRoot.AddMember("static", roadnet.convertToJson(jsonRoot.GetAllocator()), jsonRoot.GetAllocator());
        return ans;
    }

    bool Engine::loadFlow(const std::string &jsonFilename){
        rapidjson::Document root;
        if (!readJsonFromFile(jsonFilename, root)){
            std::cerr << "cannot open flow file!"<< std::endl;
            return false;
        }
        int flowIndex = 0;
        std::list<std::string> path;
        try {
            if (!root.IsArray())
                throw JsonTypeError("flow file", "array");
            for (rapidjson::SizeType i = 0 ; i< root.Size();i++){
                path.emplace_back("flow[" + std::to_string(i) + "]");
                rapidjson::Value &flow = root[i];
                std::vector<Road *> roads;
                std::vector<Intersection *> inters;
                const auto &routes = getJsonMemberArray("route", flow);
                roads.reserve(routes.Size());
                for (auto &route: routes.GetArray()){
                    path.emplace_back("route[" + std::to_string(roads.size()) + "]");
                    if (!route.IsString()){
                        throw JsonTypeError("route", "string");
                    }
                    std::string roadName = route.GetString();
                    auto road = roadnet.getRoadById(roadName);
                    if (!road)
                        throw JsonFormatError(
                            "No such road: " + roadName);
                    Intersection *endInter = road->getEndIntersection();
                    inters.push_back(endInter); // todo: which
                    roads.push_back(road);
                    path.pop_back();
                }
                Router router(roads, inters);

                const auto &vehicle = getJsonMemberObject("vehicle", flow);
                double len = getJsonMember<double>("length", vehicle);
                double width = getJsonMember<double>("width", vehicle);

                Road* firstRoad = roads[0];
                double startTime = getJsonMember<double>("startTime", flow, 0);
                double endTime = getJsonMember<double>("endTime", flow, -1);

                VehicleInfo vehicleInfo(len, width, router);
                Flow newFlow(vehicleInfo, getJsonMember<double>("interval", flow), this, startTime, endTime,
                            std::to_string(i));
                flows.push_back(newFlow);
                path.pop_back();
            }
            assert(path.empty());
        }
        catch (const JsonFormatError &e)
        {
            std::cerr << "Error occurred when reading flow file" << std::endl;
            for (const auto &node : path)
            {
                std::cerr << "/" << node;
            }
            std::cerr << " " << e.what() << std::endl;
            return false;
        }
        return true;
    }

    void Engine::vehicleControl(Vehicle &vehicle){
        //todo
    }

    void Engine::getAction(){
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::updateAction(){
        startBarrier.wait();
        endBarrier.wait();
        vehicleRemoveBuffer.clear();// todo： 可以先不管
    }

    // wyy: function_主线程——更新每辆车的dist和变道， 设置结束等信息
    void Engine::updateLocation()
    {
        updateLocationFlag = false;
        // wyy Q: Why sort?
        std::sort(pushBuffer.begin(), pushBuffer.end(), vehicleCmp);
        for (auto& vehiclePair : pushBuffer) {
            Vehicle* vehicle = vehiclePair.first;
            // std::cerr << "the " << index << "th car " << std::endl;
            double maxPosblDist = vehiclePair.second;

            Drivable* curLane = vehicle->getCurDrivable();
            Drivable* nextLane = vehicle->getNextDrivable();
            Intersection* nextInter = vehicle->getNextIntersection();
            Vehicle* leader = vehicle->getLeader();
            Road* curRoad = curLane->getBelongRoad(); // TODO: not sure whether problem here.

            if (nextLane == nullptr) { //这里的分支可能有些冗余，我分为了下一条道路是终点和不是终点的情况
                if (maxPosblDist > curLane->getLength()) {
                    if (leader == nullptr || leader->hasSetEnd() || leader->getChangedDrivable() != nullptr) {
                        // wyy Q: what is setEnd
                        double oldDist = vehicle->gettotalDist();
                        vehicle->updateTotalDist(oldDist - vehicle->getDistance() + curLane->getLength());
                        vehicle->setEnd(true);
                        vehicle->setStop(false);
                        double remainDist = curLane->getLength() - vehicle->getDistance();
                        double remainRunningTime = remainDist / vehicle->getSpeed(); //ensure speed here
                        double endtime = remainRunningTime + (double)currentTime;
                        vehicle->setEndTime(endtime);

                        avgSpeedSum += vehicle->getTotalDist() / (vehicle->getEndTime() - vehicle->getStartTime());
                        addCumulativeTravelTime(vehicle->getEndTime() , vehicle->getStartTime());
                        vehicle->setDrivable(nullptr);
                        vehicleActiveCount--;
                        finishedVehicleCnt++;
                        // curLane->popVehicle(); 注释掉的操作打算放到子线程中进行实现
                    } else {
                        double dist = leader->getBufferDist() - vehicle->getMinGap() - vehicle->getLen();
                        double totalDist = vehicle->getTotalDist() - vehicle->getDistance() + dist+ curLane->getLength();
                        vehicle->updateTotalDist(totalDist);
                        vehicle->setDis(dist);
                        if (leader->hasSetStop())
                            vehicle->setStop(true);
                        else
                            vehicle->setStop(false);

                        vehicle->updateTotalDist(totalDist);
                    }
                } else { // 汽车还在这条道路上
                    if (leader == nullptr || leader->hasSetEnd() || leader->getChangedDrivable() != nullptr) {
                        double oldDist = vehicle->getTotalDist(); 
                        vehicle->updateTotalDist(oldDist - vehicle->getDistance() + maxPosblDist);
                        vehicle->setDis(maxPosblDist);
                        vehicle->setStop(false);
                    } else {
                        if (maxPosblDist > leader->getBufferDist() - leader->getMinGap() - leader->getLen()) { // 调整位置
                            double pos = leader->getBufferDist() - leader->getMinGap() - leader->getLen();
                            double oldDist = vehicle->getTotalDist();
                            vehicle->updateTotalDist(oldDist - vehicle->getDistance() + pos);
                            vehicle->setDis(pos);
                            if (leader->hasSetStop())
                                vehicle->setStop(true);
                            else
                                vehicle->setStop(false);
                        } else { // 不超过leader的位置，不调整位置
                            double oldDist = vehicle->getTotalDist();
                            vehicle->updateTotalDist(oldDist - vehicle->getDistance() + maxPosblDist);
                            vehicle->setDis(maxPosblDist);
                            vehicle->setStop(false);
                        }
                    }
                }
                continue;
            }
            nextLane->checkBelongRoad();
            Road* nextRoad = nextLane->getBelongRoad();
            if (leader == nullptr || leader->getChangedDrivable() != nullptr || leader->hasSetEnd()) { // 这里主要考虑的是leader已经换道路了
                if (maxPosblDist > curLane->getLength()){
                    RoadLink rlink = nextInter->getRoadLink(curRoad, nextRoad);
                    if (!rlink.isAvailable()) {
                        double oldDist = vehicle->getTotalDist();
                        vehicle->setStop(true);
                        vehicle->updateTotalDist(oldDist + curLane->getLength() - vehicle->getDistance());
                        vehicle->setDis(curLane->getLength());
                    } else { // 汽车进入下一条车道
                        double deltaDist = maxPosblDist - curLane->getLength();
                        double oldDist = vehicle->getTotalDist();
                        Vehicle* leader = nextLane->getLastVehicle();
                        int nextLaneCarCnt = nextLane->getVehicleCnt();
                        if (leader == nullptr || leader->getBufferDist() >= deltaDist + leader->getMinGap() + leader->getLen()) {
                            vehicle->updateTotalDist(oldDist - vehicle->getDistance() + curLane->getLength() + deltaDist);
                            vehicle->setDis(deltaDist);
                            vehicle->setStop(false);
                            // change lane
                            vehicle->setDrivable(nextLane);
                            // todo:在子线程处理该车流的道路

                            if (nextLane == nullptr) {
                                vehicle->setEnd(true); // 结束
                                std::cerr<<"cur lane length = " << curLane->getLength() <<" vehicle " << vehicle->getId() <<" enter the point. distance = " << vehicle->getDistance()<< std::endl;
                                double remainDist = curLane->getLength() - vehicle->getDistance();
                                double remainRunningTime = remainDist / vehicle->getSpeed();
                                vehicle->setEndTime(remainRunningTime + currentTime);
                                std::cerr << " add time, speed = "<< vehicle->getSpeed() << " cur lane len = " << curLane->getLength() << " vehicle dist = " << vehicle->getDistance() << std::endl;
                                avgSpeedSum += (vehicle->getTotalDist() - deltaDist) / (vehicle->getEndTime() - vehicle->getStartTime());
                                addCumulativeTravelTime(vehicle->getEndTime(), vehicle->getStartTime());
                                vehicle->setDrivable(nullptr);
                                vehicleActiveCount--;
                                finishedVehicleCnt ++;
                            }
                            // else pushvehicle
                        } else {
                            double leaderDist = leader->getBufferDist();
                            deltaDist = leaderDist - leader->getLen() - leader->getMinGap();
                            if (deltaDist <= 0) { // 正好顶在了道路的顶端， 我为毛要把逻辑写的这么奇怪
                                vehicle->updateTotalDist(oldDist - vehicle->getDistance() + curRoad->getLength());
                                vehicle->setDis(curLane->getLength());
                                vehicle->setStop(true); // 停下来了
                                continue;
                            } else {

                                vehicle->updateTotalDist(oldDist - vehicle->getDistance() + curRoad->getLength() + deltaDist);
                                vehicle->setDis(deltaDist);
                                vehicle->setDrivable(nextLane);
                                vehicle->setStop(false);
                            }
                        }
                    }
                } else { // 留在了原来的道路
                    double oldDist = vehicle->getTotalDist();
                    vehicle->setDis(maxPosblDist);
                    vehicle->setStop(false);
                    vehicle->updateTotalDist(oldDist - vehicle->getDistance() + maxPosblDist);
                }
            } else { // leader还留在原来的道路上
                double oldDist = vehicle->getTotalDist();
                if (leader->getBufferDist() >= maxPosblDist + leader->getMinGap() + leader->getLen()) {
                    vehicle->updateTotalDist(oldDist - vehicle->getDistance() + maxPosblDist);
                    vehicle->setDis(maxPosblDist);
                    vehicle->setStop(false);
                } else {
                    double newDist = leader->getBufferDist() - leader->getMinGap() - leader->getLen();
                    vehicle->updateTotalDist(oldDist - vehicle->getDistance() + newDist);
                    vehicle->setDis(newDist);
                    if (leader->hasSetStop()) {
                        vehicle->setStop(true);
                    } else
                        vehicle->setStop(false);
                }
            }
        }
        updateLocationFlag = true;
        startBarrier.wait(); // 等运行完上述的逻辑后，让子线程对包含的vehicle进行删除换道操作
        endBarrier.wait(); // 等子线程的操作运行完之后，对包含的vehicle进行更新新道路操作push vehicle
        for (auto& vehiclePair : pushBuffer) {
            Vehicle* vehicle = vehiclePair.first;
            Drivable* drivable = vehicle->getChangedDrivable();
            if (drivable != nullptr) {
                drivable->pushVehicle(vehicle);
            }
        }
        pushBuffer.clear();
    }

    void Engine::addCumulativeTravelTime(double endTime , double startTime)
    {
        cumulativeTravelTime += endTime - startTime;
    }

    void Engine::updateLeaderAndGap(){
        startBarrier.wait();
        endBarrier.wait();
    }

    void Engine::threadController(std::set<Vehicle*>& vehicles,
        std::vector<Road*>& roads,
        std::vector<Intersection*>& intersections,
        std::vector<Drivable*>& drivables){
        //todo
        while (!finished) {
            threadGetAction(vehicles);
            threadUpdateLocation(drivables);
            threadUpdateAction(vehicles);
            threadupdateLeaderAndGap(drivables);
        }
    }

    void Engine::step(int interval, std::map<std::string, int> &actions)
    {
        for (auto inter : roadnet.getIntersections()) {
            std::string interId = inter.getId();
            int phase = actions[interId];
            inter.getTrafficLight().setPhase(phase);
        }

        for (int i = 0; i < interval; i++) { // 表示会保持当前的信号调度运行interval的时间
            nextStep();
        }
    }

    void Engine::step2(int interval)
    {
        for (int i = 0; i < interval; i++) { // 表示会保持当前的信号调度运行interval的时间
            nextStep();
        }
    }

    void Engine::nextStep()
    {
        for (auto& flow : flows) {
            flow.nextStep(1);
        }
        //std::cerr << "now cars in engine are " << totalVehicleCnt << " cur time = " << currentTime <<std::endl;
        getAction();
        updateLocation();
        updateAction();
        updateLeaderAndGap();
        steps += 1;
        currentTime += 1;

        // wyy modify: updateLog
        updateLog();
    }

    bool Engine::checkPriority(int priority)
    {
        return vehiclePool.find(priority) != vehiclePool.end();
    }

    //yzh:将vehicle放入vehiclePool、vehicleMap、threadVehiclePool
    void Engine::pushVehicle(Vehicle *const vehicle, bool pushToDrivable) {
        size_t threadIndex = rnd() % threadNum;
        vehiclePool.emplace(vehicle->getPriority(), std::make_pair(vehicle, threadIndex));
        vehicleMap.emplace(vehicle->getId(), vehicle);
        threadVehiclePool[threadIndex].insert(vehicle);

        if (pushToDrivable)
            ((Lane *) vehicle->getCurDrivable())->pushWaitingVehicle(vehicle);
    }
    
    // void Engine::pushVehicle(Vehicle *vehicle)
    // {
    //     size_t threadIndex = rnd() % threadNum;
    //     vehiclePool.emplace(vehicle->getPriority(), std::make_pair(vehicle, threadIndex)); // 加入车流
    //     if (vehicleMap.count(vehicle->getId())) {
    //         std::cerr<<"repeat insert existed vehicle"<<std::endl;
    //         throw "repeat insert existed vehicle";
    //     }

    //     vehicleMap.emplace(vehicle->getId(), vehicle);
    //     threadVehiclePool[threadIndex].insert(vehicle);
    //     Vehicle *leader = vehicle->getCurDrivable()->getLastVehicle();
    //     vehicle->getCurDrivable()->pushVehicle(vehicle); // 道路加入车流
    //     vehicle->setLeader(leader);

    //     vehicleActiveCount++;
    //     totalVehicleCnt++;
    // }

    // wyy: function-计算每个running的车下一秒应该走的距离， 存到buffer中
    void Engine::threadGetAction(std::set<Vehicle *> &vehicles)
    {
        startBarrier.wait();
        std::vector<std::pair<Vehicle * , double>> buffer;
        for (auto vehicle : vehicles) {
            // wyy:这里curlan和belongRoad有什么用
            Drivable* curLane = vehicle->getCurDrivable();
            if (curLane == 0x00 || curLane == nullptr) {
                std::cerr<<vehicle->getId()<<std::endl;
            }
            Road* belongRoad = curLane->getBelongRoad();

            if (vehicle->isRunning()) {
                double speed = vehicle->getSpeed();
                double runningTime;
                if (vehicle->getStartTime() >= currentTime && vehicle->getStartTime() < currentTime + interval) { // start time
                    runningTime = interval - (vehicle->getStartTime() - currentTime); // 这里注意一个问题就是运行时间应该只会是1
                } else {
                    runningTime = interval; // 默认的interval为10s
                }
                double deltaDist = 0;
                deltaDist = runningTime * vehicle->getSpeed();
                deltaDist += vehicle->getDistance();
                buffer.emplace_back(vehicle, deltaDist);
            }
        }
        {
            std::lock_guard<std::mutex> guard(lock);
            pushBuffer.insert(pushBuffer.end(), buffer.begin(), buffer.end());
        }
        endBarrier.wait();
    }

    // wyy: function_对每个drivable删除已变道或已end的车， 更新removeBuffer，map
    void Engine::threadUpdateLocation(const std::vector<Drivable *> &drivables)
    {
        startBarrier.wait(); //等主线程执行完后进行删除车辆操作
        assert(updateLocationFlag == true);
        for (Drivable* drivable : drivables) {  //这里引用的vehicle从drivable的vehicles中拿出来，updatelocation中是从pushbuffer里引用，源头确实是threadvehiclepool，所以在后面再次遍历的时候vehicle的状态改变
            auto &vehicles = drivable->getVehicles(); // 为什么要加上&
            auto vehicleIter = vehicles.begin(); // 这是个指针
            while (vehicleIter != vehicles.end()) {
                Vehicle* vehicle = *vehicleIter;
                if (vehicle->getChangedDrivable() != nullptr || vehicle->hasSetEnd()) {
                    vehicleIter = vehicles.erase(vehicleIter);
                } else {
                    vehicleIter++;
                }

                if (vehicle->hasSetEnd()) { // 在这里就删除车辆，不知道会不会有问题 TODO
                    std::lock_guard<std::mutex> guard(lock);
                    // std::cerr<<"delete car "<<vehicle->getId()<<std::endl; TODO
                    vehicleRemoveBuffer.insert(vehicle);
                    vehicleMap.erase(vehicle->getId());
                    auto iter = vehiclePool.find(vehicle->getPriority());
                    threadVehiclePool[iter->second.second].erase(vehicle); //在线程的vehicle池里删去了这个vehicle
                    delete vehicle;
                    vehiclePool.erase(iter);
                }
            }
        }
        endBarrier.wait(); // 等待子线程做完删除后，主线程在进行所有的换道更新
    }

    // wyy: function_对每辆车更新buffer中的信息
    void Engine::threadUpdateAction(std::set<Vehicle *> &vehicles)
    {
        startBarrier.wait();
        for (auto vehicle : vehicles) {
            if (vehicle->isRunning()) {
                vehicle->update();
            }
        }
        endBarrier.wait();
    }

    // wyy: function_对每辆车更新leader和gap
    void Engine::threadupdateLeaderAndGap(const std::vector<Drivable *> &drivables)
    {
        startBarrier.wait();
        for (Drivable* drivable : drivables) {
            Vehicle *leader = nullptr;
            for (Vehicle* vehicle : drivable->getVehicles()) {
                vehicle->updateLeaderAndGap(leader);
                leader = vehicle;
            }
        }
        endBarrier.wait();
    }

    double Engine::getAverageTravelTime()
    {
        double tt = cumulativeTravelTime;
        // std::cerr<< " cumu time = " << tt << std::endl;
        int n = finishedVehicleCnt;
        for (auto& vehiclePair : vehiclePool) {
            tt += currentTime - vehiclePair.second.first->getStartTime(); // 思考这里有什么问题
            n++;
        }
        // std::cerr<<" n = " << n << " car total time = " << tt<< std::endl;
        if (n == 0)
            return 0;
        else return tt / n;
    }

    void Engine::setTrafficLightPhase(std::string id, int phaseIndex )
    {
        roadnet.getIntersectionById(id)->getTrafficLight().setPhase(phaseIndex);
    }

    size_t Engine::getVehicleCount() const
    {
        return vehicleActiveCount;
    }

    std::vector<const Vehicle *> Engine::getRunningVehicles(bool includeWaiting) const
    {
        std::vector<const Vehicle*> ret;
        ret.reserve(vehicleActiveCount);
        for (const auto& vehiclePair : vehiclePool) {
            const Vehicle* vehicle = vehiclePair.second.first;
            if ((vehicle->isStoped() && includeWaiting) || vehicle->isRunning()) {
                ret.emplace_back(vehicle);
            }
        }
        return ret;
    }

    std::vector<std::string> Engine::getVehicles(bool includeWaiting) const
    {
        std::vector<std::string> ret;
        ret.reserve(vehicleActiveCount);
        for (const Vehicle* vehicle : getRunningVehicles(includeWaiting)) {
            ret.emplace_back(vehicle->getId());
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneVehicleCount() const
    {
        std::map<std::string, int> ret;
        for (const Lane* lane : roadnet.getLanes()) {
            ret.emplace(lane->getId(), lane->getVehicleCnt());
        }
        return ret;
    }

    std::map<std::string, int> Engine::getLaneWaitingVehicleCount() const
    {
        std::map<std::string, int> ret;
        for (const Lane* lane : roadnet.getLanes()) {
            int cnt = 0;
            for (Vehicle* vehicle : lane->getVehicles()) {
                if (vehicle->isStoped()) {
                    cnt++;
                }
            }
            ret.emplace(lane->getId(), cnt);
        }
        return ret;
    }

    std::map<std::string, std::vector<std::string>> Engine::getLaneVehicles()
    {
        std::map<std::string, std::vector<std::string>> ret;
        for (const Lane *lane : roadnet.getLanes())
        {
            std::vector<std::string> vehicles;
            for (Vehicle *vehicle : lane->getVehicles())
            { // 怎么更新lane上车辆
                vehicles.push_back(vehicle->getId());
            }
            ret.emplace(lane->getId(), vehicles);
        }
        return ret;
    }

    std::map<std::string, std::string> Engine::getVehicleInfo(const std::string &id) const
    {
        auto iter = vehicleMap.find(id);
        if (iter == vehicleMap.end())
        {
            throw std::runtime_error("Vehicle '" + id + "' not found");
        }
        else
        {
            Vehicle *vehicle = iter->second;
            return vehicle->getInfo();
        }
    }

    std::map<std::string, double> Engine::getVehicleSpeed() const
    {
        std::map<std::string, double> ret;
        for (const Vehicle *vehicle : getRunningVehicles(true))
        {
            ret.emplace(vehicle->getId(), vehicle->getSpeed());
        }
        return ret;
    }

    std::map<std::string, double> Engine::getVehicleDistance() const
    {
        std::map<std::string, double> ret;
        for (const Vehicle *vehicle : getRunningVehicles(true)) {
            ret.emplace(vehicle->getId(), vehicle->getDistance());
        }
        return ret;
    }

    std::string Engine::getLeader(const std::string &vehicleId) const
    {
        auto iter = vehicleMap.find(vehicleId);
        if (iter == vehicleMap.end()) {
            throw std::runtime_error("Vehicle '" + vehicleId + "'not found");
        } else {
            Vehicle* vehicle = iter->second;
            Vehicle* leader = vehicle->getLeader();
            if (leader) {
                return leader->getId();
            } else
                return "";
        }
    }

    void Engine::reset(bool resetRnd)
    {
        currentTime = 0;

        for (auto& vehiclePair : vehiclePool) {
            delete vehiclePair.second.first;
        }
        for (auto& pool : threadVehiclePool) {
            pool.clear();
        }
        vehiclePool.clear();
        vehicleMap.clear();
        roadnet.reset();

        finishedVehicleCnt = 0;
        cumulativeTravelTime = 0;
        for (auto &flow : flows) {
            flow.reset();
        }
        steps = 0;
        vehicleActiveCount = 0;
        if (resetRnd) {
            rnd.seed(seed);
        }
    }
}
