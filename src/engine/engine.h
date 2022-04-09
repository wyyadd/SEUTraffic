#ifndef SEUTRAFFIC_ENGINE_H
#define SEUTRAFFIC_ENGINE_H

#include "flow/flow.h"
#include "roadnet/roadnet.h"
#include "utility/barrier.h"

#include <mutex>
#include <thread>
#include <set>
#include <random>
#include <fstream>

namespace SEUTraffic {
    class Engine {
    private:
        static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b) {
            return a.second > b.second;
        }

        std::map<size_t, std::pair<Vehicle *, int>> vehiclePool;
        std::map<std::string, Vehicle *> vehicleMap;
        std::vector<std::set<Vehicle *>> threadVehiclePool;
        std::vector<std::vector<Road *>> threadRoadPool;
        std::vector<std::vector<Intersection *>> threadIntersectionPool;
        std::vector<std::vector<Drivable *>> threadDrivablePool;
        std::vector<Flow> flows;
        RoadNet roadNet;
        int threadNum;
        double interval;
        std::vector<std::pair<Vehicle *, double>> pushBuffer;

        size_t steps = 0;
        int finishedVehicleCnt = 0;
        int vehicleActiveCount = 0;
        int totalVehicleCnt = 0;
        int currentTime;
        double cumulativeTravelTime = 0;
        double cumulativeWaitingTime = 0;
        // wyy modify: jsonRoot to log
        rapidjson::Document jsonRoot;
        std::vector<std::thread> threadPool; // 线程池
        bool finished = false;
        Barrier startBarrier, endBarrier;
        std::string dir;
        bool rlTrafficLight;
        int seed;
        std::mutex lock;

        // wyy modify: logout
        std::ofstream logOut;

        struct Statistics {
            std::vector<int> time;
            std::vector<int> waitingVehicleCnt;
            std::vector<double> avgWaitingTime;
            std::vector<int> finishVehicleCnt;
            std::vector<double> avgFinishTime;

            template<class T>
            rapidjson::Value convertToStatistic(std::vector<T> &arr, rapidjson::Document::AllocatorType &allocator);
        } statistics;

    private:
        void vehicleControl(Vehicle &vehicle);

        void getAction();

        void updateAction();

        void updateLocation();

        static void updateVehicleDistWithNextDrivable(Vehicle* vehicle, double maxPossibleDist, bool& stopFlag);

        void updateLeaderAndGap();

        void threadController(std::set<Vehicle *> &vehicles,
                              std::vector<Road *> &roads,
                              std::vector<Intersection *> &intersections,
                              std::vector<Drivable *> &drivables);

        bool loadRoadNet(const std::string &jsonFile);

        bool loadFlow(const std::string &jsonFilename);

        // wyy modify: update log
        void updateLog();

        void setLogFile(const std::string &jsonFile, const std::string &logFile);

    public:
        std::mt19937 rnd;

        Engine(const std::string &configFile, int threadNum);

        bool updateLocationFlag = false;

        ~Engine();

        bool loadConfig(const std::string &configFile);

        void nextStep(bool fixedTimeTraffic);

        bool checkPriority(size_t priority);

        void pushVehicle(Vehicle *vehicle, bool pushToDrivable = true);

        void threadGetAction(std::set<Vehicle *> &vehicles);

        void threadUpdateLocation(const std::vector<Drivable *> &drivables);

        void threadUpdateAction(std::set<Vehicle *> &vehicles);

        void threadUpdateLeaderAndGap(const std::vector<Drivable *> &drivables);

        double getAverageTravelTime();

        size_t getSteps() const {
            return steps;
        }

        int getCurrentTime() const {
            return currentTime;
        }

        int getActiveCars() const {
            return vehicleActiveCount;
        }

        void setTrafficLightPhase(std::string id, int phaseIndex);

        RoadNet getRoadnet() {
            return roadNet;
        }

        std::vector<std::string> getInterIds() {
            return roadNet.getInterIds();
        }

        size_t getVehicleCount() const;

        std::vector<const Vehicle *> getRunningVehicles(bool includeWaiting = false) const;

        std::vector<std::string> getVehicles(bool includeWaiting) const;

        std::map<std::string, int> getLaneVehicleCount() const;

        std::map<std::string, int> getLaneWaitingVehicleCount() const;

        size_t getWaitingVehicleCount() const;

        std::map<std::string, double> getVehicleSpeed() const;

        std::map<std::string, double> getVehicleDistance() const;

        std::map<std::string, std::vector<std::string>> getLaneVehicles();

        std::map<std::string, std::string> getVehicleInfo(const std::string &id) const;

        void reset(bool resetRnd = false);

        void addCumulativeTravelTime(double endTime, double startTime);

        std::string getLeader(const std::string &vehicleId) const;

        void logTrafficStatistics();

        void handleWaiting();
    };
}

#endif //SEUTRAFFIC_ENGINE_H
