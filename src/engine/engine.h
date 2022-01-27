#include "flow/flow.h"
#include "roadnet/roadnet.h"
#include "utility/barrier.h"

#include <mutex>
#include <thread>
#include <set>
#include <random>
#include <fstream>

namespace SEUTraffic{
    class Engine{
    private:
        static bool vehicleCmp(const std::pair<Vehicle *, double> &a, const std::pair<Vehicle *, double> &b)
        {
            return a.second > b.second;
        }

        std::map<int, std::pair<Vehicle *, int>> vehiclePool;
        std::map<std::string, Vehicle *> vehicleMap;
        std::vector<std::set<Vehicle *>> threadVehiclePool;
        std::vector<std::vector<Road *>> threadRoadPool;
        std::vector<std::vector<Intersection *>> threadIntersectionPool;
        std::vector<std::vector<Drivable *>> threadDrivablePool;
        std::vector<Flow> flows;
        RoadNet roadnet;
        int threadNum;
        double interval;
        std::vector<std::pair<Vehicle*, double>> pushBuffer;

        size_t steps = 0;
        int finishedVehicleCnt = 0;
        int vehicleActiveCount = 0;
        int totalVehicleCnt = 0;
        int currentTime;
        double cumulativeTravelTime = 0;
        std::set<Vehicle *> vehicleRemoveBuffer;
        // wyy modify: jsonroot to log
        rapidjson::Document jsonRoot;
        std::vector<std::thread> threadPool; // 线程池
        bool finished = false;
        Barrier startBarrier, endBarrier;
        std::string dir;
        double avgSpeedSum  = 0;
        bool rlTrafficLight;
        int seed;
        std::mutex lock;

        // wyy modify: logout
        std::ofstream logOut;
        void setLogFile(const std::string &jsonFile, const std::string &logFile);

    private:
        void vehicleControl(Vehicle &vehicle);

        void getAction();

        void updateAction();

        void updateLocation();

        void updateLeaderAndGap();

        void threadController(std::set<Vehicle *> &vehicles,
                        std::vector<Road *> &roads,
                        std::vector<Intersection *> &intersections,
                        std::vector<Drivable *> &drivables);

        bool loadRoadNet(const std::string &jsonFile);

        bool loadFlow(const std::string &jsonFilename);

        // wyy modify: update log
        void updateLog();


    public:
        std::mt19937 rnd;

        Engine(const std::string& configFile, int threadNum);

        bool updateLocationFlag = false;

        ~Engine();

        bool loadConfig(const std::string& configFile);

        void nextStep();

        void step2(int interval);

        void step(int interval, std::map<std::string, int>& actions);

        bool checkPriority(int priority);

        void pushVehicle(Vehicle *const vehicle, bool pushToDrivable = true);

        void threadGetAction(std::set<Vehicle*>& vehicles);

        void threadUpdateLocation(const std::vector<Drivable*>& drivables);

        void threadUpdateAction(std::set<Vehicle*>& vehicles);

        void threadupdateLeaderAndGap(const std::vector<Drivable*>& drivables);

        double getAverageTravelTime();

        int getSteps()
        {
            return steps;
        }

        int getCurrentTime()
        {
            return currentTime;
        }

        int getActiveCars()
        {
            return vehicleActiveCount;
        }

        void setTrafficLightPhase(std::string id, int phaseIndex);

        RoadNet getRoadnet()
        {
            return roadnet;
        }

        std::vector<std::string> getInterIds()
        {
            return roadnet.getInterIds();
        }

        size_t getVehicleCount() const;

        std::vector<const Vehicle *> getRunningVehicles(bool includeWaiting = false) const;

        std::vector<std::string> getVehicles(bool includeWaiting) const;

        std::map<std::string, int> getLaneVehicleCount() const;

        std::map<std::string, int> getLaneWaitingVehicleCount() const;

        std::map<std::string, double> getVehicleSpeed() const;

        std::map<std::string, double> getVehicleDistance() const;

        std::map<std::string, std::vector<std::string>> getLaneVehicles();

        std::map<std::string, std::string> getVehicleInfo(const std::string& id) const;

        void reset(bool resetRnd = false);

        void addCumulativeTravelTime(double startTime, double endTime);

        int getTotalCars() // contain both active and finished car cnt
        {
            return vehicleActiveCount + finishedVehicleCnt;
        }

        std::string getLeader(const std::string &vehicleId) const;
    };

}
