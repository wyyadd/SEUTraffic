#ifndef CITYFLOW_VEHICLE_H
#define CITYFLOW_VEHICLE_H

#include "utility/utility.h"
#include <cerrno>
#include <type_traits>
#include <utility>
#include <memory>
#include <map>
#include "vehicle/vehicleInfo.h"


namespace SEUTraffic{
    class Lane;

    class LaneLink;

    class Intersection;

    class Drivable;

    class Engine;

    class Flow;

    class Vehicle{
        friend class Router;

    private:
        std::string id;
        int startTime;
        int priority;

        struct Buffer {
            bool isDisSet = false;
            bool isDrivableSet = false;
            bool isEndSet = false;
            double deltaDist;
            Drivable* drivable;
            double speed;
            bool end = false;
            double dis;
        };

        struct ControllerInfo{
            double dis = 0;
            Drivable* drivable = nullptr;
            double gap;
            Vehicle *leader = nullptr;
            bool running = false;
            bool end = false;
        };

        Buffer buffer;

        ControllerInfo controllerInfo;

        Engine* engine;

        VehicleInfo vehicleInfo;

        bool stopped;

        double endTime;

        double totalDist = 0; // record for avg speed compute

        double enterTime;
        Flow *flow;

    public:
        Vehicle(const std::string& id, VehicleInfo& vehicleInfo, int startTime, Engine *engine);
        
        Vehicle(const VehicleInfo &init, const std::string &id, Engine *engine, Flow *flow = nullptr);

        Vehicle(const Vehicle &vehicle, const std::string &id, Engine *engine, Flow *flow = nullptr);

        void setSpeed(double speed);

        void setDis(double dis){
            buffer.dis = dis;
            buffer.isDisSet = true;
        }

        bool isRunning() const { return controllerInfo.running; }

        double getStartTime() const { return startTime; }

        inline double getDistance() const { return controllerInfo.dis; }

        double getBufferDist() const
        {
            if (buffer.isDisSet)
                return buffer.dis;
            else return this->getDistance();
        }

        inline double getMinGap() const { return vehicleInfo.getMinGap(); }

        double getLen() const { return vehicleInfo.getLen(); }

        double getWidth() const { return vehicleInfo.getWidth(); }

        double getSpeed() const { return vehicleInfo.getSpeed();}

        std::string getId() const { return id; }

        double gettotalDist() const { return totalDist; }

        Drivable* getCurDrivable() const;

        void setLeader(Vehicle* leaderCar); // 设置leader

        Drivable* getNextDrivable();

        Drivable* getNextDrivable() const ;

        Intersection* getNextIntersection();

        Vehicle* getLeader()
        {
            return controllerInfo.leader;
        }

        bool hasSetEnd() { return buffer.isEndSet; }

        bool hasSetDrivable() const { return buffer.isDrivableSet; }

        Drivable* getChangedDrivable()
        {
            if (!buffer.isDrivableSet) {
                return nullptr;
            }
            else return buffer.drivable;
        }

        double getTotalDist() { return totalDist; }

        int getPriority() { return priority; }

        void setEnd(bool end)
        {
            buffer.end = end;
            buffer.isEndSet = true;
        }

        void setPriority(int priority) { this->priority = priority; }

        void setStop(bool stop)
        {
            stopped = stop;
        }

        void setEndTime(double endtime)
        {
            endTime = endtime;
        }

        double gettotalDist()
        {
            return totalDist;
        }

        void updateTotalDist(double dist) { totalDist = dist; }

        double getEndTime()
        {
            return endTime;
        }

        void setDrivable(Drivable* lane)
        {
            buffer.drivable = lane;
            buffer.isDrivableSet = true;
        }

        bool hasSetStop()
        {
            return stopped;
        }

        bool isStoped() const
        {
            return stopped;
        }

        void update();

        void updateLeaderAndGap(Vehicle* leader);

        std::map<std::string, std::string> getInfo() const;

        // wyy modify: vehicle get points
        Point getPoint() const;

        //yzh modify
        Lane * getCurLane() const {
            if (getCurDrivable()->isLane()) return (Lane *)getCurDrivable();
            else return nullptr;
        }

    };
}
#endif
