#ifndef SEUTRAFFIC_VEHICLE_H
#define SEUTRAFFIC_VEHICLE_H

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
        struct Buffer {
            bool isDisSet = false;
            bool isDrivableSet = false;
            bool isEndSet = false;
//            double deltaDist;
            Drivable* drivable;
            double speed;
            bool end = false;
            double dis;
        };

        struct ControllerInfo{
            double dis = 0;
            Drivable* drivable = nullptr;
//            double gap;
            Vehicle *leader = nullptr;
            bool running = false;
            bool end = false;
        };

        Buffer buffer;

        ControllerInfo controllerInfo;

        Engine* engine;

        VehicleInfo vehicleInfo;

        std::string id;

        int startTime;

        double endTime = -1;

        size_t priority;

        // TODO: 有什么用
        bool stopped = false;

        double totalDist = 0; // record for avg speed compute

        // wyy modify
        std::vector<Drivable* > planned;
        int currentDrivableIndex = 0;

        Flow *flow;

    public:
        Vehicle(const VehicleInfo &init, std::string id, Engine *engine, Flow *flow = nullptr);

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

        Drivable* getCurDrivable() const;

        void setLeader(Vehicle* leaderCar); // 设置leader

        Drivable* getNextDrivable();

        Intersection* getNextIntersection();

        Vehicle* getLeader() const
        {
            return controllerInfo.leader;
        }

        bool hasSetEnd() const { return buffer.isEndSet; }

        bool hasSetDrivable() const { return buffer.isDrivableSet; }

        Drivable* getChangedDrivable() const
        {
            if (!buffer.isDrivableSet) {
                return nullptr;
            }
            else return buffer.drivable;
        }

        double getTotalDist() const { return totalDist; }

        size_t getPriority() const { return priority; }

        void setEnd(bool end)
        {
            buffer.end = end;
            buffer.isEndSet = true;
        }

        void setPriority(size_t pri) { this->priority = pri; }

        void setStop(bool stop)
        {
            stopped = stop;
        }

        void setEndTime(double newEndTime)
        {
            this->endTime = newEndTime;
        }


        void updateTotalDist(double dist) { totalDist = dist; }

        double getEndTime() const
        {
            return endTime;
        }

        void setDrivable(Drivable* drivable)
        {
            buffer.drivable = drivable;
            buffer.isDrivableSet = true;
            ++currentDrivableIndex;
        }

        bool hasSetStop() const
        {
            return stopped;
        }

        bool isStopped() const
        {
            return stopped;
        }

        void update();

        void updateLeaderAndGap(Vehicle* leader);

        std::map<std::string, std::string> getInfo() const;

        // wyy modify: vehicle get points
        Point getPoint() const;

        //yzh modify
        Lane * getCurLane() const;

    };
}
#endif // SEUTRAFFIC_VEHICLE_H
