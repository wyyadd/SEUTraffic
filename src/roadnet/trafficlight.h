#ifndef SEUTRAFFIC_TRAFFICLIGHT_H
#define SEUTRAFFIC_TRAFFICLIGHT_H

#include <vector>

namespace SEUTraffic {
    class Intersection;

    class RoadLink;

    class TrafficLight;

    class LightPhase {
        friend class RoadNet;

        friend class RoadLink;

        friend class TrafficLight;

    private:
        unsigned int phase = 0;
        double time = 0.0;
        std::vector<bool> roadLinkAvailable;

    public:
        std::vector<bool> getRoadLinkAvailable() {
            return roadLinkAvailable;
        }

        unsigned int getPhaseIndex() const {
            return phase;
        }
    };

    class TrafficLight {
        friend class RoadNet;

    private:
        Intersection *intersection = nullptr;
        std::vector<LightPhase> phases;
        std::vector<int> roadLinkIndices;
        double remainDuration = 0.0;
        int curPhaseIndex = 0;
        struct SnapshotBuffer {
            double remainDuration = 0.0;
            int curPhaseIndex = 0;
        } snapshotBuffer;
    public:
        void init(int initPhaseIndex);

        int getCurrentPhaseIndex() const;

        LightPhase &getCurrentPhase();

        Intersection *getIntersection();

        std::vector<LightPhase> &getPhases() {
            return phases;
        }

        void passTime(double seconds);

        bool changePhase(double seconds);//yzh:经过seconds时间，当前Intersection的TrafficLight是否会改变相位

        void setPhase(int phaseIndex);

        void reset();

        void snapshot() {
            snapshotBuffer.remainDuration = remainDuration;
            snapshotBuffer.curPhaseIndex = curPhaseIndex;
        }

        void restore() {
            remainDuration = snapshotBuffer.remainDuration;
            curPhaseIndex = snapshotBuffer.curPhaseIndex;
            snapshotBuffer = SnapshotBuffer();
        }
    };
}
#endif // SEUTRAFFIC_TRAFFICLIGHT_H
