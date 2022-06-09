//
// Created by wyyadd on 6/12/22.
//

#ifndef SEUTRAFFIC_DSA_AGENT_H
#define SEUTRAFFIC_DSA_AGENT_H

#include "engine/engine.h"
#include <unordered_map>
#include <vector>
#include <cmath>
#include <cfloat>

namespace ALGO {
    using SEUTraffic::Intersection;
    using SEUTraffic::Engine;
    using std::string;
    using std::pair;
    using std::cout;
    using SEUTraffic::TrafficLight;

    typedef double cType;

    class DSA_Agent {
    private:
        struct Message {
            cType Q = 0;
        };

        enum MovementPhases {
            WE_Straight = 0, WE_Left = 1, SN_Straight = 2, SN_Left = 3
        };

        struct Traffic {
            std::vector<cType> phase_traffic;
            Traffic() {
                phase_traffic.clear();
                phase_traffic.resize(4, 0);
            }
        };

        enum Direction {
            East = Intersection::East,
            North = Intersection::North,
            West = Intersection::West,
            South = Intersection::South
        };

        int agentId;
        Intersection *intersection;
        Engine *engine;

        std::mutex *enginePredictMutex;
        std::mutex *agentMutex;
        std::condition_variable *cv;
        int currentReceivedNum = 0;
        int enginePredictTime = 10;
        int iterateTime = 20;

        std::vector<Message> receivedMessage;
        std::vector<DSA_Agent *> inAgents;
        std::vector<DSA_Agent *> outAgents;

        // 4行4列，4行分别表示当前agent的4个movementPhase, 4列表示东南西北四个邻居， 里面的元素为邻居的4个Phase产生的traffic
        std::vector<std::vector<Traffic>> costGraph;
        std::vector<cType> localCost;
        int movementPhases_to_trafficLightPhase[4]{1, 3, 2, 4};

    private:

        std::vector<cType> generateCostWithNeighbour(Intersection *neighbour, MovementPhases movementPhase);

        cType generateLocalCost(Intersection *neighbour, MovementPhases movementPhase);

        static int getNotNullAgentSize(std::vector<DSA_Agent *> &agents);

    public:
        DSA_Agent(int id, Intersection *intersection, Engine *engine, std::mutex *enginePredictMutex)
                : agentId(id), intersection(intersection), engine(engine), enginePredictMutex(enginePredictMutex) {
            inAgents.resize(4, nullptr);
            outAgents.resize(4, nullptr);
            receivedMessage.resize(4, Message());
            agentMutex = new std::mutex();
            cv = new std::condition_variable();
            costGraph.resize(4, std::vector<Traffic>(4, Traffic()));
            localCost.resize(4, 0);
        }

        ~DSA_Agent() {
            delete agentMutex;
            delete cv;
        }

        string getId() const { return intersection->getId(); }

        int getAgentId() const { return agentId; }

        void sendMessage(bool reverse);

        void receiveMessage(MovementPhases movementPhase, cType val);

        void updateInAgents(DSA_Agent *agent);

        void updateOutAgents(DSA_Agent *agent);

        void generateTrafficGraph();

        void makeDecision();

        void resetAgent();

        void run();
    };
} // ALGO

#endif //SEUTRAFFIC_DSA_AGENT_H
