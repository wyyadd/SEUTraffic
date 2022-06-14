//
// Created by wyyadd on 6/12/22.
//

#ifndef SEUTRAFFIC_DSA_AGENT_H
#define SEUTRAFFIC_DSA_AGENT_H

#include "engine/engine.h"
#include <unordered_map>
#include <vector>
#include <cmath>

namespace ALGO {
    using SEUTraffic::Intersection;
    using SEUTraffic::Engine;
    using std::string;
    using std::pair;
    using std::cout;

    class DSA_Agent : public Intersection {
    private:
        struct Message {
            double Q = 0;
            std::vector<DSA_Agent*> sender;
        };

        struct Cost {
            std::vector<std::pair<DSA_Agent *, double>> cost;
        };

        enum MovementPhases {
            WE_Straight = 0, WE_Left = 1, SN_Straight = 2, SN_Left = 3
        };
        int agentId;
        Engine *engine;

        std::mutex *agentMutex;
        std::condition_variable *cv;
        int currentReceivedNum = 0;

        std::vector<Message> receivedMessage;
        std::vector<DSA_Agent *> inAgents;
        std::vector<DSA_Agent *> outAgents;
        // 4 rows represent 4 movementPhases, 5 column: neighbour's four movementPhase, local cost
        // 4行代表我们的四个movementPhases, 5列:前四列表示neighbour采取的4个movementPhase下产生的cost, 第5列表示localCost
        std::vector<std::vector<Cost>> costGraph;
        int movementPhases_to_trafficLightPhase[4]{1, 3, 2, 4};

    private:
        void generateCostGraph(MovementPhases movementPhase);

        // direction = neighbour's direction
        void generateCost(Direction direction, MovementPhases movementPhase);

        void generateCostWithNeighbour(DSA_Agent *neighbour, MovementPhases movementPhase);

        void generateLocalCost(Intersection *neighbour, MovementPhases movementPhase);

    public:
        DSA_Agent(int id, Intersection *intersection, Engine *engine) : Intersection(*intersection), agentId(id),
                                                                        engine(engine) {
            inAgents.resize(4, nullptr);
            outAgents.resize(4, nullptr);
            receivedMessage.resize(4, Message());
            costGraph.resize(4, std::vector<Cost>(5, Cost()));
            agentMutex = new std::mutex();
            cv = new std::condition_variable();
        }

        ~DSA_Agent() {
            delete agentMutex;
            delete cv;
        }

        int getAgentId() const { return agentId; }

        void sendMessage();

        void receiveMessage(MovementPhases movementPhase, double val, DSA_Agent* sender);

        void updateInAgents(DSA_Agent *agent);

        void updateOutAgents(DSA_Agent *agent);

        void generateCostGraph();

        void makeDecision();

        void run();
    };
} // ALGO

#endif //SEUTRAFFIC_DSA_AGENT_H
