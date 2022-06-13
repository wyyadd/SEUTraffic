//
// Created by wyyadd on 6/12/22.
//

#ifndef SEUTRAFFIC_DSA_AGENT_H
#define SEUTRAFFIC_DSA_AGENT_H

#include "engine/engine.h"
#include <unordered_map>
#include <deque>
#include <vector>
#include <cmath>

namespace ALGO {
    using SEUTraffic::Intersection;
    using std::string;
    using std::pair;

    class DSA_Agent : public Intersection {
    private:
        struct Message {
            double Q = 0;
        };

        struct Cost {
            std::vector<std::pair<DSA_Agent*, double>> cost;
        };

        enum MovementPhases {
            WE_Straight = 0, WE_Left = 1, SN_Straight = 2, SN_Left = 3
        };
        int agentId;
        std::vector<Message> receivedMessage;
        std::vector<DSA_Agent *> inAgents;
        std::vector<DSA_Agent *> outAgents;
        // 4 rows represent 4 movementPhases, 5 column: neighbour's four movementPhase, local cost
        // 4行代表我们的四个movementPhases, 5列:前四列表示neighbour采取的4个movementPhase下产生的cost, 第5列表示localCost
        std::vector<std::vector<Cost>> costGraph;
    private:
        void generateCostGraph(MovementPhases movementPhase);

        // direction = neighbour's direction
        void generateCost(Direction direction, MovementPhases movementPhase);

        void generateCostWithNeighbour(DSA_Agent *neighbour, MovementPhases movementPhase);

        void generateLocalCost(Intersection *neighbour, MovementPhases movementPhase);

    public:
        DSA_Agent(int id, Intersection *intersection) : Intersection(*intersection), agentId(id) {
            inAgents.resize(4, nullptr);
            outAgents.resize(4, nullptr);
            receivedMessage.resize(4, Message());
            costGraph.resize(4, std::vector<Cost>(5, Cost()));
        }

        int getAgentId() const { return agentId; }

        std::vector<Message> &getMailBox() { return receivedMessage; }

        void sendMessage();

        void updateInAgents(DSA_Agent *agent);

        void updateOutAgents(DSA_Agent *agent);

        void generateCostGraph();

        void run();
    };
} // ALGO

#endif //SEUTRAFFIC_DSA_AGENT_H
