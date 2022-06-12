//
// Created by wyyadd on 6/12/22.
//

#ifndef SEUTRAFFIC_DSA_AGENT_H
#define SEUTRAFFIC_DSA_AGENT_H

#include "engine/engine.h"
#include <unordered_map>
#include <deque>
#include <vector>

namespace ALGO {
    using SEUTraffic::Intersection;
    using std::string;
    using std::pair;

    struct Message {
        int agentId;
        int message;
    };

    enum MovementPhases {
        WE_Straight = 0, WE_Left = 1, SN_Straight = 2, SN_Left = 3
    };

    class DSA_Agent : public Intersection {
    private:
        int agentId;
        std::deque<Message> receivedMessage;
        std::vector<DSA_Agent *> inAgents;
        std::vector<DSA_Agent *> outAgents;
    private:
        double getCost(Direction direction, MovementPhases movementPhase);
        double getCost(MovementPhases movementPhase, Intersection* neighbour);
    public:
        DSA_Agent(int id, Intersection *intersection) : Intersection(*intersection), agentId(id) {
            inAgents.resize(4, nullptr);
            outAgents.resize(4, nullptr);
        }

        int getAgentId() const { return agentId; }

        std::deque<Message> &getMailBox() { return receivedMessage; }

        static void sendMessage(Message message, DSA_Agent *neighbour);

        void updateInAgents(DSA_Agent *agent);

        void updateOutAgents(DSA_Agent *agent);

        double getCost(MovementPhases movementPhase);
    };
} // ALGO

#endif //SEUTRAFFIC_DSA_AGENT_H
