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

    struct Message {
        int agentId;
        int message;
    };

    class DSA_Agent :public Intersection{
    private:
        int agentId;
        std::deque<Message> receivedMessage;
        std::vector<DSA_Agent*> inAgents;
        std::vector<DSA_Agent*> outAgents;
    public:
        DSA_Agent(int id, Intersection *intersection) : Intersection(*intersection), agentId(id) {}

//        string getId() const {return intersection->getId();}

        int getAgentId() const { return agentId; }

        std::deque<Message> &getMailBox() { return receivedMessage; }

        static void sendMessage(Message message, DSA_Agent *neighbour);

        void updateInAgents(DSA_Agent* agent){inAgents.push_back(agent);}

        void updateOutAgents(DSA_Agent* agent){outAgents.push_back(agent);}
    };
} // ALGO

#endif //SEUTRAFFIC_DSA_AGENT_H
