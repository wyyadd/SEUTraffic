//
// Created by wyyadd on 6/12/22.
//

#include "DSA_Agent.h"

namespace ALGO {
    void DSA_Agent::sendMessage(Message message, DSA_Agent *neighbour) {
        neighbour->getMailBox().push_back(message);
    }

    void DSA_Agent::updateInAgents(DSA_Agent *agent) {
        if (neighbours[North] != nullptr && neighbours[North]->getId() == agent->getId())
            inAgents[North] = agent;
        else if (neighbours[South] != nullptr && neighbours[South]->getId() == agent->getId())
            inAgents[South] = agent;
        else if (neighbours[West] != nullptr && neighbours[West]->getId() == agent->getId())
            inAgents[West] = agent;
        else if (neighbours[East] != nullptr && neighbours[East]->getId() == agent->getId())
            inAgents[East] = agent;
    }

    void DSA_Agent::updateOutAgents(DSA_Agent *agent) {
        if (neighbours[North] != nullptr && neighbours[North]->getId() == agent->getId())
            outAgents[North] = agent;
        else if (neighbours[South] != nullptr && neighbours[South]->getId() == agent->getId())
            outAgents[South] = agent;
        else if (neighbours[West] != nullptr && neighbours[West]->getId() == agent->getId())
            outAgents[West] = agent;
        else if (neighbours[East] != nullptr && neighbours[East]->getId() == agent->getId())
            outAgents[East] = agent;
    }

    double DSA_Agent::getCost(MovementPhases movementPhase) {
        double cost = 0;
        switch (movementPhase) {
            case WE_Straight:
            case SN_Left:
            {
                cost += getCost(West, movementPhase);
                cost += getCost(East, movementPhase);
                break;
            }
            case WE_Left:
            case SN_Straight:
            {
                cost += getCost(North, movementPhase);
                cost += getCost(South, movementPhase);
                break;
            }
        }
        return cost;
    }

    double DSA_Agent::getCost(Direction direction, MovementPhases movementPhase) {
        if (outAgents[direction] == nullptr && !neighbours[direction]->isVirtualIntersection())
            return 0;
        auto neighbour = outAgents[direction] == nullptr ? neighbours[direction] : outAgents[direction];
        return getCost(movementPhase, neighbour);
    }

    double DSA_Agent::getCost(MovementPhases movementPhase, Intersection *neighbour) {
        double cost = 0;
        for (auto &roadLink: roadLinks) {
            if (roadLink.getStartRoad()->getStartIntersection() == neighbour
                && (int) roadLink.getRoadLinkType() % 2 == (int) movementPhase) {
                cost += (double)(roadLink.getVehicleCnt() * roadLink.getVehicleCnt());
            }
        }
        return cost;
    }


} // ALGO