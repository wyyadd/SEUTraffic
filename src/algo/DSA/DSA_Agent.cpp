//
// Created by wyyadd on 6/12/22.
//

#include "DSA_Agent.h"

namespace ALGO {
    void DSA_Agent::sendMessage() {
        for (int movementPhase = 0; movementPhase < 4; ++movementPhase) {
            // agent's movementPhase on myself
            auto phase = static_cast<MovementPhases>(movementPhase);
            double cost = 0;
            // add local cost
            cost += costGraph[phase][4].cost.empty() ? 0 : costGraph[phase][4].cost[0].second;
            // add message from neighbour
            cost += receivedMessage[phase].Q;
            // send cost to neighbour
            for (int i = 0; i < 4; ++i) {
                auto neighbour_phase = static_cast<MovementPhases>(i);
                for (auto &neighbour_cost_pair: costGraph[phase][neighbour_phase].cost) {
                    // TODO: add lock
                    neighbour_cost_pair.first->getMailBox()[neighbour_phase].Q += neighbour_cost_pair.second + cost;
                }
            }
        }
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

    void DSA_Agent::generateCostGraph(MovementPhases movementPhase) {
        switch (movementPhase) {
            case WE_Straight:
            case WE_Left: {
                generateCost(West, movementPhase);
                generateCost(East, movementPhase);
                break;
            }
            case SN_Straight:
            case SN_Left: {
                generateCost(North, movementPhase);
                generateCost(South, movementPhase);
                break;
            }
        }
    }

    void DSA_Agent::generateCost(Direction direction, MovementPhases movementPhase) {
        if (outAgents[direction] == nullptr && !neighbours[direction]->isVirtualIntersection())
            return;
        if (outAgents[direction] == nullptr)
            return generateLocalCost(neighbours[direction], movementPhase);
        else
            return generateCostWithNeighbour(outAgents[direction], movementPhase);
    }

    void DSA_Agent::generateCostWithNeighbour(DSA_Agent *neighbour, MovementPhases movementPhase) {
        // this agent set movementPhase
        // neighbour set four movement
        for (int phase = 0; phase < 4; ++phase) {
            double cost = 0;
            auto neighbour_phase = static_cast<MovementPhases>(movementPhase);
            // TODO: intersection set phase
            // TODO: engine predict
            // neighbour set neighbour_phase
            // engine predict
            for (auto &roadLink: roadLinks) {
                if (roadLink.getStartRoad()->getStartIntersection()->getId() == neighbour->getId()
                    && (int) roadLink.getRoadLinkType() % 2 == (int) movementPhase) {
                    cost += std::pow(roadLink.getVehicleCnt(), 2);
                }
            }
            for (auto &roadLink: neighbour->getRoadLinks()) {
                if (roadLink.getStartRoad()->getStartIntersection()->getId() == getId() &&
                    (int) roadLink.getRoadLinkType() % 2 == (int) movementPhase) {
                    cost += std::pow(roadLink.getVehicleCnt(), 2);
                }
            }
            costGraph[movementPhase][neighbour_phase].cost.emplace_back(neighbour, cost);
            // engine predict done
        }
    }

    void DSA_Agent::generateLocalCost(Intersection *neighbour, MovementPhases movementPhase) {
        double cost = 0;
        // this set phase
        // engine predict
        for (auto &roadLink: roadLinks) {
            if (roadLink.getStartRoad()->getStartIntersection() == neighbour
                && (int) roadLink.getRoadLinkType() % 2 == (int) movementPhase) {
                // TODO: 修改计算方法
                cost += std::pow(roadLink.getVehicleCnt(), 2);
            }
        }
        // engine predict done
        costGraph[movementPhase][4].cost.emplace_back(nullptr, cost);
    }

    void DSA_Agent::run() {
        // wait receive all message
        generateCostGraph();
        sendMessage();
        // reverse generateCostGraph and sendMessage
        // make decision
    }

    void DSA_Agent::generateCostGraph() {
        for (int movementPhase = 0; movementPhase < 4; ++movementPhase) {
            generateCostGraph(static_cast<MovementPhases>(movementPhase));
        }
    }


} // ALGO