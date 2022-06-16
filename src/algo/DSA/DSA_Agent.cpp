//
// Created by wyyadd on 6/12/22.
//

#include "DSA_Agent.h"

namespace ALGO {
    using SEUTraffic::Intersection;

    void DSA_Agent::sendMessage(bool reverse) {
        auto &distAgents = reverse ? inAgents : outAgents;
        for (int direction = 0; direction < 4; ++direction) {
            if (distAgents[direction] == nullptr) continue;
            auto &neighbour = distAgents[direction];
            cType messages[4];
            cType total = 0;
            for (int neighbourPhase = 0; neighbourPhase < 4; ++neighbourPhase) {
                auto minCost = DBL_MAX;
                for (int agentPhase = 0; agentPhase < 4; ++agentPhase) {
                    // add cost with neighbour
                    cType cost = costGraph[agentPhase][direction].phase_traffic[neighbourPhase];
                    // add local cost
                    cost += localCost[agentPhase];
                    // add message from neighbour
                    cost += receivedMessage[agentPhase].Q;
                    minCost = std::min(minCost, cost);
                }
                total += minCost;
                messages[neighbourPhase] = minCost;
            }
            // 防止溢出，所以减了个average
            auto average = total/4;
            for(int i = 0; i < 4; ++i){
                neighbour->receiveMessage(static_cast<MovementPhases>(i), messages[i]-average);
            }
        }
    }

    void DSA_Agent::updateInAgents(DSA_Agent *agent) {
        if (intersection->neighbours[North] != nullptr && intersection->neighbours[North]->getId() == agent->getId())
            inAgents[North] = agent;
        else if (intersection->neighbours[South] != nullptr && intersection->neighbours[South]->getId() == agent->getId())
            inAgents[South] = agent;
        else if (intersection->neighbours[West] != nullptr && intersection->neighbours[West]->getId() == agent->getId())
            inAgents[West] = agent;
        else if (intersection->neighbours[East] != nullptr && intersection->neighbours[East]->getId() == agent->getId())
            inAgents[East] = agent;
    }

    void DSA_Agent::updateOutAgents(DSA_Agent *agent) {
        if (intersection->neighbours[North] != nullptr && intersection->neighbours[North]->getId() == agent->getId())
            outAgents[North] = agent;
        else if (intersection->neighbours[South] != nullptr && intersection->neighbours[South]->getId() == agent->getId())
            outAgents[South] = agent;
        else if (intersection->neighbours[West] != nullptr && intersection->neighbours[West]->getId() == agent->getId())
            outAgents[West] = agent;
        else if (intersection->neighbours[East] != nullptr && intersection->neighbours[East]->getId() == agent->getId())
            outAgents[East] = agent;
    }

    void DSA_Agent::generateTrafficGraph() {
        for (int phase = 0; phase < 4; ++phase) {
            auto agent_phase = static_cast<MovementPhases>(phase);
            for (int direction = 0; direction < 4; ++direction) {
                Intersection *neighbour = intersection->neighbours[direction];
                if (neighbour->isVirtualIntersection()) {
                    localCost[agent_phase] += generateLocalCost(neighbour, agent_phase);
                } else {
                    costGraph[agent_phase][direction].phase_traffic = generateCostWithNeighbour(neighbour, agent_phase);
                }
            }
        }
    }

    std::vector<cType> DSA_Agent::generateCostWithNeighbour(Intersection *neighbour, MovementPhases movementPhase) {
        std::vector<cType> cost(4);
        // this agent set movementPhase
        // neighbour set four movement
        for (int phase = 0; phase < 4; ++phase) {
            cType c = 0;
            auto neighbour_phase = static_cast<MovementPhases>(phase);
            // lock
            std::unique_lock<std::mutex> lk(*enginePredictMutex);
            // snapshot and set trafficLight
            intersection->getTrafficLight().snapshot();
            neighbour->getTrafficLight().snapshot();
            intersection->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[movementPhase]);
            neighbour->getTrafficLight().setPhase(
                    movementPhases_to_trafficLightPhase[neighbour_phase]);
            // engine predict
            engine->predictPeriod(enginePredictTime);
            for (auto &road: intersection->getInRoads()) {
                if(road->getStartIntersection()->getId() == neighbour->getId())
                    c += (cType)std::pow(road->getVehicleCnt(), 2);
            }

            for (auto &road: intersection->getOutRoads()) {
                if(road->getEndIntersection()->getId() == neighbour->getId())
                    c += (cType)std::pow(road->getVehicleCnt(), 2);
            }
            // engine predict done
            engine->stopPredict();
            // restore trafficLight
            intersection->getTrafficLight().restore();
            neighbour->getTrafficLight().restore();
            // unlock
            lk.unlock();
            cost[phase] = c;
        }
        return cost;
    }

    cType DSA_Agent::generateLocalCost(Intersection *neighbour, MovementPhases movementPhase) {
        cType cost = 0;
        // lock
        std::unique_lock<std::mutex> lk(*enginePredictMutex);
        intersection->getTrafficLight().snapshot();
        intersection->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[movementPhase]);
        engine->predictPeriod(enginePredictTime);

        for (auto &road: intersection->getInRoads()) {
            if(road->getStartIntersection()->getId() == neighbour->getId())
                cost += (cType)std::pow(road->getVehicleCnt(), 2);
        }
        engine->stopPredict();
        intersection->getTrafficLight().restore();
        // unlock
        lk.unlock();
        return cost;
    }

    int DSA_Agent::getNotNullAgentSize(std::vector<DSA_Agent *> &agents) {
        int size = 0;
        for (auto &a: agents)
            size += a != nullptr;
        return size;
    }

    void DSA_Agent::receiveMessage(DSA_Agent::MovementPhases movementPhase, cType val) {
        {
            std::lock_guard<std::mutex> lock(*agentMutex);
            receivedMessage[movementPhase].Q += val;
            ++currentReceivedNum;
        }
        cv->notify_one();
    }

    void DSA_Agent::makeDecision() {
        int bestPhase = -1;
        auto minCost = DBL_MAX;
        for (int movementPhase = 0; movementPhase < 4; ++movementPhase) {
            auto phase = static_cast<MovementPhases>(movementPhase);
            cType cost = localCost[phase];
            cost += receivedMessage[phase].Q;
            if (cost < minCost) {
                minCost = cost;
                bestPhase = phase;
            }
        }
        if (bestPhase != -1) {
            intersection->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[bestPhase]);
//            std::cout << bestPhase;
        } else {
            std::cerr << "error happen, agent Id: " << getId() << '\n';
        }
    }

    void DSA_Agent::resetAgent() {
        currentReceivedNum = 0;
        receivedMessage.clear();
        receivedMessage.resize(4, Message());
        costGraph.clear();
        costGraph.resize(4, std::vector<Traffic>(4, Traffic()));
        localCost.clear();
        localCost.resize(4, 0);
    }

    void DSA_Agent::run() {
        generateTrafficGraph();
        for (int i = 0; i < iterateTime; ++i) {
            // wait receive all message
            std::unique_lock<std::mutex> lk(*agentMutex);
            cv->wait(lk, [&] { return currentReceivedNum >= 4 * getNotNullAgentSize(inAgents); });
            currentReceivedNum = 0;
            sendMessage(false);

            // reverse order
            cv->wait(lk, [&] { return currentReceivedNum >= 4 * getNotNullAgentSize(outAgents); });
            currentReceivedNum = 0;
            sendMessage(true);
        }
        makeDecision();
    }

} // ALGO