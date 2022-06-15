//
// Created by wyyadd on 6/12/22.
//

#include "DSA_Agent.h"

// 当前问题： engine只能由主线程预测
// receive message 会发生死锁
namespace ALGO {
    void DSA_Agent::sendMessage() {
        for (int movementPhase = 0; movementPhase < 4; ++movementPhase) {
            // agent's movementPhase on myself
            auto phase = static_cast<MovementPhases>(movementPhase);
            double cost = 0;
            // add local cost
            for (auto &localCost: costGraph[phase][4].cost)
                cost += localCost.second;
            // add message from neighbour
            cost += receivedMessage[phase].Q;
            // send cost to neighbour
            for (int i = 0; i < 4; ++i) {
                auto neighbour_phase = static_cast<MovementPhases>(i);
                for (auto &neighbour_cost_pair: costGraph[phase][neighbour_phase].cost) {
                    neighbour_cost_pair.first->receiveMessage(neighbour_phase, neighbour_cost_pair.second + cost, this);
                }
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

    void DSA_Agent::generateCostGraph(MovementPhases movementPhase) {
        switch (movementPhase) {
            case WE_Straight: {
                generateCost(West, movementPhase);
                generateCost(East, movementPhase);
                break;
            }
            case SN_Straight: {
                generateCost(North, movementPhase);
                generateCost(South, movementPhase);
                break;
            }
            case SN_Left:
            case WE_Left: {
                generateCost(West, movementPhase);
                generateCost(East, movementPhase);
                generateCost(North, movementPhase);
                generateCost(South, movementPhase);
            }
        }
    }

    void DSA_Agent::generateCost(Direction direction, MovementPhases movementPhase) {
        if (outAgents[direction] == nullptr && !intersection->neighbours[direction]->isVirtualIntersection())
            return;
        if (outAgents[direction] == nullptr)
            return generateLocalCost(intersection->neighbours[direction], movementPhase);
        else
            return generateCostWithNeighbour(outAgents[direction], movementPhase);
    }

    void DSA_Agent::generateCostWithNeighbour(DSA_Agent *neighbour, MovementPhases movementPhase) {
        // this agent set movementPhase
        // neighbour set four movement
        for (int phase = 0; phase < 4; ++phase) {
            double cost = 0;
            auto neighbour_phase = static_cast<MovementPhases>(phase);
            // lock
            std::unique_lock<std::mutex> lk(*enginePredictMutex);
            // snapshot and set trafficLight
            intersection->getTrafficLight().snapshot();
            intersection->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[movementPhase]);
            neighbour->getIntersection()->getTrafficLight().snapshot();
            neighbour->getIntersection()->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[neighbour_phase]);
            // engine predict
            engine->predictPeriod(30);
            for (auto &roadLink: intersection->getRoadLinks()) {
                if (roadLink.getStartRoad()->getStartIntersection()->getId() == neighbour->getId()
                    && (int) roadLink.getRoadLinkType() == (int) movementPhase % 2) {
                    cost += std::pow(roadLink.getVehicleCnt(), 2);
                }
            }
            for (auto &roadLink: neighbour->getIntersection()->getRoadLinks()) {
                if (roadLink.getStartRoad()->getStartIntersection()->getId() == getId() &&
                    (int) roadLink.getRoadLinkType() == (int) neighbour_phase % 2) {
                    cost += std::pow(roadLink.getVehicleCnt(), 2);
                }
            }
            costGraph[movementPhase][neighbour_phase].cost.emplace_back(neighbour, cost);
            // engine predict done
            engine->stopPredict();
            // restore trafficLight
            intersection->getTrafficLight().restore();
            neighbour->getIntersection()->getTrafficLight().restore();
            // unlock
            lk.unlock();
        }
    }

    void DSA_Agent::generateLocalCost(Intersection *neighbour, MovementPhases movementPhase) {
        double cost = 0;
        // lock
        std::unique_lock<std::mutex> lk(*enginePredictMutex);
        intersection->getTrafficLight().snapshot();
        intersection->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[movementPhase]);
        engine->predictPeriod(30);
        for (auto &roadLink: intersection->getRoadLinks()) {
            if (roadLink.getStartRoad()->getStartIntersection() == neighbour
                && (int) roadLink.getRoadLinkType() == (int) movementPhase % 2) {
                cost += std::pow(roadLink.getVehicleCnt(), 2);
            }
        }
        engine->stopPredict();
        intersection->getTrafficLight().restore();
        // unlock
        lk.unlock();
        costGraph[movementPhase][4].cost.emplace_back(nullptr, cost);
    }

    int DSA_Agent::getNotNullAgentSize(std::vector<DSA_Agent *> &agents) {
        int size = 0;
        for (auto &a: agents)
            size += a != nullptr;
        return size;
    }

    void DSA_Agent::generateCostGraph() {
        for (int movementPhase = 0; movementPhase < 4; ++movementPhase) {
            generateCostGraph(static_cast<MovementPhases>(movementPhase));
        }
    }

    void DSA_Agent::receiveMessage(DSA_Agent::MovementPhases movementPhase, double val, DSA_Agent *sender) {
//        std::cout << getId() << " receive message block" << '\n';
        {
            std::lock_guard<std::mutex> lock(*agentMutex);
            receivedMessage[movementPhase].Q += val;
            receivedMessage[movementPhase].sender.push_back(sender);
            ++currentReceivedNum;
        }
        cv->notify_one();
//        std::cout << getId() << " receive message block release" << '\n';
    }

    void DSA_Agent::makeDecision() {
        int bestPhase = -1;
        double minCost = INT32_MAX;
        for (int movementPhase = 0; movementPhase < 4; ++movementPhase) {
            auto phase = static_cast<MovementPhases>(movementPhase);
            double cost = 0;
            for (auto &localCost: costGraph[phase][4].cost)
                cost += localCost.second;
            cost += receivedMessage[phase].Q;
            if (cost < minCost) {
                minCost = cost;
                bestPhase = phase;
            }
        }
        if (bestPhase != -1) {
            intersection->getTrafficLight().setPhase(movementPhases_to_trafficLightPhase[bestPhase]);
//            cout << getId() << " make decision: " << bestPhase << '\n';
        } else {
            std::cerr << "error happen, agent Id: " << getId() << '\n';
        }
    }

    void DSA_Agent::resetAgent() {
        currentReceivedNum = 0;
        costGraph.clear();
        costGraph.resize(4, std::vector<Cost>(5, Cost()));
        receivedMessage.clear();
        receivedMessage.resize(4, Message());
        std::swap(inAgents, outAgents);
    }

    void DSA_Agent::run() {
        // wait receive all message
        std::unique_lock<std::mutex> lk(*agentMutex);
        cv->wait(lk, [&] { return currentReceivedNum >= 12 * getNotNullAgentSize(inAgents); });
        generateCostGraph();
        sendMessage();
        // reverse order
        currentReceivedNum = 0;
        std::swap(inAgents, outAgents);
        cv->wait(lk, [&] { return currentReceivedNum >= 12 * getNotNullAgentSize(inAgents); });
        costGraph.clear();
        costGraph.resize(4, std::vector<Cost>(5, Cost()));
        generateCostGraph();
        sendMessage();

        makeDecision();
    }
} // ALGO