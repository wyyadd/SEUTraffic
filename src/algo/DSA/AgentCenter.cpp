//
// Created by wyyadd on 6/9/22.
//
#include "engine/engine.h"
#include "utility/barrier.h"
#include "DSA_Agent.h"
#include <unordered_map>
#include <algorithm>
#include <thread>

namespace ALGO {
    using std::string;
    using std::vector;
    using std::cout;
    using SEUTraffic::Engine;
    using SEUTraffic::Intersection;
    using SEUTraffic::Barrier;

    class AgentCenter {
    private:
        vector<DSA_Agent> agents;
        vector<vector<int>> graph;
        Engine *engine;
        // {intersection id : agent id}
        std::unordered_map<string, int> interMap;
        std::vector<std::thread> agentPool; // 线程池
        Barrier *startBarrier;
        Barrier *endBarrier;
        bool finished = false;
        std::mutex enginePredictMutex;

    private:
        void generateGraph() {
            for (auto &a: agents) {
                interMap[a.getId()] = a.getAgentId();
            }
            // if two agent(a,b) has an edge, graph[a][b]=1 and graph[b][a]=1
            graph.resize(agents.size(), vector<int>(agents.size(), 0));
            for (auto &road: engine->getRoadnet().getRoads()) {
                // we don't consider virtual intersection
                if (road.getStartIntersection()->isVirtualIntersection() ||
                    road.getEndIntersection()->isVirtualIntersection())
                    continue;
                graph[interMap[road.getStartIntersection()->getId()]][interMap[road.getEndIntersection()->getId()]] = 1;
                graph[interMap[road.getEndIntersection()->getId()]][interMap[road.getStartIntersection()->getId()]] = 1;
            }
            // DOCP algorithm need, DSA don't need
            DAG();
            // update agent order
            for (int i = 0; i < (int) graph[0].size(); ++i) {
                for (int j = 0; j < (int) graph[i].size(); ++j) {
                    if (graph[i][j]) {
                        agents[i].updateOutAgents(&agents[j]);
                        agents[j].updateInAgents(&agents[i]);
                        cout << agents[i].getId() << "--->" << agents[j].getId() << std::endl;
                    }
                }
            }
        }

        // A utility function to find the vertex with minimum distance value, from
        // the set of vertices not yet included in shortest path tree
        static int minDistance(const vector<int> &dist, const vector<bool> &sptSet) {
            // Initialize min value
            int min = INT_MAX, min_index;
            for (int v = 0; v < (int) dist.size(); v++)
                if (!sptSet[v] && dist[v] <= min)
                    min = dist[v], min_index = v;
            return min_index;
        }

        // Function that implements Dijkstra's single source shortest path algorithm
        // for a graph represented using adjacency matrix representation
        vector<int> dijkstra(int src) {
            int V = (int) graph[0].size();
            vector<int> dist(V); // The output array.  dist[i] will hold the shortest
            // distance from src to i
            vector<bool> sptSet(V); // sptSet[i] will be true if vertex i is included in shortest
            // path tree or shortest distance from src to i is finalized
            // Initialize all distances as INFINITE and stpSet[] as false
            for (int i = 0; i < V; i++)
                dist[i] = INT_MAX, sptSet[i] = false;
            // Distance of source vertex from itself is always 0
            dist[src] = 0;
            // Find shortest path for all vertices
            for (int count = 0; count < V - 1; count++) {
                // Pick the minimum distance vertex from the set of vertices not
                // yet processed. u is always equal to src in the first iteration.
                int u = minDistance(dist, sptSet);
                // Mark the picked vertex as processed
                sptSet[u] = true;
                // Update dist value of the adjacent vertices of the picked vertex.
                for (int v = 0; v < V; v++)
                    // Update dist[v] only if is not in sptSet, there is an edge from
                    // u to v, and total weight of path from src to  v through u is
                    // smaller than current value of dist[v]
                    if (!sptSet[v] && graph[u][v] && dist[u] != INT_MAX
                        && dist[u] + graph[u][v] < dist[v])
                        dist[v] = dist[u] + graph[u][v];
            }
            return dist;
        }

        void DAG() {
            int dia = INT32_MAX;
            DSA_Agent *sink_agent = nullptr;
            vector<int> sink_dist;
            // firstly, find a sink agent which is closest to all agents
            for (auto &ai: agents) {
                // dist is an array which present distance from ai to other agents
                auto dist = dijkstra(interMap[ai.getId()]);
                int dia_ai = *std::max_element(dist.begin(), dist.end());
                if (dia_ai < dia) {
                    sink_agent = &ai;
                    dia = dia_ai;
                    sink_dist = dist;
                }
            }
            cout << "sink Agent: " << sink_agent->getId() << std::endl;
            // secondly, determine edged two agents order By their distance to sink agent
            // if distance(sink_agent, a) < distance(sink_agent,b) then direction is b --> a, 反之亦然
            for (auto &road: engine->getRoadnet().getRoads()) {
                if (road.getEndIntersection()->isVirtualIntersection() ||
                    road.getStartIntersection()->isVirtualIntersection())
                    continue;
                if (sink_dist[interMap[road.getStartIntersection()->getId()]] <
                    sink_dist[interMap[road.getEndIntersection()->getId()]])
                    graph[interMap[road.getStartIntersection()->getId()]][interMap[road.getEndIntersection()->getId()]] = 0;
                else
                    graph[interMap[road.getEndIntersection()->getId()]][interMap[road.getStartIntersection()->getId()]] = 0;
            }
        }

        void agentRun(DSA_Agent &agent) {
            while (true) {
                startBarrier->wait();
                if (finished) break;
                agent.run();
                endBarrier->wait();
                agent.resetAgent();
            }
        }

    public:
        explicit AgentCenter(Engine *e) {
            this->engine = e;
            // init agents
            int id = 0;
            // 这步好像很重要，vector频繁的调整大小的话，会对mutex指针造成破坏，这是为什么捏捏捏
            agents.reserve(e->getRoadnet().getIntersections().size());
            for (auto &intersection: e->getRoadnet().getIntersections()) {
                if (!intersection.isVirtualIntersection()) {
                    agents.emplace_back(id++, &intersection, engine, &enginePredictMutex);
                }
            }
            startBarrier = new Barrier(agents.size() + 1);
            endBarrier = new Barrier(agents.size() + 1);
            // generate DAG graph
            generateGraph();
            // start thread
            for (auto &agent: agents)
                agentPool.emplace_back(&AgentCenter::agentRun, this, std::ref(agent));
        }

        void run() {
            startBarrier->wait();
            endBarrier->wait();
        }

        ~AgentCenter() {
            finished = true;
            startBarrier->wait();
            for (auto &thread: agentPool) thread.join();
            delete startBarrier;
            delete endBarrier;
        }
    };
} // ALGO

