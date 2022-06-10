//
// Created by wyyadd on 6/9/22.
//
#include "engine/engine.h"
#include <unordered_map>
#include <algorithm>

using namespace SEUTraffic;
using std::string;
using std::vector;
using std::cout;

class DCOP {
private:
    vector<Intersection> agents;
    vector<vector<int>> graph;
    Engine *engine;
    // intersection : number
    std::unordered_map<string, int> interMap;
    std::unordered_map<int, string> numberMap;

    void generateGraph() {
        for (auto i = 0; i < (int) engine->getRoadnet().getIntersections().size(); ++i) {
            interMap[agents[i].getId()] = i;
            numberMap[i] = agents[i].getId();
        }
        graph.resize(interMap.size(), vector<int>(interMap.size(), 0));
        for (auto &road: engine->getRoadnet().getRoads()) {
            graph[interMap[road.getStartIntersection()->getId()]][interMap[road.getEndIntersection()->getId()]] = 1;
            graph[interMap[road.getEndIntersection()->getId()]][interMap[road.getStartIntersection()->getId()]] = 1;
        }
        DAG();
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
        Intersection *sink_agent = nullptr;
        vector<int> sink_dist;
        for (auto &ai: agents) {
            auto dist = dijkstra(interMap[ai.getId()]);
            int dia_ai = *std::max_element(dist.begin(), dist.end());
            if (dia_ai < dia) {
                sink_agent = &ai;
                dia = dia_ai;
                sink_dist = dist;
            }
        }
        cout << sink_agent->getId() << std::endl;
        for (auto &road: engine->getRoadnet().getRoads()) {
            if (sink_dist[interMap[road.getStartIntersection()->getId()]] < sink_dist[interMap[road.getEndIntersection()->getId()]])
                graph[interMap[road.getStartIntersection()->getId()]][interMap[road.getEndIntersection()->getId()]] = 0;
            else
                graph[interMap[road.getEndIntersection()->getId()]][interMap[road.getStartIntersection()->getId()]] = 0;
        }
        for(int i = 0; i < graph[0].size(); ++i)
            for(int j = 0; j < graph[i].size(); ++j)
                if(graph[i][j])
                    cout << numberMap[i] << "--->" << numberMap[j] << std::endl;
    }

public:
    explicit DCOP(Engine *engine) {
        this->engine = engine;
        agents = engine->getRoadnet().getIntersections();
        auto x = agents[3].getRoadLinkTraffic();
        auto y = agents[3].getInRoadsTraffic();
        auto z = agents[3].getOutRoadsTraffic();
        generateGraph();
    }
};