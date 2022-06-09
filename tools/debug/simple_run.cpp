#include "engine/engine.h"
#include "algo/DSA/AgentCenter.cpp"
#include "algo/maxPressure.cpp"
#include <string>
#include <iostream>
#include <ctime>

using namespace SEUTraffic;

int main() {
//    std::string configFile = "cityflow_config/trafficJam/config.json";
    std::string configFile = "cityflow_config/syn_3x3/config.json";
    size_t totalStep = 1000;
    Engine engine(configFile, 8);
    time_t startTime, endTime;
    time(&startTime);
    ALGO::AgentCenter dcop(&engine);
    for (int i = 0; i < (int) totalStep; i++) {
        if(i % 10 == 0)
            dcop.run();
        engine.nextStep(false);
//        ALGO::maxPressure(&engine);
        if (i % (totalStep / 10) == 0) {
            std::cout << "The current degree of completion: " << 100 * i / totalStep << "%" << std::endl;
        }
    }
    engine.logTrafficStatistics();
    time(&endTime);
    std::cout << "Total Step: " << totalStep << std::endl;
    std::cout << "Total Time: " << (endTime - startTime) << "s" << std::endl;
    return 0;
}
