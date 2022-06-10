#include "engine/engine.h"
#include "algo/DCOP.cpp"
#include <string>
#include <iostream>
#include <ctime>

using namespace SEUTraffic;

int main() {
    std::string configFile = "/home/wyyadd/SRTP/SEUTraffic/cityflow_config/trafficJam/config.json";
//    std::string configFile = "cityflow_config/examples/config.json";
    size_t totalStep = 1000;
    Engine engine(configFile, 8);
    time_t startTime, endTime;
    time(&startTime);
    for (int i = 0; i < (int) totalStep; i++) {
        engine.nextStep(true);
//        engine.predictPeriod(30);
        if (i % (totalStep / 10) == 0) {
            std::cout << "The current degree of completion: " << 100 * i / totalStep << "%" << std::endl;
        }
    }
    DCOP dcop(&engine);
    engine.logTrafficStatistics();
    time(&endTime);
    std::cout << "Total Step: " << totalStep << std::endl;
    std::cout << "Total Time: " << (endTime - startTime) << "s" << std::endl;
    return 0;
}
