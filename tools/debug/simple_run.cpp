#include "engine/engine.h"

#include <string>
#include <iostream>
#include <ctime>

using namespace SEUTraffic;

int main() {
    std::string configFile = "/home/wyyadd/SRTP/SEUTraffic/cityflow_config/trafficJam/config.json";
//    std::string configFile = "cityflow_config/examples/config.json";
    size_t totalStep = 1000;
    bool fixedTimeTraffic = true;//是否采用固定时长红绿灯

    Engine engine(configFile, 8);
    time_t startTime, endTime;
    time(&startTime);
    for (int i = 0; i < (int) totalStep; i++) {
        engine.nextStep(fixedTimeTraffic);
        if (i % (totalStep / 10) == 0) {
            std::cout << "The current degree of completion: " << 100 * i / totalStep << "%" << std::endl;
//            engine.predictPeriod(30);
        }
    }
    engine.logTrafficStatistics();
    time(&endTime);
    std::cout << "Total Step: " << totalStep << std::endl;
    std::cout << "Total Time: " << (endTime - startTime) << "s" << std::endl;
    return 0;
}
