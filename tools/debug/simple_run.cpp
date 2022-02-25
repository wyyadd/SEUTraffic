#include "engine/engine.h"

#include <string>
#include <iostream>
#include <ctime>

using namespace SEUTraffic;

int main() {
    std::string configFile = "examples/config.json";
    size_t totalStep = 1000;

    Engine engine(configFile, 8);
    time_t startTime, endTime;
    time(&startTime);
    for (int i = 0; i < (int)totalStep; i++) {
        engine.nextStep();
        if(i % (totalStep/10) == 0)
            std::cout << "当前模拟进度" << 100*i/totalStep << "%" << std::endl;
    }
    engine.logTrafficStatistics();
    time(&endTime);
    std::cout << "Total Step: " << totalStep << std::endl;
    std::cout << "Total Time: " << (endTime - startTime) << "s" << std::endl;
    return 0;
}