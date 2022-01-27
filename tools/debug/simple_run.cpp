#include "engine/engine.h"

#include <string>
#include <iostream>
#include <cstdlib>
#include <ctime>

using namespace SEUTraffic;

int main() {
    std::string configFile = "examples/config.json";
    size_t totalStep = 200;

    Engine engine(configFile, 8);
    time_t startTime, endTime;
    time(&startTime);
    for (int i = 0; i < (int)totalStep; i++) {
        engine.nextStep();
    }
    time(&endTime);
    std::cout << "Total Step: " << totalStep << std::endl;
    std::cout << "Total Time: " << (endTime - startTime) << "s" << std::endl;
    return 0;
}