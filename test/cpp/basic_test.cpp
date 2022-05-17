#include <gtest/gtest.h>
#include <iostream>
#include <ostream>
#include "engine/engine.h"
using namespace SEUTraffic;

size_t threads = std::min(std::thread::hardware_concurrency(), 4u);
std::string configFile = "/home/wyyadd/SRTP/SEUTraffic/cityflow_config/trafficJam/config.json";
//std::string configFile = "examples/config.json";


// TEST(Basic, Basic){
//     size_t totalStep = 50;
//     int interval = 10;
//     Engine engine(configFile, threads);
//     for (size_t i = 0; i < totalStep; i++) {
//         engine.step2(interval);
//         double avgTime = engine.getAverageTravelTime();
//         int activeCarCnt = engine.getActiveCars();
//         std::cerr<<"total car cnt = " << engine.getTotalCars()<<std::endl;
//         std::cerr<<"active car cnt = "<< activeCarCnt<<std::endl;
//         std::cerr << "step = " << engine.getSteps() << std::endl;
//         std::cerr << "average running time = " << avgTime << std::endl;
//         //
//         std::cerr << "current time = " << engine.getCurrentTime() <<std::endl;
//         std::cerr <<"---------------------------------------------"<<std::endl;
//     }
//     SUCCEED();
//     std::cerr<<"success" <<std::endl;
// }

TEST(Basic, reset) {
    size_t totalStep = 200;

    Engine engine(configFile, threads);
    for (size_t i = 0; i < totalStep; i++) {
        engine.nextStep();
    }
//    double curTime = engine.getCurrentTime();
//    size_t vehCnt = engine.getVehicleCount();
//    engine.reset(true);
//    for (size_t i = 0; i < totalStep; i++) {
//        engine.nextStep();
//    }
//    std::cerr <<"current time = " << engine.getCurrentTime() << " old currtime = " << curTime << std::endl;
//    EXPECT_EQ(engine.getCurrentTime(), curTime);
//    std::cerr<<"now vehicle cmt = " << engine.getVehicleCount() << " old ve cnt = " << vehCnt << std::endl;
//    EXPECT_EQ(engine.getVehicleCount(), vehCnt);
    SUCCEED();
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
