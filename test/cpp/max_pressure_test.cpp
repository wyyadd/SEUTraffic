#include<gtest/gtest.h>
#include <iostream>
#include <iterator>
#include <ostream>
#include "engine/engine.h"
#include "roadnet/roadnet.h"
using namespace SEUTraffic;

size_t threads = std::min(std::thread::hardware_concurrency(), 16u);
std::string configFile = "../cityflow_config/ny_1x16/config.json";
// std::string configFile = "../cityflow_config/syn_4x4/config.json";
// std::string configFile = "../cityflow_config/manhattan_16x3/config.json";
// std::string configFile = "../cityflow_config/hangzhou_4x4/config.json";

TEST(Basic, Basic){
    size_t totalStep = 460;
    int interval = 10;
    //看看多少个flow
    Engine engine(configFile, threads);
    std::vector<std::string> interIds = engine.getInterIds();
    bool isdebug = false;
    // Intersection* debugInter = engine.getRoadnet().getIntersectionById("intersection_1_1");
    // std::cerr <<" debug inter = " << debugInter->getId() << std::endl;
    // for (Road* road : debugInter->getRoads()) {
    //     for (Lane lane : road->getLanes()) {
    //         std::cerr<< " lane id = " << lane.getId() << std::endl;
    //     }
    // }
    // std::cerr << std::endl;

    for (size_t i = 0; i < totalStep; i++) {
        std::cerr <<"---------------------------------------------"<<std::endl;
        // if (i >= totalStep - 10) isdebug = true;
        for (std::string id : interIds) {
            int bestPhaseIndex;
            // maxpressure
            Intersection* inter = engine.getRoadnet().getIntersectionById(id);

            bestPhaseIndex = inter->getMaxpressurePhase(isdebug);
            // if (id == "intersection_1_1")
            //     bestPhaseIndex = 2;
            // }
            if (inter->getId() == "intersection_1_1") {
                std::cerr <<"intersection_1_1 choose phase " << bestPhaseIndex <<std::endl;
            }
            engine.setTrafficLightPhase(id, bestPhaseIndex);
        }

        // output debug info
        Intersection* debugInter = engine.getRoadnet().getIntersectionById("intersection_1_1");
        std::cerr << " debug inter = " << debugInter->getId() << std::endl;
        for (Road* road : debugInter->getRoads()) {
            for (Lane lane : road->getLanes()) {
                std::cerr << "lane id = " << lane.getId() << " car cnt = " << lane.getVehicleCnt() << std::endl;
                std::cerr << "lane length = " << lane.getLength() <<std::endl;
            }
        }
        std::cerr<<"before step ,total car cnt = " << engine.getTotalCars()<<std::endl;

        engine.step2(interval);
        double avgTime = engine.getAverageTravelTime();
        double activeCarCnt = engine.getActiveCars();
        std::cerr<<"total car cnt = " << engine.getTotalCars()<<std::endl;
        std::cerr << "active car cnt = " << activeCarCnt << std::endl;
        std::cerr << "step = " << engine.getSteps() << std::endl;
        std::cerr << "average running time = " << avgTime << std::endl;
        std::cerr << "current time = " << engine.getCurrentTime() << std::endl;

        // std::cerr <<"---------------------------------------------"<<std::endl;
    }
    SUCCEED();
    std::cerr << "success" << std::endl;
}

int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
}
