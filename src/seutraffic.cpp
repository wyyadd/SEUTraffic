#include "engine/engine.h"

#include "pybind11/pybind11.h"
#include "pybind11/stl.h"

namespace py = pybind11;
using namespace py::literals;

PYBIND11_MODULE(SEUTraffic, m)
{
    py::class_<SEUTraffic::Engine>(m, "Engine")
        .def(py::init<const std::string&, int>(),
            "config_file"_a,
            "thread_num"_a = 1)
        .def("next_step", &SEUTraffic::Engine::nextStep)
        .def("get_vehicle_count", &SEUTraffic::Engine::getVehicleCount)
        .def("get_vehicles", &SEUTraffic::Engine::getVehicles, "include_waiting"_a = false)
        .def("get_lane_vehicle_count", &SEUTraffic::Engine::getLaneVehicleCount)
        .def("get_vehicle_speed", &SEUTraffic::Engine::getVehicleSpeed)
        .def("get_lane_waiting_vehicle_count", &SEUTraffic::Engine::getLaneWaitingVehicleCount)
        .def("get_lane_vehicles", &SEUTraffic::Engine::getLaneVehicles)
        .def("get_vehicle_info", &SEUTraffic::Engine::getVehicleInfo, "vehicle_id"_a)
        .def("get_vehicle_distance", &SEUTraffic::Engine::getVehicleDistance)
        .def("get_leader", &SEUTraffic::Engine::getLeader, "vehicle_id"_a)
        .def("get_current_time", &SEUTraffic::Engine::getCurrentTime)
        .def("get_average_travel_time", &SEUTraffic::Engine::getAverageTravelTime)
        .def("set_tl_phase", &SEUTraffic::Engine::setTrafficLightPhase, "intersection_id"_a, "phaseIndex"_a)
        .def("reset", &SEUTraffic::Engine::reset, "seed"_a = false);

#ifdef VERSION
    m.attr("__version__") = VERSION;
#else
    m.attr("__version__") = "dev";
#endif
}
