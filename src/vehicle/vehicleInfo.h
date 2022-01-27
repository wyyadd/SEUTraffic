
#ifndef CITYFLOW_VEHICLEINFO_H
#define CITYFLOW_VEHICLEINFO_H

#include "vehicle/router.h"
namespace SEUTraffic {

    class Vehicle;

    class VehicleInfo {

    private:
        double length = 5;
        double width = 2;
        Router router;
        double speed = 10;
        double minGap = 2.5;

    public:
        VehicleInfo(double length, double width, Router &router ):length(length), width(width), router(router) {};

        Road *getFirstRoad() const;

        double getSpeed() const {
            return this->speed;
        }

        double getMinGap() const { return this->minGap; }

        Router& getRouter() { return router; }

        Router getRouter() const {return router;}

        double getLen() const { return this->length; }

        double getWidth() const { return this->width;}
    };
}
#endif
