#ifndef SEUTRAFFIC_VEHICLEINFO_H
#define SEUTRAFFIC_VEHICLEINFO_H

#include <utility>

#include "vehicle/router.h"

namespace SEUTraffic {

    class Vehicle;

    class VehicleInfo {

    private:
        double length = 5;
        double width = 2;
        double speed = 10;
        double minGap = 2.5;
        std::shared_ptr<const Router> router = nullptr;

    public:
        VehicleInfo(double length, double width, double minGap, std::shared_ptr<const Router> &router)
                : length(length), width(width), minGap(minGap), router(router) {};

        Road *getFirstRoad() const;

        double getSpeed() const {
            return this->speed;
        }

        double getMinGap() const { return this->minGap; }

        Router getRouter() const { return *router; }

        double getLen() const { return this->length; }

        double getWidth() const { return this->width; }
    };
}
#endif // SEUTRAFFIC_VEHICLEINFO_H
