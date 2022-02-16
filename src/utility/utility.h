#ifndef SEUTRAFFIC_UTILITY_H
#define SEUTRAFFIC_UTILITY_H
#include <cmath>
#include <iostream>
#include <random>
#include <typeinfo>
#include <cerrno>
#include "rapidjson/document.h"
#include "dtoa_milo.h"

namespace SEUTraffic
{
    constexpr double eps = 1e-8;
    constexpr size_t JSON_BUFFER_SIZE = 65536;

    class Road;
    class Lane;

    bool readJsonFromFile(const std::string &filename, rapidjson::Document &document);

    class JsonFormatError: public std::runtime_error {
    public:
        explicit JsonFormatError(const std::string &info) : std::runtime_error(info){}
    };

    class JsonMemberMiss: public JsonFormatError{
    private:
        std::string info;
    public:
        explicit JsonMemberMiss(const std::string &name) :
                JsonFormatError(name + " is required but missing in json file"){}
    };

    class JsonTypeError: public JsonFormatError{
    private:
        std::string info;
    public:
        JsonTypeError(const std::string &name, const char* type):
                JsonFormatError(name + ": expected type " + type){}
    };

    const rapidjson::Value &getJsonMemberValue(const std::string &name, const rapidjson::Value &object);

    template<typename T>
    bool jsonConvertableTo(const rapidjson::Value &value){
        return value.Is<T>();
    };

    template<>
    bool jsonConvertableTo<double>(const rapidjson::Value &value);

    template<typename T>
    T getJsonMember(const std::string &name, const rapidjson::Value &object){
        assert(object.IsObject());
        const auto &value = getJsonMemberValue(name, object);
        if (!jsonConvertableTo<T>(value))
            throw JsonTypeError(name, typeid(T).name());
        return value.Get<T>();
    }

    template<typename T>
    T getJsonMember(const std::string &name, const rapidjson::Value &object, const T &default_value){
        assert(object.IsObject());
        auto iter = object.FindMember(name.c_str());
        if (!jsonConvertableTo<T>(iter->value)) {
            std::cerr<<"why cannot convert " << name << " to int "<<std::endl;
        }

        if (iter == object.MemberEnd() || !jsonConvertableTo<T>(iter->value))
            return default_value;
        return iter->value.Get<T>();
    }

    const rapidjson::Value &
    getJsonMemberObject(const std::string &name, const rapidjson::Value &object);

    const rapidjson::Value&
    getJsonMemberArray(const std::string& name, const rapidjson::Value& object);

    // wyy modify: log
    bool writeJsonToFile(const std::string &filename, const rapidjson::Document &document);

    // wyy modify: add point class
    class Point {
    public:
        double x = 0.0;
        double y = 0.0;

        static constexpr double eps = 1e-8;
        static int sign(double x);

        Point() = default;
        Point(double x, double y);
        double len();
        Point normal();
        Point unit();
        double ang();
    };
    typedef Point Vector;

    Point operator*(const Point &A, double k);
    Point operator-(const Point &A, const Point &B);
    Point operator+(const Point &A, const Point &B);
    Point operator-(const Point &A);
    double crossMultiply(const Point &A, const Point &B);
    double dotMultiply(const Point &A, const Point &B);
    double calcDist(const Point &A, const Point &B);
    double calcAng(Point A, Point B);

    Point calcIntersectPoint(Point A, Point B, Point C, Point D);
    bool onSegment(Point A, Point B, Point P);

    inline double max2double(double x, double y) {
        return x > y ? x : y;
    }

    inline double min2double(double x, double y) {
        return x < y ? x : y;
    }

    inline void dtoa_milo(double value, char* buffer) {
    // Not handling NaN and inf
    assert(!std::isnan(value));
    assert(!std::isinf(value));

    if (value == 0) {
        buffer[0] = '0';
        buffer[1] = '.';
        buffer[2] = '0';
        buffer[3] = '\0';
    }
    else {
        if (value < 0) {
            *buffer++ = '-';
            value = -value;
        }
        int length, K;
        Grisu2(value, buffer, &length, &K);
        Prettify(buffer, length, K);
    }
}

    inline std::string double2string(double x) {
        char ret[30];
        dtoa_milo(x, ret);
        std::string str(ret);
        return str;
    }
}
#endif // SEUTRAFFIC_UTILITY_H
