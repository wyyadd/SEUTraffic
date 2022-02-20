#include "roadnet/roadnet.h"
#include "roadnet/trafficlight.h"
#include "vehicle/vehicle.h"
#include <cmath>
#include <set>
#include <iostream>
#include <iterator>
#include <algorithm>

namespace SEUTraffic{
    // 计算points中相邻两个坐标点的距离和
    static double getLengthOfPoints(const std::vector<Point> &points) {
        double length = 0.0;
        for (size_t i = 0; i + 1 < points.size(); i++)
            length += (points[i + 1] - points[i]).len();
        return length;
    }

    // wyy modify: add getPointsByDis, and Dir
    // wyy function: 根据车在lane的dis计算车的point
    static Point getPointByDistance(const std::vector<Point> &points, double dis) {
        //yzh：dis为dis和points长度中的较小值；
        dis = min2double(max2double(dis, 0), getLengthOfPoints(points));
        //yzh：dis小于等于0时，返回第一个点的坐标；
        if (dis <= 0.0)
            return points[0];
         //yzh：dis大于0，小于points长度时，返回点的对应坐标
        for (size_t i = 1; i < points.size(); i++) {
            double len = (points[i - 1] - points[i]).len();
            if (dis > len)
                dis -= len;
            else
                return points[i - 1] + (points[i] - points[i - 1]) * (dis / len);
        }
        //yzh：dis大于points的长度时，返回最后一个point的坐标
        return points.back();
    }

    Point RoadNet::getPoint(const Point &p1, const Point &p2, double a) {
        return {(p2.x - p1.x) * a + p1.x, (p2.y - p1.y) * a + p1.y};
    }

    //yzh: read roadNet.json文件
    bool RoadNet::loadFromJson(const std::string& jsonFileName){
        rapidjson::Document document;
        if (!readJsonFromFile(jsonFileName, document)){
            std::cerr << "cannot open roadNet file" << std::endl;
            return false;
        }
        /**
         *  path is like a stack to detect and avoid errors
         */
        std::list<std::string> path;
        if (!document.IsObject())
            throw JsonTypeError("roadNet config file", "object");
        try{
            const rapidjson::Value &interValues = getJsonMemberArray("intersections", document);
            const rapidjson::Value &roadValues = getJsonMemberArray("roads", document);

            //build mapping
            roads.resize(roadValues.Size());
            intersections.resize(interValues.Size());

            for (rapidjson::SizeType i = 0; i < roadValues.Size(); i++){
                path.emplace_back("road[" + std::to_string(i) + "]");
                std::string id =  getJsonMember<const char*>("id", roadValues[i]);
                roadMap[id] = &roads[i];
                roads[i].id = id;
                path.pop_back();
            }
            assert(path.empty());

            for(rapidjson::SizeType i = 0; i < interValues.Size(); i++){
                path.emplace_back("intersection[" + std::to_string(i) + "]");
                std::string id = getJsonMember<const char*>("id", interValues[i]);
                interMap[id] = &intersections[i];
                intersections[i].id = id;
                interIds.push_back(id);
                path.pop_back();
            }
            assert(path.empty());

            // read road
            path.emplace_back("roads");
            for (rapidjson::SizeType i = 0; i < roadValues.Size();i++){
                path.emplace_back(roads[i].getId());
                const auto &curRoadValue = roadValues[i]; //遍历roadValues
                if (!curRoadValue.IsObject()){
                    throw JsonTypeError("road[" + std::to_string(i) + "]" , "object");
                }

                roads[i].startIntersection = interMap[getJsonMember<const char*>("startIntersection", curRoadValue)];
                roads[i].endIntersection = interMap[getJsonMember<const char *>("endIntersection", curRoadValue)];

                //check
                if (!roads[i].startIntersection) throw JsonFormatError("startIntersection does not exist");
                if (!roads[i].endIntersection) throw JsonFormatError("endIntersection does not exist.");

                //read lanes
                const auto &lanesValue = getJsonMemberArray("lanes", curRoadValue);
                int laneIndex = 0;
                for (const auto &laneValue : lanesValue.GetArray()){
                    path.emplace_back ("lane[" + std::to_string(laneIndex) + "]");
                    if (!laneValue.IsObject()){
                        throw JsonTypeError("lane","object");
                    }
                    auto width = getJsonMember<double>("width", laneValue);
                    auto maxSpeed = getJsonMember<double>("maxSpeed", laneValue);
                    roads[i].lanes.emplace_back(width, maxSpeed, laneIndex, &roads[i]);
                    laneIndex++;
                    path.pop_back();
                }

                for (auto& lane : roads[i].lanes) {
                    drivableMap[lane.getId()] = &lane;
                }
                path.pop_back();

                //  read points
                const auto &pointsValue = getJsonMemberArray("points", curRoadValue);
                for (const auto &pointValue : pointsValue.GetArray()) {
                    path.emplace_back("point[" + std::to_string(roads[i].points.size()) + "]");
                    if (!pointValue.IsObject())
                        throw JsonTypeError("point of road", "object");
                    auto x = getJsonMember<double>("x", pointValue);
                    auto y = getJsonMember<double>("y", pointValue);
                    // for every road add its position
                    roads[i].points.emplace_back(x, y);
                    path.pop_back();
                }
            }
            path.pop_back();
            assert(path.empty());

            // read intersections
            std::map<std::string, RoadLinkType> typeMap = {{"turn_left", turn_left},
                                                       {"turn right" , turn_right},
                                                        {"go_straight" , go_straight}};

            path.emplace_back("intersections");
            for (rapidjson::SizeType i = 0; i < interValues.Size(); i++){
                path.emplace_back(intersections[i].getId());
                const auto &curInterValue = interValues[i];
                if  (!curInterValue.IsObject()){
                    throw JsonTypeError("intersection", "object");
                    return false;
                }

                // wyy modify: intersection read points
                const auto &pointValue = getJsonMemberObject("point", curInterValue);
                intersections[i].isVirtual = getJsonMember<bool>("virtual", curInterValue);
                auto x = getJsonMember<double>("x", pointValue);
                auto y = getJsonMember<double>("y", pointValue);
                intersections[i].point = Point(x, y);
                // wyy end modify

                // read roads
                const auto &roadsValue = getJsonMemberArray("roads", curInterValue);
                for (auto &roadNameValue : roadsValue.GetArray()){
                    path.emplace_back("roads[" + std::to_string(intersections[i].roads.size()) + "]");
                    std::string roadName = roadNameValue.GetString();
                    if (!roadMap.count(roadName)){
                        throw JsonFormatError("No such road: " + roadName);
                    }
                    intersections[i].roads.push_back(roadMap[roadName]);
                    path.pop_back();
                }

                // skip other information if intersection is virtual
                intersections[i].trafficLight.intersection = &intersections[i];
                if (intersections[i].isVirtual){
                    path.pop_back();
                    continue;
                }

                // read width
                intersections[i].width = getJsonMember<double>("width", curInterValue);

                // read roadLinks
                const auto &roadLinksValue = getJsonMemberArray("roadLinks", curInterValue);
                intersections[i].roadLinks.resize(roadLinksValue.Size());
                int roadLinkIndex = 0;
                for (const auto &roadLinkValue : roadLinksValue.GetArray()){
                    path.emplace_back("roadLinks[" + std::to_string(roadLinkIndex) + "]");
                    if (!roadLinkValue.IsObject()){
                        throw JsonTypeError("roadLink","object");
                    }
                    RoadLink &roadLink = intersections[i].roadLinks[roadLinkIndex];
                    roadLink.index = roadLinkIndex ++;
                    roadLink.type = typeMap[getJsonMember<const char*>("type", roadLinkValue)];
                    roadLink.startRoad = roadMap[getJsonMember<const char*>("startRoad", roadLinkValue)];
                    roadLink.endRoad = roadMap[getJsonMember<const char*>("endRoad", roadLinkValue)];

                    const auto &laneLinksValue = getJsonMemberArray("laneLinks", roadLinkValue);
                    roadLink.laneLinks.resize(laneLinksValue.Size());
                    int laneLinkIndex = 0;
                    for (const auto &laneLinkValue : laneLinksValue.GetArray()){
                        path.emplace_back("laneLinks[" + std::to_string(laneLinkIndex) + "]");
                        if (!laneLinkValue.IsObject())
                            throw JsonTypeError("laneLink", "object");
                        LaneLink &laneLink = roadLink.laneLinks[laneLinkIndex++];

                        int startLaneIndex = getJsonMember<int>("startLaneIndex", laneLinkValue);
                        int endLaneIndex = getJsonMember<int>("endLaneIndex", laneLinkValue);
                        if (startLaneIndex >= static_cast<int>(roadLink.startRoad->lanes.size()) || startLaneIndex < 0){
                            throw JsonFormatError("startLaneIndex out of range");
                        }
                        if (endLaneIndex >= static_cast<int>(roadLink.endRoad->lanes.size()) || endLaneIndex < 0) {

                            throw JsonFormatError("endLaneIndex out of range.");
                        }
                        Lane *startLane = &roadLink.startRoad->lanes[startLaneIndex];
                        Lane* endLane = &roadLink.endRoad->lanes[endLaneIndex];
                        // wyy: 此处应加上point
                        // wyy modify: read lanelink points
                        auto iter = laneLinkValue.FindMember("points");
                        if (iter != laneLinkValue.MemberEnd() && !iter->value.IsArray())
                            throw JsonTypeError("points in laneLink", "array");
                        if (iter != laneLinkValue.MemberEnd() && !iter->value.Empty())
                            for (const auto &pValue : iter->value.GetArray()) {
                                laneLink.points.emplace_back(getJsonMember<double>("x", pValue),
                                                             getJsonMember<double>("y", pValue));
                            }
                        else {
                            // TODO what does it mean
                            Point start = Point(startLane->getPointByDistance(
                                    startLane->getLength() - startLane->getEndIntersection()->width));
                            Point end = Point(
                                    endLane->getPointByDistance(0.0 + endLane->getStartIntersection()->width));
                            double len = (Point(end.x - start.x, end.y - start.y)).len();
                            Point startDirection = startLane->getDirectionByDistance(
                                    startLane->getLength() - startLane->getEndIntersection()->width);
                            Point endDirection = endLane->getDirectionByDistance(
                                    0.0 + endLane->getStartIntersection()->width);
                            double minGap = 5;
                            double gap1X = startDirection.x * len * 0.5;
                            double gap1Y = startDirection.y * len * 0.5;
                            double gap2X = -endDirection.x * len * 0.5;
                            double gap2Y = -endDirection.y * len * 0.5;
                            if (gap1X * gap1X + gap1Y * gap1Y < 25 && startLane->getEndIntersection()->width >= 5) {
                                gap1X = minGap * startDirection.x;
                                gap1Y = minGap * startDirection.y;
                            }
                            if (gap2X * gap2X + gap2Y * gap2Y < 25 && endLane->getStartIntersection()->width >= 5) {
                                gap2X = minGap * endDirection.x;
                                gap2Y = minGap * endDirection.y;
                            }
                            Point mid1 = Point(start.x + gap1X,start.y + gap1Y);
                            Point mid2 = Point(end.x + gap2X,end.y + gap2Y);
                            int numPoints = 10;
                            for (int j = 0; j <= numPoints; j++) {
                                Point p1 = getPoint(start, mid1, j / double(numPoints));
                                Point p2 = getPoint(mid1, mid2, j / double(numPoints));
                                Point p3 = getPoint(mid2, end, j / double(numPoints));
                                Point p4 = getPoint(p1, p2, j / double(numPoints));
                                Point p5 = getPoint(p2, p3, j / double(numPoints));
                                Point p6 = getPoint(p4, p5, j / double(numPoints));
                                laneLink.points.emplace_back(p6.x, p6.y);
                            }
                        }
                        // wyy end modify
                       
                        laneLink.roadLink = &roadLink;
                        laneLink.startLane = startLane;
                        laneLink.endLane = endLane;
                        // wyy modify: lanelink length
                        laneLink.length = getLengthOfPoints(laneLink.points);
                        //laneLink.length = 300;
                        startLane->laneLinks.push_back(&laneLink);
                        drivableMap.emplace(laneLink.getId(), &laneLink);
                        path.pop_back();
                    }
                    roadLink.intersection = &intersections[i];
                    path.pop_back();
                }

                // read trafficLight
                int phaseIndex = 0;
                const auto &trafficLightValue = getJsonMemberObject("trafficLight", curInterValue);
                path.emplace_back("trafficLight");
                const auto &lightPhasesValue = getJsonMemberArray("lightphases", trafficLightValue);
                for (const auto &lightPhaseValue : lightPhasesValue.GetArray()){
                    path.emplace_back("lightphases[" + std::to_string(intersections[i].trafficLight.phases.size())+"]");
                    if (!lightPhaseValue.IsObject()){
                        throw JsonTypeError("lightphase", "object");
                    }
                    LightPhase lightPhase;
                    lightPhase.time = getJsonMember<double>("time", lightPhaseValue);
                    lightPhase.phase = phaseIndex++;
                    lightPhase.roadLinkAvailable = std::vector<bool>(intersections[i].roadLinks.size(), false);
                    const auto& availableRoadLinksValue =
                            getJsonMemberArray("availableRoadLinks", lightPhaseValue);
                    for (rapidjson::SizeType index = 0; index < availableRoadLinksValue.Size() ; index++){
                        path.emplace_back("availableRoadLinks[" + std::to_string(index) + "]");
                        if (!availableRoadLinksValue[index].IsInt()){
                            throw JsonTypeError("availableRoadLink", "int");
                        }
                        size_t indexInRoadLinks = availableRoadLinksValue[index].GetUint();
                        if (indexInRoadLinks >= lightPhase.roadLinkAvailable.size()){
                            throw JsonFormatError("index out of range.");
                        }
                        lightPhase.roadLinkAvailable[indexInRoadLinks] = true;
                        path.pop_back();
                    }
                    intersections[i].trafficLight.phases.push_back(lightPhase);
                    path.pop_back();
                }
                path.pop_back(); // End of traffic light
                intersections[i].trafficLight.init(0);

                path.pop_back(); // End of intersection
            }
            path.pop_back();
            assert(path.empty());
        }catch(const JsonFormatError &e){
            std::cerr << "Error occured when reading the roadNet file: " << std::endl;
            for (const auto &node : path){
                std::cerr << "/" << node;
            }
            std::cerr << " " << e.what() << std::endl;
            return false;
        }

        // wyy modify: initlanesPoints
        for (auto &road : roads)
            road.initLanesPoints();
        // wyy end modify

        for (auto& road : roads) {
            auto& roadLanes = road.getLanePointers();
            lanes.insert(lanes.end(), roadLanes.begin(), roadLanes.end());
            drivables.insert(drivables.end(), roadLanes.begin(), roadLanes.end());
        }

        for (auto &intersection : intersections) {
            auto &intersectionLaneLinks = intersection.getLaneLinks();
            laneLinks.insert(laneLinks.end(), intersectionLaneLinks.begin(), intersectionLaneLinks.end());
            drivables.insert(drivables.end(), intersectionLaneLinks.begin(), intersectionLaneLinks.end());
        }
        return true;
    }

    //yzh:生成前端道路静态展示需要的JSON文件
    rapidjson::Value RoadNet::convertToJson(rapidjson::Document::AllocatorType &allocator) {
        rapidjson::Value jsonRoot(rapidjson::kObjectType);
        // write nodes
        rapidjson::Value jsonNodes(rapidjson::kArrayType);
        for (auto & intersection : intersections) {
            rapidjson::Value jsonNode(rapidjson::kObjectType), jsonPoint(rapidjson::kArrayType);
            rapidjson::Value idValue;
            idValue.SetString(rapidjson::StringRef(intersection.id.c_str()));
            jsonNode.AddMember("id", idValue, allocator);
            jsonPoint.PushBack(intersection.point.x, allocator);
            jsonPoint.PushBack(intersection.point.y, allocator);
            jsonNode.AddMember("point", jsonPoint, allocator);
            jsonNode.AddMember("virtual", intersection.isVirtual, allocator);
            if (!intersection.isVirtual) {
                jsonNode.AddMember("width", intersection.width, allocator);
            }

            rapidjson::Value jsonOutline(rapidjson::kArrayType);
            for (auto &point: intersection.getOutline()) {
                jsonOutline.PushBack(point.x, allocator);
                jsonOutline.PushBack(point.y, allocator);
            }
            jsonNode.AddMember("outline", jsonOutline, allocator);
            jsonNodes.PushBack(jsonNode, allocator);
        }
        jsonRoot.AddMember("nodes", jsonNodes, allocator);

        //write edges
        rapidjson::Value jsonEdges(rapidjson::kArrayType);
        for (auto & road : roads) {
            rapidjson::Value jsonEdge(rapidjson::kObjectType);
            rapidjson::Value jsonPoints(rapidjson::kArrayType);
            rapidjson::Value jsonLaneWidths(rapidjson::kArrayType);
            rapidjson::Value jsonDirs(rapidjson::kArrayType);

            rapidjson::Value idValue;
            idValue.SetString(rapidjson::StringRef(road.id.c_str()));
            jsonEdge.AddMember("id", idValue, allocator);
            rapidjson::Value startValue;
            if (road.startIntersection)
                startValue.SetString(rapidjson::StringRef(road.startIntersection->id.c_str()));
            else
                startValue.SetString("null");
            jsonEdge.AddMember("from", startValue, allocator);

            rapidjson::Value endValue;
            if (road.endIntersection)
                endValue.SetString(rapidjson::StringRef(road.endIntersection->id.c_str()));
            else
                endValue.SetString("null");
            jsonEdge.AddMember("to", endValue, allocator);
            for (size_t j = 0; j < road.points.size(); ++j) {
                rapidjson::Value jsonPoint(rapidjson::kArrayType);
                jsonPoint.PushBack(road.points[j].x, allocator);
                jsonPoint.PushBack(road.points[j].y, allocator);
                jsonPoints.PushBack(jsonPoint, allocator);
            }
            jsonEdge.AddMember("points", jsonPoints, allocator);
            jsonEdge.AddMember("nLane", static_cast<int>(road.lanes.size()), allocator);
            for (size_t j = 0; j < road.lanes.size(); ++j) {
                jsonLaneWidths.PushBack(road.lanes[j].width, allocator);
            }
            jsonEdge.AddMember("laneWidths", jsonLaneWidths, allocator);
            jsonEdges.PushBack(jsonEdge, allocator);
        }
        jsonRoot.AddMember("edges", jsonEdges, allocator);
        return jsonRoot;
    }

    void RoadNet::reset()
    {
        for (auto &road : roads)  road.reset();
        for (auto& intersection : intersections) intersection.reset();
    }

    //yzh:根据road的points计算出lane的points
    void Road::initLanesPoints() {
        double dsum = 0.0;
        std::vector<Point> roadPoints = this->points;

        assert(roadPoints.size() >= 2);

        /**
         * p2-p1 is a vector , (p2 - p1).unit() is a 单位 vector
         */
        if (!startIntersection->isVirtualIntersection()) {
            double width = startIntersection->width;
            Point p1 = roadPoints[0];
            Point p2 = roadPoints[1];
            // GUESS: 根据十字路口的宽度， 动态修正road的宽度
            roadPoints[0] = p1 + (p2 - p1).unit() * width;
        }

        if (!endIntersection->isVirtualIntersection()) {
            double width = endIntersection->width;
            Point p1 = roadPoints[roadPoints.size() - 2];
            Point p2 = roadPoints[roadPoints.size() - 1];
            roadPoints[roadPoints.size() - 1] = p2 - (p2 - p1).unit() * width;
        }

        for (Lane &lane : lanes) {
            double dmin = dsum;
            double dmax = dsum + lane.width;
            lane.points.clear();
            for (int j = 0; j < (int) roadPoints.size(); j++) {
                // TODO: the '(dmin + dmax) / 2.0' is wrong
                std::vector<Point> &lanePoints = lane.points;
                if (j == 0) {
                    // u is 单位 vector
                    Vector u = (roadPoints[1] - roadPoints[0]).unit();
                    // v is 向right旋转90度 of u
                    Vector v = -u.normal();
                    Point startPoint = roadPoints[j] + v * ((dmin + dmax) / 2.0);
                    lanePoints.push_back(startPoint);
                } else if (j + 1 == (int) roadPoints.size()) {
                    Vector u = (roadPoints[j] - roadPoints[j - 1]).unit();
                    Vector v = -u.normal();
                    Point endPoint = roadPoints[j] + v * ((dmin + dmax) / 2.0);
                    lanePoints.push_back(endPoint);
                } else {
                    Vector u1 = (roadPoints[j + 1] - roadPoints[j]).unit();
                    Vector u2 = (roadPoints[j] - roadPoints[j - 1]).unit();
                    Vector u = (u1 + u2).unit();
                    Vector v = -u.normal();
                    Point interPoint = roadPoints[j] + v * ((dmin + dmax) / 2.0);
                    lanePoints.push_back(interPoint);
                }
            }
            lane.length = getLengthOfPoints(lane.points);
            dsum += lane.width;
        }
    }

    void Road::reset() {
        for (auto &lane : lanes) lane.reset();
    }

    double Road::averageLength() const{
        double sum = 0;
        size_t laneNum = getLanes().size();
        if (laneNum == 0) return 0;
        for (const auto &lane : getLanes()){
            sum += lane.getLength();
        }
        return sum / laneNum;
    }

    Lane::Lane(){
        width = 0;
        maxSpeed = 0;
        laneIndex = -1;
        belongRoad = 0;
        drivableType = LANE;
    }

    Lane::Lane(double width, double maxSpeed, int laneIndex, Road *belongRoad){
        this->width = width;
        this->maxSpeed = maxSpeed;
        this->laneIndex = laneIndex;
        this->belongRoad = belongRoad;
        drivableType = LANE;
    }

    bool Lane::available(const Vehicle *vehicle) const{ // TODO
        if (!vehicles.empty()){
            Vehicle *tail =vehicles.back();
            return tail->getDistance() > tail->getLen() + vehicle->getMinGap(); //TODO
        }
        else return true;
    }

    bool Lane::canEnter(const Vehicle *vehicle) const {
        if (!vehicles.empty()) {
            Vehicle *tail = vehicles.back();
            return tail->getDistance() > tail->getLen() + vehicle->getLen() || //TODO
                   tail->getSpeed() >= 2; //todo: speed > 2 or?
        } else {
            return true;
        }
    }

    std::vector<LaneLink *> Lane::getLaneLinksToRoad(const Road *road) const {
        std::vector<LaneLink *> ret;
        for (auto &laneLink : laneLinks) {
            if (laneLink->getEndLane()->getBelongRoad() == road)
                ret.push_back(laneLink);
        }
        return ret;
    }

    // wyy modify: add getPointsByDistance and getDir
    Point Drivable::getPointByDistance(double dis) const {
        return SEUTraffic::getPointByDistance(points, dis);
    }

    // wyy: 根据车在lane上的dis求方向
    Point Drivable::getDirectionByDistance(double dis) const {
        double remain = dis;
        for (int i = 0; i + 1 < (int) points.size(); i++) {
            double len = (points[i + 1] - points[i]).len();
            if (remain < len)
                // 距离dis最近两点的单位向量
                return (points[i + 1] - points[i]).unit();
            else
                remain -= len;
        }
        // 最后两点的单位向量
        return (points[points.size() - 1] - points[points.size() - 2]).unit();
    }

    const std::vector<LaneLink *> &Intersection::getLaneLinks() {
        if (laneLinks.size() > 0) return laneLinks;
        for (auto &roadLink : roadLinks) {
            auto &roadLaneLinks = roadLink.getLaneLinkPointers();
            laneLinks.insert(laneLinks.end(), roadLaneLinks.begin(), roadLaneLinks.end());
        }
        return laneLinks;
    }

    std::vector<LaneLink *> &RoadLink::getLaneLinkPointers() {
        if (laneLinkPointers.size() > 0) return laneLinkPointers;
        for (auto &laneLink : laneLinks) {
            laneLinkPointers.push_back(&laneLink);
        }
        return laneLinkPointers;
    }

    void Intersection::reset() {
        trafficLight.reset();
        for (auto &roadLink : roadLinks) roadLink.reset();
    }

    // wyy modify: log
    std::vector<Point> Intersection::getOutline() {
        // Calculate the convex hull as the outline of the intersection
        std::vector<Point> points;
        points.push_back(getPosition());
        for (auto road : getRoads()){
            Vector roadDirect = road->getEndIntersection()->getPosition() - road->getStartIntersection()->getPosition();
            roadDirect = roadDirect.unit();
            Vector pDirect = roadDirect.normal();
            if (road->getStartIntersection() == this) {
                roadDirect = -roadDirect;
            }
            /*                          <deltaWidth>
             *                   [pointB *]------[pointB1 *]--------
             *                       |
             *                       v
             *                   [pDirect] <- roadDirect <- Road
             *                       |
             *                       v
             * [intersection]----[pointA *]------[pointA1 *]--------
             */
            double roadWidth = road->getWidth();
            double deltaWidth = 0.5 * min2double(width, roadWidth);
            deltaWidth = max2double(deltaWidth, 5);

            Point pointA = getPosition() -  roadDirect * width;
            Point pointB  = pointA - pDirect * roadWidth;
            points.push_back(pointA);
            points.push_back(pointB);

            if (deltaWidth < road->averageLength()) {
                Point pointA1 = pointA - roadDirect * deltaWidth;
                Point pointB1 = pointB - roadDirect * deltaWidth;
                points.push_back(pointA1);
                points.push_back(pointB1);
            }
        }

        auto minIter = std::min_element(points.begin(), points.end(),
                                        [](const Point &a, const Point &b){ return a.y < b.y; });

        Point p0 = *minIter;
        std::vector<Point> stack;
        stack.push_back(p0);
        points.erase(minIter);

        std::sort(points.begin(), points.end(),
                  [&p0](const Point &a, const Point &b)
                  {return (a - p0).ang() < (b - p0).ang(); });

        for (size_t i = 0 ; i < points.size(); ++i) {
            Point &point = points[i];
            Point p2 = stack[stack.size() - 1];
            if (stack.size() < 2) {
                if (point.x != p2.x || point.y != p2.y)
                    stack.emplace_back(point);
                continue;
            }
            Point p1 = stack[stack.size() - 2];

            while (stack.size() > 1 && crossMultiply(point - p2, p2 - p1) >= 0) {
                p2 = p1;
                stack.pop_back();
                if (stack.size() > 1) p1 = stack[stack.size() - 2];
            }
            stack.emplace_back(point);
        }

        return stack;
    }

    void RoadLink::reset() {
        for (auto &laneLink : laneLinks) laneLink.reset();
    }
    
    //yzh:清空LaneLink中的vehicles
    void LaneLink::reset() {
         vehicles.clear();
    }

    //yzh:清空vehicles和waitingBuffer中的车辆
    void Lane::reset()
    {
//        waitingBuffer.clear();
        vehicles.clear();
    }

    RoadLink Intersection::getRoadLink(Road* startRoad, Road* endRoad)
    {
        // std::cerr<<"start road = " + startRoad->getId() + "end road = " + endRoad->getId()<<std::endl;
        for (auto& rLink : roadLinks) {
            Road* stRoad = rLink.getStartRoad();
            Road* edRoad = rLink.getEndRoad();
            // std::cerr<<"stroad = " + stRoad->getId() + " ed road = " + edRoad->getId() << std::endl;
            if (stRoad->getId() == startRoad->getId() && edRoad->getId() == endRoad->getId()){
                return rLink;
            }
        }

        std::cerr<<"start road = " + startRoad->getId() + ", end road = " + endRoad->getId()<<std::endl;
        std::cerr <<  "the roadlink doesn't match the inter's roadlink."<<std::endl;
        std::cerr << "inter id = " + id << std::endl;

        throw  "the roadlink doest match the inter's roadlink. inter id = " + id + " last rlink is " + startRoad->getId() + " next rlink " + endRoad->getId();
    }

    int Intersection::getMaxpressurePhase(bool isdebug)
    {
        double maxPressure = -0x3f3f3f;
        double incomings = 0;
        double outcomings = 0;
        int bestPhaseIndex = 0;
        for (LightPhase phase : trafficLight.getPhases()) {
            incomings = 0;
            outcomings = 0;
            std::vector<bool> roadLinkAvailable = phase.getRoadLinkAvailable();
            std::set<Lane *> startLanes;
            std::set<Lane*> endLanes;

            if (getId() == "intersection_3_1" && isdebug) {
                std::cerr <<"inter 3_1, phase " << phase.getPhaseIndex() << " ok rlink: ";
                for (int i = 0; i < roadLinkAvailable.size(); i++) {
                    bool ok = roadLinkAvailable[i];
                    if (ok ==true)
                        std::cerr << i <<" ";
                }
                std::cerr<<std::endl;
            }
            for (RoadLink rlink : roadLinks) { // 这里的逻辑写错了
                if (roadLinkAvailable[rlink.getIndex()]) {
                    for (auto& lanelink : rlink.getLaneLinks()) {
                        Lane* stlane = lanelink.getStartLane();
                        Lane* edlane = lanelink.getEndLane();
                        startLanes.insert(stlane);
                        endLanes.insert(edlane);
                    }
                }
            }

            for (auto stLane : startLanes) {
                int incomingCars = stLane->getVehicleCnt();
                if (getId() == "intersection_3_1" && isdebug) {
                    Road* stRoad = stLane->getBelongRoad();
                    // comment: to see more details
                    std::cerr <<"phase = " << phase.getPhaseIndex() <<" inter = " << getId() <<" road = " << stRoad->getId() << " incoming lane " << stLane->getId() << " car cnt = " << incomingCars<< std::endl;
                }
                incomings += incomingCars;
            }
            for (auto edLane: endLanes) {
                int outcomingcars = edLane->getVehicleCnt();
                if (getId() == "intersection_3_1" && isdebug) {
                    Road *edRoad = edLane->getBelongRoad();
                    std::cerr <<"phase = " << phase.getPhaseIndex() <<" inter = " << getId() <<" road = " << edRoad->getId() << " outcoming lane " << edLane->getId() << " car cnt = " <<  outcomingcars << std::endl;
                }
                outcomings += outcomingcars;
            }
            double pressure = incomings - outcomings;

            if (getId() == "intersection_3_1" && isdebug) {
                std::cerr <<" inter id = " << getId() << " phase = " << phase.getPhaseIndex() << ", pressure = " << pressure <<std::endl;
            }
            if (maxPressure < pressure) {
                maxPressure = pressure;
                bestPhaseIndex = phase.getPhaseIndex();
            }
            startLanes.clear();
            endLanes.clear();
        }
        if (isdebug)
            std::cerr<< "inter id = " << getId() << " max pressure = " << maxPressure << " signal = " << bestPhaseIndex << std::endl;
        return bestPhaseIndex;
    }

    double Road::getWidth() const{
        double width = 0;
        for (const auto &lane : getLanes()){
            width += lane.getWidth();
        }
        return width;
    }

    //yzh：road的长度等于各lane长度之和
    double Road::getLength() const{
        double length = 0;
        for (const auto &lane : getLanes()){
            length += lane.getLength();
        }
        return length;
    }

    const std::vector<Lane*>& Road::getLanePointers()
    {
        if (!lanePointers.empty()) return lanePointers;
        for (auto& lane : lanes) {
            lanePointers.push_back(&lane);
        }
        return lanePointers;
    }
}
