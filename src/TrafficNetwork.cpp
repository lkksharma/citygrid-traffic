// TrafficNetwork.cpp
// The city map: owns every intersection, road and vehicle, and keeps road congestion
// up to date as vehicles move. The other components work on a reference to it.
#pragma once

#include <stdexcept>

#include "Entities.cpp"

using namespace std;

class TrafficNetwork {
    // std::map never relocates its elements, so the Road* stored in intersections stay valid.
    map<string, Intersection> intersections;
    map<string, Road> roads;
    map<string, Vehicle> vehicles;

public:
    void addIntersection(const string& id, const string& name) {
        if (!intersections.emplace(id, Intersection(id, name)).second)
            throw invalid_argument("duplicate intersection id " + id);
    }

    // Adds a directed road and wires it into both intersections it touches.
    void addRoad(const string& id, const string& from, const string& to, double lengthMeters, int capacity) {
        Intersection* source = findIntersection(from);
        Intersection* target = findIntersection(to);
        if (!source || !target) throw invalid_argument("road " + id + " references an unknown intersection");

        auto [it, inserted] = roads.emplace(id, Road(id, from, to, lengthMeters, capacity));
        if (!inserted) throw invalid_argument("duplicate road id " + id);
        source->outgoingRoads[id] = &it->second;
        target->incomingRoads[id] = &it->second;
    }

    Vehicle& addVehicle(const string& id, const vector<string>& route) {
        auto [it, inserted] = vehicles.emplace(id, Vehicle(id, route));
        if (!inserted) throw invalid_argument("duplicate vehicle id " + id);
        return it->second;
    }

    // Moves a vehicle onto a road, taking it off its previous road first. Every congestion
    // figure in the system comes from these per-road vehicle sets.
    void placeVehicleOnRoad(const string& vehicleId, const string& roadId) {
        Vehicle* vehicle = findVehicle(vehicleId);
        Road* road = findRoad(roadId);
        if (!vehicle || !road) throw invalid_argument("unknown vehicle " + vehicleId + " or road " + roadId);

        if (Road* previous = findRoad(vehicle->currentRoadId)) previous->vehiclesOnRoad.erase(vehicleId);
        road->vehiclesOnRoad.insert(vehicleId);
        vehicle->currentRoadId = roadId;
    }

    Intersection* findIntersection(const string& id) {
        auto it = intersections.find(id);
        return it == intersections.end() ? nullptr : &it->second;
    }
    Road* findRoad(const string& id) {
        auto it = roads.find(id);
        return it == roads.end() ? nullptr : &it->second;
    }
    Vehicle* findVehicle(const string& id) {
        auto it = vehicles.find(id);
        return it == vehicles.end() ? nullptr : &it->second;
    }
    map<string, Road>& getRoads() { return roads; }

    // A route is valid when it starts and ends at intersections and every road
    // begins exactly where the previous one ended (so it respects road direction).
    bool isValidRoute(const vector<string>& route) {
        if (route.size() < 2 || !findIntersection(route.front()) || !findIntersection(route.back())) return false;
        string at = route.front();
        for (size_t i = 1; i + 1 < route.size(); ++i) {
            Road* road = findRoad(route[i]);
            if (!road || road->fromIntersection != at) return false;
            at = road->toIntersection;
        }
        return at == route.back();
    }
};
