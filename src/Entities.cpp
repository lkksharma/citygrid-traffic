// Entities.cpp
// Core data model: Road (directed edge), Intersection (graph node) and Vehicle.
//
// Project convention: there are no header files. Only main.cpp is compiled, and it
// #includes these .cpp files. Each file is therefore guarded with #pragma once and
// defines its member functions inside the class body (implicitly inline).
#pragma once

#include <iostream>
#include <map>
#include <set>
#include <string>
#include <vector>

using namespace std;

class RouteOptimization;  // full definition in RouteOptimization.cpp

// A ONE-WAY road between two intersections. A two-way street is modelled as two Road
// objects (one per direction), so the router can never send a car against traffic.
class Road {
public:
    string id;
    string fromIntersection;
    string toIntersection;
    double lengthMeters;
    int capacity;                // vehicles the road holds when fully jammed
    set<string> vehiclesOnRoad;  // IDs of vehicles currently on this road
    bool blocked = false;        // set by AccidentAlert while a collision is active

    Road(const string& id, const string& from, const string& to, double lengthMeters, int capacity)
        : id(id), fromIntersection(from), toIntersection(to), lengthMeters(lengthMeters), capacity(capacity) {}

    // Congestion is derived purely from the vehicle count: 0.0 = empty, 1.0 = at capacity.
    // It is not clamped, so an overloaded road (> 1.0) keeps getting more expensive to route through.
    double getCongestion() const {
        if (capacity <= 0) return 1.0;
        return (double)vehiclesOnRoad.size() / capacity;
    }
};

// A junction of several roads. Every INCOMING road has its own traffic light facing the
// traffic that arrives on it; outgoing roads are what the router follows.
class Intersection {
public:
    string id;
    string name;
    map<string, Road*> incomingRoads;
    map<string, Road*> outgoingRoads;

    Intersection(const string& id, const string& name) : id(id), name(name) {}
};

class Vehicle {
public:
    string id;
    vector<string> route;  // [sourceIntersection, road1, road2, ..., destinationIntersection]
    string currentRoadId;  // empty while the vehicle waits at an intersection

    Vehicle(const string& id, const vector<string>& route) : id(id), route(route) {}

    // The vehicle asks the optimizer for a better route and adopts it.
    // Defined in RouteOptimization.cpp, where RouteOptimization is a complete type.
    vector<string> requestOptimizedRoute(RouteOptimization& optimizer);

    // True if the road appears among the planned roads (the first and last entries are intersections).
    bool plansToUse(const string& roadId) const {
        for (size_t i = 1; i + 1 < route.size(); ++i)
            if (route[i] == roadId) return true;
        return false;
    }
};

inline string routeToString(const vector<string>& route) {
    string out = "[";
    for (size_t i = 0; i < route.size(); ++i) out += (i ? ", " : "") + route[i];
    return out + "]";
}
