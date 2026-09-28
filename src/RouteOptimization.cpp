// RouteOptimization.cpp
// Congestion-aware routing. Intersections are graph nodes and roads are DIRECTED weighted
// edges; Dijkstra's algorithm finds the cheapest route to the vehicle's destination.
#pragma once

#include <limits>
#include <queue>
#include <stdexcept>

#include "TrafficNetwork.cpp"

using namespace std;

class RouteOptimization {
    TrafficNetwork& network;
    double congestionPenalty;  // hyperparameter: how strongly congestion inflates a road's cost
    double detourFactor;       // routes may be at most this many times the shortest free-flow distance
    vector<string> lastExcludedRoads;

    static constexpr double INF = numeric_limits<double>::infinity();

public:
    // congestionPenalty has no default on purpose: it is a tuning knob the caller must choose.
    //   0   -> pure shortest distance, congestion ignored
    //   k   -> a fully jammed road costs (1 + k) times its length
    RouteOptimization(TrafficNetwork& network, double congestionPenalty, double detourFactor = 1.5)
        : network(network) {
        setCongestionPenalty(congestionPenalty);
        if (detourFactor < 1.0) throw invalid_argument("detourFactor must be >= 1");
        this->detourFactor = detourFactor;
    }

    void setCongestionPenalty(double penalty) {
        if (penalty < 0) throw invalid_argument("congestionPenalty must be >= 0");
        congestionPenalty = penalty;
    }
    double getCongestionPenalty() const { return congestionPenalty; }

    // Roads the anti-detour guard ruled out during the last optimizeRoute() call.
    const vector<string>& getLastExcludedRoads() const { return lastExcludedRoads; }

    double edgeWeight(const Road& road) const {
        return road.lengthMeters * (1.0 + congestionPenalty * road.getCongestion());
    }

    // Input and output use the same shape: [source, road1, ..., roadN, destination].
    // Only the endpoints of the input matter; the roads in between are re-planned.
    vector<string> optimizeRoute(const vector<string>& currentRoute) {
        lastExcludedRoads.clear();
        if (currentRoute.size() < 2) return currentRoute;
        const string& source = currentRoute.front();
        const string& destination = currentRoute.back();
        if (!network.findIntersection(source) || !network.findIntersection(destination)) {
            printf("  [ROUTER] unknown source/destination in %s, route unchanged\n", routeToString(currentRoute).c_str());
            return currentRoute;
        }
        if (source == destination) return {source, destination};

        // Step 1 (anti-detour guard): free-flow distances from the source and back from the destination.
        // A road u->v is allowed only if SOME route through it stays within detourFactor x the shortest
        // trip. This removes dead ends (nodes that cannot reach the destination) and roads that head
        // away from it, even when they are empty and look cheap.
        map<string, double> fromSource = freeFlowDistances(source, false);
        map<string, double> toDestination = freeFlowDistances(destination, true);
        double shortest = distanceOrInf(fromSource, destination);
        if (shortest == INF) {
            printf("  [ROUTER] no open road leads from %s to %s, keep the current route and hold\n",
                   source.c_str(), destination.c_str());
            return currentRoute;
        }

        set<string> allowedRoads;
        for (auto& [roadId, road] : network.getRoads()) {
            double throughThisRoad = distanceOrInf(fromSource, road.fromIntersection) + road.lengthMeters +
                                     distanceOrInf(toDestination, road.toIntersection);
            if (throughThisRoad <= detourFactor * shortest + 1e-9) allowedRoads.insert(roadId);
            else lastExcludedRoads.push_back(roadId);
        }

        // Step 2: Dijkstra over allowed roads with congestion-weighted costs.
        map<string, double> cost;
        map<string, string> arrivedVia;  // intersection -> road used to reach it on the best route
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> frontier;
        cost[source] = 0;
        frontier.push({0, source});

        while (!frontier.empty()) {
            auto [costSoFar, at] = frontier.top();
            frontier.pop();
            if (costSoFar > distanceOrInf(cost, at)) continue;  // stale queue entry
            if (at == destination) break;

            for (const auto& [roadId, road] : network.findIntersection(at)->outgoingRoads) {
                if (!allowedRoads.count(roadId)) continue;
                double newCost = costSoFar + edgeWeight(*road);
                if (newCost < distanceOrInf(cost, road->toIntersection)) {
                    cost[road->toIntersection] = newCost;
                    arrivedVia[road->toIntersection] = roadId;
                    frontier.push({newCost, road->toIntersection});
                }
            }
        }

        // Step 3: walk the arrivedVia links back from the destination to rebuild the route.
        vector<string> roadsReversed;
        for (string at = destination; at != source; at = network.findRoad(arrivedVia[at])->fromIntersection)
            roadsReversed.push_back(arrivedVia[at]);

        vector<string> optimized = {source};
        optimized.insert(optimized.end(), roadsReversed.rbegin(), roadsReversed.rend());
        optimized.push_back(destination);
        return optimized;
    }

    // Congestion-weighted cost of a route; infinite if it is invalid or uses a blocked road.
    double routeCost(const vector<string>& route) {
        if (!network.isValidRoute(route)) return INF;
        double total = 0;
        for (size_t i = 1; i + 1 < route.size(); ++i) {
            Road* road = network.findRoad(route[i]);
            if (road->blocked) return INF;
            total += edgeWeight(*road);
        }
        return total;
    }

private:
    static double distanceOrInf(const map<string, double>& distances, const string& node) {
        auto it = distances.find(node);
        return it == distances.end() ? INF : it->second;
    }

    // Plain-length Dijkstra over open roads. With reverse = true it follows roads backwards,
    // which gives every node's distance TO `start` instead of FROM it.
    map<string, double> freeFlowDistances(const string& start, bool reverse) {
        map<string, double> distance = {{start, 0}};
        priority_queue<pair<double, string>, vector<pair<double, string>>, greater<>> frontier;
        frontier.push({0, start});

        while (!frontier.empty()) {
            auto [d, at] = frontier.top();
            frontier.pop();
            if (d > distance[at]) continue;

            Intersection* node = network.findIntersection(at);
            for (const auto& [roadId, road] : reverse ? node->incomingRoads : node->outgoingRoads) {
                if (road->blocked) continue;
                const string& next = reverse ? road->fromIntersection : road->toIntersection;
                double nd = d + road->lengthMeters;
                if (nd < distanceOrInf(distance, next)) {
                    distance[next] = nd;
                    frontier.push({nd, next});
                }
            }
        }
        return distance;
    }
};

inline vector<string> Vehicle::requestOptimizedRoute(RouteOptimization& optimizer) {
    route = optimizer.optimizeRoute(route);
    return route;
}
