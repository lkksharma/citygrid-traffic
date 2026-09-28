// TrafficLightManager.cpp
// Dynamic red/green light timing for an intersection, based on how congested each
// incoming road is.
//
// Signal model: a round-robin cycle with one green phase per incoming road. While one road
// is green, every other road is red, so  red(road) = sum of the other roads' green times.
#pragma once

#include <cstdio>

#include "TrafficNetwork.cpp"

using namespace std;

struct SignalTiming {
    string roadId;
    string tier;
    int vehicles;
    int capacity;
    double congestion;
    int greenSec;
    int redSec;
};

class TrafficLightManager {
public:
    static constexpr int MIN_GREEN = 5;         // lower bound: a nearly empty road barely interrupts the others
    static constexpr int MAX_GREEN = 120;       // upper bound: 2 minutes
    static constexpr int MAX_RED = 120;         // no approach waits more than 2 minutes, so no road backs up
    static constexpr int BASELINE_GREEN = 60;   // conventional fixed-time signal, shown for comparison

    map<string, SignalTiming> computeTimings(const Intersection& intersection) const {
        map<string, SignalTiming> timings;
        for (const auto& [roadId, road] : intersection.incomingRoads) {
            SignalTiming t;
            t.roadId = roadId;
            t.vehicles = (int)road->vehiclesOnRoad.size();
            t.capacity = road->capacity;
            t.congestion = road->getCongestion();
            tie(t.tier, t.greenSec) = greenForRoad(*road);
            t.redSec = 0;
            timings[roadId] = t;
        }

        fitRedsUnderCap(timings, intersection.id);

        int cycle = cycleLength(timings);
        for (auto& [roadId, t] : timings) t.redSec = cycle - t.greenSec;
        return timings;
    }

    void printTimings(const Intersection& intersection, const map<string, SignalTiming>& timings) const {
        int n = (int)timings.size();
        int baselineRed = BASELINE_GREEN * (n - 1);
        printf("Intersection %s (%s): %d signal(s), dynamic cycle %d s\n",
               intersection.id.c_str(), intersection.name.c_str(), n, cycleLength(timings));
        printf("  %-6s %-7s %-10s %-9s %6s %6s   | fixed-time green / red\n",
               "Road", "Load", "Congestion", "Tier", "Green", "Red");
        for (const auto& [roadId, t] : timings) {
            printf("  %-6s %2d/%-4d %-10.2f %-9s %4d s %4d s   | %d s / %d s%s\n",
                   roadId.c_str(), t.vehicles, t.capacity, t.congestion, t.tier.c_str(), t.greenSec, t.redSec,
                   BASELINE_GREEN, baselineRed, baselineRed > MAX_RED ? "  (over the 2-min cap!)" : "");
        }
    }

private:
    // Rule table: the first rule whose upper bound exceeds the congestion wins.
    struct Rule {
        double congestionBelow;
        const char* tier;
        int greenSec;
    };
    static constexpr Rule RULES[] = {
        {0.25, "LIGHT", 15},
        {0.50, "MODERATE", 35},
        {0.75, "HEAVY", 60},
        {0.90, "SEVERE", 90},
    };

    static pair<string, int> greenForRoad(const Road& road) {
        // An accident upstream stops traffic reaching the light, so a long green would be wasted.
        if (road.blocked) return {"BLOCKED", MIN_GREEN};
        if (road.vehiclesOnRoad.empty()) return {"EMPTY", MIN_GREEN};
        for (const Rule& rule : RULES)
            if (road.getCongestion() < rule.congestionBelow) return {rule.tier, rule.greenSec};
        return {"JAMMED", MAX_GREEN};
    }

    static int cycleLength(const map<string, SignalTiming>& timings) {
        int cycle = 0;
        for (const auto& [roadId, t] : timings) cycle += t.greenSec;
        return cycle;
    }

    // The longest red belongs to the road with the shortest green: cycle - min(green).
    // If that exceeds MAX_RED, shrink every green above MIN_GREEN by the same factor. Using one
    // factor keeps the priority order, so the most congested road still gets the longest green.
    static void fitRedsUnderCap(map<string, SignalTiming>& timings, const string& intersectionId) {
        while (true) {
            int cycle = cycleLength(timings);
            int shortestGreen = MAX_GREEN;
            for (const auto& [roadId, t] : timings) shortestGreen = min(shortestGreen, t.greenSec);
            int longestRed = cycle - shortestGreen;
            if (longestRed <= MAX_RED) return;

            double factor = (double)MAX_RED / longestRed;
            bool changed = false;
            for (auto& [roadId, t] : timings) {
                if (t.greenSec <= MIN_GREEN) continue;
                t.greenSec = max(MIN_GREEN, (int)(t.greenSec * factor));  // truncation guarantees progress
                changed = true;
            }
            if (!changed) {  // every road is already at MIN_GREEN: too many approaches to honour the cap
                printf("  [WARN] %s: %zu approaches cannot all stay under a %d s red; using %d s greens\n",
                       intersectionId.c_str(), timings.size(), MAX_RED, MIN_GREEN);
                return;
            }
        }
    }
};
