// AccidentAlert.cpp
// Simulated collision sensors plus an idempotent alert service.
//
// Idempotency: a crash stays in front of the sensor until it is cleared, so the same collision
// is reported on every tick. Each road therefore has at most ONE active accident. Repeated reports
// map to the existing accident ID, and each vehicle is alerted at most once per accident. A new
// crash after clearance gets a new ID. IDs come from a counter and are never reused.
#pragma once

#include <cstdio>
#include <random>

#include "TrafficNetwork.cpp"

using namespace std;

struct SensorReading {
    string roadId;
    bool collisionDetected;
    int tick;
};

// Stands in for the per-road collision sensors. Seeded, so every run gives the same output.
class CollisionSensor {
    mt19937 rng;
    uniform_real_distribution<double> chance{0.0, 1.0};
    double randomCollisionRate;   // per road, per tick
    set<string> activeCollisions; // crashes still physically on the road (sticky until cleared)

public:
    CollisionSensor(unsigned seed, double randomCollisionRate) : rng(seed), randomCollisionRate(randomCollisionRate) {}

    void setRandomCollisionRate(double rate) { randomCollisionRate = rate; }
    void injectCollision(const string& roadId) { activeCollisions.insert(roadId); }
    void clearCollision(const string& roadId) { activeCollisions.erase(roadId); }

    SensorReading poll(const string& roadId, int tick) {
        if (chance(rng) < randomCollisionRate) activeCollisions.insert(roadId);
        return {roadId, activeCollisions.count(roadId) > 0, tick};
    }
};

struct Accident {
    string accidentId;
    string roadId;
    int detectedAtTick;
    int clearedAtTick = -1;
    bool active = true;
    set<string> notifiedVehicles;  // makes alerts idempotent per vehicle
};

struct AlertNotification {
    string accidentId;
    string vehicleId;
    string roadId;
    bool vehicleIsOnRoad;  // true: stuck on the road; false: only planned to use it, so should reroute
};

class AccidentAlert {
    CollisionSensor& sensor;
    map<string, Accident> accidents;           // accidentId -> accident (full history)
    map<string, string> activeAccidentOnRoad;  // roadId -> active accidentId (the idempotency key)
    map<string, Vehicle*> subscribers;
    int nextAccidentNumber = 1;
    int currentTick = 0;

public:
    explicit AccidentAlert(CollisionSensor& sensor) : sensor(sensor) {}

    void subscribe(Vehicle* vehicle) { subscribers[vehicle->id] = vehicle; }  // subscribing twice is harmless

    // Polls every road's sensor once and returns only the NEW notifications from this tick.
    // Replaying a tick with the same readings produces no new accidents and no repeat alerts.
    vector<AlertNotification> processTick(TrafficNetwork& network) {
        ++currentTick;
        vector<AlertNotification> sent;
        bool anyCollision = false;

        for (auto& [roadId, road] : network.getRoads()) {
            SensorReading reading = sensor.poll(roadId, currentTick);
            if (!reading.collisionDetected) continue;
            anyCollision = true;

            Accident& accident = registerCollision(reading, road);
            for (auto& [vehicleId, vehicle] : subscribers) {
                bool onRoad = vehicle->currentRoadId == roadId;
                if (!onRoad && !vehicle->plansToUse(roadId)) continue;
                if (!accident.notifiedVehicles.insert(vehicleId).second) continue;  // already alerted

                if (onRoad)
                    printf("  [ALERT %s] %s: you are ON %s (%s -> %s) where a collision occurred, proceed with caution\n",
                           accident.accidentId.c_str(), vehicleId.c_str(), roadId.c_str(),
                           road.fromIntersection.c_str(), road.toIntersection.c_str());
                else
                    printf("  [ALERT %s] %s: collision on %s (%s -> %s), which is on your route, rerouting advised\n",
                           accident.accidentId.c_str(), vehicleId.c_str(), roadId.c_str(),
                           road.fromIntersection.c_str(), road.toIntersection.c_str());
                sent.push_back({accident.accidentId, vehicleId, roadId, onRoad});
            }
        }

        if (!anyCollision) printf("  [SENSOR t=%d] all roads clear\n", currentTick);
        return sent;
    }

    // Marks the accident resolved and reopens the road. Clearing twice is a no-op that returns false.
    bool clearAccident(const string& accidentId, TrafficNetwork& network) {
        auto it = accidents.find(accidentId);
        if (it == accidents.end() || !it->second.active) {
            printf("  [CLEAR] %s is unknown or already cleared, nothing to do\n", accidentId.c_str());
            return false;
        }
        Accident& accident = it->second;
        accident.active = false;
        accident.clearedAtTick = currentTick;
        activeAccidentOnRoad.erase(accident.roadId);
        sensor.clearCollision(accident.roadId);
        if (Road* road = network.findRoad(accident.roadId)) road->blocked = false;
        printf("  [CLEAR] %s cleared, road %s reopened\n", accidentId.c_str(), accident.roadId.c_str());
        return true;
    }

    const map<string, Accident>& getAccidents() const { return accidents; }

    void printAccidentLog() const {
        printf("  %-9s %-6s %-9s %-8s %-8s %s\n", "ID", "Road", "Detected", "Cleared", "Status", "Vehicles alerted");
        for (const auto& [id, a] : accidents) {
            string cleared = a.active ? "-" : "t=" + to_string(a.clearedAtTick);
            printf("  %-9s %-6s t=%-7d %-8s %-8s %zu\n", id.c_str(), a.roadId.c_str(), a.detectedAtTick,
                   cleared.c_str(), a.active ? "ACTIVE" : "CLEARED", a.notifiedVehicles.size());
        }
    }

private:
    // Returns the road's existing active accident, or opens a new one with a fresh unique ID.
    Accident& registerCollision(const SensorReading& reading, Road& road) {
        auto existing = activeAccidentOnRoad.find(reading.roadId);
        if (existing != activeAccidentOnRoad.end()) {
            printf("  [SENSOR t=%d] %s: collision still reported, already tracked as %s (duplicate ignored)\n",
                   reading.tick, reading.roadId.c_str(), existing->second.c_str());
            return accidents.at(existing->second);
        }

        char id[16];
        snprintf(id, sizeof(id), "ACC-%04d", nextAccidentNumber++);
        Accident accident;
        accident.accidentId = id;
        accident.roadId = reading.roadId;
        accident.detectedAtTick = reading.tick;

        activeAccidentOnRoad[reading.roadId] = id;
        road.blocked = true;
        printf("  [SENSOR t=%d] %s: collision detected, NEW accident %s, road closed\n",
               reading.tick, reading.roadId.c_str(), id);
        return accidents.emplace(id, accident).first->second;
    }
};
