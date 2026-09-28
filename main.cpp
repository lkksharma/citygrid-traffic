// main.cpp
// Smart Traffic Management System: runs three scenarios to show dynamic signal timing,
// congestion-aware routing and idempotent accident alerts.
//
// Build: make   (compiles only this file; the src/*.cpp modules are #included below)
// Run:   ./traffic_sim
#include "src/Entities.cpp"
#include "src/TrafficNetwork.cpp"
#include "src/TrafficLightManager.cpp"
#include "src/RouteOptimization.cpp"
#include "src/AccidentAlert.cpp"

using namespace std;

// Sample city. Every road is one-way and holds 20 vehicles; lengths are in metres.
//   A Home   B Market   C Central Square   D School   E Hospital   F Office   G Outskirts
// Main corridors from A to F:   A-B-C-F (1200 m)   and   A-D-E-F (1350 m).
// G is a far-out ring road. It is always empty, which makes it a tempting detour.
void buildCity(TrafficNetwork& city) {
    city.addIntersection("A", "Home");
    city.addIntersection("B", "Market");
    city.addIntersection("C", "Central Square");
    city.addIntersection("D", "School");
    city.addIntersection("E", "Hospital");
    city.addIntersection("F", "Office");
    city.addIntersection("G", "Outskirts");

    const int CAP = 20;
    city.addRoad("R_AB", "A", "B", 400, CAP);   city.addRoad("R_BA", "B", "A", 400, CAP);
    city.addRoad("R_BC", "B", "C", 400, CAP);   city.addRoad("R_CB", "C", "B", 400, CAP);
    city.addRoad("R_CF", "C", "F", 400, CAP);
    city.addRoad("R_AD", "A", "D", 450, CAP);   city.addRoad("R_DA", "D", "A", 450, CAP);
    city.addRoad("R_DE", "D", "E", 450, CAP);   city.addRoad("R_ED", "E", "D", 450, CAP);
    city.addRoad("R_EF", "E", "F", 450, CAP);   // one-way: there is no F -> E road
    city.addRoad("R_DC", "D", "C", 500, CAP);
    city.addRoad("R_EC", "E", "C", 300, CAP);   city.addRoad("R_CE", "C", "E", 300, CAP);
    city.addRoad("R_GC", "G", "C", 900, CAP);
    city.addRoad("R_AG", "A", "G", 800, CAP);
    city.addRoad("R_GF", "G", "F", 1100, CAP);  city.addRoad("R_FG", "F", "G", 1100, CAP);
}

// Puts `count` background vehicles on a road to create congestion.
void addTraffic(TrafficNetwork& city, const string& roadId, int count) {
    static int counter = 0;
    Road* road = city.findRoad(roadId);
    for (int i = 0; i < count; ++i) {
        string id = "BG" + to_string(++counter);
        city.addVehicle(id, {road->fromIntersection, roadId, road->toIntersection});
        city.placeVehicleOnRoad(id, roadId);
    }
}

void section(const string& title) {
    printf("\n==============================================================================\n");
    printf(" %s\n", title.c_str());
    printf("==============================================================================\n");
}

string costText(double cost) {
    return cost == numeric_limits<double>::infinity() ? "BLOCKED" : to_string((int)cost);
}

void showOptimization(TrafficNetwork& city, RouteOptimization& optimizer, const vector<string>& route) {
    vector<string> optimized = optimizer.optimizeRoute(route);
    printf("  congestionPenalty = %.1f\n", optimizer.getCongestionPenalty());
    printf("    input : %-32s cost %s\n", routeToString(route).c_str(), costText(optimizer.routeCost(route)).c_str());
    printf("    output: %-32s cost %s   (valid: %s)\n", routeToString(optimized).c_str(),
           costText(optimizer.routeCost(optimized)).c_str(), city.isValidRoute(optimized) ? "yes" : "NO");
    printf("    roads excluded by the anti-detour guard: %s\n", routeToString(optimizer.getLastExcludedRoads()).c_str());
}

// ---------------------------------------------------------------------------------------------
// Scenario 1: dynamic signal timing at a busy junction and at a quiet one.
// ---------------------------------------------------------------------------------------------
void scenarioRushHourSignals() {
    section("SCENARIO 1: Rush hour, dynamic signal timing");
    TrafficNetwork city;
    buildCity(city);
    addTraffic(city, "R_BC", 19);  // jammed
    addTraffic(city, "R_DC", 12);  // heavy
    addTraffic(city, "R_EC", 3);   // light
                                   // R_GC stays empty
    TrafficLightManager lights;
    Intersection& central = *city.findIntersection("C");
    lights.printTimings(central, lights.computeTimings(central));
    printf("  -> The jammed road gets the longest green, the empty road gets only %d s,\n"
           "     and every red stays at or under %d s, where fixed 60 s greens would force 180 s reds.\n\n",
           TrafficLightManager::MIN_GREEN, TrafficLightManager::MAX_RED);

    addTraffic(city, "R_AB", 1);  // quiet junction: one car arriving, the other approach empty
    Intersection& market = *city.findIntersection("B");
    lights.printTimings(market, lights.computeTimings(market));
    printf("  -> With little traffic, the cycle shrinks to a few seconds, so drivers barely wait.\n");
}

// ---------------------------------------------------------------------------------------------
// Scenario 2: route optimization for a vehicle, and the effect of the congestionPenalty setting.
// ---------------------------------------------------------------------------------------------
void scenarioRouteOptimization() {
    section("SCENARIO 2: Route optimization (Dijkstra on congestion-weighted roads)");
    TrafficNetwork city;
    buildCity(city);
    addTraffic(city, "R_AB", 2);
    addTraffic(city, "R_BC", 18);  // the usual route is jammed at the Market -> Central Square road
    addTraffic(city, "R_CF", 4);
    addTraffic(city, "R_AD", 1);
    addTraffic(city, "R_DE", 12);
    addTraffic(city, "R_EF", 2);
    addTraffic(city, "R_DC", 10);

    Vehicle& car = city.addVehicle("CAR_1", {"A", "R_AB", "R_BC", "R_CF", "F"});
    printf("CAR_1 plans %s. R_BC holds 18/20 vehicles.\n\n", routeToString(car.route).c_str());

    printf("Low penalty: congestion is only a small cost, so the shorter route is still best.\n");
    RouteOptimization lowPenalty(city, 0.5);
    showOptimization(city, lowPenalty, car.route);

    printf("\nHigh penalty: congestion costs much more, so the car avoids the jam.\n");
    RouteOptimization highPenalty(city, 3.0);
    showOptimization(city, highPenalty, car.route);

    printf("\nWith the guard switched off (detourFactor = 100), the empty outskirts road wins,\n"
           "even though it is a 1900 m trip where the direct route is 1200 m:\n");
    RouteOptimization unguarded(city, 3.0, 100.0);
    showOptimization(city, unguarded, car.route);

    printf("\nCAR_1 calls requestOptimizedRoute() with the high-penalty optimizer:\n");
    car.requestOptimizedRoute(highPenalty);
    printf("  CAR_1 route is now %s\n", routeToString(car.route).c_str());
}

// ---------------------------------------------------------------------------------------------
// Scenario 3: a collision is reported, alerts are sent once, cars reroute, and the lights adapt.
// ---------------------------------------------------------------------------------------------
void scenarioAccidentAlerts() {
    section("SCENARIO 3: Accident alerts (idempotent, unique accident IDs)");
    TrafficNetwork city;
    buildCity(city);
    addTraffic(city, "R_AB", 3);
    addTraffic(city, "R_BC", 6);
    addTraffic(city, "R_CF", 3);
    addTraffic(city, "R_AD", 2);
    addTraffic(city, "R_DC", 2);
    addTraffic(city, "R_DE", 8);
    addTraffic(city, "R_CE", 4);

    // V1, V2 are already driving on R_DE. V3, V4 plan to use it. V5 does not. V6 plans to use it
    // but has not subscribed to alerts.
    vector<string> viaDE = {"A", "R_AD", "R_DE", "R_EF", "F"};
    city.addVehicle("V1", {"D", "R_DE", "R_EF", "F"});
    city.addVehicle("V2", {"D", "R_DE", "R_EF", "F"});
    city.placeVehicleOnRoad("V1", "R_DE");
    city.placeVehicleOnRoad("V2", "R_DE");
    city.addVehicle("V3", viaDE);
    city.addVehicle("V4", viaDE);
    city.addVehicle("V5", {"A", "R_AB", "R_BC", "R_CF", "F"});
    city.addVehicle("V6", viaDE);

    CollisionSensor sensor(42, 0.0);  // no random crashes yet, so the demo is deterministic
    AccidentAlert alerts(sensor);
    for (string id : {"V1", "V2", "V3", "V4", "V5"}) alerts.subscribe(city.findVehicle(id));

    TrafficLightManager lights;
    Intersection& hospital = *city.findIntersection("E");
    printf("Signals at E before the accident:\n");
    lights.printTimings(hospital, lights.computeTimings(hospital));

    printf("\nA collision happens on R_DE. The sensor keeps reporting it on the next 3 ticks:\n");
    sensor.injectCollision("R_DE");
    vector<AlertNotification> toReroute;
    for (int tick = 1; tick <= 3; ++tick) {
        vector<AlertNotification> sent = alerts.processTick(city);
        printf("  tick %d: %zu new notification(s)\n", tick, sent.size());
        toReroute.insert(toReroute.end(), sent.begin(), sent.end());
    }
    printf("  -> One accident record (%zu in total) and each vehicle is alerted once. V5 is unaffected,\n"
           "     and V6 is not subscribed.\n", alerts.getAccidents().size());

    printf("\nAlerted vehicles that are not yet on R_DE reroute around the closed road:\n");
    RouteOptimization router(city, 2.0);
    for (const AlertNotification& n : toReroute) {
        if (n.vehicleIsOnRoad) continue;
        Vehicle& v = *city.findVehicle(n.vehicleId);
        string before = routeToString(v.route);
        v.requestOptimizedRoute(router);
        printf("  %s: %s  ->  %s\n", v.id.c_str(), before.c_str(), routeToString(v.route).c_str());
    }

    printf("\nSignals at E after the accident (R_DE is closed, so its green drops to the minimum):\n");
    lights.printTimings(hospital, lights.computeTimings(hospital));

    printf("\nThe road is cleared, then a second crash happens on the same road later:\n");
    alerts.clearAccident("ACC-0001", city);
    alerts.clearAccident("ACC-0001", city);  // repeated clear is a harmless no-op
    sensor.injectCollision("R_DE");
    alerts.processTick(city);

    printf("\nThe sensor network now runs with random collisions (seed 42, 3%% per road per tick):\n");
    sensor.setRandomCollisionRate(0.03);
    for (int i = 0; i < 3; ++i) alerts.processTick(city);

    printf("\nAccident log:\n");
    alerts.printAccidentLog();
}

int main() {
    scenarioRushHourSignals();
    scenarioRouteOptimization();
    scenarioAccidentAlerts();
    return 0;
}
