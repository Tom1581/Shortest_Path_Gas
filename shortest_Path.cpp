
#include <iostream>
#include <vector>
#include <queue>
#include <map>
#include <limits>
#include <cmath>  // For heuristic calculations if needed

struct GasStation {
    int id;
    double pricePerLiter;
    std::vector<std::pair<int, double>> connections;  // Neighboring station and distance
    std::pair<double, double> coordinates;  // Assuming we have coordinates for heuristic calculation

    GasStation(int id, double price, std::pair<double, double> coords) : id(id), pricePerLiter(price), coordinates(coords) {}
};

struct State {
    int station;
    double remainingFuel;
    double cost;
    double heuristic;

    // Priority in A* is based on cost + heuristic
    bool operator>(const State& other) const {
        return (cost + heuristic) > (other.cost + other.heuristic);
    }
};

double heuristicEstimate(int currentStation, int goalStation, const std::vector<GasStation>& stations) {
    auto [x1, y1] = stations[currentStation].coordinates;
    auto [x2, y2] = stations[goalStation].coordinates;
    // Using Euclidean distance as a simple heuristic
    return sqrt(pow(x2 - x1, 2) + pow(y2 - y1, 2));
}

double dijkstraModified(const std::vector<GasStation>& stations, int startStation, int goalStation, double startFuel, double maxFuel, double fuelEfficiency) {
    std::priority_queue<State, std::vector<State>, std::greater<State>> pq;
    std::map<std::pair<int, double>, double> bestCost;

    double initialHeuristic = heuristicEstimate(startStation, goalStation, stations);
    pq.push({startStation, startFuel, 0, initialHeuristic});
    bestCost[{startStation, startFuel}] = 0;

    while (!pq.empty()) {
        State current = pq.top();
        pq.pop();

        if (current.station == goalStation) {  // Early exit if goal is reached
            return current.cost;
        }

        // Explore refueling at current station
        if (current.remainingFuel < maxFuel) {
            double costToRefuel = (maxFuel - current.remainingFuel) * stations[current.station].pricePerLiter;
            State refueled = {current.station, maxFuel, current.cost + costToRefuel, 0};
            refueled.heuristic = heuristicEstimate(current.station, goalStation, stations);
            if (bestCost.find({refueled.station, refueled.remainingFuel}) == bestCost.end() || refueled.cost < bestCost[{refueled.station, refueled.remainingFuel}]) {
                pq.push(refueled);
                bestCost[{refueled.station, refueled.remainingFuel}] = refueled.cost;
            }
        }

        // Try moving to each connected station
        for (const auto& connection : stations[current.station].connections) {
            double distance = connection.second;
            double requiredFuel = distance / fuelEfficiency;
            if (current.remainingFuel >= requiredFuel) {
                State next = {connection.first, current.remainingFuel - requiredFuel, current.cost, 0};
                next.heuristic = heuristicEstimate(next.station, goalStation, stations);
                if (bestCost.find({next.station, next.remainingFuel}) == bestCost.end() || next.cost < bestCost[{next.station, next.remainingFuel}]) {
                    pq.push(next);
                    bestCost[{next.station, next.remainingFuel}] = next.cost;
                }
            }
        }
    }

    // If no path was found, return a large number
    return std::numeric_limits<double>::max();
}

int main() {
    std::vector<GasStation> stations = {
        GasStation(0, 1.5, {0, 0}), GasStation(1, 1.45, {10, 10}), GasStation(2, 1.55, {20, 20})
    };
    stations[0].connections.emplace_back(1, 10);
    stations[1].connections.emplace_back(0, 10);
    stations[1].connections.emplace_back(2, 15);
    stations[2].connections.emplace_back(1, 15);

    int startStation = 0;
    int goalStation = 2;
    double startFuel = 10.0;
    double maxFuel = 20.0;
    double fuelEfficiency = 0.1;

    double minCost = dijkstraModified(stations, startStation, goalStation, startFuel, maxFuel, fuelEfficiency);
    std::cout << "Minimum cost to reach the goal station: $" << minCost << std::endl;

    return 0;
}
