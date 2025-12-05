// main.cpp - Drone Flight Path Planner Main Application
//g++ -std=c++14 -o DronePlanner.exe main.cpp -static
#include <iostream>
#include <iomanip>
#include <string>
#include <memory>
#include <limits>
#include <conio.h>

#include "Common.h"
#include "Battery.h"
#include "Drone.h"
#include "Map.h"
#include "PathFinder.h"
#include "Logger.h"
#include "Simulator.h"
using namespace std;

// Template function for safe input
template <typename T>
T getInput(const string &prompt, T minVal, T maxVal)
{
    T value;
    while (true)
    {
        ::cout << prompt;
        if (cin >> value && value >= minVal && value <= maxVal)
        {
            return value;
        }
        cin.clear();
        cin.ignore(numeric_limits<streamsize>::max(), '\n');
        cout << "Invalid input. Please enter a value between "
                  << minVal << " and " << maxVal << "\n";
    }
}

// Inline function for displaying separator
inline void printSeparator(char c = '=', int len = 50)
{
    cout << string(len, c) << endl;
}

class FlightPlanner
{
private:
    Map3D map;
    vector<unique_ptr<Drone>> drones;
    MissionLogger logger;
    ConsoleSimulator simulator;
    PathFinder3D *pathFinder;
    int activeDroneIdx;

public:
    FlightPlanner() : activeDroneIdx(0), pathFinder(nullptr)
    {
        // Initialize map
        map = Map3D(50, 25, 20, "Metro City");
        map.loadPredefinedMap();

        // Initialize pathfinder
        pathFinder = new PathFinder3D(&map, 1.0);

        // Create different drone types (demonstrating polymorphism)
        drones.push_back(make_unique<Drone>("DRN-001", "Standard"));
        drones.push_back(make_unique<SurveyDrone>("SRV-001"));
        drones.push_back(make_unique<DeliveryDrone>("DLV-001"));
        drones.push_back(make_unique<RacingDrone>("RCR-001"));
    }

    ~FlightPlanner()
    {
        delete pathFinder;
    }

    void showMainMenu()
    {
        cout << "\n";
        printSeparator('=', 50);
        cout << "       DRONE FLIGHT PATH PLANNER v1.0\n";
        printSeparator('=', 50);
        cout << "\n";
        cout << "  1. View Map & Obstacles\n";
        cout << "  2. Select Drone\n";
        cout << "  3. Plan & Execute Flight Mission\n";
        cout << "  4. Quick Flight (Random Destination)\n";
        cout << "  5. View Drone Status\n";
        cout << "  6. Recharge Drone Battery\n";
        cout << "  7. View Mission Logs\n";
        cout << "  8. Mission Summary & Statistics\n";
        cout << "  9. Compare Drone Efficiency\n";
        cout << "  10. Clear Mission Logs\n";
        cout << "  0. Exit\n";
        cout << "\n";
        printSeparator('-', 50);
        cout << "Active Drone: " << drones[activeDroneIdx]->getInfo() << "\n";
        printSeparator('-', 50);
    }

    void viewMap()
    {
        simulator.drawMap(map, drones[activeDroneIdx]->getPosition(),
                          Vector3D(), Vector3D(), vector<Vector3D>(), false);

        cout << "\n\nObstacles in map:\n";
        printSeparator('-', 60);
        cout << left << setw(15) << "Type"
                  << setw(15) << "Position"
                  << setw(20) << "Dimensions (LxWxH)\n";
        printSeparator('-', 60);

        for (const auto &obs : map.getObstacles())
        {
            cout << left << setw(15) << obs.getType()
                      << setw(15) << (string)obs.getPosition()
                      << obs.getLength() << "x" << obs.getWidth() << "x" << obs.getHeight() << "\n";
        }

        cout << "\nPress any key to continue...";
        _getch();
    }

    void selectDrone()
    {
        cout << "\n--- Available Drones ---\n";
        for (size_t i = 0; i < drones.size(); i++)
        {
            cout << (i + 1) << ". " << drones[i]->getInfo() << "\n";
        }

        int choice = getInput<int>("Select drone (1-" + to_string(drones.size()) + "): ",
                                   1, (int)drones.size());
        activeDroneIdx = choice - 1;
        cout << "Selected: " << drones[activeDroneIdx]->getId() << "\n";
    }

    void planAndExecuteFlight()
    {
        Drone *drone = drones[activeDroneIdx].get();

        cout << "\n--- Flight Mission Planning ---\n";
        cout << "Map bounds: X[0-" << map.getWidth() - 1 << "], Y[0-" << map.getDepth() - 1
                  << "], Z[0-" << map.getHeight() - 1 << "]\n\n";

        // Get start position
        cout << "Start Position:\n";
        double sx = getInput<double>("  X: ", 0, map.getWidth() - 1);
        double sy = getInput<double>("  Y: ", 0, map.getDepth() - 1);
        double sz = getInput<double>("  Z: ", 0, map.getHeight() - 1);
        Vector3D start(sx, sy, sz);

        // Check if start is blocked
        if (map.isBlocked(start))
        {
            cout << "Start position is blocked by obstacle. Please try again.\n";
            return;
        }

        // Get destination
        cout << "\nDestination:\n";
        double dx = getInput<double>("  X: ", 0, map.getWidth() - 1);
        double dy = getInput<double>("  Y: ", 0, map.getDepth() - 1);
        double dz = getInput<double>("  Z: ", 0, map.getHeight() - 1);
        Vector3D dest(dx, dy, dz);

        if (map.isBlocked(dest))
        {
            cout << "Destination is blocked by obstacle. Please try again.\n";
            return;
        }

        executeFlight(drone, start, dest);
    }

    void quickFlight()
    {
        Drone *drone = drones[activeDroneIdx].get();

        // Random safe positions
        Vector3D start(2, 2, 1);
        Vector3D dest(45, 20, 2);

        cout << "\nQuick flight from " << start << " to " << dest << "\n";
        executeFlight(drone, start, dest);
    }

    void executeFlight(Drone *drone, const Vector3D &start, const Vector3D &dest)
    {
        cout << "\nCalculating optimal path...\n";

        // Find path
        auto path = pathFinder->findPath(start, dest);
        double pathDist = pathFinder->calculatePathDistance(path);

        cout << "Path found with " << path.size() << " waypoints\n";
        cout << "Estimated distance: " << fixed << setprecision(2) << pathDist << " units\n";

        // Check battery
        if (!drone->getBattery().canTravel(pathDist))
        {
            cout << "\nWARNING: Insufficient battery for this mission!\n";
            cout << "Current: " << drone->getBattery().getPercentage() << "%\n";
            cout << "Required: ~" << (pathDist * drone->getBattery().getConsumptionRate()) << "%\n";
            cout << "Continue anyway? (y/n): ";
            char c;
            cin >> c;
            if (c != 'y' && c != 'Y')
                return;
        }

        double startBattery = drone->getBattery().getPercentage();
        drone->resetDistance();

        // Simulate flight
        double estTime = pathDist / drone->getSpeed();
        int delayMs = max(50, (int)(5000 / path.size())); // Adjust animation speed

        simulator.simulateFlight(*drone, map, path, start, dest, delayMs);

        double endBattery = drone->getBattery().getPercentage();
        double batteryUsed = startBattery - endBattery;

        // Log mission
        MissionResult result;
        result.droneId = drone->getId();
        result.startPos = (string)start;
        result.endPos = (string)dest;
        result.distance = drone->getTotalDistance();
        result.batteryUsed = batteryUsed;
        result.duration = estTime;
        result.status = (endBattery > 5) ? "Completed" : "Emergency Landing";

        logger.logMission(result);
        drone->incrementMission();

        // Display results
        printSeparator('=', 50);
        cout << "         MISSION RESULTS\n";
        printSeparator('=', 50);
        cout << "Drone: " << drone->getId() << " (" << drone->getModel() << ")\n";
        cout << "Route: " << start << " -> " << dest << "\n";
        cout << "Distance: " << fixed << setprecision(2) << result.distance << " units\n";
        cout << "Battery Used: " << result.batteryUsed << "%\n";
        cout << "Battery Remaining: " << endBattery << "%\n";
        cout << "Flight Time: " << result.duration << " seconds\n";
        cout << "Status: " << result.status << "\n";
        cout << "Waypoints: " << path.size() << "\n";
        printSeparator('=', 50);

        cout << "\nResults saved to mission log.\n";
        cout << "Press any key to continue...";
        _getch();
    }

    void viewDroneStatus()
    {
        cout << "\n--- All Drones Status ---\n";
        printSeparator('-', 70);

        for (size_t i = 0; i < drones.size(); i++)
        {
            auto &d = drones[i];
            cout << (i == activeDroneIdx ? ">> " : "   ");
            cout << d->getInfo() << "\n";
            cout << "      Position: " << d->getPosition()
                      << " | Speed: " << d->getSpeed() << " u/s"
                      << " | Missions: " << d->getMissionCount() << "\n";
            cout << "      Battery Type: " << d->getBattery().getBatteryType()
                      << " | Status: " << d->getBattery().getStatus() << "\n\n";
        }

        cout << "Press any key to continue...";
        _getch();
    }

    void rechargeBattery()
    {
        cout << "\nRecharging " << drones[activeDroneIdx]->getId() << "...\n";

        for (int i = 0; i <= 100; i += 5)
        {
            cout << "\r";
            simulator.drawProgressBar(i / 100.0, 40);
            Sleep(50);
        }

        drones[activeDroneIdx]->getBattery().recharge();
        cout << "\nBattery fully charged to 100%!\n";
        cout << "Press any key to continue...";
        _getch();
    }

    void viewMissionLogs()
    {
        auto missions = logger.readAllMissions();

        cout << "\n--- Mission Logs ---\n";
        if (missions.empty())
        {
            cout << "No missions logged yet.\n";
        }
        else
        {
            printSeparator('-', 90);
            cout << left << setw(12) << "Drone"
                      << setw(15) << "Start"
                      << setw(15) << "End"
                      << setw(10) << "Distance"
                      << setw(10) << "Battery"
                      << setw(10) << "Time"
                      << "Status\n";
            printSeparator('-', 90);

            for (const auto &m : missions)
            {
                cout << left << setw(12) << m.droneId
                          << setw(15) << m.startPos
                          << setw(15) << m.endPos
                          << setw(10) << fixed << setprecision(1) << m.distance
                          << setw(10) << m.batteryUsed
                          << setw(10) << m.duration
                          << m.status << "\n";
            }
        }

        cout << "\nPress any key to continue...";
        _getch();
    }

    void showSummary()
    {
        logger.printSummary();
        cout << "Press any key to continue...";
        _getch();
    }

    void compareEfficiency()
    {
        logger.compareEfficiency();
        cout << "Press any key to continue...";
        _getch();
    }

    void clearLogs()
    {
        cout << "Are you sure you want to clear all logs? (y/n): ";
        char c;
        cin >> c;
        if (c == 'y' || c == 'Y')
        {
            logger.clearLogs();
            cout << "Logs cleared.\n";
        }
    }

    void run()
    {
        int choice;
        do
        {
            system("cls");
            showMainMenu();
            choice = getInput<int>("Enter choice: ", 0, 10);

            switch (choice)
            {
            case 1:
                viewMap();
                break;
            case 2:
                selectDrone();
                break;
            case 3:
                planAndExecuteFlight();
                break;
            case 4:
                quickFlight();
                break;
            case 5:
                viewDroneStatus();
                break;
            case 6:
                rechargeBattery();
                break;
            case 7:
                viewMissionLogs();
                break;
            case 8:
                showSummary();
                break;
            case 9:
                compareEfficiency();
                break;
            case 10:
                clearLogs();
                break;
            case 0:
                cout << "Exiting...\n";
                break;
            }
        } while (choice != 0);
    }
};

int main()
{
    SetConsoleTitle(TEXT("Drone Flight Path Planner"));

    // Set console size
    system("mode con: cols=100 lines=40");

    cout << "Initializing Drone Flight Path Planner...\n";

    FlightPlanner planner;
    planner.run();

    return 0;
}