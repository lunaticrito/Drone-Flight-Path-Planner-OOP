// Simulator.h - Windows console simulation with graphics
#ifndef SIMULATOR_H
#define SIMULATOR_H

#ifndef _GLIBCXX_HAS_GTHREADS
#define _GLIBCXX_HAS_GTHREADS
#endif

#include <windows.h>
#include <iostream>
#include <vector>
#include <string>
#include <iomanip>
#include "Common.h"
#include "Map.h"
#include "Drone.h"
using namespace std;

class ConsoleSimulator {
private:
    HANDLE hConsole;
    int consoleWidth, consoleHeight;
    
    // Color constants using enum
    enum Colors {
        GRAY = 8,
        BLUE = 9,
        GREEN = 10,
        CYAN = 11,
        RED = 12,
        MAGENTA = 13,
        YELLOW = 14,
        WHITE = 15
    };
    
    void setCursor(int x, int y) {
        COORD pos = {(SHORT)x, (SHORT)y};
        SetConsoleCursorPosition(hConsole, pos);
    }
    
    void setColor(int color) {
        SetConsoleTextAttribute(hConsole, color);
    }
    
    void clearScreen() {
        COORD topLeft = {0, 0};
        CONSOLE_SCREEN_BUFFER_INFO screen;
        DWORD written;
        
        GetConsoleScreenBufferInfo(hConsole, &screen);
        FillConsoleOutputCharacterA(hConsole, ' ', screen.dwSize.X * screen.dwSize.Y, topLeft, &written);
        FillConsoleOutputAttribute(hConsole, FOREGROUND_GREEN | FOREGROUND_RED | FOREGROUND_BLUE,
            screen.dwSize.X * screen.dwSize.Y, topLeft, &written);
        SetConsoleCursorPosition(hConsole, topLeft);
    }

public:
    ConsoleSimulator() {
        hConsole = GetStdHandle(STD_OUTPUT_HANDLE);
        consoleWidth = 100;
        consoleHeight = 40;
        
        // Hide cursor for smoother animation
        CONSOLE_CURSOR_INFO cursorInfo;
        GetConsoleCursorInfo(hConsole, &cursorInfo);
        cursorInfo.bVisible = FALSE;
        SetConsoleCursorInfo(hConsole, &cursorInfo);
    }
    
    ~ConsoleSimulator() {
        // Restore cursor
        CONSOLE_CURSOR_INFO cursorInfo;
        GetConsoleCursorInfo(hConsole, &cursorInfo);
        cursorInfo.bVisible = TRUE;
        SetConsoleCursorInfo(hConsole, &cursorInfo);
    }
    
    void drawMap(const Map3D& map, const Vector3D& dronePos, 
                 const Vector3D& start, const Vector3D& dest,
                 const vector<Vector3D>& path, bool forAnimation = false) {
        
        if (!forAnimation) {
            clearScreen();
        } else {
            setCursor(0, 0);
        }
        
        // Map dimensions for display
        int mapW = min(map.getWidth(), 50);
        int mapD = min(map.getDepth(), 25);
        
        // Title
        setColor(CYAN);
        cout << "=== DRONE FLIGHT SIMULATOR - " << map.getName() << " ===                    \n";
        setColor(WHITE);
        cout << "Map: " << map.getWidth() << "x" << map.getDepth() << "x" << map.getHeight() << " | ";
        cout << "Drone: " << dronePos << "                    \n\n";
        
        // Create 2D view (top-down)
        vector<vector<char>> display(mapD, vector<char>(mapW, '.'));
        vector<vector<int>> colors(mapD, vector<int>(mapW, GRAY));
        
        // Mark obstacles
        for (const auto& obs : map.getObstacles()) {
            int ox = (int)obs.getPosition().getX();
            int oy = (int)obs.getPosition().getY();
            int ol = (int)obs.getLength();
            int ow = (int)obs.getWidth();
            
            for (int x = ox; x < ox + ol && x < mapW; x++) {
                for (int y = oy; y < oy + ow && y < mapD; y++) {
                    if (x >= 0 && y >= 0) {
                        if (obs.getHeight() > 10) {
                            display[y][x] = '#';
                            colors[y][x] = RED;
                        } else if (obs.getHeight() > 5) {
                            display[y][x] = 'B';
                            colors[y][x] = YELLOW;
                        } else {
                            display[y][x] = 'o';
                            colors[y][x] = GREEN;
                        }
                    }
                }
            }
        }
        
        // Mark path
        for (const auto& p : path) {
            int px = (int)p.getX();
            int py = (int)p.getY();
            if (px >= 0 && px < mapW && py >= 0 && py < mapD) {
                if (display[py][px] == '.') {
                    display[py][px] = '*';
                    colors[py][px] = CYAN;
                }
            }
        }
        
        // Mark start and destination
        int sx = (int)start.getX(), sy = (int)start.getY();
        int dx = (int)dest.getX(), dy = (int)dest.getY();
        if (sx >= 0 && sx < mapW && sy >= 0 && sy < mapD) {
            display[sy][sx] = 'S';
            colors[sy][sx] = GREEN;
        }
        if (dx >= 0 && dx < mapW && dy >= 0 && dy < mapD) {
            display[dy][dx] = 'D';
            colors[dy][dx] = MAGENTA;
        }
        
        // Mark drone position (overwrites path for visibility)
        int droneX = (int)dronePos.getX();
        int droneY = (int)dronePos.getY();
        if (droneX >= 0 && droneX < mapW && droneY >= 0 && droneY < mapD) {
            display[droneY][droneX] = '@';
            colors[droneY][droneX] = BLUE;
        }
        
        // Draw the map
        cout << "   ";
        setColor(WHITE);
        for (int x = 0; x < mapW; x += 5) {
            cout << x;
            int digits = (x == 0) ? 1 : (int)log10(x) + 1;
            cout << string(5 - digits, ' ');
        }
        cout << "\n";
        
        for (int y = 0; y < mapD; y++) {
            setColor(WHITE);
            cout << setw(2) << y << " ";
            for (int x = 0; x < mapW; x++) {
                setColor(colors[y][x]);
                cout << display[y][x];
            }
            cout << "  \n";  // Extra spaces to clear any residual characters
        }
        
        // Legend
        setColor(WHITE);
        cout << "\nLegend: ";
        setColor(BLUE); cout << "@";
        setColor(WHITE); cout << "=Drone ";
        setColor(GREEN); cout << "S";
        setColor(WHITE); cout << "=Start ";
        setColor(MAGENTA); cout << "D";
        setColor(WHITE); cout << "=Dest ";
        setColor(RED); cout << "#";
        setColor(WHITE); cout << "=Tall ";
        setColor(YELLOW); cout << "B";
        setColor(WHITE); cout << "=Building ";
        setColor(GREEN); cout << "o";
        setColor(WHITE); cout << "=Low ";
        setColor(CYAN); cout << "*";
        setColor(WHITE); cout << "=Path      \n";
    }
    
    void drawFlightStatus(const Drone& drone, int currentWaypoint, int totalWaypoints) {
        setColor(CYAN);
        cout << "\n--- Flight Status ---                              \n";
        setColor(WHITE);
        cout << "Position: " << drone.getPosition() << "                    \n";
        cout << "Waypoint: " << currentWaypoint << "/" << totalWaypoints << "                    \n";
        cout << "Battery: ";
        
        double batt = drone.getBattery().getPercentage();
        if (batt > 50) setColor(GREEN);
        else if (batt > 20) setColor(YELLOW);
        else setColor(RED);
        cout << fixed << setprecision(1) << batt << "%                    \n";
        
        setColor(WHITE);
        cout << "Distance Traveled: " << fixed << setprecision(2) 
                  << drone.getTotalDistance() << " units                    \n";
        cout << "Altitude: " << fixed << setprecision(1) 
                  << drone.getPosition().getZ() << " units                    \n";
    }
    
    void simulateFlight(Drone& drone, const Map3D& map, 
                        const vector<Vector3D>& path,
                        const Vector3D& start, const Vector3D& dest,
                        int delayMs = 200) {
        
        if (path.empty()) {
            cout << "No path to simulate!\n";
            return;
        }
        
        drone.setPosition(start);
        drone.takeOff();
        
        // Initial draw
        clearScreen();
        
        for (size_t i = 0; i < path.size(); i++) {
            // Move drone to next waypoint
            drone.move(path[i]);
            
            // Redraw map with updated drone position
            drawMap(map, drone.getPosition(), start, dest, path, true);
            
            // Draw status below map
            drawFlightStatus(drone, (int)(i + 1), (int)path.size());
            
            // Flush output to ensure immediate display
            cout.flush();
            
            // Wait before next frame
            Sleep(delayMs);
            
            // Check for critical battery
            if (drone.getBattery().getPercentage() < 5) {
                setColor(RED);
                cout << "\n!!! CRITICAL BATTERY - EMERGENCY LANDING !!!                    \n";
                setColor(WHITE);
                break;
            }
        }
        
        drone.land();
        setColor(GREEN);
        cout << "\n=== FLIGHT COMPLETE ===                              \n";
        setColor(WHITE);
    }
    
    void drawProgressBar(double progress, int width = 40) {
        int filled = (int)(progress * width);
        cout << "[";
        for (int i = 0; i < width; i++) {
            if (i < filled) cout << "=";
            else if (i == filled) cout << ">";
            else cout << " ";
        }
        cout << "] " << fixed << setprecision(1) << (progress * 100) << "%";
    }
};

#endif