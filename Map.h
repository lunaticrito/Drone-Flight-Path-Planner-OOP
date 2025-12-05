// Map.h - 3D Map with obstacles
#ifndef MAP_H
#define MAP_H

#include "Common.h"
#include <vector>
#include <string>
using namespace std;

class Map3D {
private:
    int width, depth, height;
    vector<Obstacle> obstacles;
    string mapName;
    
public:
    Map3D(int w = 50, int d = 30, int h = 20, string name = "Default City")
        : width(w), depth(d), height(h), mapName(name) {}
    
    void addObstacle(const Obstacle& obs) {
        obstacles.push_back(obs);
    }
    
    void loadPredefinedMap() {
        obstacles.clear();
        // Buildings
        obstacles.push_back(Obstacle(Vector3D(5, 5, 0), 4, 4, 12, "Tower A"));
        obstacles.push_back(Obstacle(Vector3D(15, 8, 0), 6, 5, 8, "Office Block"));
        obstacles.push_back(Obstacle(Vector3D(25, 3, 0), 3, 3, 15, "Radio Tower"));
        obstacles.push_back(Obstacle(Vector3D(35, 10, 0), 5, 4, 6, "Warehouse"));
        obstacles.push_back(Obstacle(Vector3D(10, 18, 0), 4, 6, 10, "Apartment"));
        obstacles.push_back(Obstacle(Vector3D(28, 18, 0), 7, 5, 7, "Mall"));
        obstacles.push_back(Obstacle(Vector3D(42, 5, 0), 4, 4, 9, "Hospital"));
        obstacles.push_back(Obstacle(Vector3D(20, 12, 0), 3, 3, 5, "Small Building"));
        // Trees (lower obstacles)
        obstacles.push_back(Obstacle(Vector3D(12, 3, 0), 1, 1, 4, "Tree"));
        obstacles.push_back(Obstacle(Vector3D(38, 20, 0), 1, 1, 3, "Tree"));
        obstacles.push_back(Obstacle(Vector3D(45, 15, 0), 1, 1, 4, "Tree"));
    }
    
    bool isBlocked(const Vector3D& point, double margin = 0.5) const {
        if (point.getX() < 0 || point.getX() >= width ||
            point.getY() < 0 || point.getY() >= depth ||
            point.getZ() < 0 || point.getZ() >= height) {
            return true;
        }
        for (const auto& obs : obstacles) {
            if (obs.containsPoint(point, margin)) return true;
        }
        return false;
    }
    
    // Check if line segment is clear
    bool isPathClear(const Vector3D& from, const Vector3D& to, double step = 0.5) const {
        Vector3D dir = to - from;
        double dist = dir.magnitude();
        if (dist < 0.01) return true;
        
        Vector3D unitDir = dir.normalize();
        for (double t = 0; t <= dist; t += step) {
            Vector3D point = from + unitDir * t;
            if (isBlocked(point)) return false;
        }
        return true;
    }
    
    const vector<Obstacle>& getObstacles() const { return obstacles; }
    int getWidth() const { return width; }
    int getDepth() const { return depth; }
    int getHeight() const { return height; }
    string getName() const { return mapName; }
    
    // Get safe altitude above all obstacles
    double getSafeAltitude() const {
        double maxH = 0;
        for (const auto& obs : obstacles) {
            double top = obs.getPosition().getZ() + obs.getHeight();
            if (top > maxH) maxH = top;
        }
        return maxH + 2;
    }
};

#endif