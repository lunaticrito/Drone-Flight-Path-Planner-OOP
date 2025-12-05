// PathFinder.h - Path planning with A* algorithm (uses STL and Dynamic Memory Allocation)
#ifndef PATHFINDER_H
#define PATHFINDER_H

#include "Common.h"
#include "Map.h"
#include <vector>
#include <queue>
#include <unordered_map>
#include <algorithm>
#include <functional>
using namespace std;
// pathfinding
struct PathNode {
    Vector3D pos;
    double gCost, hCost;
    int parentIdx;
    
    PathNode(Vector3D p = Vector3D(), double g = 0, double h = 0, int parent = -1)
        : pos(p), gCost(g), hCost(h), parentIdx(parent) {}
    
    double fCost() const { return gCost + hCost; }
    
    bool operator>(const PathNode& other) const {
        return fCost() > other.fCost();
    }
};

// abstract pathfinder interface
class IPathFinder {
public:
    virtual vector<Vector3D> findPath(const Vector3D& start, const Vector3D& end) = 0;
    virtual ~IPathFinder() {}
};

// dynamic memory allocation
struct PathCacheEntry {
    Vector3D* waypoints;    // Dynamic array of waypoints
    int waypointCount;
    double totalDistance;
    
    // constructor
    PathCacheEntry() : waypoints(nullptr), waypointCount(0), totalDistance(0.0) {}
    
    // deep copy
    PathCacheEntry(const PathCacheEntry& other) {
        waypointCount = other.waypointCount;
        totalDistance = other.totalDistance;
        if (other.waypoints && waypointCount > 0) {
            waypoints = new Vector3D[waypointCount];
            for (int i = 0; i < waypointCount; i++) {
                waypoints[i] = other.waypoints[i];
            }
        } else {
            waypoints = nullptr;
        }
    }
    
    // assignment operator
    PathCacheEntry& operator=(const PathCacheEntry& other) {
        if (this != &other) {
            // delete old data
            delete[] waypoints;
            
            // copy new data
            waypointCount = other.waypointCount;
            totalDistance = other.totalDistance;
            if (other.waypoints && waypointCount > 0) {
                waypoints = new Vector3D[waypointCount];
                for (int i = 0; i < waypointCount; i++) {
                    waypoints[i] = other.waypoints[i];
                }
            } else {
                waypoints = nullptr;
            }
        }
        return *this;
    }
    
    // free memory
    ~PathCacheEntry() {
        delete[] waypoints;
        waypoints = nullptr;
    }
    
    // Store path in cache
    void storePath(const vector<Vector3D>& path, double dist) {
        delete[] waypoints;  // Clear old data
        
        waypointCount = (int)path.size();
        totalDistance = dist;
        
        if (waypointCount > 0) {
            waypoints = new Vector3D[waypointCount];  // Dynamic allocation
            for (int i = 0; i < waypointCount; i++) {
                waypoints[i] = path[i];
            }
        } else {
            waypoints = nullptr;
        }
    }
    
    // Retrieve path from cache
    vector<Vector3D> retrievePath() const {
        vector<Vector3D> path;
        for (int i = 0; i < waypointCount; i++) {
            path.push_back(waypoints[i]);
        }
        return path;
    }
};

// 3D A* Pathfinder implementation with path caching
class PathFinder3D : public IPathFinder {
private:
    const Map3D* map;
    double gridStep;
    PathCacheEntry* pathCache;  // Dynamic memory for path cache
    int cacheSize;
    int cacheCapacity;
    
    inline string posKey(const Vector3D& v) const {
        return to_string((int)v.getX()) + "," + 
               to_string((int)v.getY()) + "," + 
               to_string((int)v.getZ());
    }
    
    string getCacheKey(const Vector3D& start, const Vector3D& end) const {
        return posKey(start) + "->" + posKey(end);
    }
    
    vector<Vector3D> getNeighbors(const Vector3D& pos) const {
        vector<Vector3D> neighbors;
        // 26-directional movement in 3D
        for (int dx = -1; dx <= 1; dx++) {
            for (int dy = -1; dy <= 1; dy++) {
                for (int dz = -1; dz <= 1; dz++) {
                    if (dx == 0 && dy == 0 && dz == 0) continue;
                    Vector3D next(pos.getX() + dx * gridStep,
                                  pos.getY() + dy * gridStep,
                                  pos.getZ() + dz * gridStep);
                    if (!map->isBlocked(next)) {
                        neighbors.push_back(next);
                    }
                }
            }
        }
        return neighbors;
    }
    
    vector<Vector3D> smoothPath(const vector<Vector3D>& path) const {
        if (path.size() <= 2) return path;
        vector<Vector3D> smoothed;
        smoothed.push_back(path[0]);
        size_t i = 0;
        while (i < path.size() - 1) {
            size_t j = path.size() - 1;
            while (j > i + 1) {
                if (map->isPathClear(path[i], path[j], 0.5)) {
                    break;
                }
                j--;
            }
            smoothed.push_back(path[j]);
            i = j;
        }
        return smoothed;
    }
    
    void addToCache(const Vector3D& start, const Vector3D& end, 
                    const vector<Vector3D>& path, double distance) {
        if (cacheSize >= cacheCapacity) {
            // expand cache if full (dynamic reallocation)
            int newCapacity = cacheCapacity * 2;
            PathCacheEntry* newCache = new PathCacheEntry[newCapacity];
            
            // copy old entries
            for (int i = 0; i < cacheSize; i++) {
                newCache[i] = pathCache[i];
            }
            
            // delete old cache and use new one
            delete[] pathCache;
            pathCache = newCache;
            cacheCapacity = newCapacity;
        }
        
        // store new entry
        pathCache[cacheSize].storePath(path, distance);
        cacheSize++;
    }

public:
    PathFinder3D(const Map3D* m, double step = 1.0) 
        : map(m), gridStep(step), cacheSize(0), cacheCapacity(10) {
        // Dynamic memory allocation for cache
        pathCache = new PathCacheEntry[cacheCapacity];
    }
    
    // Destructor to free dynamically allocated memory
    ~PathFinder3D() {
        delete[] pathCache;
        pathCache = nullptr;
    }
    
    // Copy constructor (deep copy)
    PathFinder3D(const PathFinder3D& other) 
        : map(other.map), gridStep(other.gridStep), 
          cacheSize(other.cacheSize), cacheCapacity(other.cacheCapacity) {
        pathCache = new PathCacheEntry[cacheCapacity];
        for (int i = 0; i < cacheSize; i++) {
            pathCache[i] = other.pathCache[i];
        }
    }
    
    // Assignment operator
    PathFinder3D& operator=(const PathFinder3D& other) {
        if (this != &other) {
            delete[] pathCache;
            
            map = other.map;
            gridStep = other.gridStep;
            cacheSize = other.cacheSize;
            cacheCapacity = other.cacheCapacity;
            
            pathCache = new PathCacheEntry[cacheCapacity];
            for (int i = 0; i < cacheSize; i++) {
                pathCache[i] = other.pathCache[i];
            }
        }
        return *this;
    }
    
    vector<Vector3D> findPath(const Vector3D& start, const Vector3D& end) override {
        vector<Vector3D> path;
        
        // Quick check for direct path
        if (map->isPathClear(start, end)) {
            path.push_back(start);
            path.push_back(end);
            double dist = start.distanceTo(end);
            addToCache(start, end, path, dist);
            return path;
        }
        
        // A* algorithm using STL priority_queue
        priority_queue<PathNode, vector<PathNode>, greater<PathNode>> openSet;
        unordered_map<string, double> closedSet;
        vector<PathNode> allNodes;
        
        PathNode startNode(start, 0, start.distanceTo(end), -1);
        openSet.push(startNode);
        allNodes.push_back(startNode);
        
        int iterations = 0;
        const int maxIter = 10000;
        
        while (!openSet.empty() && iterations < maxIter) {
            iterations++;
            PathNode current = openSet.top();
            openSet.pop();
            
            string key = posKey(current.pos);
            if (closedSet.find(key) != closedSet.end()) continue;
            closedSet[key] = current.gCost;
            
            // Check if reached destination
            if (current.pos.distanceTo(end) < gridStep * 1.5) {
                // Reconstruct path
                int idx = (int)allNodes.size() - 1;
                for (size_t i = 0; i < allNodes.size(); i++) {
                    if (allNodes[i].pos == current.pos) { idx = (int)i; break; }
                }
                
                while (idx != -1 && idx < (int)allNodes.size()) {
                    path.push_back(allNodes[idx].pos);
                    idx = allNodes[idx].parentIdx;
                }
                reverse(path.begin(), path.end());
                path.push_back(end);
                
                vector<Vector3D> smoothedPath = smoothPath(path);
                double dist = calculatePathDistance(smoothedPath);
                addToCache(start, end, smoothedPath, dist);
                
                return smoothedPath;
            }
            
            // Explore neighbors
            int currentIdx = (int)allNodes.size() - 1;
            for (const auto& neighbor : getNeighbors(current.pos)) {
                string nKey = posKey(neighbor);
                if (closedSet.find(nKey) != closedSet.end()) continue;
                
                double newG = current.gCost + current.pos.distanceTo(neighbor);
                PathNode newNode(neighbor, newG, neighbor.distanceTo(end), currentIdx);
                openSet.push(newNode);
                allNodes.push_back(newNode);
            }
        }
        
        // fly high above obstacles
        double safeAlt = map->getSafeAltitude();
        path.push_back(start);
        path.push_back(Vector3D(start.getX(), start.getY(), safeAlt));
        path.push_back(Vector3D(end.getX(), end.getY(), safeAlt));
        path.push_back(end);
        
        double dist = calculatePathDistance(path);
        addToCache(start, end, path, dist);
        
        return path;
    }
    
    double calculatePathDistance(const vector<Vector3D>& path) const {
        double total = 0;
        for (size_t i = 1; i < path.size(); i++) {
            total += path[i-1].distanceTo(path[i]);
        }
        return total;
    }
    
    // Get cache statistics
    void printCacheStats() const {
        cout << "Path Cache Statistics:\n";
        cout << "  Entries: " << cacheSize << "/" << cacheCapacity << "\n";
        cout << "  Memory Used: " << (cacheSize * sizeof(PathCacheEntry)) << " bytes\n";
        
        int totalWaypoints = 0;
        for (int i = 0; i < cacheSize; i++) {
            totalWaypoints += pathCache[i].waypointCount;
        }
        cout << "  Total Waypoints Cached: " << totalWaypoints << "\n";
    }
};

#endif