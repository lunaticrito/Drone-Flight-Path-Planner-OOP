// Common definitions
#ifndef COMMON_H
#define COMMON_H

#include <iostream>
#include <string>
#include <vector>
#include <cmath>
#include <limits>

using namespace std;
// Polymorphism
class Vector3D {
private:
    double x, y, z;
public:
    Vector3D(double x = 0, double y = 0, double z = 0) : x(x), y(y), z(z) {}
    // getters 
    inline double getX() const { return x; }
    inline double getY() const { return y; }
    inline double getZ() const { return z; }
    
    // setters
    inline void setX(double val) { x = val; }
    inline void setY(double val) { y = val; }
    inline void setZ(double val) { z = val; }
    
    // operator overloading
    Vector3D operator+(const Vector3D& v) const {
        return Vector3D(x + v.x, y + v.y, z + v.z);
    }
    Vector3D operator-(const Vector3D& v) const {
        return Vector3D(x - v.x, y - v.y, z - v.z);
    }
    Vector3D operator*(double scalar) const {
        return Vector3D(x * scalar, y * scalar, z * scalar);
    }
    Vector3D operator/(double scalar) const {
        return Vector3D(x / scalar, y / scalar, z / scalar);
    }
    bool operator==(const Vector3D& v) const {
        return (abs(x - v.x) < 0.1 && abs(y - v.y) < 0.1 && abs(z - v.z) < 0.1);
    }
    
    // distance calculation
    double distanceTo(const Vector3D& v) const {
        return sqrt(pow(x - v.x, 2) + pow(y - v.y, 2) + pow(z - v.z, 2));
    }
    double magnitude() const {
        return sqrt(x*x + y*y + z*z);
    }
    Vector3D normalize() const {
        double mag = magnitude();
        if (mag > 0) return *this / mag;
        return Vector3D();
    }
    
    // type conversion to string
    operator string() const {
        return "(" + to_string((int)x) + "," + to_string((int)y) + "," + to_string((int)z) + ")";
    }
    // '<<' overload
    friend ostream& operator<<(ostream& os, const Vector3D& v) {
        os << "(" << v.x << ", " << v.y << ", " << v.z << ")";
        return os;
    }
    
    // Friend function for interpolation between two vectors
    // Significant use: Required for smooth flight path generation
    friend Vector3D interpolate(const Vector3D& start, const Vector3D& end, double t);
    
    // friend function for calculating midpoint with elevation
    // used in obstacle avoidance to find safe waypoints
    friend Vector3D calculateSafeMidpoint(const Vector3D& a, const Vector3D& b, double elevationBoost);
};

// friend function - Interpolation
// needs direct access to private x,y,z for efficiency
// used in flight animation for smooth transitions between waypoints
inline Vector3D interpolate(const Vector3D& start, const Vector3D& end, double t) {
    // direct access to private members
    return Vector3D(
        start.x + (end.x - start.x) * t,
        start.y + (end.y - start.y) * t,
        start.z + (end.z - start.z) * t
    );
}

// Safe Midpoint Calculation
// needs direct access to calculate elevated waypoint for obstacle avoidance
inline Vector3D calculateSafeMidpoint(const Vector3D& a, const Vector3D& b, double elevationBoost) {
    Vector3D mid;
    mid.x = (a.x + b.x) / 2.0;
    mid.y = (a.y + b.y) / 2.0;
    mid.z = (a.z + b.z) / 2.0 + elevationBoost;  // add elevation for safety
    return mid;
};

class Obstacle {
private:
    Vector3D position;
    double length, width, height;
    string type;
public:
    Obstacle(Vector3D pos, double l, double w, double h, string t = "Building")
        : position(pos), length(l), width(w), height(h), type(t) {}
    
    // check if point is inside obstacle (with margin)
    bool containsPoint(const Vector3D& p, double margin = 1.0) const {
        return (p.getX() >= position.getX() - margin && 
                p.getX() <= position.getX() + length + margin &&
                p.getY() >= position.getY() - margin && 
                p.getY() <= position.getY() + width + margin &&
                p.getZ() >= position.getZ() - margin && 
                p.getZ() <= position.getZ() + height + margin);
    }
    
    Vector3D getPosition() const { return position; }
    double getLength() const { return length; }
    double getWidth() const { return width; }
    double getHeight() const { return height; }
    string getType() const { return type; }
    
    Vector3D getCenter() const {
        return Vector3D(position.getX() + length/2, position.getY() + width/2, position.getZ() + height/2);
    }
};
#endif