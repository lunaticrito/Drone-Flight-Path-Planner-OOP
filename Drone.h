// Drone class hierarchy
#ifndef DRONE_H
#define DRONE_H

#include "Common.h"
#include "Battery.h"
#include <string>
#include <memory>
using namespace std;

// Abstract Vehicle class
class Vehicle {
public:
    virtual void move(const Vector3D& direction) = 0;
    virtual Vector3D getPosition() const = 0;
    virtual string getInfo() const = 0;
    virtual ~Vehicle() {}
};

// Flying capability interface
class IFlyable {
public:
    virtual void takeOff() = 0;
    virtual void land() = 0;
    virtual void hover() = 0;
    virtual bool isFlying() const = 0;
    virtual ~IFlyable() {}
};

// Base Drone class (Multiple inheritance)
class Drone : public Vehicle, public IFlyable {
protected:
    string id;
    string model;
    Vector3D position;
    Vector3D velocity;
    Battery battery;
    double speed;         // units per second
    double totalDistance;
    bool flying;
    int missionCount;
    
public:
    // Constructor overloading
    Drone() : id("DRN-001"), model("Basic"), position(0,0,0), speed(2.0), 
              totalDistance(0), flying(false), missionCount(0) {}
    
    Drone(string id, string model, double spd = 2.0)
        : id(id), model(model), position(0,0,0), speed(spd), 
          totalDistance(0), flying(false), missionCount(0) {}
    
    Drone(string id, string model, Battery bat, double spd)
        : id(id), model(model), position(0,0,0), battery(bat), speed(spd),
          totalDistance(0), flying(false), missionCount(0) {}
    
    // virtual function implementations
    void move(const Vector3D& target) override {
        if (!flying) return;
        double dist = position.distanceTo(target);
        battery.consume(dist);
        totalDistance += dist;
        position = target;
    }
    
    Vector3D getPosition() const override { return position; }
    
    string getInfo() const override {
        return "Drone[" + id + "] Model: " + model + " Battery: " + 
               to_string((int)battery.getPercentage()) + "%";
    }
    
    // IFlyable implementations
    void takeOff() override { 
        flying = true; 
        if (position.getZ() < 1) position.setZ(1);
    }
    void land() override { 
        flying = false; 
        position.setZ(0);
    }
    void hover() override { velocity = Vector3D(0, 0, 0); }
    bool isFlying() const override { return flying; }
    
    // getters
    string getId() const { return id; }
    string getModel() const { return model; }
    double getSpeed() const { return speed; }
    double getTotalDistance() const { return totalDistance; }
    Battery& getBattery() { return battery; }
    const Battery& getBattery() const { return battery; }
    int getMissionCount() const { return missionCount; }
    
    // setters
    void setPosition(const Vector3D& pos) { position = pos; }
    void incrementMission() { missionCount++; }
    void resetDistance() { totalDistance = 0; }
    
    // type conversion 
    operator string() const {
        return id + " (" + model + ")";
    }
};

// Inheritance
class SurveyDrone : public Drone {
private:
    double cameraResolution;
public:
    SurveyDrone(string id) 
        : Drone(id, "Survey-X1", Battery(120, 0.6, "Li-Po"), 1.5), cameraResolution(4.0) {}
    
    string getInfo() const override {
        return Drone::getInfo() + " [Survey: " + to_string((int)cameraResolution) + "K Camera]";
    }
};

class DeliveryDrone : public Drone {
private:
    double maxPayload;
    double currentPayload;
public:
    DeliveryDrone(string id)
        : Drone(id, "Delivery-D1", Battery(150, 0.7, "Li-Ion HD"), 2.5), 
          maxPayload(5.0), currentPayload(0) {}
    
    void setPayload(double weight) { currentPayload = min(weight, maxPayload); }
    
    string getInfo() const override {
        return Drone::getInfo() + " [Payload: " + to_string((int)currentPayload) + 
               "/" + to_string((int)maxPayload) + "kg]";
    }
};

class RacingDrone : public Drone {
private:
    double maxSpeed;
public:
    RacingDrone(string id)
        : Drone(id, "Racer-R1", Battery(80, 0.8, "Li-Po Racing"), 5.0), maxSpeed(8.0) {}
    
    string getInfo() const override {
        return Drone::getInfo() + " [Max Speed: " + to_string((int)maxSpeed) + " units/s]";
    }
};

#endif