// Battery management
#ifndef BATTERY_H
#define BATTERY_H

#include <string>
using namespace std;

// Abstract base class 
class PowerSource {
public:
    virtual double getCharge() const = 0;
    virtual void consume(double amount) = 0;
    virtual void recharge() = 0;
    virtual bool isLow() const = 0;
    virtual string getStatus() const = 0;
    virtual ~PowerSource() {}
};

// Derived class implementing PowerSource
class Battery : public PowerSource {
private:
    double capacity;
    double currentCharge;
    double consumptionRate; //dist per unit
    string batteryType;
    
public:
    // constructor overloading
    Battery() : capacity(100.0), currentCharge(100.0), consumptionRate(0.5), batteryType("Li-Ion") {}
    Battery(double cap) : capacity(cap), currentCharge(cap), consumptionRate(0.5), batteryType("Li-Ion") {}
    Battery(double cap, double rate, string type) 
        : capacity(cap), currentCharge(cap), consumptionRate(rate), batteryType(type) {}
    
    // virtual function
    double getCharge() const override { return currentCharge; }
    
    void consume(double distance) override {
        double consumption = distance * consumptionRate;
        currentCharge = max(0.0, currentCharge - consumption);
    }
    
    void recharge() override { currentCharge = capacity; }
    
    bool isLow() const override { return currentCharge < (capacity * 0.2); }
    
    string getStatus() const override {
        if (currentCharge > capacity * 0.6) return "Good";
        if (currentCharge > capacity * 0.3) return "Moderate";
        if (currentCharge > capacity * 0.1) return "Low";
        return "Critical";
    }
    
    // methods
    double getCapacity() const { return capacity; }
    double getPercentage() const { return (currentCharge / capacity) * 100.0; }
    double getConsumptionRate() const { return consumptionRate; }
    string getBatteryType() const { return batteryType; }
    
    bool canTravel(double distance) const {
        return (currentCharge - distance * consumptionRate) > (capacity * 0.1);
    }
};

// High-capacity battery (Inheritance)
class HighCapacityBattery : public Battery {
private:
    bool fastChargeEnabled;
public:
    HighCapacityBattery() : Battery(200.0, 0.4, "Li-Po High-Cap"), fastChargeEnabled(true) {}
    
    string getStatus() const override {
        string base = Battery::getStatus();
        return base + (fastChargeEnabled ? " [Fast-Charge Ready]" : "");
    }
};

#endif