// Logger.h - File handling and mission logging with Templates
#ifndef LOGGER_H
#define LOGGER_H

#include <fstream>
#include <string>
#include <vector>
#include <sstream>
#include <iomanip>
#include <ctime>
#include <iostream>
#include <unordered_map>

using namespace std;

// Mission result structure
struct MissionResult {
    string droneId;
    string startPos;
    string endPos;
    double distance;
    double batteryUsed;
    double duration;
    string status;
    string timestamp;
    
    string toCSV() const {
        stringstream ss;
        ss << droneId << "," << startPos << "," << endPos << ","
           << fixed << setprecision(2) << distance << ","
           << batteryUsed << "," << duration << "," << status << "," << timestamp;
        return ss.str();
    }
    
    static MissionResult fromCSV(const string& line) {
        MissionResult result;
        stringstream ss(line);
        string token;
        vector<string> tokens;
        while (getline(ss, token, ',')) tokens.push_back(token);
        
        if (tokens.size() >= 8) {
            result.droneId = tokens[0];
            result.startPos = tokens[1];
            result.endPos = tokens[2];
            result.distance = stod(tokens[3]);
            result.batteryUsed = stod(tokens[4]);
            result.duration = stod(tokens[5]);
            result.status = tokens[6];
            result.timestamp = tokens[7];
        }
        return result;
    }
};

// template class for generic data storage
template<typename T>
class DataStore {
private:
    vector<T> data;
public:
    void add(const T& item) { data.push_back(item); }
    void clear() { data.clear(); }
    size_t size() const { return data.size(); }
    const T& get(size_t idx) const { return data[idx]; }
    T& get(size_t idx) { return data[idx]; }
    const vector<T>& getAll() const { return data; }
    
    // Function overloading for finding
    bool find(const T& item) const {
        for (const auto& d : data) {
            if (d == item) return true;
        }
        return false;
    }
};

// file handling
class MissionLogger {
private:
    string logFile;
    DataStore<MissionResult> missionStore;
    
    string getCurrentTimestamp() {
        time_t now = time(0);
        tm* ltm = localtime(&now);
        stringstream ss;
        ss << 1900 + ltm->tm_year << "-" 
           << setfill('0') << setw(2) << 1 + ltm->tm_mon << "-"
           << setw(2) << ltm->tm_mday << " "
           << setw(2) << ltm->tm_hour << ":"
           << setw(2) << ltm->tm_min << ":"
           << setw(2) << ltm->tm_sec;
        return ss.str();
    }
    
public:
    MissionLogger(string file = "mission_log.csv") : logFile(file) {}
    
    //write file
    void logMission(MissionResult& result) {
        result.timestamp = getCurrentTimestamp();
        missionStore.add(result);
    //append file
        ofstream file(logFile, ios::app);
        if (file.is_open()) {
            file << result.toCSV() << endl;
            file.close();
        }
    }
    //read file
    vector<MissionResult> readAllMissions() {
        vector<MissionResult> missions;
        ifstream file(logFile);
        if (file.is_open()) {
            string line;
            while (getline(file, line)) {
                if (!line.empty()) {
                    missions.push_back(MissionResult::fromCSV(line));
                }
            }
            file.close();
        }
        return missions;
    }
    
    // generate summary report
    void printSummary() {
        auto missions = readAllMissions();
        if (missions.empty()) {
            cout << "No mission logs found.\n";
            return;
        }
        
        double totalDist = 0, totalBattery = 0, totalTime = 0;
        int successCount = 0;
        unordered_map<string, int> droneUsage;
        
        for (const auto& m : missions) {
            totalDist += m.distance;
            totalBattery += m.batteryUsed;
            totalTime += m.duration;
            if (m.status == "Completed") successCount++;
            droneUsage[m.droneId]++;
        }
        
        cout << "\n========== MISSION SUMMARY ==========\n";
        cout << "Total Missions: " << missions.size() << endl;
        cout << "Successful: " << successCount << " (" 
             << (100.0 * successCount / missions.size()) << "%)\n";
        cout << "Total Distance: " << fixed << setprecision(2) 
             << totalDist << " units\n";
        cout << "Total Battery Used: " << totalBattery << "%\n";
        cout << "Total Flight Time: " << totalTime << " seconds\n";
        cout << "Avg Distance/Mission: " << totalDist / missions.size() << " units\n";
        cout << "\nDrone Usage:\n";
        for (const auto& p : droneUsage) {
            cout << "  " << p.first << ": " << p.second << " missions\n";
        }
        cout << "======================================\n";
    }
    
    // compare efficiency between drones
    void compareEfficiency() {
        auto missions = readAllMissions();
        if (missions.empty()) {
            cout << "No mission logs for comparison.\n";
            return;
        }
        
        unordered_map<string, vector<double>> droneEfficiency;
        
        for (const auto& m : missions) {
            if (m.distance > 0) {
                double eff = m.batteryUsed / m.distance;
                droneEfficiency[m.droneId].push_back(eff);
            }
        }
        
        cout << "\n======= EFFICIENCY COMPARISON =======\n";
        cout << left << setw(15) << "Drone" 
             << setw(12) << "Missions" 
             << setw(15) << "Avg Batt/Unit" << endl;
        cout << string(42, '-') << endl;
        
        for (const auto& p : droneEfficiency) {
            double avgEff = 0;
            for (double e : p.second) avgEff += e;
            avgEff /= p.second.size();
            
            cout << left << setw(15) << p.first 
                 << setw(12) << p.second.size()
                 << fixed << setprecision(3) << avgEff << "%\n";
        }
        cout << "=====================================\n";
    }
    //clear file
    void clearLogs() {
        ofstream file(logFile, ios::trunc);
        file.close();
        missionStore.clear();
    }
};

#endif