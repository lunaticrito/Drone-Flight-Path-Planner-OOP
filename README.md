# Drone Flight Path Planner 

## Project Structure
DroneFlightPlanner/
├── Common.h        - Vector3D class, Obstacle class
├── Battery.h       - Abstract PowerSource, Battery classes
├── Drone.h         - Vehicle interface, Drone hierarchy
├── Map.h           - 3D Map with obstacles
├── PathFinder.h    - A* pathfinding algorithm
├── Logger.h        - File handling, Templates, Mission logging
├── Simulator.h     - Windows console visualization
├── main.cpp        - Main application
└── README.md
## Compilation Instructions (Windows)

### Using g++ (MinGW):
g++ -std=c++14 -o DroneFlanner.exe main.cpp -static

### Using Visual Studio Developer Command Prompt:
cl /EHsc /std:c++14 main.cpp /Fe:DronePlanner.exe

### Using Code::Blocks:
1. Create new Console Application project
2. Add all .h files to Header Files
3. Add main.cpp to Source Files
4. Build and Run

## OOP Concepts Demonstrated

### 1. Classes and Objects
- `Vector3D` - 3D point/vector representation
- `Obstacle` - Map obstacles with dimensions
- `Drone` - Autonomous drone with state
- `Map3D` - 3D environment
- `PathFinder3D` - Pathfinding engine
- `MissionLogger` - Log management
- `ConsoleSimulator` - Visualization

### 2. Inheritance
- `PowerSource` (abstract) → `Battery` → `HighCapacityBattery`
- `Vehicle` (abstract) + `IFlyable` (interface) → `Drone`
- `Drone` → `SurveyDrone`, `DeliveryDrone`, `RacingDrone`

### 3. Polymorphism
#### Operator Overloading (Vector3D):
- `+`, `-`, `*`, `/` for vector math
- `==` for comparison
- `<<` for stream output

#### Function Overloading:
- `DataStore::find()` - multiple signatures
- `getInput<T>()` - template function

#### Constructor Overloading:
- `Battery()`, `Battery(cap)`, `Battery(cap, rate, type)`
- `Drone()`, `Drone(id, model)`, `Drone(id, model, battery, speed)`

### 4. Abstraction (Virtual Functions)
- `PowerSource` - pure virtual: `getCharge()`, `consume()`, `recharge()`, `isLow()`, `getStatus()`
- `Vehicle` - pure virtual: `move()`, `getPosition()`, `getInfo()`
- `IFlyable` - interface: `takeOff()`, `land()`, `hover()`, `isFlying()`
- `IPathFinder` - pure virtual: `findPath()`

### 5. Encapsulation
- Private member variables with public getters/setters
- `Vector3D`: x, y, z accessible via `getX()`, `setX()`, etc.
- `Battery`: internal state protected, controlled access
- `Drone`: position, battery managed internally

### 6. Inline Functions
- `Vector3D::getX()`, `getY()`, `getZ()` - inline getters
- `printSeparator()` - inline utility function
- `PathFinder3D::posKey()` - inline helper

### 7. Type Conversion
- `Vector3D::operator std::string()` - converts to "(x,y,z)" string
- `Drone::operator std::string()` - converts to "id (model)"

### 8. File Handling
- `MissionLogger::logMission()` - writes to CSV file
- `MissionLogger::readAllMissions()` - reads from CSV file
- Append mode (`std::ios::app`) for logging
- Truncate mode (`std::ios::trunc`) for clearing

### 9. Templates
- `DataStore<T>` - generic container class
- `getInput<T>()` - type-safe input function

### 10. STL Usage
- `std::vector` - dynamic arrays
- `std::priority_queue` - A* open set
- `std::unordered_map` - closed set, statistics
- `std::unique_ptr` - smart pointers for drone management
- `std::string`, `std::stringstream` - string handling

## Features

1. **3D Map Visualization** - Top-down view with colored obstacles
2. **Multiple Drone Types** - Standard, Survey, Delivery, Racing
3. *A* Pathfinding** - Optimal route avoiding obstacles
4. **Path Smoothing** - Removes unnecessary waypoints
5. **Battery Management** - Consumption tracking, low battery warnings
6. **Flight Simulation** - Animated drone movement
7. **Mission Logging** - CSV file storage
8. **Statistics** - Summary and efficiency comparison

## Usage

1. Run the executable
2. View map to see obstacle layout
3. Select a drone type
4. Enter start and destination coordinates
5. Watch the flight simulation
6. View mission logs and statistics

## Console Controls
- Number keys: Menu selection
- Any key: Continue after viewing screens
