# Grizzly Controller Class Documentation

## Overview
The Grizzly Controller Class is the foundation for all controller implementations in the Grizzly system. It provides a consistent interface and shared functionality for managing services, threads, and coordination between different system components.

## Class Hierarchy

```
BaseController (Abstract)
├── RoverMovementController
├── SensorDataController
├── NetworkController
├── PowerController
└── NavigationController
```

## BaseController Class

### Purpose
The `BaseController` class serves as the abstract base class that all specific controllers inherit from. It provides common functionality for service management, thread coordination, lifecycle management, and local inter-thread communication through integrated ASIO support.

### Class Definition

```cpp
class BaseController {
protected:
    // Core controller members
    std::string controller_name_;                    // Unique identifier for the controller
    std::atomic<bool> running_;                     // Controller running state
    std::map<std::string, std::thread> service_threads_;      // Active service threads
    std::map<std::string, std::atomic<bool>> service_running_; // Service running states
    std::mutex controller_mutex_;                   // Thread synchronization
    
    // ASIO local inter-thread communication infrastructure
    boost::asio::io_context io_context_;            // Shared I/O context for async operations
    std::thread io_thread_;                         // Dedicated thread for I/O context
    std::atomic<bool> io_running_;                  // I/O context running state
    std::mutex message_mutex_;                      // Message handling mutex
    
    // Inter-controller messaging
    std::map<std::string, std::function<void(const std::string&)>> message_handlers_;
    std::map<std::string, std::queue<std::string>> message_queues_;
    
public:
    // Constructor
    BaseController(const std::string& name);
    
    // Virtual destructor
    virtual ~BaseController();
    
    // Pure virtual functions (must be implemented by derived classes)
    virtual void initialize() = 0;
    virtual void start() = 0;
    virtual void stop() = 0;
    virtual void pause() = 0;
    virtual void resume() = 0;
    
    // Common functionality
    void start_service(const std::string& service_name, std::function<void()> service_func);
    void stop_service(const std::string& service_name);
    void stop_all_services();
    bool is_running() const;
    std::string get_name() const;
    
    // ASIO local inter-thread communication functionality
    void start_io_context();
    void stop_io_context();
    void send_message_to_controller(const std::string& target_controller, const std::string& message);
    void register_message_handler(const std::string& message_type, std::function<void(const std::string&)> handler);
    void post_async_task(std::function<void()> task);
    void schedule_periodic_task(std::function<void()> task, std::chrono::milliseconds interval);
};
```

## Core Functionality

### 1. Service Management
**Purpose**: Manages the lifecycle of individual services within a controller.

**Key Methods**:
- `start_service()` - Launch a service on a dedicated thread
- `stop_service()` - Gracefully shutdown a specific service
- `stop_all_services()` - Shutdown all services managed by the controller

**Implementation**:
```cpp
void BaseController::start_service(const std::string& service_name, std::function<void()> service_func) {
    std::lock_guard<std::mutex> lock(controller_mutex_);
    service_running_[service_name] = true;
    service_threads_[service_name] = std::thread([this, service_name, service_func]() {
        while (service_running_[service_name]) {
            service_func();
        }
    });
}
```

### 2. Thread Coordination
**Purpose**: Coordinates between multiple service threads within a controller.

**Features**:
- Thread-safe service management
- Atomic state tracking
- Mutex-protected operations
- Graceful shutdown handling

### 3. Lifecycle Management
**Purpose**: Provides consistent lifecycle management for all controllers.

**States**:
- **Initialized** - Controller is set up but not running
- **Running** - Controller and services are active
- **Paused** - Controller is running but services are paused
- **Stopped** - Controller and all services are shutdown

### 4. ASIO Local Inter-Thread Communication
**Purpose**: Provides centralized asynchronous communication capabilities for local inter-thread and inter-controller messaging.

**Key Features**:
- Shared I/O context for efficient async operations
- Inter-controller message passing
- Asynchronous task scheduling
- Periodic task execution
- Event-driven programming support

**Implementation**:
```cpp
void BaseController::start_io_context() {
    std::lock_guard<std::mutex> lock(message_mutex_);
    if (!io_running_) {
        io_running_ = true;
        io_thread_ = std::thread([this]() {
            io_context_.run();
        });
    }
}

void BaseController::send_message_to_controller(const std::string& target_controller, const std::string& message) {
    std::lock_guard<std::mutex> lock(message_mutex_);
    message_queues_[target_controller].push(message);
    
    // Post async task to process the message
    boost::asio::post(io_context_, [this, target_controller]() {
        process_messages_for_controller(target_controller);
    });
}

void BaseController::schedule_periodic_task(std::function<void()> task, std::chrono::milliseconds interval) {
    auto timer = std::make_shared<boost::asio::steady_timer>(io_context_, interval);
    timer->async_wait([this, task, timer, interval](boost::system::error_code ec) {
        if (!ec) {
            task();
            schedule_periodic_task(task, interval); // Reschedule
        }
    });
}
```

**Benefits**:
- **Local Communication**: Controllers communicate within the same process
- **Asynchronous Operations**: Non-blocking inter-thread communication
- **Event-Driven**: Controllers can react to events and messages
- **Performance**: Efficient task scheduling and message passing
- **Scalability**: Handle many concurrent operations without blocking

## Inter-Controller Messaging

### Purpose
Inter-controller messaging enables asynchronous communication between controllers within the same process using a shared message system. This approach decouples controllers and provides event-driven communication without direct dependencies.

### Message Infrastructure
The BaseController provides:
- **Message Queues**: Thread-safe queues for each target controller
- **Message Handlers**: Registration system for message type callbacks
- **ASIO Integration**: Asynchronous message processing and dispatch

### Key Methods
- `send_message_to_controller()` - Send messages to specific controllers
- `register_message_handler()` - Register callbacks for message types
- `post_async_task()` - Schedule asynchronous tasks
- `schedule_periodic_task()` - Set up recurring operations

### Message Flow
1. **Sending**: Controllers send messages using `send_message_to_controller()`
2. **Queuing**: Messages are queued per target controller
3. **Processing**: ASIO context processes messages asynchronously
4. **Handling**: Target controllers receive messages through registered handlers

### Benefits
- **Decoupling**: Controllers don't need direct references to each other
- **Asynchronous**: Non-blocking message sending and processing
- **Scalable**: Multiple controllers can communicate simultaneously
- **Flexible**: Easy to add new message types and handlers

## Controller Implementation Requirements

### Core Functions (Must Implement)
All controllers must implement these pure virtual functions:
- `initialize()` - Set up controller resources and configuration
- `start()` - Begin controller operations and start services
- `stop()` - Gracefully shutdown controller and all services
- `pause()` - Temporarily halt controller operations
- `resume()` - Resume controller operations after pause

### Inherited ASIO Functions
All controllers inherit these asynchronous capabilities:
- `start_io_context()` - Initialize the shared I/O context
- `stop_io_context()` - Shutdown the I/O context
- `send_message_to_controller()` - Send messages to other controllers
- `register_message_handler()` - Register callbacks for message types
- `post_async_task()` - Schedule asynchronous tasks
- `schedule_periodic_task()` - Set up recurring operations

### Thread Communication
Controllers can use these inherited thread management functions:
- `start_service()` - Launch a service on a dedicated thread
- `stop_service()` - Gracefully shutdown a specific service
- `stop_all_services()` - Shutdown all services managed by the controller
- `is_running()` - Check controller running state
- `get_name()` - Get controller identifier

## Controller Factory Pattern

### Purpose
Provides a centralized way to create and manage different types of controllers.

```cpp
class ControllerFactory {
public:
    static std::unique_ptr<BaseController> create_controller(const std::string& type) {
        if (type == "rover_movement") {
            return std::make_unique<RoverMovementController>();
        }
        else if (type == "sensor_data") {
            return std::make_unique<SensorDataController>();
        }
        else if (type == "power") {
            return std::make_unique<PowerController>();
        }
        else if (type == "navigation") {
            return std::make_unique<NavigationController>();
        }
        return nullptr;
    }
    
    static std::vector<std::string> get_available_controllers() {
        return {"rover_movement", "sensor_data", "power", "navigation"};
    }
};
```

## Usage Patterns

### 1. Basic Controller Usage
```cpp
// Create a controller
auto controller = ControllerFactory::create_controller("rover_movement");

// Initialize and start
controller->initialize();
controller->start();

// Use controller-specific functionality
auto rover = static_cast<RoverMovementController*>(controller.get());
rover->move_forward(1.0);

// Cleanup
controller->stop();
```

### 2. Multiple Controller Coordination
```cpp
class GrizzlySystem {
private:
    std::map<std::string, std::unique_ptr<BaseController>> controllers_;
    
public:
    void initialize_system() {
        // Create multiple controllers
        controllers_["rover"] = ControllerFactory::create_controller("rover_movement");
        controllers_["sensor"] = ControllerFactory::create_controller("sensor_data");
        controllers_["power"] = ControllerFactory::create_controller("power");
        
        // Initialize all controllers
        for (auto& pair : controllers_) {
            pair.second->initialize();
        }
    }
    
    void start_system() {
        // Start all controllers
        for (auto& pair : controllers_) {
            pair.second->start();
        }
    }
    
    void stop_system() {
        // Stop all controllers
        for (auto& pair : controllers_) {
            pair.second->stop();
        }
    }
};
```

### 3. Controller Communication
```cpp
class ControllerCoordinator {
private:
    std::map<std::string, std::unique_ptr<BaseController>> controllers_;
    
public:
    void coordinate_controllers() {
        // Get sensor data
        auto sensor_controller = static_cast<SensorDataController*>(controllers_["sensor"].get());
        double speed = sensor_controller->get_sensor_value("speed");
        
        // Use sensor data to control rover
        auto rover_controller = static_cast<RoverMovementController*>(controllers_["rover"].get());
        if (speed > 0.5) {
            rover_controller->emergency_stop();
        }
    }
};
```

## Design Principles

### 1. Single Responsibility
Each controller is responsible for one specific aspect of the system:
- **RoverMovementController** - Movement and locomotion
- **SensorDataController** - Data collection and processing
- **NetworkController** - Communication and data transmission

### 2. Open/Closed Principle
- **Open for extension** - Easy to add new controller types
- **Closed for modification** - Base class remains stable

### 3. Dependency Inversion
- Controllers depend on abstractions (BaseController)
- Not dependent on concrete implementations

### 4. Interface Segregation
- Each controller only implements what it needs
- No forced dependencies on unused functionality

### 5. ASIO Integration Benefits
**Why ASIO is included in BaseController**:

- **Local Inter-Thread Communication**: Controllers communicate within the same process
- **Asynchronous Task Scheduling**: Non-blocking task execution and message passing
- **Event-Driven Architecture**: Controllers can react to events and messages
- **Performance**: Efficient async operations without blocking service threads
- **Scalability**: Handle many concurrent operations efficiently
- **Maintainability**: Centralized async communication code

**Architecture Benefits**:
```
┌─────────────────────────────────────────────────────────────┐
│                    BaseController                          │
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Service   │  │   Service   │  │   ASIO      │        │
│  │ Management  │  │   Threads   │  │   Local     │        │
│  │             │  │             │  │   Comm      │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Rover     │  │   Sensor    │  │   Power     │        │
│  │ Controller  │  │ Controller  │  │ Controller │        │
│  │             │  │             │  │             │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
│         │               │               │                │
│         └───────────────┼───────────────┘                │
│                         │                                │
│              ┌─────────────┐                             │
│              │   Shared    │                             │
│              │   Data     │                             │
│              │   Bus      │                             │
│              │ (ASIO)     │                             │
│              └─────────────┘                             │
└─────────────────────────────────────────────────────────────┘
```

**Local Communication Examples**:
```cpp
// Any controller can use local communication functionality
class SensorDataController : public BaseController {
public:
    void send_sensor_data() {
        // Use inherited ASIO functionality for local communication
        send_message_to_controller("RoverMovement", "SENSOR:temp:23.5");
    }
};

class RoverMovementController : public BaseController {
public:
    void send_movement_command() {
        // Use inherited ASIO functionality for local communication
        send_message_to_controller("PowerController", "CMD:set_power:0.8");
    }
};
```

## Thread Safety

### 1. Atomic Operations
- `running_` state is atomic for thread-safe access
- `service_running_` states are atomic for service coordination

### 2. Mutex Protection
- `controller_mutex_` protects thread management operations
- Prevents race conditions during service start/stop

### 3. RAII Principles
- Automatic cleanup in destructor
- Exception-safe resource management

## Error Handling

### 1. Service Failure Recovery
```cpp
void BaseController::handle_service_failure(const std::string& service_name) {
    // Log the failure
    logger_.error("Service " + service_name + " failed");
    
    // Attempt to restart the service
    if (service_running_[service_name]) {
        stop_service(service_name);
        std::this_thread::sleep_for(100ms);
        start_service(service_name, service_functions_[service_name]);
    }
}
```

### 2. Graceful Shutdown
```cpp
void BaseController::graceful_shutdown() {
    running_ = false;
    
    // Stop all services gracefully
    for (auto& pair : service_running_) {
        pair.second = false;
    }
    
    // Wait for threads to finish
    for (auto& pair : service_threads_) {
        if (pair.second.joinable()) {
            pair.second.join();
        }
    }
}
```

## Performance Considerations

### 1. Thread Management
- Each service runs on its own thread
- Threads are created/destroyed as needed
- No thread pooling (can be added if needed)

### 2. Memory Management
- Smart pointers for automatic cleanup
- RAII for resource management
- Minimal memory overhead per controller

### 3. Synchronization
- Minimal locking for performance
- Atomic operations where possible
- Lock-free operations for hot paths

This controller class architecture provides a robust, extensible foundation for building complex control systems while maintaining clean separation of concerns and thread safety.
