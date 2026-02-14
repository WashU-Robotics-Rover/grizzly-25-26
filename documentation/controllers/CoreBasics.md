# Grizzly Controller Architecture

## Overview
The Grizzly Controller is the central orchestrator that manages and coordinates multiple services running on separate threads. It uses Boost.Asio for asynchronous I/O and integrates with the existing Grizzly scheduler system.

## Controller Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Grizzly Controller                       │
│                  (Thread Orchestrator)                     │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Service   │  │   Service   │  │   Service   │        │
│  │   Thread 1  │  │   Thread 2  │  │   Thread N  │        │
│  │             │  │             │  │             │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Thread    │  │   Service   │  │   Service   │        │
│  │   Manager   │  │   Registry  │  │   Monitor   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
├─────────────────────────────────────────────────────────────┤
│  ┌─────────────┐  ┌─────────────┐  ┌─────────────┐        │
│  │   Asio      │  │   Scheduler │  │   Logger    │        │
│  │   Manager   │  │   Manager   │  │   Manager   │        │
│  └─────────────┘  └─────────────┘  └─────────────┘        │
└─────────────────────────────────────────────────────────────┘
```

## Core Components

### 1. Controller (Thread Orchestrator)
**Purpose**: Central coordinator that manages multiple services running on separate threads.

**Key Functions**:
- `start_service()` - Launch a service on a dedicated thread
- `stop_service()` - Gracefully shutdown a service
- `monitor_services()` - Monitor health and performance of all services
- `coordinate_services()` - Manage communication between services
- `load_balance()` - Distribute workload across service threads

**Usage Example**:
```cpp
Controller controller;
controller.start_service("sensor_service", sensor_service_func);
controller.start_service("control_service", control_service_func);
controller.start_service("network_service", network_service_func);
```

### 2. Service Thread Manager
**Purpose**: Manages the lifecycle of individual service threads.

**Implementation**:
```cpp
class ServiceThreadManager {
private:
    std::map<std::string, std::thread> service_threads_;
    std::map<std::string, std::atomic<bool>> service_running_;
    
public:
    void start_service(const std::string& name, std::function<void()> service_func) {
        service_running_[name] = true;
        service_threads_[name] = std::thread([this, name, service_func]() {
            while (service_running_[name]) {
                service_func();
            }
        });
    }
    
    void stop_service(const std::string& name) {
        service_running_[name] = false;
        if (service_threads_[name].joinable()) {
            service_threads_[name].join();
        }
    }
};
```

### 3. Service Registry
**Purpose**: Tracks and manages all registered services.

**Implementation**:
```cpp
class ServiceRegistry {
private:
    std::map<std::string, ServiceInfo> services_;
    
public:
    void register_service(const std::string& name, ServiceInfo info) {
        services_[name] = info;
    }
    
    ServiceInfo get_service(const std::string& name) {
        return services_[name];
    }
    
    std::vector<std::string> list_services() {
        std::vector<std::string> names;
        for (const auto& pair : services_) {
            names.push_back(pair.first);
        }
        return names;
    }
};
```

### 4. Service Coordination Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Controller                                │
│                  (Orchestrator)                             │
└─────────────────────┬───────────────────────────────────────┘
                      │
        ┌─────────────┼─────────────┐
        │             │             │
        ▼             ▼             ▼
┌─────────────┐ ┌─────────────┐ ┌─────────────┐
│   Sensor    │ │   Control   │ │   Network   │
│   Service   │ │   Service   │ │   Service   │
│   (Thread)  │ │   (Thread)  │ │   (Thread)  │
└─────────────┘ └─────────────┘ └─────────────┘
        │             │             │
        └─────────────┼─────────────┘
                      │
              ┌─────────────┐
              │   Shared    │
              │   Data     │
              │   Bus      │
              └─────────────┘
```

## Message Types and Protocols

### 1. Sensor Data Messages
**Format**: `SENSOR:<id>:<value>:<unit>:<timestamp>`
**Example**: `SENSOR:temp_1:23.5:C:1640995200`

**Usage**:
```cpp
void send_sensor_data(const std::string& id, double value, const std::string& unit) {
    auto timestamp = std::chrono::system_clock::now().time_since_epoch().count();
    std::string message = "SENSOR:" + id + ":" + std::to_string(value) + ":" + unit + ":" + std::to_string(timestamp);
    udp_socket_->async_send_to(buffer(message), remote_endpoint_, [](error_code ec, size_t bytes) {
        // Handle send result
    });
}
```

### 2. Control Commands
**Format**: `CMD:<command>:<target>:<value>`
**Example**: `CMD:set_speed:motor_1:1500`

**Usage**:
```cpp
void send_control_command(const std::string& command, const std::string& target, const std::string& value) {
    std::string message = "CMD:" + command + ":" + target + ":" + value;
    // Send via UDP
}
```

### 3. Status Updates
**Format**: `STATUS:<component>:<status>:<details>`
**Example**: `STATUS:scheduler:running:loop_count_1250`

## Controller Service Management

### 1. Service Lifecycle Management
```cpp
class GrizzlyController {
private:
    ServiceThreadManager thread_manager_;
    ServiceRegistry service_registry_;
    AsioManager asio_mgr_;
    Scheduler scheduler_;
    
public:
    void initialize() {
        // Initialize core components
        asio_mgr_.start();
        
        // Register and start services
        register_services();
        start_all_services();
    }
    
    void register_services() {
        // Register sensor service
        service_registry_.register_service("sensor_service", {
            .name = "sensor_service",
            .frequency = 50,
            .priority = 1,
            .function = [this]() { sensor_service_loop(); }
        });
        
        // Register control service
        service_registry_.register_service("control_service", {
            .name = "control_service", 
            .frequency = 50,
            .priority = 2,
            .function = [this]() { control_service_loop(); }
        });
        
        // Register network service
        service_registry_.register_service("network_service", {
            .name = "network_service",
            .frequency = 10,
            .priority = 3,
            .function = [this]() { network_service_loop(); }
        });
    }
    
    void start_all_services() {
        auto services = service_registry_.list_services();
        for (const auto& service_name : services) {
            auto service_info = service_registry_.get_service(service_name);
            thread_manager_.start_service(service_name, service_info.function);
        }
    }
};
```

### 2. Service Threading Model
```
┌─────────────────┐    ┌─────────────────┐    ┌─────────────────┐
│   Controller    │    │   Sensor        │    │   Control       │
│   Thread        │    │   Service       │    │   Service       │
│                 │    │   Thread        │    │   Thread        │
│ - Service Mgmt  │    │ - Data Reading  │    │ - Control Logic │
│ - Coordination  │    │ - Processing    │    │ - Calculations  │
│ - Monitoring    │    │ - Transmission │    │ - Commands      │
└─────────────────┘    └─────────────────┘    └─────────────────┘
        │                       │                       │
        └───────────────────────┼───────────────────────┘
                                │
                    ┌─────────────────┐
                    │   Network       │
                    │   Service       │
                    │   Thread        │
                    │                 │
                    │ - UDP/TCP I/O   │
                    │ - Message Proc  │
                    │ - Async Ops     │
                    └─────────────────┘
```

## Performance Characteristics

### 1. Timing Precision
- **Target Frequency**: 50 Hz (20ms intervals)
- **Jitter Tolerance**: < 1ms
- **Execution Time**: < 5ms per cycle
- **Network Latency**: < 10ms

### 2. Resource Usage
- **CPU Usage**: < 10% on modern hardware
- **Memory**: < 100MB for typical applications
- **Network Bandwidth**: < 1Mbps for sensor data

### 3. Scalability
- **Max Sensors**: 100+ concurrent
- **Max Actuators**: 50+ concurrent
- **Network Connections**: 10+ simultaneous
- **Message Rate**: 1000+ messages/second

## Error Handling and Recovery

### 1. Network Errors
```cpp
void handle_network_error(const boost::system::error_code& ec) {
    if (ec == boost::asio::error::connection_refused) {
        // Retry connection
        reconnect_after_delay();
    } else if (ec == boost::asio::error::timed_out) {
        // Handle timeout
        handle_timeout();
    }
}
```

### 2. Control Loop Errors
```cpp
void handle_control_error(const std::exception& e) {
    // Log error
    logger_.error("Control loop error: " + std::string(e.what()));
    
    // Enter safe mode
    enter_safe_mode();
    
    // Attempt recovery
    attempt_recovery();
}
```

## Configuration and Setup

### 1. Configuration File
```json
{
    "controller": {
        "frequency": 50,
        "network": {
            "udp_port": 5000,
            "tcp_port": 5001,
            "timeout_ms": 1000
        },
        "sensors": {
            "max_count": 100,
            "update_rate": 50
        },
        "actuators": {
            "max_count": 50,
            "command_timeout": 500
        }
    }
}
```

### 2. Command Line Usage
```bash
# Start controller
./grizzly_controller --config config.json

# Start with specific frequency
./grizzly_controller --frequency 50 --port 5000

# Start in debug mode
./grizzly_controller --debug --verbose
```

## Monitoring and Diagnostics

### 1. Health Monitoring
- **Control Loop Health**: Monitor 50 Hz timing
- **Network Health**: Monitor connection status
- **Sensor Health**: Monitor data freshness
- **Actuator Health**: Monitor command execution

### 2. Performance Metrics
- **Loop Execution Time**: Track per-cycle timing
- **Network Latency**: Monitor message round-trip times
- **Error Rates**: Track failed operations
- **Resource Usage**: Monitor CPU and memory

This architecture provides a robust, high-performance control system that integrates seamlessly with your existing Grizzly scheduler while adding powerful networking capabilities through Boost.Asio.