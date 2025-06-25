# Task Class Documentation

## Overview

The `Task` class provides a modern, RAII-based wrapper around FreeRTOS tasks, offering improved lifecycle management, thread safety, and ease of use compared to direct FreeRTOS API usage.

## Key Features

- **RAII Resource Management**: Tasks are automatically cleaned up when the object is destroyed
- **Thread-Safe State Management**: All state changes are atomic and thread-safe
- **Flexible Memory Allocation**: Supports both dynamic and static memory allocation
- **Coordinated Suspension**: Uses mutexes to ensure safe suspension points
- **Factory Methods**: Provides convenient factory methods for common task patterns

## Task States

The Task class manages four distinct states:

- `Created`: Task object exists but FreeRTOS task hasn't been started
- `Running`: FreeRTOS task is actively running
- `Suspended`: FreeRTOS task is suspended (paused)
- `Stopped`: FreeRTOS task has been deleted

## Basic Usage

### Creating a Simple Task

```cpp
#include "Task.h"

// Configure the task
Task::Config config{
    .name = "MyTask",
    .stack_size = 2048,
    .priority = 5,
    .core_id = tskNO_AFFINITY,  // Any core
    .use_static_allocation = false
};

// Create task with lambda function
auto my_task = std::make_unique<Task>(config, []() {
    ESP_LOGI("MyTask", "Task is running!");
    
    // Your task logic here
    while (true) {
        // Do work
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
});

// Start the task
if (my_task->start()) {
    ESP_LOGI("Main", "Task started successfully");
}
```

### Using Static Allocation

```cpp
// Declare static memory buffers
static StackType_t task_stack[2048];
static StaticTask_t task_tcb;

Task::Config config{
    .name = "StaticTask",
    .stack_size = 2048,
    .priority = 5,
    .core_id = 0,  // Pin to core 0
    .use_static_allocation = true
};

auto task = std::make_unique<Task>(
    config,
    []() { /* task function */ },
    task_stack,
    &task_tcb
);
```

## Factory Methods

### Periodic Tasks

For tasks that need to run at regular intervals:

```cpp
auto periodic_task = TaskFactory::createPeriodicTask(
    config,
    []() {
        ESP_LOGI("Periodic", "Running periodic work");
        // Do periodic work here
    },
    10,  // 10 Hz frequency
    coordination_mutex  // Optional mutex for safe suspension
);
```

### Notification-Based Tasks

For tasks that wait for notifications:

```cpp
auto notification_task = TaskFactory::createNotificationTask(
    config,
    [](uint32_t notification_value) {
        if (notification_value & (1 << 0)) {
            // Handle bit 0
        }
        if (notification_value & (1 << 1)) {
            // Handle bit 1
        }
    }
);

// Send notification to the task
TaskHandle_t handle = notification_task->getHandle();
xTaskNotify(handle, (1 << 0), eSetBits);
```

## Task Lifecycle Management

### Starting and Stopping

```cpp
// Start task
if (task->start()) {
    ESP_LOGI("Main", "Task started");
}

// Stop task (with timeout)
if (task->stop(1000)) {  // 1 second timeout
    ESP_LOGI("Main", "Task stopped gracefully");
}
```

### Suspension and Resumption

```cpp
// Suspend task
if (task->suspend(500)) {  // 500ms timeout
    ESP_LOGI("Main", "Task suspended");
    
    // Do something while task is suspended
    
    // Resume task
    if (task->resume()) {
        ESP_LOGI("Main", "Task resumed");
    }
}
```

### State Queries

```cpp
// Check task state
if (task->isRunning()) {
    ESP_LOGI("Main", "Task is currently running");
}

if (task->isSuspended()) {
    ESP_LOGI("Main", "Task is currently suspended");
}

// Get detailed state
TaskState state = task->getState();
switch (state) {
    case TaskState::Created:   /* ... */ break;
    case TaskState::Running:   /* ... */ break;
    case TaskState::Suspended: /* ... */ break;
    case TaskState::Stopped:   /* ... */ break;
}
```

## Thread Safety and Coordination

### Suspension Mutex

For tasks that need coordinated suspension (to avoid suspending during critical sections):

```cpp
SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

// Set suspension mutex
task->setSuspensionMutex(mutex);

// In your task function, acquire mutex during critical sections
auto periodic_task = TaskFactory::createPeriodicTask(
    config,
    []() {
        // Mutex is automatically acquired before this function
        // and released after it completes
        criticalWorkFunction();
    },
    frequency,
    mutex  // Pass the same mutex here
);
```

## Integration with Existing Code

### Replacing Traditional FreeRTOS Tasks

**Before (Traditional FreeRTOS):**
```cpp
TaskHandle_t task_handle = nullptr;
StackType_t task_stack[2048];
StaticTask_t task_tcb;

// Create task
task_handle = xTaskCreateStatic(
    taskFunction, "TaskName", 2048, nullptr, 5,
    task_stack, &task_tcb
);

// Manual cleanup required
if (task_handle) {
    vTaskDelete(task_handle);
}
```

**After (Task Class):**
```cpp
auto task = std::make_unique<Task>(
    Task::Config{
        .name = "TaskName",
        .stack_size = 2048,
        .priority = 5,
        .use_static_allocation = true
    },
    []() { /* task logic */ },
    task_stack,
    &task_tcb
);

task->start();
// Automatic cleanup when task goes out of scope
```

## ModernTaskManager Example

The `ModernTaskManager` class demonstrates how to refactor the existing `TaskManager` to use the new Task class:

```cpp
// In ModernTaskManager
std::unique_ptr<Task> control_task_;
std::unique_ptr<Task> odo_broadcast_task_;

// Creating tasks becomes simpler
void createControlTask() {
    control_task_ = TaskFactory::createPeriodicTask(
        config,
        [this]() { wheel_container_.executeControlLoop(); },
        control_frequency_,
        control_mutex_
    );
}

// Task actions are simplified
bool controlLoopTaskAction(TaskAction action) {
    switch (action) {
        case TaskAction::Start:   return control_task_->start();
        case TaskAction::Stop:    return control_task_->stop();
        case TaskAction::Suspend: return control_task_->suspend();
        case TaskAction::Resume:  return control_task_->resume();
    }
}
```

## Best Practices

1. **Use RAII**: Always create tasks as `std::unique_ptr<Task>` to ensure automatic cleanup
2. **Check Return Values**: Always check the return value of `start()`, `stop()`, `suspend()`, and `resume()`
3. **Set Suspension Mutex**: For tasks that modify shared data, use a suspension mutex for safe coordination
4. **Use Factory Methods**: Prefer `TaskFactory` methods for common patterns like periodic tasks
5. **Proper Timeouts**: Use appropriate timeouts for `stop()` and `suspend()` operations
6. **Static Allocation**: Use static allocation for real-time systems where dynamic allocation is undesirable

## Error Handling

The Task class provides comprehensive error handling:

- All operations return boolean values indicating success/failure
- Detailed logging using ESP-IDF logging framework
- State validation prevents invalid operations
- Automatic resource cleanup on failure

## Performance Considerations

- Task state is managed using atomic operations for thread safety
- Minimal overhead compared to direct FreeRTOS API usage
- Optional static allocation eliminates heap fragmentation
- Suspension coordination prevents unnecessary blocking

## Migration Guide

To migrate from the existing TaskManager to use the new Task class:

1. Replace task handles with `std::unique_ptr<Task>`
2. Use factory methods for creating tasks
3. Replace manual task creation with Task constructor calls
4. Simplify task action methods using Task member functions
5. Remove manual resource cleanup code (handled by RAII)

This new design provides better encapsulation, safer resource management, and more maintainable code while preserving all the functionality of the original TaskManager.
