#pragma once

#include <vector>
#include "Wheel.h"
#include "ControlInterface.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * @brief Manages dynamic creation and reinitialization of Wheel objects based on task notifications.
 */
class WheelManager
{
public:
    WheelManager(controller_data_t *controllerData, TaskHandles *handles);

    /**
     * @brief Static FreeRTOS task entry point that calls instance method.
     */
    static void TaskEntry(void *pvParameters);

    /**
     * @brief Actual task method that handles wheel lifecycle.
     */
    void Task();

private:
    controller_data_t *data_;
    TaskHandles *taskHandles_;
    std::vector<Wheel> wheels_;

    void ReallocateWheels();
    void UpdateRequestedWheels(uint16_t wheelMask);
};
