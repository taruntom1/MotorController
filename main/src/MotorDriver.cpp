// MotorDriver.cpp
#include "MotorDriver.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

MotorDriver::MotorDriver(const MotorDriverConfig &config) : config(config)
{
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "MotorDriver created.");
}

MotorDriver::~MotorDriver()
{
    if (generator)
    {
        mcpwm_del_generator(generator);
    }
    if (comparator)
    {
        mcpwm_del_comparator(comparator);
    }
    if (operator_)
    {
        mcpwm_del_operator(operator_);
    }
    if (timer)
    {
        mcpwm_timer_disable(timer);
        mcpwm_del_timer(timer);
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "MotorDriver destroyed.");
}

bool MotorDriver::init()
{
    // Configure the MCPWM timer
    if (mcpwm_timer_config_t timerConfig = {
            .group_id = 0,
            .clk_src = MCPWM_TIMER_CLK_SRC_DEFAULT,
            .resolution_hz = config.clockFrequencyHz,
            .count_mode = MCPWM_TIMER_COUNT_MODE_UP,
            .period_ticks = config.pwmResolution,
            .intr_priority = 0};
        mcpwm_new_timer(&timerConfig, &timer) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to configure MCPWM timer.");
        return false;
    }
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM timer configured.");

    // Configure the MCPWM operator

    if (mcpwm_operator_config_t operatorConfig = {.group_id = 0, .intr_priority = 0};
        mcpwm_new_operator(&operatorConfig, &operator_) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to configure MCPWM operator.");
        return false;
    }
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM operator configured.");

    // Connect the timer and the operator
    if (mcpwm_operator_connect_timer(operator_, timer) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to connect timer to operator.");
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM timer and operator connected.");

    // Configure the comparator

    if (mcpwm_comparator_config_t comparatorConfig = {.intr_priority = 0};
        mcpwm_new_comparator(operator_, &comparatorConfig, &comparator) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to configure MCPWM comparator.");
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM comparator configured.");

    // Configure the generator
    if (mcpwm_generator_config_t generatorConfig = { .gen_gpio_num = config.pwmPin };
        mcpwm_new_generator(operator_, &generatorConfig, &generator) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to configure MCPWM generator.");
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM generator configured.");

    // Set the generator action
    if (mcpwm_generator_set_action_on_timer_event(generator,
                                                  MCPWM_GEN_TIMER_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, MCPWM_TIMER_EVENT_EMPTY, MCPWM_GEN_ACTION_HIGH)) != ESP_OK ||
        mcpwm_generator_set_action_on_compare_event(generator,
                                                    MCPWM_GEN_COMPARE_EVENT_ACTION(MCPWM_TIMER_DIRECTION_UP, comparator, MCPWM_GEN_ACTION_LOW)) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to set generator actions.");
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM generator actions set.");

    // Enable and start the timer
    if (mcpwm_timer_enable(timer) != ESP_OK || mcpwm_timer_start_stop(timer, MCPWM_TIMER_START_NO_STOP) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to start MCPWM timer.");
        return false;
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "MCPWM timer started.");

    // Configure the direction pin
    gpio_set_direction(config.directionPin, GPIO_MODE_OUTPUT);

    gpio_set_level(config.directionPin, 0);

    if (mcpwm_comparator_set_compare_value(comparator, 0) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to set comparator value.");
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Speed set to %.2f   duty: %d", 0.00, 0);
    }

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "MotorDriver initialized successfully.");
    return true;
}

void MotorDriver::setSpeed(float speed)
{
    // Speed is expected to be in the range -1.0 to 1.0
    speed = std::clamp(speed, -1.0f, 1.0f);

    // Set direction
    if (speed >= 0)
    {
        gpio_set_level(config.directionPin, 1);
    }
    else
    {
        gpio_set_level(config.directionPin, 0);
        speed = -speed; // Convert to positive for PWM duty
    }

    // Set comparator value based on duty cycle
    auto dutyTicks = static_cast<uint32_t>(speed * config.pwmResolution);
    if (mcpwm_comparator_set_compare_value(comparator, dutyTicks) != ESP_OK)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Failed to set comparator value.");
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Speed: %.2f   duty: %lu", speed, dutyTicks);
    }
}
