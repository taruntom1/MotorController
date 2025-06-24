#ifndef ENCODER_PULSE_READER_H
#define ENCODER_PULSE_READER_H

#include "driver/pulse_cnt.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_cpu.h"
#include "MyStructs.h"

/**
 * @defgroup EncoderPulseReaderModule Encoder Pulse Reader
 * @brief Module for managing encoder pulse counting and velocity calculation.
 *
 * This module provides an interface for configuring and controlling
 * the ESP32 Pulse Counter (PCNT) hardware to read encoder pulses
 * and calculate velocity.
 * @{
 */

/**
 * @brief Configuration structure for the Pulse Counter (PCNT) module.
 */
struct pcnt_config_t
{
    pcnt_unit_config_t unit_config = {/**< Configuration for the PCNT unit. */
                                      .low_limit = -0x7fff,
                                      .high_limit = 0x7fff,
                                      .intr_priority = 0,
                                      .flags = {.accum_count = 1}};
    pcnt_chan_config_t channel_config;                                                /**< Configuration for the PCNT channel. */
    pcnt_glitch_filter_config_t glitch_filter_config;                                 /**< Configuration for the PCNT glitch filter. */
    pcnt_channel_edge_action_t pos_edge_action = PCNT_CHANNEL_EDGE_ACTION_INCREASE;   /**< Action on positive edge. */
    pcnt_channel_edge_action_t neg_edge_action = PCNT_CHANNEL_EDGE_ACTION_DECREASE;   /**< Action on negative edge. */
    pcnt_channel_level_action_t high_level_action = PCNT_CHANNEL_LEVEL_ACTION_KEEP;   /**< Action for high level signal. */
    pcnt_channel_level_action_t low_level_action = PCNT_CHANNEL_LEVEL_ACTION_INVERSE; /**< Action for low level signal. */
};

/**
 * @class EncoderPulseReader
 * @brief Class for reading encoder pulses using the PCNT (Pulse Counter) peripheral.
 *
 * This class encapsulates the configuration and management of a pulse counter unit,
 * providing methods to start/stop counting, retrieve pulse counts, clear counts,
 * and calculate raw angle and velocity based on encoder pulses.
 *
 * @note The class manages the lifetime of the PCNT unit and channel handles.
 *
 * @var EncoderPulseReader::TAG
 *      Tag string for logging and debugging purposes.
 * @var EncoderPulseReader::_pcnt_config
 *      Pointer to the configuration structure for the PCNT peripheral.
 * @var EncoderPulseReader::pcnt_unit
 *      Handle for the PCNT unit.
 * @var EncoderPulseReader::pcnt_channel
 *      Handle for the PCNT channel.
 * @var EncoderPulseReader::previous_count
 *      Previous pulse count, used for velocity calculation.
 * @var EncoderPulseReader::previous_time
 *      Previous timestamp, used for velocity calculation.
 */

class EncoderPulseReader
{
private:
    const char *TAG = "EncoderPulseReader";
    pcnt_config_t *_pcnt_config;                  /**< Pointer to the configuration structure for PCNT. */
    pcnt_unit_handle_t pcnt_unit = nullptr;       /**< Handle for the PCNT unit. */
    pcnt_channel_handle_t pcnt_channel = nullptr; /**< Handle for the PCNT channel. */
    float inverse_cycles_per_sec;                 /**< Cycles per microsecond of processor. */
    int previous_count = 0;                       /**< Previous pulse count for velocity calculation. */
    uint64_t previous_cpu_cycles = 0;             /**< Previous timestamp for velocity calculation. */

public:
    /**
     * @brief Constructs and initializes an EncoderPulseReader instance.
     * @param _pcnt_config Pointer to the PCNT configuration structure.
     */
    explicit EncoderPulseReader(pcnt_config_t *_pcnt_config);

    /**
     * @brief Destructor to clean up resources.
     */
    ~EncoderPulseReader();

    /**
     * @brief Starts the pulse counter.
     * @return True if starting is successful, otherwise false.
     */
    bool start_pulse_counter();

    /**
     * @brief Stops the pulse counter.
     * @return True if stopping is successful, otherwise false.
     */
    bool stop_pulse_counter();

    /**
     * @brief Clears the pulse count of the counter.
     */
    void clear_pulse_count();

    /**
     * @brief Calculates and returns the raw angle based on the current pulse count.
     * @return The raw angle in degrees or radians, depending on implementation.
     */
    encoder_ticks_t get_ticks();

    /**
     * @brief Calculates and returns the raw velocity based on pulse count changes over time.
     * @return The raw velocity, typically in pulses per second or equivalent units.
     */
    encoder_tickrate_t get_tickrate();

    void get_tick_tickrate(encoder_ticks_t &ticks, encoder_tickrate_t &tickrate);
};

/** @} */ // End of EncoderPulseReaderModule

#endif // ENCODER_PULSE_READER_H
