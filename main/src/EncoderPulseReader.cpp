#include "EncoderPulseReader.h"

EncoderPulseReader::EncoderPulseReader(pcnt_config_t *_pcnt_config)
    : _pcnt_config(_pcnt_config)
{
    ESP_ERROR_CHECK(pcnt_new_unit(&_pcnt_config->unit_config, &pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &_pcnt_config->channel_config, &pcnt_channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel, _pcnt_config->pos_edge_action, _pcnt_config->neg_edge_action));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel, _pcnt_config->high_level_action, _pcnt_config->low_level_action));
    ESP_ERROR_CHECK(pcnt_unit_enable(pcnt_unit));
}

EncoderPulseReader::~EncoderPulseReader()
{
    ESP_ERROR_CHECK(pcnt_unit_disable(pcnt_unit));
    ESP_ERROR_CHECK(pcnt_del_channel(pcnt_channel));
    ESP_ERROR_CHECK(pcnt_del_unit(pcnt_unit));
}

bool EncoderPulseReader::start_pulse_counter()
{
    ESP_ERROR_CHECK(pcnt_unit_start(pcnt_unit));
    return true;
}

bool EncoderPulseReader::stop_pulse_counter()
{
    ESP_ERROR_CHECK(pcnt_unit_stop(pcnt_unit));
    return true;
}

void EncoderPulseReader::get_pulse_count(int *count)
{
    pcnt_unit_get_count(pcnt_unit, count);
}

void EncoderPulseReader::clear_pulse_count()
{
    esp_err_t err = ESP_OK;
    err = pcnt_unit_clear_count(pcnt_unit);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to clear pulse count: %s", esp_err_to_name(err));
    }
}

encoder_ticks_t EncoderPulseReader::get_ticks()
{
    int current_count;
    pcnt_unit_get_count(pcnt_unit, &current_count);
    return current_count;
}

encoder_tickrate_t EncoderPulseReader::get_tickrate()
{
    timestamp_t timestamp;
    return get_ticks(timestamp);
}

encoder_ticks_t EncoderPulseReader::get_ticks(timestamp_t &timestamp)
{
    timestamp = esp_timer_get_time();
    return get_ticks();
}

encoder_tickrate_t EncoderPulseReader::get_tickrate(timestamp_t &timestamp)
{
    int current_count;
    pcnt_unit_get_count(pcnt_unit, &current_count);

    timestamp = esp_timer_get_time();
    float delta_time_us = timestamp - previous_time_us;

    encoder_tickrate_t tickrate = (current_count - previous_count) / delta_time_us;

    previous_count = current_count;
    previous_time_us = timestamp;

    return tickrate;
}

void EncoderPulseReader::get_tick_tickrate(encoder_ticks_t &ticks, encoder_tickrate_t &tickrate, timestamp_t &timestamp)
{
    pcnt_unit_get_count(pcnt_unit, &ticks);

    timestamp = esp_timer_get_time();
    float delta_time_us = timestamp - previous_time_us;

    tickrate = (ticks - previous_count) / delta_time_us;

    previous_count = ticks;
    previous_time_us = timestamp;
}
