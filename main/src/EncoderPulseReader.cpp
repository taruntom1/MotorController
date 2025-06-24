#include "EncoderPulseReader.h"

EncoderPulseReader::EncoderPulseReader(pcnt_config_t *_pcnt_config)
    : _pcnt_config(_pcnt_config),
      inverse_cycles_per_sec(1.0f / static_cast<float>(esp_rom_get_cpu_ticks_per_us() * 1000000))
{
    ESP_ERROR_CHECK(pcnt_new_unit(&_pcnt_config->unit_config, &pcnt_unit));
    ESP_ERROR_CHECK(pcnt_new_channel(pcnt_unit, &_pcnt_config->channel_config, &pcnt_channel));
    ESP_ERROR_CHECK(pcnt_channel_set_edge_action(pcnt_channel, _pcnt_config->pos_edge_action, _pcnt_config->neg_edge_action));
    ESP_ERROR_CHECK(pcnt_channel_set_level_action(pcnt_channel, _pcnt_config->high_level_action, _pcnt_config->low_level_action));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, _pcnt_config->unit_config.high_limit));
    ESP_ERROR_CHECK(pcnt_unit_add_watch_point(pcnt_unit, _pcnt_config->unit_config.low_limit));
    ESP_ERROR_CHECK(pcnt_unit_clear_count(pcnt_unit));
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

void EncoderPulseReader::clear_pulse_count()
{
    esp_err_t err = ESP_OK;
    err = pcnt_unit_clear_count(pcnt_unit);
    if (err != ESP_OK)
    {
        ESP_LOGW(TAG, "Failed to clear pulse count: %s", esp_err_to_name(err));
    }
    previous_count = 0;
}

encoder_ticks_t EncoderPulseReader::get_ticks()
{
    int hw_count = 0;
    pcnt_unit_get_count(pcnt_unit, &hw_count);
    return hw_count;
}

encoder_tickrate_t EncoderPulseReader::get_tickrate()
{
    int current_count = get_ticks();
    uint32_t current_cpu_cycles = esp_cpu_get_cycle_count();
    float delta_time_us = static_cast<float>(current_cpu_cycles - previous_cpu_cycles) * inverse_cycles_per_sec;

    encoder_tickrate_t tickrate = static_cast<float>(current_count - previous_count) / delta_time_us;

    previous_count = current_count;
    previous_cpu_cycles = current_cpu_cycles;

    return tickrate;
}

void EncoderPulseReader::get_tick_tickrate(encoder_ticks_t &ticks, encoder_tickrate_t &tickrate)
{
    ticks = get_ticks();
    uint32_t current_cpu_cycles = esp_cpu_get_cycle_count();
    float delta_time_us = static_cast<float>(current_cpu_cycles - previous_cpu_cycles) * inverse_cycles_per_sec;

    tickrate = static_cast<float>(ticks - previous_count) / delta_time_us;

    previous_count = ticks;
    previous_cpu_cycles = current_cpu_cycles;
}
