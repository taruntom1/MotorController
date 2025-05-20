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

angle_t EncoderPulseReader::get_raw_angle()
{
    int current_count;
    pcnt_unit_get_count(pcnt_unit, &current_count);
    angle_t angle = static_cast<float>(current_count);
    return angle;
}

angularvelocity_t EncoderPulseReader::get_raw_velocity()
{
    int current_count;
    uint64_t current_time = esp_timer_get_time();
    pcnt_unit_get_count(pcnt_unit, &current_count);
    angularvelocity_t velocity = (current_count - previous_count) / static_cast<float>(current_time - previous_time);
    previous_count = current_count;
    previous_time = current_time;
    return velocity;
}
