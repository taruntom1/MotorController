#include "UART_Protocol.h"

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

UARTProtocol::UARTProtocol(protocol_config config)
    : config{config},
      header{config.header},
      maxPacketSize{config.maxPacketSize} {}

void UARTProtocol::begin()
{
    uart_config.baud_rate = config.baudRate;

    ESP_ERROR_CHECK(uart_param_config(config.port, &uart_config));

    ESP_ERROR_CHECK(uart_set_pin(config.port, config.pinTX, config.pinRX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_ERROR_CHECK(uart_driver_install(config.port, config.bufferSize, config.bufferSize, 20, &uart_queue, 0));

    ESP_ERROR_CHECK(uart_enable_pattern_det_baud_intr(config.port, 0xAA, 3, 9, 0, 0));

    ESP_ERROR_CHECK(uart_pattern_queue_reset(config.port, 20));

    xTaskCreate(&UARTProtocol::uartTask, "uart_task", 3700, this, 10, NULL);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "UART initialized successfully");
}

void UARTProtocol::uartTask(void *arg)
{
    UARTProtocol *self = static_cast<UARTProtocol *>(arg);
    self->processEvents();
}

void UARTProtocol::processEvents()
{
    uart_event_t event;
    uint8_t data[128];

    while (xQueueReceive(uart_queue, &event, portMAX_DELAY))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "UART event received Event type: %d", event.type);

        if (event.type == UART_PATTERN_DET)
        {
            int pattern_pos = uart_pattern_pop_pos(config.port);
            if (pattern_pos >= 0)
            {
                int read_len = uart_read_bytes(config.port, data, pattern_pos + 4, pdMS_TO_TICKS(20));
                if (read_len >= pattern_pos + 3)
                {
                    uint8_t cmd = data[pattern_pos + 3];
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Command received: 0x%02X", cmd);
                    onCommandReceived(static_cast<Command>(cmd));
                }
                else
                {
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Partial read: %d bytes", read_len);
                }
            }
            else
            {
                uart_flush_input(config.port);
                ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Pattern pop failed. Input flushed.");
            }
        }
    }

    vTaskDelete(NULL);
}

void UARTProtocol::SendCommand(Command command)
{
    static uint8_t commandArr[4] = {header, header, header, 0x00};

    commandArr[3] = static_cast<uint8_t>(command);

    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Sending command: 0x%02X", static_cast<uint8_t>(command));

    uart_write_bytes(config.port, reinterpret_cast<const char *>(commandArr), 4);
}

void UARTProtocol::SendData(const std::vector<uint8_t> &data)
{
    uart_write_bytes(config.port, data.data(), data.size());
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Sent %hhu bytes of data", data.size());
}

void UARTProtocol::SendData(const uint8_t *data, const uint8_t length)
{
    uart_write_bytes(config.port, data, length);
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Sent %hhu bytes of data", length);
}

bool UARTProtocol::ReadCommand(Command &command, uint32_t timeout_ms)
{
    uint32_t tickCount;
    tickCount = xTaskGetTickCount();
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Attempting to read command with timeout: %lu ms", timeout_ms);
    while (xTaskGetTickCount() - tickCount < pdMS_TO_TICKS(timeout_ms))
    {
        while (uart_read_bytes(config.port, &command, 1, 0) > 0)
        {
            if (static_cast<uint8_t>(command) == header)
            {
                ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Header received");
                if (uart_read_bytes(config.port, &command, 1, pdMS_TO_TICKS(10)) > 0)
                {
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Received command: 0x%02X", static_cast<uint8_t>(command));
                    return true;
                }
                else
                {
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Timeout while reading command");
                    return false;
                }
            }
        }
    }
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "No header received within timeout");
    return false;
}

std::vector<uint8_t> UARTProtocol::ReadData(uint8_t length, uint32_t timeout_ms)
{
    std::vector<uint8_t> data(length);
    if (unlikely(uart_read_bytes(config.port, data.data(), length, pdMS_TO_TICKS(timeout_ms)) != length))
        data.clear();
    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Successfully received %hhu bytes of data", length);
    return data;
}

bool UARTProtocol::ReadData(uint8_t *data, uint8_t length, uint32_t timeout_ms)
{
    if (likely(uart_read_bytes(config.port, data, length, pdMS_TO_TICKS(timeout_ms)) == length))
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Successfully received %hhu bytes of data", length);
        return true;
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Timeout while reading data");
        return false;
    }
}
