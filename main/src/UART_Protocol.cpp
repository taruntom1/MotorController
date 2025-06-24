#include "UART_Protocol.h"

#include <utility>

#define LOG_LOCAL_LEVEL ESP_LOG_INFO // Set local log level for this file

UARTProtocol::UARTProtocol(const protocol_config &config)
    : config{config},
      header{config.header},
      maxPacketSize{config.maxPacketSize}
{
    // Validate configuration parameters
    if (config.baudRate <= 0)
    {
        throw UARTConfigurationException("Invalid baud rate: " + std::to_string(config.baudRate));
    }

    if (config.bufferSize == 0 || config.bufferSize > 8192)
    {
        throw UARTConfigurationException("Invalid buffer size: " + std::to_string(config.bufferSize));
    }

    if (config.maxPacketSize == 0 || config.maxPacketSize > 255)
    {
        throw UARTConfigurationException("Invalid max packet size: " + std::to_string(config.maxPacketSize));
    }

    if (config.port < UART_NUM_0 || config.port >= UART_NUM_MAX)
    {
        throw UARTConfigurationException("Invalid UART port: " + std::to_string(config.port));
    }
}

void UARTProtocol::begin()
{
    try
    {
        uart_config.baud_rate = config.baudRate;

        esp_err_t err = uart_param_config(config.port, &uart_config);
        if (err != ESP_OK)
        {
            throw UARTInitializationException("Failed to configure UART parameters: " + std::string(esp_err_to_name(err)));
        }

        err = uart_set_pin(config.port, config.pinTX, config.pinRX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
        if (err != ESP_OK)
        {
            throw UARTInitializationException("Failed to set UART pins: " + std::string(esp_err_to_name(err)));
        }

        err = uart_driver_install(config.port, config.bufferSize, config.bufferSize, 20, &uart_queue, 0);
        if (err != ESP_OK)
        {
            throw UARTInitializationException("Failed to install UART driver: " + std::string(esp_err_to_name(err)));
        }

        err = uart_enable_pattern_det_baud_intr(config.port, 0xAA, 3, 9, 0, 0);
        if (err != ESP_OK)
        {
            throw UARTInitializationException("Failed to enable pattern detection: " + std::string(esp_err_to_name(err)));
        }

        err = uart_pattern_queue_reset(config.port, 20);
        if (err != ESP_OK)
        {
            throw UARTInitializationException("Failed to reset pattern queue: " + std::string(esp_err_to_name(err)));
        }

        BaseType_t taskResult = xTaskCreate(&UARTProtocol::uartTask, "uart_task", 3700, this, 10, nullptr);
        if (taskResult != pdPASS)
        {
            throw UARTInitializationException("Failed to create UART task");
        }

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_INFO, TAG, "UART initialized successfully");
    }
    catch (const UARTInitializationException &)
    {
        // Clean up any partially initialized resources
        uart_driver_delete(config.port);
        throw; // Re-throw the exception
    }
    catch (const std::bad_alloc &)
    {
        // Handle memory allocation failures
        uart_driver_delete(config.port);
        throw UARTInitializationException("Memory allocation failed during UART initialization");
    }
    catch (...)
    {
        // Handle any other unexpected exceptions
        uart_driver_delete(config.port);
        throw UARTInitializationException("Unexpected error during UART initialization");
    }
}

void UARTProtocol::uartTask(void *arg)
{
    auto *self = static_cast<UARTProtocol *>(arg);
    self->processEvents();
}

void UARTProtocol::processEvents()
{
    uart_event_t event;
    std::array<uint8_t, 128> data; /**< Buffer for received data. */

    try
    {
        while (xQueueReceive(uart_queue, &event, portMAX_DELAY))
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "UART event received Event type: %d", event.type);

            if (event.type == UART_PATTERN_DET)
            {
                try
                {
                    int pattern_pos = uart_pattern_pop_pos(config.port);
                    if (pattern_pos >= 0)
                    {
                        processCommand(data, pattern_pos);
                    }
                    else
                    {
                        uart_flush_input(config.port);
                        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Pattern pop failed. Input flushed.");
                    }
                }
                catch (const std::exception &e)
                {
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Exception processing UART pattern: %s", e.what());
                    uart_flush_input(config.port);
                }
                catch (...)
                {
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Unknown exception processing UART pattern");
                    uart_flush_input(config.port);
                }
            }
        }
    }
    catch (const std::exception &e)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Exception in UART event processing: %s", e.what());
    }
    catch (...)
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Unknown exception in UART event processing");
    }

    vTaskDelete(nullptr);
}

void UARTProtocol::processCommand(std::array<uint8_t, 128U> &data, int pattern_pos)
{
    int read_len = uart_read_bytes(config.port, data.data(), pattern_pos + 4, pdMS_TO_TICKS(20));
    if (read_len >= pattern_pos + 3)
    {
        uint8_t cmd = data[pattern_pos + 3];
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Command received: 0x%02X", cmd);

        // Call the callback safely
        if (onCommandReceived)
        {
            try
            {
                onCommandReceived(static_cast<Command>(cmd));
            }
            catch (const std::exception &e)
            {
                ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Exception in command callback: %s", e.what());
            }
            catch (...)
            {
                ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Unknown exception in command callback");
            }
        }
    }
    else
    {
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_DEBUG, TAG, "Partial read: %d bytes", read_len);
    }
}

void UARTProtocol::SendCommand(Command command)
{
    try
    {
        commandArr[3] = std::to_underlying(command);

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Sending command: 0x%02X", static_cast<uint8_t>(command));

        int bytes_written = uart_write_bytes(config.port, commandArr.data(), 4);
        if (bytes_written != 4)
        {
            throw UARTCommunicationException("Failed to send complete command. Sent " +
                                             std::to_string(bytes_written) + " of 4 bytes");
        }
    }
    catch (const std::bad_alloc &)
    {
        throw UARTCommunicationException("Memory allocation failed while sending command");
    }
}

void UARTProtocol::SendData(const std::vector<uint8_t> &data) const
{
    try
    {
        if (data.empty())
        {
            throw UARTCommunicationException("Cannot send empty data");
        }

        if (data.size() > maxPacketSize)
        {
            throw UARTCommunicationException("Data size (" + std::to_string(data.size()) +
                                             ") exceeds maximum packet size (" + std::to_string(maxPacketSize) + ")");
        }

        int bytes_written = uart_write_bytes(config.port, data.data(), data.size());
        if (bytes_written != static_cast<int>(data.size()))
        {
            throw UARTCommunicationException("Failed to send complete data. Sent " +
                                             std::to_string(bytes_written) + " of " + std::to_string(data.size()) + " bytes");
        }

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Sent %hhu bytes of data", data.size());
    }
    catch (const std::bad_alloc &)
    {
        throw UARTCommunicationException("Memory allocation failed while sending data");
    }
}

void UARTProtocol::SendData(const uint8_t *data, const uint8_t length) const
{
    try
    {
        if (data == nullptr)
        {
            throw UARTCommunicationException("Cannot send null data pointer");
        }

        if (length == 0)
        {
            throw UARTCommunicationException("Cannot send zero-length data");
        }

        if (length > maxPacketSize)
        {
            throw UARTCommunicationException("Data length (" + std::to_string(length) +
                                             ") exceeds maximum packet size (" + std::to_string(maxPacketSize) + ")");
        }

        int bytes_written = uart_write_bytes(config.port, data, length);
        if (bytes_written != length)
        {
            throw UARTCommunicationException("Failed to send complete data. Sent " +
                                             std::to_string(bytes_written) + " of " + std::to_string(length) + " bytes");
        }

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Sent %hhu bytes of data", length);
    }
    catch (const std::bad_alloc &)
    {
        throw UARTCommunicationException("Memory allocation failed while sending data");
    }
}

bool UARTProtocol::ReadCommand(Command &command, uint32_t timeout_ms) const
{
    try
    {
        uint32_t tickCount = xTaskGetTickCount();
        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Attempting to read command with timeout: %lu ms", timeout_ms);

        while (xTaskGetTickCount() - tickCount < pdMS_TO_TICKS(timeout_ms))
        {
            while (uart_read_bytes(config.port, &command, 1, 0) > 0)
            {
                if (std::to_underlying(command) == header)
                {
                    ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Header received");
                    int bytes_read = uart_read_bytes(config.port, &command, 1, pdMS_TO_TICKS(10));
                    if (bytes_read > 0)
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
    catch (const std::bad_alloc &)
    {
        throw UARTCommunicationException("Memory allocation failed while reading command");
    }
    catch (...)
    {
        throw UARTCommunicationException("Unexpected error while reading command");
    }
}

std::vector<uint8_t> UARTProtocol::ReadData(uint8_t length, uint32_t timeout_ms) const
{
    try
    {
        if (length == 0)
        {
            throw UARTCommunicationException("Cannot read zero-length data");
        }

        if (length > maxPacketSize)
        {
            throw UARTCommunicationException("Requested data length (" + std::to_string(length) +
                                             ") exceeds maximum packet size (" + std::to_string(maxPacketSize) + ")");
        }

        std::vector<uint8_t> data(length);
        int bytes_read = uart_read_bytes(config.port, data.data(), length, pdMS_TO_TICKS(timeout_ms));

        if (bytes_read != length) [[unlikely]]
        {
            if (bytes_read < 0)
            {
                throw UARTCommunicationException("UART read error occurred");
            }
            else
            {
                throw UARTTimeoutException("Timeout while reading data. Read " +
                                           std::to_string(bytes_read) + " of " + std::to_string(length) + " bytes");
            }
        }

        ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Successfully received %hhu bytes of data", length);
        return data;
    }
    catch (const std::bad_alloc &)
    {
        throw UARTCommunicationException("Memory allocation failed while reading data");
    }
}

bool UARTProtocol::ReadData(uint8_t *data, uint8_t length, uint32_t timeout_ms) const
{
    try
    {
        if (data == nullptr)
        {
            throw UARTCommunicationException("Cannot read into null data pointer");
        }

        if (length == 0)
        {
            throw UARTCommunicationException("Cannot read zero-length data");
        }

        if (length > maxPacketSize)
        {
            throw UARTCommunicationException("Requested data length (" + std::to_string(length) +
                                             ") exceeds maximum packet size (" + std::to_string(maxPacketSize) + ")");
        }

        int bytes_read = uart_read_bytes(config.port, data, length, pdMS_TO_TICKS(timeout_ms));

        if (bytes_read == length) [[likely]]
        {
            ESP_LOG_LEVEL_LOCAL(ESP_LOG_VERBOSE, TAG, "Successfully received %hhu bytes of data", length);
            return true;
        }
        else
        {
            if (bytes_read < 0)
            {
                throw UARTCommunicationException("UART read error occurred");
            }
            else
            {
                ESP_LOG_LEVEL_LOCAL(ESP_LOG_ERROR, TAG, "Timeout while reading data. Read %d of %hhu bytes", bytes_read, length);
                return false;
            }
        }
    }
    catch (const UARTException &)
    {
        throw; // Re-throw UART-specific exceptions
    }
    catch (const std::bad_alloc &)
    {
        throw UARTCommunicationException("Memory allocation failed while reading data");
    }
    catch (...)
    {
        throw UARTCommunicationException("Unexpected error while reading data");
    }
}
