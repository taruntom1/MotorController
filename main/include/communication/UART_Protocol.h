/**
 * @file UARTProtocol.h
 * @brief Header file for the UARTProtocol class, providing UART communication utilities.
 */

#ifndef UARTPROTOCOL_H
#define UARTPROTOCOL_H

#include <stdint.h>
#include "driver/uart.h"
#include "esp_log.h"
#include <functional>
#include <vector>
#include <stdexcept>
#include <string>
#include "communication/Commands.h"

/**
 * @brief Base exception class for UART protocol errors
 */
class UARTException : public std::runtime_error
{
public:
    explicit UARTException(const std::string &message) : std::runtime_error(message) {}
};

/**
 * @brief Exception thrown when UART initialization fails
 */
class UARTInitializationException : public UARTException
{
public:
    explicit UARTInitializationException(const std::string &message)
        : UARTException("UART Initialization Error: " + message) {}
};

/**
 * @brief Exception thrown when UART communication fails
 */
class UARTCommunicationException : public UARTException
{
public:
    explicit UARTCommunicationException(const std::string &message)
        : UARTException("UART Communication Error: " + message) {}
};

/**
 * @brief Exception thrown when UART timeout occurs
 */
class UARTTimeoutException : public UARTException
{
public:
    explicit UARTTimeoutException(const std::string &message)
        : UARTException("UART Timeout Error: " + message) {}
};

/**
 * @brief Exception thrown when invalid configuration is provided
 */
class UARTConfigurationException : public UARTException
{
public:
    explicit UARTConfigurationException(const std::string &message)
        : UARTException("UART Configuration Error: " + message) {}
};

/**
 * @struct protocol_config
 * @brief Configuration structure for UARTProtocol.
 *
 * This structure holds the configuration parameters required to initialize
 * the UARTProtocol class, such as baud rate, UART port, buffer size, and pin settings.
 */
struct protocol_config
{
    int baudRate = 115200;         /**< Baud rate for UART communication. */
    uart_port_t port = UART_NUM_1; /**< UART port number. */
    uint16_t bufferSize = 2048;    /**< Size of the UART buffer. */
    uint8_t pinRX = 18;            /**< RX pin number. */
    uint8_t pinTX = 17;            /**< TX pin number. */
    uint8_t header = 0xAA;         /**< Header byte for the protocol. */
    uint8_t maxPacketSize = 100;   /**< Maximum size of a data packet. */
};

/**
 * @class UARTProtocol
 * @brief Provides a protocol layer for UART communication.
 *
 * The UARTProtocol class encapsulates UART communication functionality, including
 * sending and receiving commands and data, with header and checksum validation.
 */
class UARTProtocol
{
private:
    protocol_config config; /**< Configuration settings for the protocol. */
    uint8_t header;         /**< Header byte for the protocol. */
    uint8_t maxPacketSize;  /**< Maximum size of a data packet. */
    const char *TAG = "UART";
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .rx_flow_ctrl_thresh = 0,
        .source_clk = UART_SCLK_APB,                       // Ensure all fields are initialized
        .flags = {.allow_pd = 0, .backup_before_sleep = 0} // Flags for power management
    };
    QueueHandle_t uart_queue;
    static void uartTask(void *arg);

    std::array<uint8_t, 4> commandArr = {header, header, header, 0x00}; /**< Array to hold command bytes, initialized with the header. */

    void processEvents();

    void processCommand(std::array<uint8_t, 128U> &data, int pattern_pos);

public:
    std::function<void(int)> onEvent;

    /**
     * @brief Constructs a UARTProtocol object.
     * @param config Configuration settings for the protocol.
     */
    explicit UARTProtocol(const protocol_config &config);

    /**
     * @brief Initializes the UART protocol with the specified configuration.
     */
    void begin();

    /**
     * @brief Sends a command over UART.
     * @param commandType The type of command to send.
     */
    void SendCommand(const Command commandType);
    /**
     * @brief Sends data over UART.
     * @param data Pointer to the data array to send.
     * @param length Length of the data array.
     */
    void SendData(const std::vector<uint8_t> &data) const;
    void SendData(const uint8_t *data, const uint8_t length) const;
    /**
     * @brief Reads a command from UART with an timeout.
     * @param commandType Reference to store the received command type.
     * @param timeout Timeout duration in milliseconds (default is 1000 ms).
     * @return True if a command was successfully read, false otherwise.
     */
    bool ReadCommand(Command &commandType, uint32_t timeout = 1000) const;

    /**
     * @brief Callback function to handle received commands.
     *
     * This function is called when a command is received via UART.
     * The received command is passed as an 8-bit unsigned integer parameter.
     *
     * @param command The received command byte.
     */
    std::function<void(Command)> onCommandReceived;
    /**
     * @brief Reads data from UART with an optional timeout.
     * @param data Pointer to store the received data.
     * @param length Length of the data to read.
     * @param timeout Timeout duration in milliseconds (default is 1000 ms).
     * @return True if data was successfully read, false otherwise.
     */
    bool ReadData(uint8_t *data, uint8_t length, uint32_t timeout = 1000) const;
    std::vector<uint8_t> ReadData(uint8_t length, uint32_t timeout = 1000) const;
};

#endif // UART_PROTOCOL_H
