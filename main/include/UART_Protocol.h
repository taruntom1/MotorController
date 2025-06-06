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
#include "Commands.h"

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
        .source_clk = UART_SCLK_APB, // Ensure all fields are initialized
    };
    QueueHandle_t uart_queue;
    static void uartTask(void *arg);
    void processEvents();

public:
    std::function<void(int)> onEvent;

    /**
     * @brief Constructs a UARTProtocol object.
     * @param config Configuration settings for the protocol.
     */
    UARTProtocol(protocol_config config);

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
    void SendData(const std::vector<uint8_t>& data);
    void SendData(const uint8_t *data, const uint8_t length);

    /**
     * @brief Reads a command from UART with an timeout.
     * @param commandType Reference to store the received command type.
     * @param timeout Timeout duration in milliseconds (default is 1000 ms).
     * @return True if a command was successfully read, false otherwise.
     */
    bool ReadCommand(Command &commandType, uint32_t timeout = 1000);

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
    bool ReadData(uint8_t *data, uint8_t length, uint32_t timeout = 1000);
    std::vector<uint8_t> ReadData(uint8_t length, uint32_t timeout = 1000);
};

#endif // UART_PROTOCOL_H
