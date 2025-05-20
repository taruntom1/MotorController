#pragma once

#include <cstdint>
#include <cstddef>

class UARTProtocol; // Forward declaration to avoid circular dependency

// Computes CRC-16-CCITT (0x1021)
uint16_t crc16_ccitt(const uint8_t* data, size_t length);

// Handles time synchronization request from UART
void handleTimeSyncRequest(UARTProtocol* protocol);
