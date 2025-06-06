#include "TimeSyncServer.h"

#include <vector>
#include "esp_timer.h"
#include "Commands.h"
#include "UART_Protocol.h"

uint16_t crc16_ccitt(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFF;
    for (size_t i = 0; i < length; ++i)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (int j = 0; j < 8; ++j)
            crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
    }
    return crc;
}

void handleTimeSyncRequest(UARTProtocol *protocol)
{
    uint64_t t1 = esp_timer_get_time(); // microseconds

    std::vector<uint8_t> response;
    response.reserve(14);
    constexpr uint8_t header[4] = {0xAA, 0xAA, 0xAA, static_cast<uint8_t>(Command::SYNC_TIME)};
    response.insert(response.begin(), header, header + 4);

    response.insert(response.end(), reinterpret_cast<const uint8_t *>(&t1), reinterpret_cast<const uint8_t *>(&t1) + 8);

    uint16_t resp_crc = crc16_ccitt(response.data() + 4, 8);
    response.insert(response.end(), reinterpret_cast<uint8_t *>(&resp_crc), reinterpret_cast<uint8_t *>(&resp_crc) + 2);
    protocol->SendData(response);
}
