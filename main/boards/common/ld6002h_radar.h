#pragma once

#include <cstdint>
#include <functional>

struct RadarData {
    bool person = false;
    uint8_t resp_rate = 0;
    uint8_t heart_rate = 0;
    uint8_t distance_raw = 0;
    uint16_t distance_min = 0;
    uint16_t distance_max = 0;
    bool motion = false;
    uint16_t motion_mag = 0;
    bool valid = false;
    uint32_t last_update = 0;
};

class LD6002HRadar {
public:
    LD6002HRadar(int tx_pin, int rx_pin);
    ~LD6002HRadar();
    void Start();
    void Stop();
    RadarData GetLatestData() const;
    using DataCallback = std::function<void(const RadarData&)>;
    void OnData(DataCallback cb) { data_callback_ = cb; }
private:
    int tx_pin_, rx_pin_, uart_port_num_;
    bool running_ = false;
    DataCallback data_callback_;
    uint8_t rx_buffer_[32];
    int rx_len_ = 0;
    static void RadarTask(void* arg);
    void ReadLoop();
    bool ParseFrame(const uint8_t* data, int len, RadarData& result);
};
