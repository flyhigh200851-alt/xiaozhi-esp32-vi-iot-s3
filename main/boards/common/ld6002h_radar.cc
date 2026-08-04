#include "ld6002h_radar.h"
#include <esp_log.h>
#include <driver/uart.h>
#include <driver/gpio.h>
#include <cstring>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

#define TAG "LD6002H"
#define RADAR_UART_BUF_SIZE 256
#define RADAR_FRAME_SIZE 15
#define RADAR_BAUD 115200

LD6002HRadar::LD6002HRadar(int tx_pin, int rx_pin)
    : tx_pin_(tx_pin), rx_pin_(rx_pin), uart_port_num_(UART_NUM_1) {}

LD6002HRadar::~LD6002HRadar() { Stop(); }

void LD6002HRadar::Start() {
    if (running_) return;
    
    uart_config_t uart_config = {
        .baud_rate = RADAR_BAUD,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
    };
    
    ESP_ERROR_CHECK(uart_param_config(uart_port_num_, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(uart_port_num_, tx_pin_, rx_pin_, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));
    ESP_ERROR_CHECK(uart_driver_install(uart_port_num_, RADAR_UART_BUF_SIZE, 0, 0, NULL, 0));
    
    running_ = true;
    xTaskCreate(RadarTask, "radar_task", 4096, this, 5, NULL);
    ESP_LOGI(TAG, "Radar started on UART1 (TX:%d, RX:%d)", tx_pin_, rx_pin_);
}

void LD6002HRadar::Stop() {
    running_ = false;
    uart_driver_delete(uart_port_num_);
}

void LD6002HRadar::RadarTask(void* arg) {
    auto* self = static_cast<LD6002HRadar*>(arg);
    self->ReadLoop();
}

void LD6002HRadar::ReadLoop() {
    while (running_) {
        int len = uart_read_bytes(uart_port_num_, rx_buffer_ + rx_len_, 
                                   sizeof(rx_buffer_) - rx_len_, pdMS_TO_TICKS(100));
        if (len > 0) rx_len_ += len;
        
        while (rx_len_ >= RADAR_FRAME_SIZE) {
            // Find frame header AA 55
            int header_pos = -1;
            for (int i = 0; i < rx_len_ - 1; i++) {
                if (rx_buffer_[i] == 0xAA && rx_buffer_[i+1] == 0x55) {
                    header_pos = i;
                    break;
                }
            }
            if (header_pos < 0) { rx_len_ = 0; break; }
            if (header_pos > 0) {
                memmove(rx_buffer_, rx_buffer_ + header_pos, rx_len_ - header_pos);
                rx_len_ -= header_pos;
            }
            if (rx_len_ < RADAR_FRAME_SIZE) break;
            
            // Check footer 0D 0A
            if (rx_buffer_[13] == 0x0D && rx_buffer_[14] == 0x0A) {
                RadarData data;
                if (ParseFrame(rx_buffer_, RADAR_FRAME_SIZE, data)) {
                    if (data_callback_) data_callback_(data);
                }
                memmove(rx_buffer_, rx_buffer_ + RADAR_FRAME_SIZE, rx_len_ - RADAR_FRAME_SIZE);
                rx_len_ -= RADAR_FRAME_SIZE;
            } else {
                // Bad frame, skip first byte
                memmove(rx_buffer_, rx_buffer_ + 1, rx_len_ - 1);
                rx_len_--;
            }
        }
        
        if (rx_len_ > sizeof(rx_buffer_) - RADAR_FRAME_SIZE) rx_len_ = 0;
    }
}

bool LD6002HRadar::ParseFrame(const uint8_t* data, int len, RadarData& result) {
    if (len < RADAR_FRAME_SIZE) return false;
    if (data[0] != 0xAA || data[1] != 0x55) return false;
    if (data[13] != 0x0D || data[14] != 0x0A) return false;
    
    result.person = (data[2] == 0x01);
    result.resp_rate = data[3];
    result.heart_rate = data[4];
    result.distance_raw = data[5];
    result.distance_min = (data[5] > 0) ? (data[5] - 1) * 15 : 0;
    result.distance_max = data[5] * 15;
    result.motion = (data[6] == 0x01);
    result.motion_mag = (uint16_t)data[7] | ((uint16_t)data[8] << 8);
    result.valid = true;
    result.last_update = (uint32_t)(esp_timer_get_time() / 1000);
    return true;
}

RadarData LD6002HRadar::GetLatestData() const { return RadarData(); }
