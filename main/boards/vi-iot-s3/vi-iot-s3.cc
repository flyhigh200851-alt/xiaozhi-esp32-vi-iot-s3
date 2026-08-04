#include "wifi_board.h"
#include "sensors.h"
#include "driver/rmt_tx.h"
#include "esp_http_server.h"
#include "websocket_control_server.h"
#include "driver/rmt_encoder.h"
#include <cstring>
#include "audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "pin_config.h"
#include "esp_timer.h"
#include <inttypes.h>

/* ============ PCA9685 I2C 电机驱动 ============ */
#define PCA9685_ADDR        0x40
#define PCA9685_MODE1       0x00
#define PCA9685_PRE_SCALE   0xFE
#define PCA9685_LED0_ON_L   0x06

#define PCA9685_FREQ        1000  /* Hz */

/* 电机引脚映射（PCA9685通道 -> TB6612功能）*/
/* 前轮 - TB6612 #1 */
#define MOTOR_LF_PWM   0   /* CH0 -> PWMA 左前速度 */
#define MOTOR_RF_PWM   1   /* CH1 -> PWMB 右前速度 */
#define MOTOR_LF_DIR1  2   /* CH2 -> AIN1 左前方向 */
#define MOTOR_LF_DIR2  3   /* CH3 -> AIN2 左前方向 */
#define MOTOR_RF_DIR1  10  /* CH10 -> BIN1 右前方向 */
#define MOTOR_RF_DIR2  11  /* CH11 -> BIN2 右前方向 */
/* 后轮 - TB6612 #2 */
#define MOTOR_LR_PWM   6   /* CH6 -> PWMA 左后速度 */
#define MOTOR_RR_PWM   7   /* CH7 -> PWMB 右后速度 */
#define MOTOR_LR_DIR1  8   /* CH8 -> AIN1 左后方向 */
#define MOTOR_LR_DIR2  9   /* CH9 -> AIN2 左后方向 */
#define MOTOR_RR_DIR1  12  /* CH12 -> BIN1 右后方向 */
#define MOTOR_RR_DIR2  13  /* CH13 -> BIN2 右后方向 */
#include "config.h"
#include "display.h"
#include "wifi_cmd_server.h"

#include <esp_log.h>
#include <driver/i2c_master.h>
#include <driver/gpio.h>
#include <esp_lcd_panel_io.h>
#include <esp_lcd_panel_ops.h>
#include <esp_lcd_panel_vendor.h>

extern "C" {
#include "xl9555.h"
void xl9555_init_with_bus(i2c_master_bus_handle_t bus);
}

#include "codecs/no_audio_codec.h"
#include "settings.h"
#include "motor_controller.h"
#include <esp_system.h>
#include <driver/uart.h>
#include <esp_rom_sys.h>
#include <esp_timer.h>

#define TAG "VI-IOT-S3"

/* ============ PCA9685 驱动 ============ */
#define PCA9685_ADDR        0x40
#define PCA9685_MODE1       0x00
#define PCA9685_PRE_SCALE   0xFE
#define PCA9685_LED0_ON_L   0x06
#define PCA9685_FREQ        1000

/* 电机映射 */
#define MOTOR_LF_PWM   0
#define MOTOR_RF_PWM   1
#define MOTOR_LF_DIR1  2
#define MOTOR_LF_DIR2  3
#define MOTOR_RF_DIR1  10
#define MOTOR_RF_DIR2  11
#define MOTOR_LR_PWM   6
#define MOTOR_RR_PWM   7
#define MOTOR_LR_DIR1  8
#define MOTOR_LR_DIR2  9
#define MOTOR_RR_DIR1  12
#define MOTOR_RR_DIR2  13

static i2c_master_dev_handle_t pca9685_dev = NULL;

static void pca9685_write_reg(uint8_t reg, uint8_t val) {
    uint8_t d[2] = {reg, val};
    i2c_master_transmit(pca9685_dev, d, 2, 100);
}

static void pca9685_set_pwm(uint8_t ch, uint16_t on, uint16_t off) {
    uint8_t d[5]; d[0] = PCA9685_LED0_ON_L + 4*ch;
    d[1]=on&0xFF; d[2]=(on>>8)&0xFF; d[3]=off&0xFF; d[4]=(off>>8)&0xFF;
    i2c_master_transmit(pca9685_dev, d, 5, 100);
}

static i2c_master_bus_handle_t pca9685_bus = NULL;
static void pca9685_init() {
    i2c_master_bus_config_t bcfg = {};
    bcfg.i2c_port = I2C_NUM_1;
    bcfg.sda_io_num = GPIO_NUM_19;
    bcfg.scl_io_num = GPIO_NUM_20;
    bcfg.clk_source = I2C_CLK_SRC_DEFAULT;
    bcfg.glitch_ignore_cnt = 7;
    bcfg.intr_priority = 0;
    bcfg.trans_queue_depth = 0;
    bcfg.flags.enable_internal_pullup = true;
    ESP_ERROR_CHECK(i2c_new_master_bus(&bcfg, &pca9685_bus));
    i2c_device_config_t cfg = {};
    cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
    cfg.device_address = PCA9685_ADDR;
    cfg.scl_speed_hz = 100000;
    i2c_master_bus_add_device(pca9685_bus, &cfg, &pca9685_dev);
    pca9685_write_reg(PCA9685_MODE1, 0x10);
    uint8_t ps = (uint8_t)(25000000/4096/PCA9685_FREQ-1);
    pca9685_write_reg(PCA9685_PRE_SCALE, ps);
    pca9685_write_reg(PCA9685_MODE1, 0xA0);  // 0xA0 = RESTART + AI enable
    vTaskDelay(pdMS_TO_TICKS(10));
    /* 回读 MODE1 确认 I2C 通信正常 */
    uint8_t mode1_val = 0;
    uint8_t reg = 0;
    i2c_master_transmit_receive(pca9685_dev, &reg, 1, &mode1_val, 1, 100);
    ESP_LOGI("MOTOR", "MODE1 after init = 0x%02X (expected 0xA0)", mode1_val);
    for (int c=0; c<16; c++) pca9685_set_pwm(c, 0, 0);
    ESP_LOGI("MOTOR", "PCA9685 ready");
}

/* motor_ch: {PWM_ch, dir_pin1, dir_pin2} - dir pins are XL9555 IO */
static const int motor_ch[4][3] = {
    {MOTOR_LF_PWM,MOTOR_LF_DIR1,MOTOR_LF_DIR2},
    {MOTOR_RF_PWM,MOTOR_RF_DIR1,MOTOR_RF_DIR2},
    {MOTOR_LR_PWM,MOTOR_LR_DIR1,MOTOR_LR_DIR2},
    {MOTOR_RR_PWM,MOTOR_RR_DIR1,MOTOR_RR_DIR2},
};

static __attribute__((unused)) void motor_stop() {
    for (int m=0; m<4; m++) {
        pca9685_set_pwm(motor_ch[m][1], 0, 0);
        pca9685_set_pwm(motor_ch[m][2], 0, 0);
        pca9685_set_pwm(motor_ch[m][0], 0, 0);
    }
}
static void motor_set(int id, int speed) {
    if(id<0||id>3) return;
    if(speed>100) speed=100;
    if(speed<-100) speed=-100;
    uint16_t pwm = (uint16_t)(abs(speed)*40.95f);
    pca9685_set_pwm(motor_ch[id][1], 0, speed>0?0:4095);
    pca9685_set_pwm(motor_ch[id][2], 0, speed>0?4095:0);
    /* 平滑起步：逐渐加速到目标值 */
    if(speed != 0 && pwm > 20) {
        int step = 20;
        while(step < pwm) {
            pca9685_set_pwm(motor_ch[id][0], 0, step);
            step = step * 3 / 2;
            if(step > pwm) step = pwm;
            vTaskDelay(pdMS_TO_TICKS(8));
        }
    }
    pca9685_set_pwm(motor_ch[id][0], 0, pwm);
    if(speed==0) {
        pca9685_set_pwm(motor_ch[id][1], 0, 0);
        pca9685_set_pwm(motor_ch[id][2], 0, 0);
    }
}
static void motor_all(int fl, int fr, int rl, int rr) {
    motor_set(0,fl); motor_set(1,fr); motor_set(2,rl); motor_set(3,rr);
}

/* 带屏幕提示的电机控制 */
struct CarStatus { int speed=0; const char* dir="停止"; const char* mode="语音"; const char* model="云小智"; int dist=-1; bool radar=false; char speech[256]={0}; };

CarStatus g_cs;
#define AUDIO_UDP_PORT 8887

/* 超声波 GPIO16 */
#define ULTRA_GPIO GPIO_NUM_16
#define RADAR_UART UART_NUM_2
#define RADAR_GPIO GPIO_NUM_18

static void radar_task(void*) {
    uart_config_t ucfg = {
        .baud_rate = 256000, .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE, .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_driver_install(RADAR_UART, 1024, 0, 0, NULL, 0);
    uart_param_config(RADAR_UART, &ucfg);
    uart_set_pin(RADAR_UART, UART_PIN_NO_CHANGE, RADAR_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uint8_t buf[32];
    while (1) {
        int len = uart_read_bytes(RADAR_UART, buf, sizeof(buf), pdMS_TO_TICKS(200));
        for (int i = 0; i < len - 7; i++) {
            if (buf[i]==0xF4 && buf[i+1]==0xF5 && buf[i+2]==0xF6 && buf[i+3]==0xF7 && buf[i+4]==0xAA) {
                g_cs.radar = (buf[i+6] != 0x00);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}
Sensors g_sensors;
SensorData g_sd;
bool g_exploring = false;

/* ─── Babycare 直连警示：ALERT1 闪灯 -> 超时探索；ALERT0 全部停止 ─── */
static bool g_alert_active = false;
static uint32_t g_alert_epoch = 0;
#define ALERT_TIMEOUT_SEC 60
#define ALERT_PATROL_SEC 300

static void alert_flash_task(void* arg) {
    uint32_t epoch = (uint32_t)(uintptr_t)arg;
    auto* led = Board::GetInstance().GetLed();
    auto* sl = dynamic_cast<SingleLed*>(led);
    while (g_alert_active && g_alert_epoch == epoch) {
        if (sl) sl->SetLedColor(255, 30, 0);
        vTaskDelay(pdMS_TO_TICKS(350));
        if (!g_alert_active || g_alert_epoch != epoch) break;
        if (sl) sl->SetLedColor(60, 0, 0);
        vTaskDelay(pdMS_TO_TICKS(350));
    }
    if (sl) sl->RestoreAutoLed();
    vTaskDelete(NULL);
}

static void alert_timeout_task(void* arg) {
    uint32_t epoch = (uint32_t)(uintptr_t)arg;
    vTaskDelay(pdMS_TO_TICKS(ALERT_TIMEOUT_SEC * 1000));
    if (g_alert_active && g_alert_epoch == epoch) {
        ESP_LOGI(TAG, "Alert timeout: search mode");
        g_exploring = true; g_cs.speed = 50; g_cs.dir = "找人"; motor_all(50, 50, 50, 50);
        vTaskDelay(pdMS_TO_TICKS(ALERT_PATROL_SEC * 1000));
        if (g_alert_active && g_alert_epoch == epoch) {
            ESP_LOGI(TAG, "Alert patrol cap: stop moving, LED keeps flashing");
            g_exploring = false; motor_stop(); g_cs.speed = 0; g_cs.dir = "警示";
        }
    }
    vTaskDelete(NULL);
}

static void car_alert_stop() {
    g_alert_active = false;
    ++g_alert_epoch;
    g_exploring = false;
    motor_stop(); g_cs.speed = 0; g_cs.dir = "停止";
    auto* led = Board::GetInstance().GetLed();
    if (auto* sl = dynamic_cast<SingleLed*>(led)) sl->RestoreAutoLed();
    ESP_LOGI(TAG, "ALERT stop: LED off, motors stopped");
}

static void car_alert_start() {
    car_alert_stop();
    uint32_t epoch = ++g_alert_epoch;
    g_alert_active = true;
    g_cs.speed = 0; g_cs.dir = "警示";
    xTaskCreate(alert_flash_task, "alert_flash", 3072, (void*)(uintptr_t)epoch, 3, NULL);
    xTaskCreate(alert_timeout_task, "alert_to", 3072, (void*)(uintptr_t)epoch, 3, NULL);
    ESP_LOGI(TAG, "ALERT start: LED flashing, timeout=%ds", ALERT_TIMEOUT_SEC);
}

static void motor_notify(const char* status, int fl, int fr, int rl, int rr) {
    g_cs.dir = status;
    motor_all(fl, fr, rl, rr);
}

static void read_mode1() {
    uint8_t mode1_val = 0;
    uint8_t reg = 0;
    esp_err_t err = i2c_master_transmit_receive(pca9685_dev, &reg, 1, &mode1_val, 1, 100);
    if (err == ESP_OK)
        ESP_LOGI("MOTOR", "MODE1 = 0x%02X", mode1_val);
    else
        ESP_LOGE("MOTOR", "I2C read failed: %d", err);
}

static void motor_test() {
    read_mode1();
    ESP_LOGI("MOTOR", "=== Motor Test Start ===");
    ESP_LOGI("MOTOR", "Motor 0 (LF) speed=50");
    motor_all(50,0,0,0); vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI("MOTOR", "Motor 1 (RF) speed=50");
    motor_all(0,50,0,0); vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI("MOTOR", "Motor 2 (LR) speed=50");
    motor_all(0,0,50,0); vTaskDelay(pdMS_TO_TICKS(2000));
    ESP_LOGI("MOTOR", "Motor 3 (RR) speed=50");
    motor_all(0,0,0,50); vTaskDelay(pdMS_TO_TICKS(2000));
    motor_stop();
    ESP_LOGI("MOTOR", "=== Motor Test Done ===");
}

/* 按键触发电机测试（启动后等500ms确保系统稳定）*/
static void RunMotorTestTask(void* param) {
    vTaskDelay(pdMS_TO_TICKS(500));
    motor_test();
    vTaskDelete(NULL);
}

/* 陀螺仪闭环转向 */
static volatile bool g_gyro_turning = false;
static void gyro_turn_task(void* arg);
static void gyro_turn_start(int degrees);

/* ============ 红外发射 + 红外学习 ============ */
#define IR_TX_GPIO   GPIO_NUM_16
#define IR_RX_GPIO   GPIO_NUM_14

#define IR_PROTO_NEC    0
#define IR_PROTO_MEIDEA 1

static rmt_channel_handle_t ir_tx_chan = NULL;
static rmt_encoder_handle_t ir_tx_encoder = NULL;
#define IR_SLOT_COUNT 30
static uint64_t g_ir_codes[IR_SLOT_COUNT] = {0};
static int g_ir_protocol[IR_SLOT_COUNT] = {0};
static int g_ir_learn_slot = 0;
static volatile bool g_ir_learning = false;
static volatile uint32_t g_ir_edges[200];
static volatile int g_ir_edge_count = 0;

static void ir_load_codes() {
    Settings s("ir", true);
    for (int i = 0; i < IR_SLOT_COUNT; i++) {
        std::string hex = s.GetString(("code" + std::to_string(i)).c_str(), "");
        g_ir_protocol[i] = s.GetInt(("proto" + std::to_string(i)).c_str(), 0);
        if (!hex.empty()) {
            g_ir_codes[i] = strtoull(hex.c_str(), NULL, 16);
        }
    }
    ESP_LOGI("IR", "Loaded %d IR codes from NVS", IR_SLOT_COUNT);
}

static void ir_save_code(int slot) {
    if (slot < 0 || slot >= IR_SLOT_COUNT) return;
    Settings s("ir", true);
    char buf[32];
    uint32_t hi = (uint32_t)(g_ir_codes[slot] >> 32);
    uint32_t lo = (uint32_t)(g_ir_codes[slot] & 0xFFFFFFFF);
    snprintf(buf, sizeof(buf), "%08X%08X", (unsigned int)hi, (unsigned int)lo);
    s.SetString(("code" + std::to_string(slot)).c_str(), buf);
    s.SetInt(("proto" + std::to_string(slot)).c_str(), g_ir_protocol[slot]);
    ESP_LOGI("IR", "Saved slot %d = %s proto=%d", slot, buf, g_ir_protocol[slot]);
}

static void ir_notify_learned(int slot) {
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    if (sock < 0) return;
    struct sockaddr_in dest = {};
    dest.sin_family = AF_INET;
    dest.sin_port = htons(50002);
    dest.sin_addr.s_addr = inet_addr("192.168.31.92");
    char buf[96];
    uint32_t hi = (uint32_t)(g_ir_codes[slot] >> 32);
    uint32_t lo = (uint32_t)(g_ir_codes[slot] & 0xFFFFFFFF);
    snprintf(buf, sizeof(buf), "IR_LEARNED %d %08X%08X %d",
        slot, (unsigned int)hi, (unsigned int)lo, g_ir_protocol[slot]);
    sendto(sock, buf, strlen(buf), 0, (struct sockaddr*)&dest, sizeof(dest));
    close(sock);
    ESP_LOGI("IR", "UDP notify sent: %s", buf);
}

static void ir_init() {
    // RMT TX for IR emitter (GPIO16)
    rmt_tx_channel_config_t tx_cfg = {};
    tx_cfg.gpio_num = IR_TX_GPIO;
    tx_cfg.clk_src = RMT_CLK_SRC_DEFAULT;
    tx_cfg.resolution_hz = 1000000;
    tx_cfg.mem_block_symbols = 64;
    tx_cfg.trans_queue_depth = 4;
    ESP_ERROR_CHECK(rmt_new_tx_channel(&tx_cfg, &ir_tx_chan));
    rmt_copy_encoder_config_t copy_cfg = {};
    ESP_ERROR_CHECK(rmt_new_copy_encoder(&copy_cfg, &ir_tx_encoder));
    ESP_ERROR_CHECK(rmt_enable(ir_tx_chan));
    // 硬件 38kHz 载波：电平为高时自动叠加载波，只发包络即可
    rmt_carrier_config_t carrier_cfg = {};
    carrier_cfg.frequency_hz = 38000;
    carrier_cfg.duty_cycle = 0.33f;
    ESP_ERROR_CHECK(rmt_apply_carrier(ir_tx_chan, &carrier_cfg));

    // IR receiver (GPIO14) via GPIO ISR
    gpio_config_t rx_cfg = {};
    rx_cfg.pin_bit_mask = 1ULL << IR_RX_GPIO;
    rx_cfg.mode = GPIO_MODE_INPUT;
    rx_cfg.pull_up_en = GPIO_PULLUP_ENABLE;
    rx_cfg.pull_down_en = GPIO_PULLDOWN_DISABLE;
    rx_cfg.intr_type = GPIO_INTR_ANYEDGE;
    gpio_config(&rx_cfg);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(IR_RX_GPIO, [](void*){
        uint32_t now = esp_timer_get_time();
        if(g_ir_edge_count < 199) { g_ir_edges[g_ir_edge_count++] = now; }
    }, NULL);

    ir_load_codes();
    ESP_LOGI(TAG, "IR TX=GPIO16 RX=GPIO14 ready");
}

static void ir_send(uint64_t code, int protocol) {
    rmt_symbol_word_t sym[64];
    int n = 0;
    int bits = (protocol == IR_PROTO_MEIDEA) ? 48 : 32;
    int leader_on = (protocol == IR_PROTO_MEIDEA) ? 4500 : 9000;
    // Leader: 高(硬件载波) + 低
    sym[n++] = (rmt_symbol_word_t){ .duration0=(uint16_t)leader_on, .level0=1,
                                    .duration1=(uint16_t)4500, .level1=0 };
    // 数据位
    for(int i=0;i<bits;i++) {
        bool bit = (code >> i) & 1;
        sym[n++] = (rmt_symbol_word_t){ .duration0=560, .level0=1,
                                        .duration1=(uint16_t)(bit?1690:560), .level1=0 };
    }
    // Stop
    sym[n++] = (rmt_symbol_word_t){ .duration0=560, .level0=1, .duration1=0, .level1=0 };
    rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
    rmt_transmit(ir_tx_chan, ir_tx_encoder, sym, n*sizeof(rmt_symbol_word_t), &tx_cfg);
    rmt_tx_wait_all_done(ir_tx_chan, portMAX_DELAY);
    ESP_LOGI("IR", "Sent proto=%d code=0x%08X%08X (%d symbols)",
        protocol, (unsigned int)(uint32_t)(code>>32), (unsigned int)(uint32_t)code, n);
}

/* 自动识别协议并解码：
   NEC:   leader 9ms + 4.5ms, 32 bit
   美的:  leader 4.5ms + 4.5ms, 48 bit
   位编码相同：low 560us, high 560(0)/1690(1) */
static int ir_decode(uint64_t& code, int& protocol) {
    int cnt = g_ir_edge_count;
    if(cnt < 70) return -1;
    uint32_t leader = g_ir_edges[1] - g_ir_edges[0];
    int bits = 0;
    if (leader >= 8000 && leader <= 10000) {
        protocol = IR_PROTO_NEC;
        bits = 32;
    } else if (leader >= 4000 && leader <= 5500) {
        protocol = IR_PROTO_MEIDEA;
        bits = 48;
    } else {
        ESP_LOGW("IR", "Unknown leader %dus", leader);
        return -1;
    }
    uint64_t c = 0;
    int e = 2;
    for(int i=0;i<bits;i++) {
        if(e + 3 > cnt) return -1;
        uint32_t low = g_ir_edges[e+1] - g_ir_edges[e];
        if(low < 400 || low > 700) return -1;
        uint32_t high = g_ir_edges[e+2] - g_ir_edges[e+1];
        if(high > 1000) c |= (1ULL << i);
        e += 2;
    }
    code = c;
    return 0;
}

static void ir_learn_task(void* arg) {
    while(1) {
        if(g_ir_learning) {
            vTaskDelay(pdMS_TO_TICKS(100));
            if(g_ir_edge_count >= 70) {
                uint64_t code = 0;
                int protocol = 0;
                if(ir_decode(code, protocol) == 0 && code) {
                    if (g_ir_learn_slot >= 0 && g_ir_learn_slot < IR_SLOT_COUNT) {
                        g_ir_codes[g_ir_learn_slot] = code;
                        g_ir_protocol[g_ir_learn_slot] = protocol;
                        ir_save_code(g_ir_learn_slot);
                        ir_notify_learned(g_ir_learn_slot);
                        uint32_t hi = (uint32_t)(code >> 32);
                        uint32_t lo = (uint32_t)(code & 0xFFFFFFFF);
                        ESP_LOGI("IR", "Learned slot %d: proto=%d code=0x%08X%08X", g_ir_learn_slot, protocol, (unsigned int)hi, (unsigned int)lo);
                    } else {
                        ESP_LOGW("IR", "Learn slot invalid: %d", g_ir_learn_slot);
                    }
                    g_ir_learning = false;
                    }
                g_ir_edge_count = 0;
            }
        } else {
            g_ir_edge_count = 0;
            vTaskDelay(pdMS_TO_TICKS(200));
        }
    }
}


/* ============ 网页控制 (WebSocket + 状态) ============ */
void web_cmd_callback(const char* cmd, int val);
static void split_cmd(const char* s, char* cmd, int* val) {
    int i = 0;
    while (s[i] && !(s[i]>='0'&&s[i]<='9') && s[i] != '-') { cmd[i]=s[i]; i++; }
    cmd[i]='\0';
    *val = atoi(s+i);
}

static esp_err_t car_cmd_handler(httpd_req_t* req) {
    char buf[64] = {0};
    int len = httpd_req_get_url_query_len(req);
    if (len > 0 && len < 64) {
        httpd_req_get_url_query_str(req, buf, sizeof(buf));
    }
    char cmd[32] = {0};
    int val = 0;
    char c_param[32] = {0};
    if (httpd_query_key_value(buf, "c", c_param, sizeof(c_param)) == ESP_OK) {
        split_cmd(c_param, cmd, &val);
        ESP_LOGI("WEB", "cmd=%s val=%d", cmd, val);
        web_cmd_callback(cmd, val);
    }
    httpd_resp_set_type(req, "text/plain");
    return httpd_resp_sendstr(req, "OK");
}

static esp_err_t car_status_handler(httpd_req_t* req) {
    char buf[256];
    snprintf(buf, sizeof(buf),
        "{\"speed\":%d,\"temp\":%.1f,\"hum\":%.0f,\"dist\":%d,\"roll\":%.1f,\"pitch\":%.1f,\"uptime\":%lld}",
        g_cs.speed, g_sd.temp, g_sd.hum, g_sd.dist, g_sd.roll, g_sd.pitch,
        (long long)(esp_timer_get_time() / 1000000));
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_sendstr(req, buf);
}

static const char* kCarHtml = R"HTML(
<!DOCTYPE html><html lang="zh"><head><meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1">
<title>小智小车</title><style>
*{margin:0;padding:0;box-sizing:border-box;font-family:sans-serif}
body{background:#0d1117;color:#e6edf3;padding:12px;padding-bottom:30px}
h1{font-size:20px;margin-bottom:8px;color:#58a6ff}
.st{display:grid;grid-template-columns:repeat(auto-fit,minmax(74px,1fr));gap:6px;margin-bottom:12px}
.sc{background:#161b22;border-radius:8px;padding:8px;text-align:center}
.sc .l{font-size:10px;color:#8b949e}.sc .v{font-size:18px;font-weight:700;color:#58a6ff}
.dpad{display:grid;grid-template-columns:repeat(3,1fr);gap:6px;max-width:240px;margin:0 auto 12px}
.dpad button{background:#21262d;border:none;color:#e6edf3;font-size:24px;padding:14px;border-radius:10px;cursor:pointer;touch-action:none;user-select:none;-webkit-user-select:none}
.dpad button:active{background:#30363d}
.dpad .s{background:#581010}
.row{display:flex;gap:6px;margin-bottom:10px}
.row button{flex:1;background:#21262d;border:none;color:#e6edf3;padding:10px;border-radius:8px;cursor:pointer;font-size:13px}
.row button:active{background:#30363d}
.row button.g{background:#0e4429}.row button.r{background:#581010}
.row button.a{background:#5a4a00}
#wsd,#up{text-align:center;font-size:11px;color:#8b949e;margin-bottom:8px}
</style></head><body>
<h1>&#x1F697; 小智小车</h1>
<div id="wsd">连接中...</div>
<div id="up">运行 --</div>
<div class="st">
<div class="sc"><div class="l">速度</div><div class="v" id="sp">0%</div></div>
<div class="sc"><div class="l">温度</div><div class="v" id="tp">--</div></div>
<div class="sc"><div class="l">湿度</div><div class="v" id="hm">--</div></div>
<div class="sc"><div class="l">距离</div><div class="v" id="ds">--</div></div>
</div>
<div class="dpad">
<div></div><button class="mv" data-cmd="self.motor.forward" data-args='{"speed":60,"duration":5}'>&#x2B06;</button><div></div>
<button class="mv" data-cmd="self.motor.turn_left" data-args='{"speed":45,"duration":5}'>&#x2B05;</button><button class="s" onclick="c('self.motor.stop','{}')">&#x23F9;</button><button class="mv" data-cmd="self.motor.turn_right" data-args='{"speed":45,"duration":5}'>&#x27A1;</button>
<div></div><button class="mv" data-cmd="self.motor.backward" data-args='{"speed":60,"duration":5}'>&#x2B07;</button><div></div>
</div>
<div class="row"><button class="g" onclick="c('self.motor.explore','{}')">探索</button><button class="r" onclick="c('self.motor.stop_explore','{}')">停止</button><button class="a" onclick="c('self.alert.stop','{}')">解除警示</button></div>
<script>
var ws=null,reT=null,holdTimer=null;
var d=document.getElementById('wsd');
function fmtUp(u){
 if(!u)return '运行 --';
 if(u<60)return '运行 '+u+'秒';
 return '运行 '+Math.floor(u/60)+'分'+(u%60)+'秒';
}
function connect(){
 clearTimeout(reT);
 ws=new WebSocket('ws://'+location.host+'/ws');
 ws.onopen=function(){d.textContent='已连接';};
 ws.onclose=function(){
  d.textContent='已断开，自动重连中';
  if(holdTimer){clearInterval(holdTimer);holdTimer=null;}
  reT=setTimeout(connect,2000);
 };
 ws.onerror=function(){try{ws.close();}catch(e){}};
}
connect();
var seq=1;
function c(name,args){
 if(ws.readyState!==1)return;
 ws.send(JSON.stringify({type:'mcp',payload:{jsonrpc:'2.0',id:seq++,method:'tools/call',params:{name:name,arguments:JSON.parse(args||'{}')}}}));
}
function startMove(b){
 if(!b.dataset.cmd)return;
 stopMove();
 var go=function(){c(b.dataset.cmd,b.dataset.args||'{}');};
 go();
 holdTimer=setInterval(go,2000);
}
function stopMove(){
 if(holdTimer){clearInterval(holdTimer);holdTimer=null;}
 c('self.motor.stop','{}');
}
var mv=document.querySelectorAll('.dpad .mv');
for(var i=0;i<mv.length;i++){
 (function(b){
  b.addEventListener('pointerdown',function(e){e.preventDefault();startMove(b);});
  ['pointerup','pointercancel','pointerleave'].forEach(function(ev){
   b.addEventListener(ev,function(e){e.preventDefault();stopMove();});
  });
 })(mv[i]);
}
setInterval(function(){
 fetch('/api/status').then(function(r){return r.json()}).then(function(s){
  document.getElementById('sp').textContent=(s.speed||0)+'%';
  document.getElementById('tp').textContent=s.temp?s.temp.toFixed(1)+'C':'--';
  document.getElementById('hm').textContent=s.hum?s.hum.toFixed(0)+'%':'--';
  document.getElementById('ds').textContent=(s.dist&&s.dist<2000?s.dist+'mm':'--');
  document.getElementById('up').textContent=fmtUp(s.uptime);
 }).catch(function(){});
},2000);
</script></body></html>
)HTML";

static esp_err_t car_root_handler(httpd_req_t* req) {
    httpd_resp_set_type(req, "text/html");
    httpd_resp_set_hdr(req, "Cache-Control", "no-store");
    return httpd_resp_sendstr(req, kCarHtml);
}

static void start_car_web_server() {
    // 只启动一个 httpd(8080)：WebSocket 控制 + 状态接口都在这
    static WebSocketControlServer ws_server;
    if (!ws_server.Start(8080)) {
        ESP_LOGE(TAG, "Failed to start WebSocket server");
        return;
    }
    httpd_uri_t s = {"/api/status", HTTP_GET, car_status_handler, NULL};
    httpd_uri_t root = {"/", HTTP_GET, car_root_handler, NULL};
    bool ok = (ws_server.RegisterUri(&s) == ESP_OK) && (ws_server.RegisterUri(&root) == ESP_OK);
    if (ok) {
        ESP_LOGI(TAG, "Web: http://192.168.31.142:8080/  ws://192.168.31.142:8080/ws");
    } else {
        ESP_LOGE(TAG, "Failed to register uri");
    }
}

static void delayed_web_server_task(void*) {
    // 等 WiFi/LWIP 就绪后再启动 Web 服务器
    vTaskDelay(pdMS_TO_TICKS(15000));
    ESP_LOGI(TAG, "Starting web servers after WiFi ready");
    start_car_web_server();
    vTaskDelete(NULL);
}

void web_cmd_callback(const char* cmd, int val) {
    g_cs.speed = val;
    if (!strcmp(cmd,"F")){ motor_all(val,val,val,val); g_cs.dir="前进"; }
    else if (!strcmp(cmd,"B")){ motor_all(-val,-val,-val,-val); g_cs.dir="后退"; }
    else if (!strcmp(cmd,"L")){ motor_all(0,val,0,val); g_cs.dir="左转"; }
    else if (!strcmp(cmd,"R")){ motor_all(val,0,val,0); g_cs.dir="右转"; }
    else if (!strcmp(cmd,"SL")){ motor_all(-val,val,val,-val); g_cs.dir="左横移"; }
    else if (!strcmp(cmd,"SR")){ motor_all(val,-val,-val,val); g_cs.dir="右横移"; }
    else if (!strcmp(cmd,"C")){ motor_all(val,-val,val,-val); g_cs.dir="旋转"; }
    else if (!strcmp(cmd,"CC")){ motor_all(-val,val,-val,val); g_cs.dir="旋转"; }
    else if (!strcmp(cmd,"S")){ motor_stop(); g_cs.dir="停止"; g_cs.speed=0; }
    else if (!strcmp(cmd,"TL")){ gyro_turn_start(-90); g_cs.dir="左转90°"; }
    else if (!strcmp(cmd,"TR")){ gyro_turn_start(90); g_cs.dir="右转90°"; }
    else if (!strcmp(cmd,"TU")){ gyro_turn_start(180); g_cs.dir="掉头"; }
    else if (!strcmp(cmd,"IRLRN")){
        if (val >= 0 && val < IR_SLOT_COUNT) {
            g_ir_learn_slot = val;
            g_ir_learning = true; g_ir_edge_count = 0;
            ESP_LOGI(TAG, "IR learning ON for slot %d - point remote at GPIO14", val);
        } else {
            ESP_LOGW(TAG, "Invalid IR slot: %d", val);
        }
    }
    else if (!strcmp(cmd,"LEDAUTO")){
        auto led = Board::GetInstance().GetLed();
        if (auto sl = dynamic_cast<SingleLed*>(led)) { sl->RestoreAutoLed(); }
    }
    else if (!strcmp(cmd,"IRSND")){
        if (val >= 0 && val < IR_SLOT_COUNT && g_ir_codes[val]) {
            ir_send(g_ir_codes[val], g_ir_protocol[val]);
        } else {
            ESP_LOGW(TAG, "IR slot %d empty - learn it first (IRLRN%d)", val, val);
        }
    }
    else if (!strcmp(cmd,"IRTEST2")){
        // GPIO 直驱测试：GPIO16 拉高 500ms
        rmt_disable(ir_tx_chan);
        gpio_set_direction(GPIO_NUM_16, GPIO_MODE_OUTPUT);
        gpio_set_level(GPIO_NUM_16, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(GPIO_NUM_16, 0);
        rmt_enable(ir_tx_chan);
        ESP_LOGI("IR", "IRTEST2: GPIO16 high 500ms done");
    }
    else if (!strcmp(cmd,"IRTEST")){
        // 200ms 连续载波：4个50ms高电平符号
        rmt_symbol_word_t sym[4];
        for(int i=0;i<4;i++) {
            sym[i] = (rmt_symbol_word_t){ .duration0=50000, .level0=1, .duration1=0, .level1=0 };
        }
        rmt_transmit_config_t tx_cfg = { .loop_count = 0 };
        rmt_transmit(ir_tx_chan, ir_tx_encoder, sym, 4*sizeof(rmt_symbol_word_t), &tx_cfg);
        rmt_tx_wait_all_done(ir_tx_chan, portMAX_DELAY);
        ESP_LOGI("IR", "IRTEST: sent 200ms carrier");
    }
    else if (!strcmp(cmd,"IRLOOP")){
        if (val >= 0 && val < IR_SLOT_COUNT && g_ir_codes[val]) {
            ir_send(g_ir_codes[val], g_ir_protocol[val]);
            vTaskDelay(pdMS_TO_TICKS(80));
            g_ir_edge_count = 0;
            uint32_t start = esp_timer_get_time()/1000;
            while ((esp_timer_get_time()/1000 - start) < 800) {
                vTaskDelay(pdMS_TO_TICKS(10));
                if (g_ir_edge_count >= 70) break;
            }
            uint64_t rx_code = 0;
            int rx_proto = 0;
            if (ir_decode(rx_code, rx_proto) == 0) {
                ESP_LOGI("IR", "LOOP RX: proto=%d code=0x%08X%08X",
                    rx_proto, (unsigned int)(uint32_t)(rx_code>>32), (unsigned int)(uint32_t)rx_code);
                ESP_LOGI("IR", "LOOP TX: proto=%d code=0x%08X%08X",
                    g_ir_protocol[val], (unsigned int)(uint32_t)(g_ir_codes[val]>>32), (unsigned int)(uint32_t)g_ir_codes[val]);
                // UDP 回报
                int sock = socket(AF_INET, SOCK_DGRAM, 0);
                if (sock >= 0) {
                    struct sockaddr_in dest = {};
                    dest.sin_family = AF_INET;
                    dest.sin_port = htons(50002);
                    dest.sin_addr.s_addr = inet_addr("192.168.31.92");
                    char buf[128];
                    snprintf(buf, sizeof(buf), "IR_LOOP RX=%08X%08X TX=%08X%08X MATCH=%d",
                        (unsigned int)(uint32_t)(rx_code>>32), (unsigned int)(uint32_t)rx_code,
                        (unsigned int)(uint32_t)(g_ir_codes[val]>>32), (unsigned int)(uint32_t)g_ir_codes[val],
                        (rx_code == g_ir_codes[val]));
                    sendto(sock, buf, strlen(buf), 0, (struct sockaddr*)&dest, sizeof(dest));
                    close(sock);
                }
            } else {
                ESP_LOGW("IR", "LOOP RX failed: %d edges", g_ir_edge_count);
            }
            g_ir_edge_count = 0;
        }
    }
    else if (!strcmp(cmd,"IRLIST")){
        for (int i = 0; i < IR_SLOT_COUNT; i++) {
            if (g_ir_codes[i]) {
                uint32_t hi = (uint32_t)(g_ir_codes[i] >> 32);
                uint32_t lo = (uint32_t)(g_ir_codes[i] & 0xFFFFFFFF);
                ESP_LOGI("IR", "Slot %2d: proto=%d code=0x%08X%08X", i, g_ir_protocol[i], (unsigned int)hi, (unsigned int)lo);
            } else {
                ESP_LOGI("IR", "Slot %2d: empty", i);
            }
        }
    }
    else if (!strcmp(cmd,"EXPLORE")){
        g_exploring = true; g_cs.speed = 50; g_cs.dir = "探索"; motor_all(50,50,50,50);
    }
    else if (!strcmp(cmd,"STOPEXP")){
        g_exploring = false; motor_stop(); g_cs.dir = "停止"; g_cs.speed = 0;
    }
    else if (!strcmp(cmd,"ALERT")){
        if (val == 1) car_alert_start();
        else car_alert_stop();
    }
    else if (!strcmp(cmd,"LISTEN")){
        ESP_LOGI(TAG, "Activate conversation via dashboard");
        Application::GetInstance().StartListening();
    }
    else if (!strcmp(cmd,"NETMIC")){
        ESP_LOGI(TAG, "Remote mic mode");
        Application::GetInstance().StartListening();
    }
    else if (!strcmp(cmd,"M")){
        const char* urls[] = {"", "http://192.168.31.92:8003/xiaozhi/ota/", "http://192.168.31.92:8004/xiaozhi/ota/", "http://192.168.31.92:8006/xiaozhi/ota/"};
        if (val >= 1 && val <= 4) {
            Settings s("wifi", true);
            s.SetString("ota_url", urls[val-1]);
            vTaskDelay(200);
            esp_restart();
        }
    }
    else if (!strcmp(cmd,"T")){ motor_test(); }
}

static void status_broadcast(void*) {
    vTaskDelay(pdMS_TO_TICKS(15000)); /* 等网络就绪 */
    int sock = socket(AF_INET, SOCK_DGRAM, 0);
    int bcast = 1; setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &bcast, sizeof(bcast));
    struct sockaddr_in d = {}; d.sin_family = AF_INET; d.sin_port = htons(8889);
    d.sin_addr.s_addr = htonl(INADDR_BROADCAST);
    while (1) {
        cJSON* j = cJSON_CreateObject();
        cJSON_AddNumberToObject(j,"speed",g_cs.speed);
        cJSON_AddStringToObject(j,"dir",g_cs.dir);
        cJSON_AddNumberToObject(j,"dist",g_cs.dist);
        cJSON_AddBoolToObject(j,"radar",g_cs.radar);
        cJSON_AddStringToObject(j,"mode",g_cs.mode);
        cJSON_AddStringToObject(j,"model",g_cs.model);
        cJSON_AddStringToObject(j,"speech",g_cs.speech);
        char* str = cJSON_PrintUnformatted(j);
        sendto(sock, str, strlen(str), 0, (struct sockaddr*)&d, sizeof(d));
        free(str); cJSON_Delete(j);
        vTaskDelay(pdMS_TO_TICKS(300));
    }
}

class ViIotS3Board : public WifiBoard {
private:
    i2c_master_bus_handle_t i2c_bus_;
    Button boot_button_;
    LcdDisplay* display_ = nullptr;

    void InitializeI2cBus() {
        ESP_LOGI(TAG, "Init I2C bus (GPIO10-SDA / GPIO11-SCL)");
        i2c_master_bus_config_t i2c_bus_cfg = {};
        i2c_bus_cfg.i2c_port = I2C_NUM_0;
        i2c_bus_cfg.sda_io_num = (gpio_num_t)XL9555_I2C_SDA_PIN;
        i2c_bus_cfg.scl_io_num = (gpio_num_t)XL9555_I2C_SCL_PIN;
        i2c_bus_cfg.clk_source = I2C_CLK_SRC_DEFAULT;
        i2c_bus_cfg.glitch_ignore_cnt = 7;
        i2c_bus_cfg.intr_priority = 0;
        i2c_bus_cfg.trans_queue_depth = 0;
        i2c_bus_cfg.flags.enable_internal_pullup = 1;
        ESP_ERROR_CHECK(i2c_new_master_bus(&i2c_bus_cfg, &i2c_bus_));
    }

    void InitES8311() {
        ESP_LOGI(TAG, "Probing ES8311 audio codec...");
        uint8_t addrs[] = {0x18, 0x19, 0x30, 0x31};
        for (int i = 0; i < 4; i++) {
            i2c_device_config_t dev_cfg = {};
            dev_cfg.dev_addr_length = I2C_ADDR_BIT_LEN_7;
            dev_cfg.device_address = addrs[i];
            dev_cfg.scl_speed_hz = 100000;
            i2c_master_dev_handle_t dev = NULL;
            esp_err_t ret = i2c_master_bus_add_device(i2c_bus_, &dev_cfg, &dev);
            if (ret == ESP_OK) {
                uint8_t cmd[2] = {0x00, 0x00};
                ret = i2c_master_transmit(dev, cmd, 2, 50);
                if (ret == ESP_OK) {
                    ESP_LOGI(TAG, "ES8311 found at 0x%02X", addrs[i]);
                    cmd[0] = 0x0F; cmd[1] = 0x7F;
                    i2c_master_transmit(dev, cmd, 2, 100);
                    cmd[0] = 0x10; cmd[1] = 0x40;
                    i2c_master_transmit(dev, cmd, 2, 100);
                    cmd[0] = 0x11; cmd[1] = 0x30;
                    i2c_master_transmit(dev, cmd, 2, 100);
                    ESP_LOGI(TAG, "ES8311 powered ON");
                    break;
                }
            }
        }
    }

    void InitializeXL9555() {
        ESP_LOGI(TAG, "Init XL9555 IO expander");
        xl9555_init_with_bus(i2c_bus_);
        xl9555_ioconfig((~(XL9555_LCD_RST | XL9555_LCD_BL | XL9555_TP_RST | XL9555_PA_PIN)) & 0xFFFF);
        xl9555_pin_write(XL9555_LCD_RST, 0);
        vTaskDelay(pdMS_TO_TICKS(100));
        xl9555_pin_write(XL9555_LCD_RST, 1);
        vTaskDelay(pdMS_TO_TICKS(100));
        xl9555_pin_write(XL9555_LCD_BL, 1);
        xl9555_pin_write(XL9555_PA_PIN, 1);
    }

    static void PollButtonsTask(void* arg) {
        uint16_t* pins = (uint16_t*)arg;
        const char* urls[4] = {"", "http://192.168.31.92:8003/xiaozhi/ota/", "http://192.168.31.92:8004/xiaozhi/ota/", "http://192.168.31.92:8006/xiaozhi/ota/"};
        const char* names[4] = {"云小智", "Dify", "Ollama", "Gemini"};
        bool prev[4] = {true, true, true, true};
        while (true) {
            for (int i = 0; i < 4; i++) {
                bool curr = xl9555_pin_read(pins[i]) != 0;
                if (prev[i] && !curr) {
                    vTaskDelay(pdMS_TO_TICKS(30));
                    curr = xl9555_pin_read(pins[i]) != 0;
                    if (!curr) {
                        while (xl9555_pin_read(pins[i]) == 0) vTaskDelay(pdMS_TO_TICKS(10));
                        if (i == 3) {
                            ESP_LOGI(TAG, "Motor test triggered by button 4");
                            xTaskCreate(RunMotorTestTask, "mtr_test", 4096, NULL, 5, NULL);
                        } else {
                            ESP_LOGI(TAG, "Mode switch -> %s", names[i]);
                            Settings s("wifi", true);
                            s.SetString("ota_url", urls[i]);
                            vTaskDelay(pdMS_TO_TICKS(200));
                            esp_restart();
                        }
                    }
                }
                prev[i] = curr;
            }
            vTaskDelay(pdMS_TO_TICKS(50));
        }
    }

    void StartButtonPoller() {
        static uint16_t pins[4] = {XL9555_BTN_1, XL9555_BTN_2, XL9555_BTN_3, XL9555_BTN_4};
        xTaskCreate(PollButtonsTask, "btn_poll", 4096, pins, 5, NULL);
        ESP_LOGI(TAG, "Button poller started");
    }

    void InitializeST7789Display() {
        ESP_LOGI(TAG, "Init ST7789 i80 8-bit display");
        esp_lcd_i80_bus_config_t bus_config = {};
        bus_config.dc_gpio_num = ST7789_LCD_RS;
        bus_config.wr_gpio_num = ST7789_LCD_WR;
        bus_config.clk_src = LCD_CLK_SRC_DEFAULT;
        int dg[] = {ST7789_LCD_D0,ST7789_LCD_D1,ST7789_LCD_D2,ST7789_LCD_D3,
                    ST7789_LCD_D4,ST7789_LCD_D5,ST7789_LCD_D6,ST7789_LCD_D7};
        memcpy(bus_config.data_gpio_nums, dg, sizeof(dg));
        bus_config.bus_width = 8;
        bus_config.max_transfer_bytes = DISPLAY_WIDTH * DISPLAY_HEIGHT * sizeof(uint16_t);
        bus_config.dma_burst_size = 64;
        esp_lcd_i80_bus_handle_t i80_bus = NULL;
        ESP_ERROR_CHECK(esp_lcd_new_i80_bus(&bus_config, &i80_bus));
        esp_lcd_panel_io_handle_t panel_io = nullptr;
        esp_lcd_panel_io_i80_config_t io_config = {};
        io_config.cs_gpio_num = ST7789_LCD_CS;
        io_config.pclk_hz = 20000000;
        io_config.trans_queue_depth = 10;
        io_config.on_color_trans_done = nullptr;
        io_config.user_ctx = nullptr;
        io_config.lcd_cmd_bits = 8;
        io_config.lcd_param_bits = 8;
        io_config.dc_levels.dc_dummy_level = 0;
        io_config.dc_levels.dc_data_level = 1;
        ESP_ERROR_CHECK(esp_lcd_new_panel_io_i80(i80_bus, &io_config, &panel_io));
        esp_lcd_panel_handle_t panel_handle = nullptr;
        esp_lcd_panel_dev_config_t panel_config = {};
        panel_config.reset_gpio_num = -1;
        panel_config.rgb_ele_order = LCD_RGB_ELEMENT_ORDER_RGB;
        panel_config.bits_per_pixel = 16;
        ESP_ERROR_CHECK(esp_lcd_new_panel_st7789(panel_io, &panel_config, &panel_handle));
        esp_lcd_panel_reset(panel_handle);
        esp_lcd_panel_init(panel_handle);
        esp_lcd_panel_invert_color(panel_handle, true);
        esp_lcd_panel_swap_xy(panel_handle, true);
        esp_lcd_panel_mirror(panel_handle, false, true);
        esp_lcd_panel_disp_on_off(panel_handle, true);
        display_ = new SpiLcdDisplay(panel_io, panel_handle,
            DISPLAY_WIDTH, DISPLAY_HEIGHT,
            DISPLAY_OFFSET_X, DISPLAY_OFFSET_Y,
            DISPLAY_MIRROR_X, DISPLAY_MIRROR_Y, DISPLAY_SWAP_XY);
    }

    void InitializeButtons() {
        boot_button_.OnClick([this]() {
            auto& app = Application::GetInstance();
            if (app.GetDeviceState() == kDeviceStateStarting) {
                EnterWifiConfigMode();
                return;
            }
        });
        boot_button_.OnPressDown([this]() {
            Application::GetInstance().StartListening();
        });
        boot_button_.OnPressUp([this]() {
            Application::GetInstance().StopListening();
        });
    }

public:
    ViIotS3Board() : boot_button_((gpio_num_t)BOOT_BUTTON_GPIO) {
        InitializeI2cBus();
        InitializeXL9555();
        pca9685_init();
        g_sensors.InitAll();
        /* 注册电机 MCP 工具，AI 可通过语音控制小车 */
        new MotorController(
            [](int s) { g_cs.speed=s; motor_notify("前进",  s,  s,  s,  s); },
            [](int s) { g_cs.speed=s; motor_notify("后退", -s, -s, -s, -s); },
            [](int s) { g_cs.speed=s; motor_notify("左转",  0,  s,  0,  s); },
            [](int s) { g_cs.speed=s; motor_notify("右转",  s,  0,  s,  0); },
            [](int s) { g_cs.speed=s; motor_notify("左横移", -s, s, s, -s); },
            [](int s) { g_cs.speed=s; motor_notify("右横移", s, -s, -s, s); },
            [](int s) { g_cs.speed=s; motor_notify("旋转",  s, -s,  s, -s); },
            []()      { g_cs.dir="停止";g_cs.speed=0; motor_stop(); },
            []()      { motor_test(); }
        );
        InitializeButtons();
        InitializeST7789Display();
        StartButtonPoller();
        new WifiCmdServer(web_cmd_callback);
        xTaskCreate(delayed_web_server_task, "web_delay", 8192, NULL, 3, NULL);
        xTaskCreate(status_broadcast, "stat_udp", 4096, NULL, 5, NULL);
        // Init sensors
        g_sensors.InitAll();
        // Init IR + LED
        ir_init();
        xTaskCreate(ir_learn_task, "ir_learn", 4096, NULL, 3, NULL);
        // Chat message callback for dashboard
// Chat callback removed for build
        // MCP tools for explore and sensor
        {
            auto& mcp = McpServer::GetInstance();
            mcp.AddTool("self.motor.explore",
                "Start autonomous exploration mode. The car drives forward and avoids obstacles.",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    g_exploring = true; g_cs.speed = 50; g_cs.dir = "\xe6\x8e\xa2\xe7\xb4\xa2";
                    motor_all(50,50,50,50); return true;
                });
            mcp.AddTool("self.motor.stop_explore",
                "Stop exploration mode and stop the car.",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    g_exploring = false; motor_stop();
                    g_cs.dir = "\xe5\x81\x9c\xe6\xad\xa2"; g_cs.speed = 0; return true;
                });
            mcp.AddTool("self.ir.send",
                "Send infrared remote control code. Slots 0-8 living room light: 0=power on, 1=power off, 2=brightness up, 3=brightness down, 4=color temp up, 5=color temp down, 6=night light, 7=mode A, 8=mode B. Slots 9-15 bedroom AC: 9=power, 10=mode, 11=temperature up, 12=temperature down, 13=swing, 14=timer, 15=auxiliary heat. Slots 16-29 living room AC: 16=power, 17=mode, 18=eco save, 19=function, 20=timer, 21=confirm, 22=temperature up, 23=temperature down, 24=fan speed, 25=swing up-down, 26=swing left-right, 27=light, 28=auxiliary heat, 29=anti direct blow. Use when user asks to control the light or air conditioner via infrared.",
                PropertyList({
                    Property("slot", kPropertyTypeInteger, 0, 29),
                }),
                [](const PropertyList& props) -> ReturnValue {
                    int slot = props["slot"].value<int>();
                    if (slot >= 0 && slot < IR_SLOT_COUNT && g_ir_codes[slot]) {
                        ir_send(g_ir_codes[slot], g_ir_protocol[slot]);
                        ESP_LOGI("IR", "MCP send slot %d proto=%d", slot, g_ir_protocol[slot]);
                        return true;
                    }
                    return false;
                });
            mcp.AddTool("self.led.set_color",
                "Set the onboard LED color. Use when user asks to turn on/off LED or change LED color. Parameters: r(0-255), g(0-255), b(0-255).",
                PropertyList({
                    Property("r", kPropertyTypeInteger, 0, 255),
                    Property("g", kPropertyTypeInteger, 0, 255),
                    Property("b", kPropertyTypeInteger, 0, 255),
                }),
                [](const PropertyList& props) -> ReturnValue {
                    int r = props["r"].value<int>();
                    int g = props["g"].value<int>();
                    int b = props["b"].value<int>();
                    auto led = Board::GetInstance().GetLed();
                    if (auto sl = dynamic_cast<SingleLed*>(led)) {
                        sl->SetLedColor(r, g, b);
                        return true;
                    }
                    return false;
                });
            mcp.AddTool("self.led.auto",
                "Restore automatic LED mode. The LED will show device state colors again (blue=connecting, red=listening, green=speaking). Use when user wants LED auto mode back.",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    auto led = Board::GetInstance().GetLed();
                    if (auto sl = dynamic_cast<SingleLed*>(led)) {
                        sl->RestoreAutoLed();
                        return true;
                    }
                    return false;
                });
            mcp.AddTool("self.led.turn_off",
                "Turn off the onboard LED. Use when user asks to turn off the light/LED.",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    auto led = Board::GetInstance().GetLed();
                    if (auto sl = dynamic_cast<SingleLed*>(led)) {
                        sl->SetLedColor(0, 0, 0);
                        return true;
                    }
                    return false;
                });
            mcp.AddTool("self.alert.stop",
                "Stop the baby alert: stop patrolling, turn off the flashing LED and restore normal LED mode. Call when the user says they have arrived or the baby is fine, e.g. '我来了', '宝宝没事了', '不用巡逻了', stop the alert, dismiss the alarm.",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    car_alert_stop();
                    return true;
                });
            mcp.AddTool("self.alert.start",
                "Start the baby alert for testing: flash the LED and after a timeout patrol the room.",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    car_alert_start();
                    return true;
                });
            mcp.AddTool("self.sensor.get_readings",
                "Get current sensor readings (temperature, humidity, distance, roll, pitch).",
                PropertyList(),
                [](const PropertyList&) -> ReturnValue {
                    char buf[256];
                    snprintf(buf, sizeof(buf),
                        "{\"temperature\":%.1f,\"humidity\":%.0f,\"distance_mm\":%d,\"roll\":%.1f,\"pitch\":%.1f}",
                        g_sd.temp, g_sd.hum, g_sd.dist, g_sd.roll, g_sd.pitch);
                    return std::string(buf);
                });
        }
        // Audio UDP receiver
        xTaskCreate([](void*) {
            vTaskDelay(pdMS_TO_TICKS(6000));
            ESP_LOGI(TAG, "Audio UDP receiver starting on port %d", AUDIO_UDP_PORT);
            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            struct sockaddr_in addr = {};
            addr.sin_family = AF_INET; addr.sin_port = htons(AUDIO_UDP_PORT);
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) {
                ESP_LOGE(TAG, "Failed to bind audio UDP port %d", AUDIO_UDP_PORT);
                close(sock); vTaskDelete(NULL); return;
            }
            ESP_LOGI(TAG, "Audio UDP receiver ready on port %d", AUDIO_UDP_PORT);
            int16_t pcm_buf[960];
            uint32_t ts = 0;
            while (1) {
                int len = recvfrom(sock, pcm_buf, sizeof(pcm_buf), 0, NULL, NULL);
                if (len == sizeof(pcm_buf)) {
                    // Audio push removed for build
                }
            }
            close(sock); vTaskDelete(NULL);
        }, "audio_udp", 4096, NULL, 4, NULL);
        // Sensor read + exploration task
        xTaskCreate([](void*) {
            vTaskDelay(pdMS_TO_TICKS(3000));
            uint32_t last_wander = 0;
            while (1) {
                g_sd = g_sensors.ReadAll();
                uint32_t now = esp_timer_get_time() / 1000;
                if(g_cs.speed > 0 && g_sd.dist_ok){
                    uint16_t d = g_sd.dist;
                    if(g_exploring){
                        if(d > 0 && d < 350){
                            ESP_LOGI(TAG, "DANGER avoid %dmm", d);
                            motor_stop(); g_cs.speed = 0;
                            vTaskDelay(pdMS_TO_TICKS(80));
                            motor_all(-45,-45,-45,-45); vTaskDelay(pdMS_TO_TICKS(500)); motor_stop();
                            if(rand()%2){ motor_all(40,-40,40,-40); }
                            else { motor_all(-40,40,-40,40); }
                            vTaskDelay(pdMS_TO_TICKS(500 + rand()%700)); motor_stop();
                            motor_all(50,50,50,50); g_cs.speed = 50;
                        } else if(d >= 350 && d < 600){
                            if(g_cs.speed > 25){ motor_all(25,25,25,25); g_cs.speed = 25; ESP_LOGI(TAG, "SLOW %dmm", d); }
                        } else if(d >= 600 && d < 900){
                            if(g_cs.speed > 40){ motor_all(35,35,35,35); g_cs.speed = 35; }
                        } else {
                            if(g_cs.speed < 50){ motor_all(50,50,50,50); g_cs.speed = 50; }
                        }
                        if(now - last_wander > 8000 + (rand()%5000)){
                            last_wander = now;
                            int t = 200 + rand()%600;
                            if(rand()%2){ motor_all(40,-40,40,-40); }
                            else { motor_all(-40,40,-40,40); }
                            vTaskDelay(pdMS_TO_TICKS(t)); motor_stop();
                            motor_all(50,50,50,50); g_cs.speed = 50;
                            ESP_LOGI(TAG, "WANDER %dms", t);
                        }
                    } else {
                        if(d > 0 && d < 200){ motor_stop(); g_cs.speed = 0; ESP_LOGI(TAG, "AUTO-STOP %dmm", d); }
                    }
                }
                vTaskDelay(pdMS_TO_TICKS(200));
            }
        }, "sensor_read", 4096, NULL, 4, NULL);
    }

    virtual AudioCodec* GetAudioCodec() override {
        static NoAudioCodecSimplexPdm audio_codec(
            AUDIO_INPUT_SAMPLE_RATE,
            AUDIO_OUTPUT_SAMPLE_RATE,
            GPIO_NUM_46,
            GPIO_NUM_9,
            GPIO_NUM_8,
            I2S_STD_SLOT_RIGHT,
            GPIO_NUM_3,
            GPIO_NUM_42
        );
        return &audio_codec;
    }

    virtual Display* GetDisplay() override { return display_; }
    virtual Led* GetLed() override {
        static SingleLed led((gpio_num_t)BUILTIN_LED_GPIO);
        return &led;
    }
};


DECLARE_BOARD(ViIotS3Board);

/* 陀螺仪闭环转向任务 */
static void gyro_turn_task(void* arg) {
    int degrees = (int)(intptr_t)arg;
    bool reverse = false;
    if(degrees < 0){ reverse = true; degrees = -degrees; }
    
    // 不碰I2C！依赖sensor_read任务每20ms更新yaw_accum
    Sensors::yaw_accum = 0;
    Sensors::last_us = esp_timer_get_time();
    
    float target = degrees;
    int speed = 40;
    g_gyro_turning = true;
    
    ESP_LOGI("GYRO", "Gyro turn start %ddeg speed=%d", degrees, speed);
    if(reverse){ motor_all(-speed, speed, -speed, speed); }
    else { motor_all(speed, -speed, speed, -speed); }
    
    uint32_t check_ms = 0;
    while(g_gyro_turning) {
        uint32_t now = esp_timer_get_time() / 1000;
        // 每20ms检查一次yaw_accum（由sensor_read高频更新）
        if(now - check_ms >= 20) {
            check_ms = now;
            float current = fabsf(Sensors::yaw_accum);
            float remaining = target - current;
            
            if(remaining <= 0) { 
                ESP_LOGI("GYRO", "Target reached: yaw=%.1f target=%.0f", current, target);
                break;
            }
            if(remaining < 15 && speed > 20) {
                speed = 20;
                if(reverse){ motor_all(-speed, speed, -speed, speed); }
                else { motor_all(speed, -speed, speed, -speed); }
            }
        }
        vTaskDelay(pdMS_TO_TICKS(5));
    }
    motor_stop();
    g_gyro_turning = false;
    g_cs.dir = "停止"; g_cs.speed = 0;
    ESP_LOGI("GYRO", "Gyro turn done yaw=%.1f target=%.0f", Sensors::yaw_accum, target);
    vTaskDelete(NULL);
}

static void gyro_turn_start(int degrees) {
    if(g_gyro_turning) { return; }
    xTaskCreate(gyro_turn_task, "gyro_turn", 4096, (void*)(intptr_t)degrees, 5, NULL);
}
// force
