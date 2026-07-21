#include "wifi_board.h"
#include "audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "pin_config.h"

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
static void motor_notify(const char* status, int fl, int fr, int rl, int rr) {
    auto display = Board::GetInstance().GetDisplay();
    if (display) display->ShowNotification(status, 2500);
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
                    cmd[0] = 0x0F; cmd[1] = 0x40;
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
        /* 注册电机 MCP 工具，AI 可通过语音控制小车 */
        new MotorController(
            [](int s) { motor_notify("前进",  s,  s,  s,  s); },  // 前进
            [](int s) { motor_notify("后退", -s, -s, -s, -s); },  // 后退
            [](int s) { motor_notify("左转",  0,  s,  0,  s); },  // 左转
            [](int s) { motor_notify("右转",  s,  0,  s,  0); },  // 右转
            [](int s) { motor_notify("左横移", s, -s, -s,  s); },  // 左平移（横移）
            [](int s) { motor_notify("右横移",-s,  s,  s, -s); },  // 右平移（横移）
            [](int s) { motor_notify("旋转",  s, -s,  s, -s); },  // 旋转（正数=顺时针, 负数=逆时针）
            []()      { auto d = Board::GetInstance().GetDisplay(); if(d) d->ShowNotification("已停止", 1500); motor_stop(); },                 // 停止
            []()      { motor_test(); }                  // 测 试
        );
        InitializeButtons();
        InitializeST7789Display();
        StartButtonPoller();
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
