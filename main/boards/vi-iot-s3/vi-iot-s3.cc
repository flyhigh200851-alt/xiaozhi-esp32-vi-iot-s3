#include "wifi_board.h"
#include "audio_codec.h"
#include "display/lcd_display.h"
#include "application.h"
#include "button.h"
#include "led/single_led.h"
#include "pin_config.h"
#include "config.h"

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

#define TAG "VI-IOT-S3"

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
        esp_lcd_panel_set_gap(panel_handle, 0, 80);
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
        InitializeButtons();
        InitializeST7789Display();
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
