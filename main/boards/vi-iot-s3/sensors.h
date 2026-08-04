#pragma once
#include <esp_log.h>
#include <driver/gpio.h>
#include <cstring>
#include <math.h>
#include <esp_timer.h>
#include "rom/ets_sys.h"

#define SDA_GPIO  GPIO_NUM_12
#define SCL_GPIO  GPIO_NUM_13
#define AHT20_ADDR  0x38
#define VL53_ADDR   0x29
#define MPU_ADDR    0x68

struct SensorData {
    float temp=0, hum=0;
    uint16_t dist=0;
    float roll=0, pitch=0, yaw=0;
    bool ok=false, dist_ok=false, mpu_ok=false;
};

class Sensors {
public:
    bool aht20_ok=false, vl53_ok=false, mpu_ok=false;
    static float yaw_accum;
    static uint64_t last_us;

    void sw_init() {
        gpio_set_direction(SDA_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_direction(SCL_GPIO, GPIO_MODE_INPUT_OUTPUT_OD);
        gpio_set_pull_mode(SDA_GPIO, GPIO_PULLUP_ONLY);
        gpio_set_pull_mode(SCL_GPIO, GPIO_PULLUP_ONLY);
        gpio_set_level(SDA_GPIO,1); gpio_set_level(SCL_GPIO,1); ets_delay_us(20);
    }
    void sw_delay() { ets_delay_us(100); }
    void sw_start() {
        gpio_set_level(SDA_GPIO,1);sw_delay();gpio_set_level(SCL_GPIO,1);sw_delay();
        gpio_set_level(SDA_GPIO,0);sw_delay();gpio_set_level(SCL_GPIO,0);sw_delay();
    }
    void sw_stop() {
        gpio_set_level(SDA_GPIO,0);sw_delay();gpio_set_level(SCL_GPIO,1);sw_delay();
        gpio_set_level(SDA_GPIO,1);sw_delay();
    }
    bool sw_write_byte(uint8_t b) {
        for(int i=0;i<8;i++){
            gpio_set_level(SDA_GPIO,(b>>(7-i))&1);sw_delay();
            gpio_set_level(SCL_GPIO,1);sw_delay();gpio_set_level(SCL_GPIO,0);sw_delay();
        }
        gpio_set_level(SDA_GPIO,1);gpio_set_level(SCL_GPIO,1);sw_delay();
        bool ack=gpio_get_level(SDA_GPIO)==0;
        gpio_set_level(SCL_GPIO,0);return ack;
    }
    uint8_t sw_read_byte(bool ack){
        uint8_t b=0;gpio_set_level(SDA_GPIO,1);
        for(int i=0;i<8;i++){
            b<<=1;gpio_set_level(SCL_GPIO,1);sw_delay();
            if(gpio_get_level(SDA_GPIO))b|=1;
            gpio_set_level(SCL_GPIO,0);sw_delay();
        }
        gpio_set_level(SDA_GPIO,ack?0:1);gpio_set_level(SCL_GPIO,1);sw_delay();
        gpio_set_level(SCL_GPIO,0);gpio_set_level(SDA_GPIO,1);
        return b;
    }
    bool sw_write_bytes(uint8_t addr,const uint8_t* d,size_t len){
        sw_start();if(!sw_write_byte(addr<<1)){sw_stop();return false;}
        for(size_t i=0;i<len;i++)if(!sw_write_byte(d[i])){sw_stop();return false;}
        sw_stop();return true;
    }
    bool sw_read_bytes(uint8_t addr,uint8_t reg,uint8_t* out,size_t len){
        // Write register address
        sw_start();if(!sw_write_byte(addr<<1)){sw_stop();return false;}
        if(!sw_write_byte(reg)){sw_stop();return false;}
        sw_stop();ets_delay_us(50);
        // Then read data
        sw_start();if(!sw_write_byte((addr<<1)|1)){int a=0;sw_stop();ESP_LOGE("I2C","read NAK at 0x%02X",addr);return false;}
        for(size_t i=0;i<len;i++)out[i]=sw_read_byte(i<len-1);
        sw_stop();return true;
    }
    bool sw_read_bytes_ss(uint8_t addr,uint8_t reg,uint8_t* out,size_t len){
        sw_start();if(!sw_write_byte(addr<<1)){sw_stop();return false;}
        if(!sw_write_byte(reg)){sw_stop();return false;}
        sw_stop();ets_delay_us(5);
        sw_start();if(!sw_write_byte((addr<<1)|1)){sw_stop();return false;}
        for(size_t i=0;i<len;i++)out[i]=sw_read_byte(i<len-1);
        sw_stop();return true;
    }
    
    bool sw_read_raw(uint8_t addr,uint8_t* out,size_t len){
        sw_start();if(!sw_write_byte((addr<<1)|1)){sw_stop();return false;}
        for(size_t i=0;i<len;i++)out[i]=sw_read_byte(i<len-1);
        sw_stop();return true;
    }

    bool InitAHT20(){
        // Soft reset first
        sw_write_bytes(AHT20_ADDR,(uint8_t[]){0xBA},1);
        vTaskDelay(pdMS_TO_TICKS(20));
        // Init with calibration enable
        sw_write_bytes(AHT20_ADDR,(uint8_t[]){0xBE,0x08,0x00},3);
        vTaskDelay(pdMS_TO_TICKS(50));
        // Check calibration bit
        uint8_t st=0;
        if(sw_read_raw(AHT20_ADDR,&st,1)){
            ESP_LOGI("AHT","init status=0x%02X cal=%d",st,(st&0x08)!=0);
            if(!(st&0x08)){
                // Calibration bit not set, retry
                sw_write_bytes(AHT20_ADDR,(uint8_t[]){0xBE,0x08,0x00},3);
                vTaskDelay(pdMS_TO_TICKS(50));
                sw_read_raw(AHT20_ADDR,&st,1);
                ESP_LOGI("AHT","retry status=0x%02X cal=%d",st,(st&0x08)!=0);
            }
        } else {
            ESP_LOGE("AHT","init status read failed");
        }
        aht20_ok=true;ESP_LOGI("SENS","AHT20 ready");return true;
    }
    bool ReadAHT20(float& t,float& h){
        uint8_t trig[]={0xAC,0x33,0x00};
        if(!sw_write_bytes(AHT20_ADDR,trig,3)){ESP_LOGE("AHT","trig write fail");return false;}
        vTaskDelay(pdMS_TO_TICKS(150));
        uint8_t d[6];
        if(!sw_read_raw(AHT20_ADDR,d,6)){ESP_LOGE("AHT","read raw fail");return false;}
        ESP_LOGI("AHT","raw=%02x%02x%02x%02x%02x%02x busy=%d",d[0],d[1],d[2],d[3],d[4],d[5],(d[0]&0x80)!=0);
        if(d[0]&0x80)return false;
        uint32_t rh=((uint32_t)d[1]<<12)|((uint32_t)d[2]<<4)|(d[3]>>4);
        uint32_t rt=((uint32_t)(d[3]&0x0F)<<16)|((uint32_t)d[4]<<8)|d[5];
        if(rt==0)return false;
        h=rh*100.0f/1048576.0f;t=(rt*200.0f/1048576.0f)-50.0f;
        ESP_LOGI("AHT","OK t=%.1f h=%.0f",t,h);
        return true;
    }

    bool InitVL53L0X(){
        uint8_t id=0;
        vl53_ok=(sw_read_bytes(VL53_ADDR,0xC0,&id,1)&&id==0xEE);
        ESP_LOGI("SENS","VL53L0X ready (ID=0x%02X)",id);
        if(vl53_ok){
            // Diagnostic: one ranging attempt + clear interrupt
            sw_write_bytes(VL53_ADDR,(uint8_t[]){0x00,1},2);
            vTaskDelay(pdMS_TO_TICKS(100));
            uint8_t st=0;
            if(sw_read_bytes(VL53_ADDR,0x14,&st,1)){
                ESP_LOGI("VL53","diag: st=0x%02X bit0=%d",st,st&1);
                if(st&1){
                    uint8_t gb[12];
                    if(sw_read_bytes(VL53_ADDR,0x14,gb,12)){
                        uint16_t dd=((uint16_t)gb[10]<<8)|gb[11];
                        ESP_LOGI("VL53","diag: dist=%dmm",dd);
                    }
                }
            }
            sw_write_bytes(VL53_ADDR,(uint8_t[]){0x15,1},2); // clear interrupt
        }
        return vl53_ok;
    }
    bool ReadVL53L0X(uint16_t& dist){
        for(int k=0;k<3;k++){
            sw_write_bytes(VL53_ADDR,(uint8_t[]){0x00,1},2);
            vTaskDelay(pdMS_TO_TICKS(100));
            uint8_t st=0;
            if(!sw_read_bytes(VL53_ADDR,0x14,&st,1)){vTaskDelay(1);continue;}
            if(!(st&1)){vTaskDelay(1);continue;}
            uint8_t gb[12];
            if(!sw_read_bytes(VL53_ADDR,0x14,gb,12)){vTaskDelay(1);continue;}
            uint16_t d=((uint16_t)gb[10]<<8)|gb[11];
            sw_write_bytes(VL53_ADDR,(uint8_t[]){0x15,1},2); // clear interrupt
            ESP_LOGI("VL53","dist=%dmm raw=%02x%02x",d,gb[10],gb[11]);
            if(d>0&&d<2000){dist=d;return true;}
        }
        return false;
    }

    bool InitMPU6050(){
        sw_write_bytes(MPU_ADDR,(uint8_t[]){0x6B,0},2);vTaskDelay(pdMS_TO_TICKS(50));
        sw_write_bytes(MPU_ADDR,(uint8_t[]){0x19,0x07},2);  // Sample rate divider 125Hz
        sw_write_bytes(MPU_ADDR,(uint8_t[]){0x1A,0x06},2);  // Low pass filter 5Hz
        sw_write_bytes(MPU_ADDR,(uint8_t[]){0x1B,0x18},2);  // Gyro ±2000°/s
        sw_write_bytes(MPU_ADDR,(uint8_t[]){0x1C,0x01},2);  // Accel ±2G + filter
                uint8_t id=0;
        if(sw_read_bytes(MPU_ADDR,0x75,&id,1)){ESP_LOGI("MPU","WHO_AM_I=0x%02X",id);}
        mpu_ok=true;ESP_LOGI("SENS","MPU6050 ready");return true;
    }
    bool ReadMPU6050(float& roll,float& pitch,float& yaw){
        uint8_t r[14];  // Read all: accel(6)+temp(2)+gyro(6)
        for(int retry=0;retry<3;retry++){
            if(sw_read_bytes(MPU_ADDR,0x3B,r,14)){
                int16_t ax=(r[0]<<8)|r[1],ay=(r[2]<<8)|r[3],az=(r[4]<<8)|r[5];
                int16_t gz=(r[12]<<8)|r[13];  // Gyro Z from 0x47-0x48
                
                // Accelerometer -> roll/pitch
                float accX=ax/16384.0f,accY=ay/16384.0f,accZ=az/16384.0f;
                if(fabsf(accX)<0.001f&&fabsf(accY)<0.001f&&fabsf(accZ)<0.001f){vTaskDelay(1);continue;}
                roll=atan2f(accY,accZ)*57.2958f;
                pitch=atan2f(-accX,sqrtf(accY*accY+accZ*accZ))*57.2958f;
                
                // Gyro Z -> yaw integration
                float yaw_rate = gz / 16.4f;  // +/-2000deg/s -> 16.4 LSB/deg/s
                uint64_t now = esp_timer_get_time();
                if(last_us == 0){ last_us = now; }
                float dt = (now - last_us) / 1000000.0f;
                last_us = now;
                if(dt > 0.5f) dt = 0;  // Ignore large gaps
                yaw_accum += yaw_rate * dt;
                yaw = yaw_accum;
                
                return true;
            }
            vTaskDelay(pdMS_TO_TICKS(10));
        }
        ESP_LOGE("MPU","read fail after 3 retries");
        return false;
    }

    void InitAll(){
        sw_init();vTaskDelay(pdMS_TO_TICKS(5));
        aht20_ok=InitAHT20();vl53_ok=InitVL53L0X();mpu_ok=InitMPU6050();
        ESP_LOGI("SENS","Sensors: AHT20=%d VL53=%d MPU=%d",aht20_ok,vl53_ok,mpu_ok);
    }

    SensorData ReadAll(){
        SensorData d;
        float t,h;uint16_t dis;float r,pi,y;
        if(ReadAHT20(t,h)){d.temp=t;d.hum=h;d.ok=true;}
        if(ReadVL53L0X(dis)){d.dist=dis;d.dist_ok=true;}
        if(ReadMPU6050(r,pi,y)){d.roll=r;d.pitch=pi;d.yaw=y;d.mpu_ok=true;}
        return d;
    }
};
float Sensors::yaw_accum=0;
uint64_t Sensors::last_us=0;
