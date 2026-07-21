#ifndef __WIFI_CMD_SERVER_H__
#define __WIFI_CMD_SERVER_H__
#include <string.h>
#include <esp_log.h>
#include <lwip/sockets.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#define CMD_PORT 8888
#define TAG_CMD "CMD"
using CmdHandler = void(*)(const char* cmd, int val);

class WifiCmdServer {
public:
    WifiCmdServer(CmdHandler handler) {
        xTaskCreate([](void* arg) {
            CmdHandler h = (CmdHandler)arg;
            vTaskDelay(pdMS_TO_TICKS(12000));
            int sock = socket(AF_INET, SOCK_DGRAM, 0);
            if (sock < 0) { vTaskDelete(NULL); return; }
            struct sockaddr_in addr = {};
            addr.sin_family = AF_INET; addr.sin_port = htons(CMD_PORT);
            addr.sin_addr.s_addr = htonl(INADDR_ANY);
            if (bind(sock, (struct sockaddr*)&addr, sizeof(addr)) < 0) { close(sock); vTaskDelete(NULL); return; }
            ESP_LOGI(TAG_CMD, "UDP cmd on port %d", CMD_PORT);
            char buf[64]; struct sockaddr_in from; socklen_t fl = sizeof(from);
            while (1) {
                int len = recvfrom(sock, buf, sizeof(buf)-1, 0, (struct sockaddr*)&from, &fl);
                if (len > 0) {
                    buf[len] = 0; char* s = buf;
                    while(*s==' '||*s=='\r'||*s=='\n') s++;
                    char cmd[8]={0};int val=0,i=0;
                    while(s[i]&&(s[i]<'0'||s[i]>'9')&&s[i]!='-'&&i<6){cmd[i]=s[i];i++;}
                    if(s[i])val=atoi(&s[i]);
                    h(cmd, val);
                }
            }
            close(sock); vTaskDelete(NULL);
        }, "cmd_udp", 4096, (void*)handler, 5, NULL);
    }
};
#endif
