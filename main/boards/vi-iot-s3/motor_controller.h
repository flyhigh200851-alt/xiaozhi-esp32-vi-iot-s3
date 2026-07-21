#ifndef __MOTOR_CONTROLLER_H__
#define __MOTOR_CONTROLLER_H__

#include "mcp_server.h"
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>

/* 文件级静态变量，供 auto-stop 任务使用 */
namespace {
    std::function<void()> g_stop_fn;
    static void auto_stop_task(void* arg) {
        int sec = (int)(intptr_t)arg;
        vTaskDelay(pdMS_TO_TICKS(sec * 1000));
        if (g_stop_fn) g_stop_fn();
        vTaskDelete(NULL);
    }
}

/* 电机控制器：把四个轮子的运动暴露为 MCP 工具，让 AI 通过语音控制小车身体 */
/* 注意：这些是"我自己的腿/轮子"，不是"在遥控别的东西" */
class MotorController {
public:
    using MotorAction = std::function<void(int speed)>;

    MotorController(MotorAction forward_fn, MotorAction backward_fn,
                    MotorAction turn_left_fn, MotorAction turn_right_fn,
                    MotorAction strafe_left_fn, MotorAction strafe_right_fn,
                    MotorAction spin_fn,
                    std::function<void()> stop_fn,
                    std::function<void()> test_fn) {
        auto& mcp = McpServer::GetInstance();

        mcp.AddTool("self.motor.forward",
            "Drive forward. Call this when the user says: 往前走, 向前开, 前进, 直走, 往前走几步, go forward, drive straight."
            " This is MY body moving forward — I, the car, drive myself.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 50, 1, 100),
                Property("duration", kPropertyTypeInteger, 3, 1, 30)
            }),
            [forward_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                forward_fn(props["speed"].value<int>());
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.backward",
            "Drive backward. Call when user says: 后退, 向后开, 倒车, go back, reverse."
            " I drive myself backward.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 50, 1, 100),
                Property("duration", kPropertyTypeInteger, 3, 1, 30)
            }),
            [backward_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                backward_fn(props["speed"].value<int>());
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.turn_left",
            "Turn left (pivot). Call when user says: 左转, 向左转, 往左拐, turn left."
            " I turn my body to the left.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 50, 1, 100),
                Property("duration", kPropertyTypeInteger, 2, 1, 30)
            }),
            [turn_left_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                turn_left_fn(props["speed"].value<int>());
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.turn_right",
            "Turn right (pivot). Call when user says: 右转, 向右转, 往右拐, turn right."
            " I turn my body to the right.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 50, 1, 100),
                Property("duration", kPropertyTypeInteger, 2, 1, 30)
            }),
            [turn_right_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                turn_right_fn(props["speed"].value<int>());
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.strafe_left",
            "Strafe (crab walk) to the left without turning. All four wheels move sideways to the left."
            " Call when user says: 左平移, 往左横着走, 向左横移, 螃蟹走, strafe left."
            " This uses mecanum wheel sideways motion — I crab-walk to the left.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 40, 1, 100),
                Property("duration", kPropertyTypeInteger, 2, 1, 30)
            }),
            [strafe_left_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                strafe_left_fn(props["speed"].value<int>());
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.strafe_right",
            "Strafe (crab walk) to the right without turning. All four wheels move sideways to the right."
            " Call when user says: 右平移, 往右横着走, 向右横移, strafe right."
            " This uses mecanum wheel sideways motion — I crab-walk to the right.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 40, 1, 100),
                Property("duration", kPropertyTypeInteger, 2, 1, 30)
            }),
            [strafe_right_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                strafe_right_fn(props["speed"].value<int>());
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.spin",
            "Spin/pivot in place. Call when user says: 转圈, 转一圈, 原地转圈, 掉头, spin, turn around, do a 360."
            " Left and right wheels counter-rotate so I spin around my center axis without moving forward/backward."
            " direction=1 for clockwise (向右转圈), direction=-1 for counter-clockwise (向左转圈)."
            " I spin myself in place.",
            PropertyList({
                Property("speed", kPropertyTypeInteger, 40, 1, 100),
                Property("duration", kPropertyTypeInteger, 3, 1, 30),
                Property("direction", kPropertyTypeInteger, 1, -1, 1)
            }),
            [spin_fn, stop_fn](const PropertyList& props) -> ReturnValue {
                int speed = props["speed"].value<int>();
                int dir = props["direction"].value<int>();
                spin_fn(dir > 0 ? speed : -speed);
                auto sec = props["duration"].value<int>();
                g_stop_fn = stop_fn;
                xTaskCreate(auto_stop_task, "mtr_auto", 2048, (void*)(intptr_t)sec, 5, NULL);
                return true;
            });

        mcp.AddTool("self.motor.stop",
            "Emergency stop. Call this when you need to stop moving immediately."
            " Triggered by user commands like: 停, 停下, 停下来, 刹车, 别动, stop, halt, freeze."
            " This stops MY wheels — I stop myself.",
            PropertyList(),
            [stop_fn](const PropertyList& props) -> ReturnValue {
                stop_fn();
                return true;
            });

        mcp.AddTool("self.motor.run_test",
            "Run the motor test sequence. Each wheel spins for 2 seconds in order: left-front → right-front → left-rear → right-rear."
            " Useful for hardware debugging.",
            PropertyList(),
            [test_fn](const PropertyList& props) -> ReturnValue {
                test_fn();
                return true;
            });
    }
};

#endif
