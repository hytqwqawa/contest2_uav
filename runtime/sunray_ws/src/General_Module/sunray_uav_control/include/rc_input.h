#ifndef RC_INPUT_H
#define RC_INPUT_H

#include <ros/ros.h>
#include <mavros_msgs/RCIn.h>
#include <cmath>
#include <iomanip>
#include <iostream>

#include "printf_utils.h"

class RC_Input
{
public:
    RC_Input(){};
    void init();
    void handle_rc_data(mavros_msgs::RCInConstPtr pMsg);
    void printf_info();

    double ch[4];     // 1-4通道数值: roll pitch yaw thrust

    double channel_5; // 通道5（二或三段）- arm/disarm，保留原逻辑
    double channel_6; // 通道6（三段）    - PX4/QGC flight mode，sunray不再使用
    double channel_7; // 通道7（三段）    - sunray切换 INIT / RC_CONTROL / CMD_CONTROL
    double channel_8; // 通道8（二段）    - PX4 emergency kill，sunray不再使用
    double channel_9; // 通道9（二段）    - 未启用

    double last_channel_5;
    double last_channel_6;
    double last_channel_7;
    double last_channel_8;

    bool have_init_channel_5{false};
    bool have_init_channel_6{false};
    bool have_init_channel_7{false};
    bool have_init_channel_8{false};

    mavros_msgs::RCIn msg;
    ros::Time rcv_stamp; // 收到遥控器消息的时间

    bool enter_init;
    bool in_init;
    bool enter_rc_pos_control;
    bool in_rc_pos_control;
    bool enter_command_control;
    bool in_command_control;

    bool toggle_arm;
    bool toggle_disarm;
    bool toggle_kill;
    bool toggle_reboot;
    bool toggle_land;

private:
    static constexpr double channel_5_threshold_value_1 = 0.25;
    static constexpr double channel_5_threshold_value_2 = 0.75;

    // CH6 现在只给 PX4/QGC 使用，保留变量和打印，但不再用于 sunray 模式切换
    static constexpr double channel_6_threshold_value_1 = 0.25;
    static constexpr double channel_6_threshold_value_2 = 0.75;

    // CH7 改为 sunray 三段模式切换
    static constexpr double channel_7_threshold_value_1 = 0.25;
    static constexpr double channel_7_threshold_value_2 = 0.75;

    // CH8 由 PX4 解析为 emergency kill，sunray 不再使用
    static constexpr double channel_8_threshold_value = 0.75;

    static constexpr double DEAD_ZONE = 0.05; // 死区

    void check_validity();
};

void RC_Input::init()
{
    rcv_stamp = ros::Time(0);

    for (int i = 0; i < 4; ++i)
    {
        ch[i] = 0.0;
    }

    channel_5 = 0.0;
    channel_6 = 0.0;
    channel_7 = 0.0;
    channel_8 = 0.0;
    channel_9 = 0.0;

    last_channel_5 = -1.0;
    last_channel_6 = -1.0;
    last_channel_7 = -1.0;
    last_channel_8 = -1.0;

    have_init_channel_5 = false;
    have_init_channel_6 = false;
    have_init_channel_7 = false;
    have_init_channel_8 = false;

    toggle_arm = false;
    toggle_disarm = false;
    toggle_kill = false;
    toggle_reboot = false;
    toggle_land = false;

    enter_init = false;
    in_init = false;

    enter_command_control = false;
    in_command_control = false;

    enter_rc_pos_control = false;
    in_rc_pos_control = false;
}

void RC_Input::printf_info()
{
    std::cout << GREEN << ">>>>>>>>>>>>>>>>>> RC INFO  <<<<<<<<<<<<<<<<<<" << TAIL << std::endl;

    std::cout.setf(std::ios::fixed);
    std::cout << std::setprecision(2);
    std::cout.setf(std::ios::left);
    std::cout.setf(std::ios::showpoint);
    std::cout.setf(std::ios::showpos);

    std::cout << GREEN << "ch1 : " << ch[0] << TAIL << std::endl;
    std::cout << GREEN << "ch2 : " << ch[1] << TAIL << std::endl;
    std::cout << GREEN << "ch3 : " << ch[2] << TAIL << std::endl;
    std::cout << GREEN << "ch4 : " << ch[3] << TAIL << std::endl;

    std::cout << GREEN << "ch5(arm/disarm)     : " << channel_5
              << "  last_ch5 : " << last_channel_5 << TAIL << std::endl;

    std::cout << GREEN << "ch6(PX4 flight mode): " << channel_6
              << "  last_ch6 : " << last_channel_6 << TAIL << std::endl;

    std::cout << GREEN << "ch7(sunray mode)    : " << channel_7
              << "  last_ch7 : " << last_channel_7 << TAIL << std::endl;

    std::cout << GREEN << "ch8(PX4 kill)       : " << channel_8
              << "  last_ch8 : " << last_channel_8 << TAIL << std::endl;

    std::cout << GREEN << "toggle_arm : " << toggle_arm << TAIL << std::endl;
    std::cout << GREEN << "toggle_disarm : " << toggle_disarm << TAIL << std::endl;
    std::cout << GREEN << "toggle_kill : " << toggle_kill << TAIL << std::endl;
    std::cout << GREEN << "toggle_land : " << toggle_land << TAIL << std::endl;
    std::cout << GREEN << "toggle_reboot : " << toggle_reboot << TAIL << std::endl;

    std::cout << GREEN << "enter_init : " << enter_init << TAIL << std::endl;
    std::cout << GREEN << "in_init : " << in_init << TAIL << std::endl;

    std::cout << GREEN << "enter_rc_pos_control : " << enter_rc_pos_control << TAIL << std::endl;
    std::cout << GREEN << "in_rc_pos_control : " << in_rc_pos_control << TAIL << std::endl;

    std::cout << GREEN << "enter_command_control : " << enter_command_control << TAIL << std::endl;
    std::cout << GREEN << "in_command_control : " << in_command_control << TAIL << std::endl;
}

void RC_Input::handle_rc_data(mavros_msgs::RCInConstPtr pMsg)
{
    msg = *pMsg;
    rcv_stamp = ros::Time::now();

    if (msg.channels.size() < 8)
    {
        ROS_ERROR_THROTTLE(1.0,
                           "RC channels size is %lu, expected at least 8.",
                           msg.channels.size());

        toggle_arm = false;
        toggle_disarm = false;
        toggle_kill = false;
        toggle_reboot = false;
        toggle_land = false;
        return;
    }

    if (msg.channels[0] < 1100 &&
        msg.channels[1] < 1100 &&
        msg.channels[2] < 1100 &&
        msg.channels[3] > 1900)
    {
        toggle_reboot = true;
        return;
    }
    else
    {
        toggle_reboot = false;
    }

    // 1-4通道数值
    for (int i = 0; i < 4; i++)
    {
        // 映射至[-1,1]区间
        ch[i] = ((double)msg.channels[i] - 1500.0) / 500.0;

        if (ch[i] > DEAD_ZONE)
        {
            ch[i] = (ch[i] - DEAD_ZONE) / (1.0 - DEAD_ZONE);
        }
        else if (ch[i] < -DEAD_ZONE)
        {
            ch[i] = (ch[i] + DEAD_ZONE) / (1.0 - DEAD_ZONE);
        }
        else
        {
            ch[i] = 0.0;
        }
    }

    // 5通道，映射至[0,1]区间
    channel_5 = ((double)msg.channels[4] - 1000.0) / 1000.0;

    // 6通道，映射至[0,1]区间
    // 注意：CH6现在只由PX4/QGC作为flight mode channel使用，sunray不再使用它切控制模式
    channel_6 = ((double)msg.channels[5] - 1000.0) / 1000.0;

    // 7通道，映射至[0,1]区间
    // 注意：CH7现在作为sunray的 INIT / RC_CONTROL / CMD_CONTROL 三段开关
    channel_7 = ((double)msg.channels[6] - 1000.0) / 1000.0;

    // 8通道，映射至[0,1]区间
    // 注意：CH8已在PX4中配置为RC_MAP_KILL_SW=8，sunray不再使用它触发Land
    channel_8 = ((double)msg.channels[7] - 1000.0) / 1000.0;

    // 检验有效性
    check_validity();

    // 初始化last值，避免节点启动时因为开关初始位置触发边沿事件
    if (!have_init_channel_5)
    {
        have_init_channel_5 = true;
        last_channel_5 = channel_5;
    }

    if (!have_init_channel_6)
    {
        have_init_channel_6 = true;
        last_channel_6 = channel_6;
    }

    if (!have_init_channel_7)
    {
        have_init_channel_7 = true;
        last_channel_7 = channel_7;
    }

    if (!have_init_channel_8)
    {
        have_init_channel_8 = true;
        last_channel_8 = channel_8;
    }

    // 判断通道5 - arm/disarm
    // 保留原逻辑：CH5上跳到高位触发arm，CH5下跳到低位触发disarm
    if (last_channel_5 < channel_5_threshold_value_2 &&
        channel_5 > channel_5_threshold_value_2)
    {
        toggle_arm = true;
        toggle_disarm = false;
    }
    else if (last_channel_5 > channel_5_threshold_value_1 &&
             channel_5 < channel_5_threshold_value_1)
    {
        toggle_arm = false;
        toggle_disarm = true;
    }
    else
    {
        toggle_arm = false;
        toggle_disarm = false;
    }

    // CH6由PX4/QGC作为Flight Mode Channel解析：
    // Stabilized / Altitude / Position 等模式切换仍然由PX4处理。
    // sunray不再使用CH6切换 INIT / RC_CONTROL / CMD_CONTROL。

    // 判断通道7 - sunray控制模式切换
    // CH7低位  -> INIT
    // CH7中位  -> RC_CONTROL
    // CH7高位  -> CMD_CONTROL
    if ((std::fabs(channel_7 - last_channel_7) > 0.4) &&
        channel_7 <= channel_7_threshold_value_1)
    {
        enter_init = true;
        enter_rc_pos_control = false;
        enter_command_control = false;
    }
    else if ((std::fabs(channel_7 - last_channel_7) > 0.4) &&
             (channel_7 > channel_7_threshold_value_1 &&
              channel_7 < channel_7_threshold_value_2))
    {
        enter_init = false;
        enter_rc_pos_control = true;
        enter_command_control = false;
    }
    else if ((std::fabs(channel_7 - last_channel_7) > 0.4) &&
             channel_7 >= channel_7_threshold_value_2)
    {
        enter_init = false;
        enter_rc_pos_control = false;
        enter_command_control = true;
    }

    // CH7现在用于sunray三段控制模式切换，不再触发sunray KILL
    toggle_kill = false;

    // CH8由PX4作为Emergency Kill解析，sunray不再使用CH8触发Land
    toggle_land = false;

    last_channel_5 = channel_5;
    last_channel_6 = channel_6;
    last_channel_7 = channel_7;
    last_channel_8 = channel_8;
}

void RC_Input::check_validity()
{
    if (channel_5 >= -1.1 && channel_5 <= 1.1 &&
        channel_6 >= -1.1 && channel_6 <= 1.1 &&
        channel_7 >= -1.1 && channel_7 <= 1.1 &&
        channel_8 >= -1.1 && channel_8 <= 1.1)
    {
        // pass
    }
    else
    {
        ROS_ERROR("RC data validity check fail. channel_5=%f, channel_6=%f, channel_7=%f, channel_8=%f",
                  channel_5, channel_6, channel_7, channel_8);
    }
}

#endif