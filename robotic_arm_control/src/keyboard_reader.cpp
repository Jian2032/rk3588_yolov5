#include "keyboard_reader.hpp"
#include <iostream>
#include <fcntl.h>

KeyboardReader::KeyboardReader() : configured_(false)
{
    // 保存当前终端配置
    tcgetattr(STDIN_FILENO, &orig_settings_);
    struct termios new_settings = orig_settings_;

    // 关闭行缓冲和回显
    new_settings.c_lflag &= ~(ICANON | ECHO);
    tcsetattr(STDIN_FILENO, TCSANOW, &new_settings);

    // 设置 stdin 为非阻塞模式
    fcntl(STDIN_FILENO, F_SETFL, O_NONBLOCK);

    configured_ = true;
}

KeyboardReader::~KeyboardReader()
{
    if (configured_)
    {
        // 恢复终端配置
        tcsetattr(STDIN_FILENO, TCSANOW, &orig_settings_);
    }
}

int KeyboardReader::readKey()
{
    unsigned char c;
    if (read(STDIN_FILENO, &c, 1) > 0)
    {
        return c; // 返回按键的 ASCII 码
    }
    return -1; // 没有输入
}
