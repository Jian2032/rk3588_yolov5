#ifndef KEYBOARD_READER_H
#define KEYBOARD_READER_H

#include <termios.h>
#include <unistd.h>
#include <stdio.h>

class KeyboardReader
{
public:
    KeyboardReader();
    ~KeyboardReader();

    // 读取一个按键（非阻塞），如果没有按键返回 -1
    int readKey();

private:
    struct termios orig_settings_;
    bool configured_;
};

#endif // KEYBOARD_READER_H
