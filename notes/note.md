## C++ 语法
- [**\#pragma once**](https://en.wikipedia.org/wiki/Pragma_once) \
用途与 [\include guards](https://en.wikipedia.org/wiki/Include_guard) 类似，保证同一个头文件在编译的时候只被包含一次，防止被重复引用，即 \#pragma once 类似于在 head.h 头文件中写
    > \#ifndef HEAD_H \
    > \#define HEAD_H \
    > ... \
    > \#endif