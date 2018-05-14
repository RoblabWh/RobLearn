#ifndef STATUS_HELPER_H
#define STATUS_HELPER_H

#include <iostream>

#define MESSAGE_INFO(message)  (std::cout << "[INFO]: " << "\x1b[39;49m" << message << "\x1b[39;49m" << std::endl)
#define MESSAGE_WARN(message)  (std::cout << "[WARN]: " << "\x1b[33;49m" << message << "\x1b[39;49m" << std::endl)
#define MESSAGE_ERROR(message) (std::cout << "[ERROR]: " << "\x1b[31;49m" << message << "\x1b[39;49m" << std::endl)

#endif //STATUS_HELPER_H
