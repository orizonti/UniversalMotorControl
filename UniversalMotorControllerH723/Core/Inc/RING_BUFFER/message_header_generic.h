#ifndef MESSAGE_HEADER_GENERIC_H
#define MESSAGE_HEADER_GENERIC_H
#include <stdint.h>

class MESSAGE_HEADER_GENERIC
{
  public:
    uint16_t HEADER = 0x8220; 
    uint16_t  MESSAGE_IDENT = 1;
    uint16_t DATA_SIZE = 1; 
    uint16_t DATA_SIZE2 = 1; 
    uint32_t MESSAGE_NUMBER = 0x0;

    bool isValid() const { return HEADER == 0x8220; }
};



#endif //MESSAGE_HEADER_GENERIC_H
