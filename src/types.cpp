#include <cstdint>
#include "types.hpp"

typedef union
{
    float f;
    uint32_t u;
} float_byte_t;

uint32_t float2byte(float f)
{
    float_byte_t float_byte;
    float_byte.f = f;
    return float_byte.u;
}

float parseHexStringToFloat(char *hex)
{
    float_byte_t float_byte;
    sscanf(hex, "%x", &float_byte.u);
    return float_byte.f;
}

uint32_t parseHexStringToUint32(char *hex)
{
    uint32_t result;
    sscanf(hex, "%x", &result); // %x works for 32-bit hex input
    return result;
}

uint16_t parseHexStringToUint16(char *hex)
{
    uint16_t result;
    sscanf(hex, "%4hx", &result); // %4hx limits input to 16-bit hex
    return result;
}

uint8_t parseHexStringToUint8(char *hex)
{
    uint8_t result;
    sscanf(hex, "%2hhx", &result); // %2hhx limits input to 8-bit hex
    return result;
}