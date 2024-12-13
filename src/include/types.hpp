#ifndef TYPES_H
#define TYPES_H

#include <cstdio>

uint32_t float2byte(float f);
float hexStringToFloat(char *hex);

float parseHexStringToFloat(char *hex);
uint32_t parseHexStringToUint32(char *hex);
uint16_t parseHexStringToUint16(char *hex);
uint8_t parseHexStringToUint8(char *hex);

#endif