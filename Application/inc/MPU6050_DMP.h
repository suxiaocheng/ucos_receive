#ifndef MPU6050_DMP_H
#define MPU6050_DMP_H

#ifdef __cplusplus
extern "C" {
#endif

#include "includes.h"
#include "MPU6050.h"

#define PROGMEM
#define PGM_P  const char *
#define PSTR(str) (str)
#define F(x) x

typedef void prog_void;
typedef char prog_char;
typedef unsigned char prog_uchar;
typedef int8_t prog_int8_t;
typedef uint8_t prog_uint8_t;
typedef int16_t prog_int16_t;
typedef uint16_t prog_uint16_t;
typedef int32_t prog_int32_t;
typedef uint32_t prog_uint32_t;

#define pgm_read_byte(addr) (*(const unsigned char *)(addr))
#define pgm_read_word(addr) (*(const unsigned short *)(addr))
#define pgm_read_dword(addr) (*(const unsigned long *)(addr))
#define pgm_read_float(addr) (*(const float *)(addr))

#define pgm_read_byte_near(addr) pgm_read_byte(addr)
#define pgm_read_word_near(addr) pgm_read_word(addr)
#define pgm_read_dword_near(addr) pgm_read_dword(addr)
#define pgm_read_float_near(addr) pgm_read_float(addr)
#define pgm_read_byte_far(addr) pgm_read_byte(addr)
#define pgm_read_word_far(addr) pgm_read_word(addr)
#define pgm_read_dword_far(addr) pgm_read_dword(addr)
#define pgm_read_float_far(addr) pgm_read_float(addr)

uint32_t dmpInitialize(void) ;
uint8_t dmpGetQuaternion(int16_t *data, const uint8_t* packet);
uint16_t dmpGetFIFOPacketSize(void);

#ifdef __cplusplus
}
#endif

#endif


