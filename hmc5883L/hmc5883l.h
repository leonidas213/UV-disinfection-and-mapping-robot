#include "pico/stdlib.h"
#include "hardware/i2c.h"
#include <stdio.h>
using namespace std;
#include <string.h>
#include <string>
#include "../i2csoft/i2csoft.h"
#include "math.h"
#ifndef hmc5883l_h
#define hmc5883l_h

#define devAddress 0x1e // 0x1E

#define ConfigRegA 0x00
#define ConfigRegB 0x01
#define ModeReg 0x02
#define DataOutXMSB 0x03
#define DataOutXLSB 0x04
#define DataOutZMSB 0x05
#define DataOutZLSB 0x06
#define DataOutYMSB 0x07
#define DataOutYLSB 0x08
#define StatusReg 0x09
#define IDRegA 0x0A
#define IDRegB 0x0B
#define IDRegC 0x0C

#define outRate0_75 0b00000
#define outRate1_5 0b00100
#define outRate3 0b01000
#define outRate7_5 0b01100
#define outRate15 0b10000
#define outRate30 0b10100
#define outRate75 0b11000

#define sample_1 0b0000000
#define sample_2 0b0100000
#define sample_4 0b0100000
#define sample_8 0b1100000

#define mode_normal 0x00
#define mode_bias_pos 0x01
#define mode_bias_neg 0x02

#define gain_1370 0b00000000
#define gain_1090 0b00100000
#define gain_820 0b01000000
#define gain_660 0b01100000
#define gain_440 0b10000000
#define gain_390 0b10100000
#define gain_330 0b11000000
#define gain_230 0b11100000

#define i2c_34000kHz 0b10000000
#define slow_i2c 0b00000000

#define continuous 0x00
#define single 0x01

class hmc5883l
{

public:
    hmc5883l(i2c_inst *);
    hmc5883l(i2csoft *);
    void read(int *);
    void begin();
    float heading();
    float heading(int *data);
    float declination = 0.0;
    float headingValue = 0.0;

private:
    bool isSoft;
    bool canRead();
    i2csoft *i2csof;
    i2c_inst *i2cHdw;
};

#endif