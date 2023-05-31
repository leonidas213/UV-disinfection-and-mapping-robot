#include "hmc5883l.h"

hmc5883l::hmc5883l(i2c_inst *i2c)
{
    this->i2cHdw = i2c;
}

hmc5883l::hmc5883l(i2csoft *i2c)
{
    this->i2csof = i2c;
}
void hmc5883l::begin()
{
    sleep_us(100);
    uint8_t msg[2] = {0x00, 0x00};
    msg[0] = ConfigRegA;
    msg[1] = sample_8 | outRate15 | mode_normal;
    if (isSoft)
        i2csof->writeBufferTo(devAddress, msg, 2);
    else
        i2c_write_blocking(i2cHdw, devAddress, msg, 2, false);
    msg[0] = ConfigRegB;
    msg[1] = gain_390;
    if (isSoft)
        i2csof->writeBufferTo(devAddress, msg, 2);
    else
        i2c_write_blocking(i2cHdw, devAddress, msg, 2, false);
    msg[0] = ModeReg;
    msg[1] = slow_i2c | continuous;
    if (isSoft)
        i2csof->writeBufferTo(devAddress, msg, 2);
    else
        i2c_write_blocking(i2cHdw, devAddress, msg, 2, false);
}
bool hmc5883l::canRead()
{
    uint8_t status = 0;
    int times = 0;
    sleep_ms(6);
    while (times < 10)
    {
        uint8_t msg = StatusReg;
        if (isSoft)
            status = i2csof->readByteFrom(devAddress, msg);
        else
            i2c_read_blocking(i2cHdw, devAddress, &status, 1, false);

        sleep_us(2);
        times += 1;
        if (status > 0)
        {

            // printf("status: %d\n", status);
            return true;
        }
    }
    printf("status: %d\n", status);
    return false;
}

void TOtwosCompliment(int *data)
{
    data[0] = (data[0] > 0x7FFF) ? (data[0] - 0xFFFF) : data[0];
    data[1] = (data[1] > 0x7FFF) ? (data[1] - 0xFFFF) : data[1];
    data[2] = (data[2] > 0x7FFF) ? (data[2] - 0xFFFF) : data[2];
}

void hmc5883l::read(int *data)
{

    // if (canRead())
    //{
    uint8_t buffer[6];
    uint8_t msg = 0x03;
    if (isSoft)
        i2csof->readBufferFrom(devAddress, msg, buffer, 6);
    else
        i2c_read_blocking(i2cHdw, devAddress, buffer, 6, false);
    data[0] = ((buffer[0] << 8) | buffer[1]);
    data[1] = ((buffer[4] << 8) | buffer[5]);
    data[2] = ((buffer[2] << 8) | buffer[3]);
    TOtwosCompliment(data);
    return;
    //}

    data[0] = 0;
    data[1] = 0;
    data[2] = 0;
}

float hmc5883l::heading()
{
    int data[3];
    read(data);
    float heading = atan2(data[1], data[0]);
    if (heading < 0)
        heading += 2 * M_PI;
    if (heading > 2 * M_PI)
        heading -= 2 * M_PI;
    return (heading * 180 / M_PI) + declination;
}
float hmc5883l::heading(int *data)
{
    float heading = atan2(data[1], data[0]);
    if (heading < 0)
        heading += 2 * M_PI;
    if (heading > 2 * M_PI)
        heading -= 2 * M_PI;
    return (heading * 180 / M_PI) + declination;
}