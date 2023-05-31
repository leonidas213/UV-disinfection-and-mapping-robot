#include "pico/stdlib.h"
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include "stdlib.h"

#include "hardware/pio.h"
#include "hardware/pwm.h"
#include "hardware/uart.h"
#include "hardware/timer.h"
#include "hardware/regs/rosc.h"       //random number gen
#include "hardware/regs/addressmap.h" //random number gen
#include "pico/binary_info.h"

#include "pico/util/queue.h"
#include "pico/multicore.h"
#include "pico/bootrom.h"

#include "StepperDriver/driver.h"
#include "MPU6050/MPU6050_6Axis_MotionApps_V6_12.h"
#include "hmc5883l/hmc5883l.h"
#include "mapping/mapping.h"

#pragma region Variables
double hcLeft = -1, hcFront = -1, hcRight = -1;
mapping mapper = mapping(&hcLeft, &hcFront, &hcRight);

i2c_inst *i2c = i2c0;
i2c_inst *maini2c = i2c1;
uart_inst *uart = uart0;

hc04 distanceHc = hc04(pio0);
//-----timers-----
repeating_timer_t timer;
//----controller------------------------------------------------------

Stepper leftMotor = Stepper(15, 16, 17, 18);
Stepper rightMotor = Stepper(19, 20, 21, 22);
driver Steppercontroller = driver(&leftMotor, &rightMotor);

//-------------------
int compassBuf[3];
hmc5883l compass = hmc5883l(maini2c);
//---------mpu6050--------

MPU6050 mpu;
int mpuCounter = 0;
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

Quaternion q;        // [w, x, y, z]         quaternion container
VectorInt16 aa;      // [x, y, z]            accel sensor measurements
VectorInt16 gy;      // [x, y, z]            gyro sensor measurements
VectorInt16 aaReal;  // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld; // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity; // [x, y, z]            gravity vector
float euler[3];      // [psi, theta, phi]    Euler angle container
float ypr[3];        // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector
float yaw, pitch, roll;

bool mpuInterrupt = false; // indicates whether MPU interrupt pin has gone high

//---------mpu6050--------
//----------------------------------------------------------
#pragma endregion
float getDistance(int echo, int trig)
{
    gpio_put(trig, 1);
    sleep_us(10);
    gpio_put(trig, 0);

    uint64_t width = 0;
    uint64_t width2 = 0;

    while (gpio_get(echo) == 0)
    {
        sleep_us(1);
        width2++;
        if (width2 > 26000)
            return 0;
    }
    absolute_time_t startTime = get_absolute_time();
    while (gpio_get(echo) == 1)
    {
        width++;
        sleep_us(1);
        if (width > 26000)
            return 0;
    }
    absolute_time_t endTime = get_absolute_time();

    return absolute_time_diff_us(startTime, endTime) / 29.0f / 2.0f;
}

#pragma region mapping

void MapperMapping()
{

    mapper.CreateMap(25, 25);
    mapper.updatePosition(12, 12);

    /// is there a map
    uint8_t isThereAMap = 0;
    const uint8_t regAd[] = {0x00, 0x03};

    i2c_write_blocking(i2c0, 0x50, regAd, 2, true);
    i2c_read_blocking(i2c0, 0x50, (uint8_t *)isThereAMap, 1, false);
    if (isThereAMap == 1)
    {
        // check the map data
        const uint8_t regAd[] = {0x00, 0x10};
        uint8_t mapSize[] = {0, 0};

        i2c_write_blocking(i2c0, 0x50, regAd, 2, true);
        i2c_read_blocking(i2c0, 0x50, mapSize, 2, false);

        mapper.width = mapSize[0];
        mapper.height = mapSize[1];
        uint8_t *mapBufer = (uint8_t *)malloc(mapper.width * mapper.height * sizeof(uint8_t));
        const uint8_t regAd2[] = {0x00, 0x20};
        i2c_write_blocking(i2c0, 0x50, regAd2, 2, true);
        i2c_read_blocking(i2c0, 0x50, mapBufer, int((mapper.width * mapper.height / 3) + 1), false);
        mapper.putCharMap((char *)mapBufer, int((mapper.width * mapper.height / 3) + 1));
    }

    bool executingCommand = false;
    mapCom pos;
    command c;
    for (int i = 0; i < 3; i++)
    {

        hcFront = getDistance(14, 13);
        hcLeft = getDistance(12, 11);
        hcRight = getDistance(10, 9);
    }

    printf("#RTPOR;%d;%d;%d;%d;%d;%d;", mapper.posx, mapper.posy, mapper.leftBlocked, mapper.frontBlocked, mapper.rightBlocked, mapper.direction);
    printf("%.2f;%.2f;%.2f;\n", hcLeft, hcFront, hcRight); // pos = mapper.getPos();
    int a = 0b00000000;
    bool turBack = false;
    while (true)
    {
        printf("#MPDAT;%d;%d;", mapper.height, mapper.width);
        int size;
        uart_write_blocking(uart, (const uint8_t*)mapper.getCharmap(&size), size); // mapper.getCharmap()
        printf("\n");
        if (uart_is_readable(uart))
        {
            uint8_t ch = uart_getc(uart);
            if (ch != '\r' & ch != '\n')
            {
                if (ch == 'u')
                {

                    reset_usb_boot(0, 0);
                }
                if (ch == 'm')
                {
                    return;
                }
            }
        }

        if (executingCommand)
        {
            if (Steppercontroller.update())
            {
                executingCommand = false;
            }
        }
        else
        {
            if (pos.rotate == 2)
            {
                turBack = true;
            }
            if (pos.rotate > 0)
            {
                printf("rotate\n");
                c.direction = pos.direction ? 'l' : 'r';
                c.counterTimer = 2;
                c.continueTurning = true;

                c.TurnTimer = time_us_64();

                pos.rotate -= 1;
                Steppercontroller.setCommand(c);
                executingCommand = true;
            }
            else if (pos.rotate == 0 && !turBack)
            {
                printf("forw after rotate\n");
                c.direction = 'f';
                c.counterTimer = 1800;
                pos.rotate = -3;
                c.continueTurning = false;

                Steppercontroller.setCommand(c);
                executingCommand = true;
            }
            else if (pos.rotate == -1)
            {
                printf("forward\n");
                c.direction = 'f';
                c.counterTimer = 1800;
                c.continueTurning = false;
                pos.rotate -= 3;
                Steppercontroller.setCommand(c);
                executingCommand = true;
            }
            else
            {
                for (int i = 0; i < 4; i++)
                {
                    hcFront = getDistance(14, 13);
                    hcLeft = getDistance(12, 11);
                    hcRight = getDistance(10, 9);
                }

                turBack = false;
                printf("mapUpdate\n");
                mapper.updateMap();
                pos = mapper.getPos();

                printf("#RTPOR;%d;%d;%d;%d;%d;%d;", mapper.posx, mapper.posy, mapper.leftBlocked, mapper.frontBlocked, mapper.rightBlocked, mapper.direction);
                printf("%.2f;%.2f;%.2f;\n", hcLeft, hcFront, hcRight);
            }

            if (mapper.Goal())
            {
                int size = 0;
                char *mapDat = mapper.getCharmap(&size);
                int eprom = 0;
                char *values = (char *)malloc(128);

                // the is a map
                const uint8_t regAd[] = {0x00, 0x03};

                i2c_write_blocking(i2c0, 0x50, regAd, 2, true);
                i2c_write_blocking(i2c0, 0x50, (const uint8_t *)0x01, 1, false);

                // map size location
                const uint8_t regAd[] = {0x00, 0x10};
                const uint8_t mapSize[] = {mapper.height, mapper.width};

                i2c_write_blocking(i2c0, 0x50, regAd, 2, true);
                i2c_write_blocking(i2c0, 0x50, mapSize, 2, false);

                // map data location
                const uint8_t regAd[] = {0x00, 0x70};
                i2c_write_blocking(i2c0, 0x50, regAd, 2, true);
                for (int i = 0; i < size; i++)
                {
                    values[eprom] = mapDat[i];
                    eprom++;
                    if (eprom == 128)
                    {
                        i2c_write_blocking(i2c0, 0x50, (const uint8_t *)values, 128, false);
                        eprom = 0;
                        sleep_ms(40); // need to wait for the eeprom to write the page
                    }
                }
                if (eprom != 0)
                {
                    i2c_write_blocking(i2c0, 0x50, (const uint8_t *)values, eprom, false);
                }
                free(values);
                printf("map saved\n");

                break;
            }
        }
    }
}

bool timer_callbacks(repeating_timer_t *rt)
{
    Steppercontroller.update();
    return true;
}

#pragma endregion

#pragma region HelpfulFunctions

void seed_random_from_rosc()
{
    uint32_t random = 0x811c9dc5;
    uint8_t next_byte = 0;
    volatile uint32_t *rnd_reg = (uint32_t *)(ROSC_BASE + ROSC_RANDOMBIT_OFFSET);

    for (int i = 0; i < 16; i++)
    {
        for (int k = 0; k < 8; k++)
        {
            next_byte = (next_byte << 1) | (*rnd_reg & 1);
        }

        random ^= next_byte;
        random *= 0x01000193;
    }

    srand(random);
}

void waitUsb()
{
    gpio_init(25);
    gpio_set_dir(25, GPIO_OUT);

    while (!stdio_usb_connected())
    {
        if (uart_is_readable(uart))
        {
            uint8_t ch = uart_getc(uart);
            if (ch == 'f')
            {

                break;
            }
        }
        gpio_put(25, 1);
        sleep_ms(250);
        gpio_put(25, 0);
        sleep_ms(250);
    }
}
bool isBetween(float input, float refValue, float offset)
{ // 120  100    90
    // min=10 max=190

    // difference between 2 degrees
    float diff = abs(input - refValue);
    if (diff > 180)
    {
        diff = 360 - diff;
    }

    return diff < offset;
}
#pragma endregion

#pragma region Interrupts

void mpuRead(uint gpio, uint32_t events)
{
    mpuCounter++;
    if (mpuCounter >= 2)
    {
        mpuCounter = 0;
        mpuInterrupt = true;
        if (fifoCount < packetSize)
        {
            fifoCount = mpu.getFIFOCount(); // wait for correct available data length, should be a VERY short wait
        }
        else
        {
            mpu.getFIFOBytes(fifoBuffer, packetSize);
        }
    }

}
bool hmcOnce = true;
void hmcRead(uint gpio, uint32_t events)
{


    compass.read(compassBuf);
    // printf("x: %d, y: %d, z: %d\n", compassBuf[0], compassBuf[1], compassBuf[2]);

}
#pragma endregion

void core1_entry()
{ //---------mpu6050--------100hz
    bool completed;
    printf("starting mpu\n");
    mpu.initialize(i2c);
    devStatus = mpu.dmpInitialize();

    printf("calibrating mpu\n");
    mpu.CalibrateAccel(6);
    mpu.CalibrateGyro(6);
    mpu.PrintActiveOffsets();
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true); // turn on the DMP, now that it's ready
        // mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;                         // set our DMP Ready flag so the main loop() function knows it's okay to use it
        packetSize = mpu.dmpGetFIFOPacketSize(); // get expected DMP packet size for later comparison
        printf("success\n");
        completed = true;
        gpio_set_irq_enabled_with_callback(8, GPIO_IRQ_EDGE_FALL, true, &mpuRead);
    }
    else
    { // ERROR!        1 = initial memory load failed         2 = DMP configuration updates failed        (if it's going to break, usually the code will be 1)
        printf("DMP Initialization failed (code %d)", devStatus);
        sleep_ms(2000);
    }
    //---------mpu6050--------

    while (1)
    {

        //---------mpu6050--------
        if (dmpReady)
        // if programming failed, don't try to do anything
        {
            if (mpuInterrupt)
            {

                mpuIntStatus = mpu.getIntStatus();
                mpuInterrupt = false;
                fifoCount = mpu.getFIFOCount(); // get current FIFO count
                // printing mpuIntStatus in hex

                if ((mpuIntStatus & 0x10) || fifoCount == 1024) // check for overflow (this should never happen unless our code is too inefficient)
                {
                    mpu.resetFIFO(); // reset so we can continue cleanly
                    // printf("FIFO overflow!");
                }
                else if (mpuIntStatus & 0x01) // otherwise, check for DMP data ready interrupt (this should happen frequently)
                {

                    while (fifoCount < packetSize)
                        fifoCount = mpu.getFIFOCount();       // wait for correct available data length, should be a VERY short wait
                    mpu.getFIFOBytes(fifoBuffer, packetSize); // read a packet from FIFO

                    fifoCount -= packetSize; // track FIFO count here in case there is > 1 packet available
                                             // display Euler angles in degrees
                    mpu.dmpGetQuaternion(&q, fifoBuffer);
                    mpu.dmpGetGravity(&gravity, &q);
                    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
                    yaw = (ypr[0] * 180 / PI) > 0 ? (ypr[0] * 180 / PI) : 360 + (ypr[0] * 180 / PI);
                    // yaw = (ypr[0] * 180 / PI);
                    pitch = (ypr[1] * 180 / PI) > 0 ? (ypr[1] * 180 / PI) : 360 + (ypr[1] * 180 / PI);
                    roll = (ypr[2] * 180 / PI) > 0 ? (ypr[2] * 180 / PI) : 360 + (ypr[2] * 180 / PI);
                    // printf("ypr: %f,\t %f,\t %f\n", yaw, pitch, roll);

                    // display real acceleration, adjusted to remove gravity
                    mpu.dmpGetAccel(&aa, fifoBuffer);
                    mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
                    // printf("areal: %d,\t %d,\t %d\n", aaReal.x, aaReal.y, aaReal.z);
                }
            }
        }
        //---------mpu6050--------

    }
}

int main()
{

    stdio_init_all();

    Steppercontroller.initialize();
    seed_random_from_rosc();

    //------------i2c------------

    bi_decl(bi_2pins_with_func(4, 5, GPIO_FUNC_I2C)); // debug
    bi_decl(bi_2pins_with_func(2, 3, GPIO_FUNC_I2C)); // debug

    i2c_init(i2c, 300 * 1000);
    i2c_init(maini2c, 200 * 1000);

    gpio_set_function(4, GPIO_FUNC_I2C); // i2c1
    gpio_set_function(5, GPIO_FUNC_I2C); // i2c1
    gpio_pull_up(4);                     // i2c1
    gpio_pull_up(5);                     // i2c1
    gpio_set_function(3, GPIO_FUNC_I2C); // i2c0
    gpio_set_function(2, GPIO_FUNC_I2C); // i2c0
    gpio_pull_up(3);                     // i2c0
    gpio_pull_up(2);                     // i2c0

    //------------i2c------------s

    //---------bluetooth------
    gpio_init(0);
    gpio_init(1);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
    stdio_uart_init_full(uart, 115200, 0, 1);
    // uart_init(uart0, 115200);
    //---------bluetooth------

    //---------hc-sr04--------
   
    gpio_init(14);
    gpio_init(13);
    gpio_init(12);
    gpio_init(11);
    gpio_init(10);
    gpio_init(9);

    gpio_set_dir(14, GPIO_IN);
    gpio_set_dir(13, GPIO_OUT);

    gpio_set_dir(12, GPIO_IN);
    gpio_set_dir(11, GPIO_OUT);

    gpio_set_dir(10, GPIO_IN);
    gpio_set_dir(9, GPIO_OUT);
    //---------hc-sr04--------

    waitUsb();
 
    string uartvalue = "";


    compass.begin();
    compass.declination = 5.967f; // 5 58' E // https://www.magnetic-declination.com/

    printf("started main\n");
    multicore_launch_core1(core1_entry);
    sleep_ms(5000);

    gpio_set_irq_enabled_with_callback(7, GPIO_IRQ_EDGE_FALL, true, &hmcRead);

    printf("ColdStart\n");

    sleep_ms(500);
    printf("finished\n");

    // if (!add_repeating_timer_us(2, timer_callbacks, NULL, &timer))
    //     printf("cant added calback");
    Steppercontroller.angle = &yaw;
    Steppercontroller.initialYaw = yaw;

    Steppercontroller.magAngle = &compass.headingValue;
    Steppercontroller.initialmagAngle = compass.headingValue;
    uint64_t messaprinter = time_us_64();
    while (1)
    {

        Steppercontroller.update();
        

        if (uart_is_readable(uart))
        {
            uint8_t ch = uart_getc(uart);
            if (ch != '\r' & ch != '\n')
            {
                if (ch == 'u')
                {

                    reset_usb_boot(0, 0);
                }

                printf("ypr: %f, %f, %f\n", yaw, pitch, roll);

                command contCom;

                switch (ch)
                {
                case 'f':
                    contCom.direction = 'f';
                    contCom.counterTimer = 1900;
                    contCom.continueTurning = false;
                    Steppercontroller.setCommand(contCom);
                    printf("turning forward\n");

                    break;
                case 'b':
                    contCom.direction = 'b';
                    contCom.counterTimer = 1900;
                    contCom.continueTurning = false;
                    Steppercontroller.setCommand(contCom);
                    printf("turning backward\n");
                    break;
                case 'l':
                    contCom.direction = 'l';
                    contCom.counterTimer = 2;
                    contCom.continueTurning = true;
                    contCom.TurnTimer = time_us_64();

                    printf("turning left\n");

                    printf("ypr: %.3f, %.3f, %.3f\n", yaw, pitch, roll);
                    Steppercontroller.setCommand(contCom);
                    break;
                case 'r':
                    contCom.direction = 'r';
                    contCom.counterTimer = 2;
                    contCom.continueTurning = true;
                    contCom.TurnTimer = time_us_64();

                    printf("turning right\n");

                    printf("ypr: %.3f, %.3f, %.3f\n", yaw, pitch, roll);
                    Steppercontroller.setCommand(contCom);
                    break;

                case 'm':

                    MapperMapping();
                    printf("mappingDone");

                    break;
                default:
                    contCom.direction = 's';
                    printf("%c\n", ch);
                    Steppercontroller.setCommand(contCom);
                    break;
                }
            }
        }
    }
}
