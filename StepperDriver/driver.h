
#include "pico/stdlib.h"
#include "math.h"
#include <stdio.h>
#define Pi 3.14159265
enum StepMode
{
    halfStep,
    fullStep
};
class Stepper
{

public:
    Stepper(int pin1, int pin2, int pin3, int pin4, StepMode mode = StepMode::fullStep);
    void Stop();
    void turnLeft(bool count = true);
    void turnRight(bool count = true);
    void initialize();

    unsigned long TotalSteps = 0;
    unsigned long forwardSteps = 0;
    unsigned long backwardSteps = 0;
    StepMode mode;

private:
    void halfStep(bool dir);
    void fullStep(bool dir);
    int stepNum = 0;
    uint pin1, pin2, pin3, pin4;
};
struct command
{
    char direction;
    int counterTimer;
    bool continueTurning;
    uint64_t TurnTimer;
};
class driver
{

public:
    driver(Stepper *Left, Stepper *Right);
    void initialize();
    void forward();
    void turnLeft();
    void turnRight();
    void backward();
    void stop();
    void calculateDistance(double yaw);
    bool update();
    void setCommand(command direction);
    long double x = 0, y = 0;
    float initialYaw;
    float *angle;
    float *magAngle;
    float initialmagAngle;

    bool isDone;

private:
    bool isBetween(float input, float refValue, float offset);

    command currentCommand;
    float controllerYaw;
    float turningInitialYaw;
    long double distance;
    unsigned long totalSteps = 0;
    // int wheel_diameter = 6; // cm çap
    int wheel_circumference = 300 * Pi * 2;

    int pin1, pin2, pin3, pin4;
    Stepper *LeftMotor, *RightMotor;
    uint stepNum = 0;

    uint64_t TurnTimer, DistanceUpdateTime, contTime;
    int counterTimer;
    bool continueTurning, controllerOnce = true;
    char direction = 's';
};