#pragma once
#include "driver.h"

Stepper::Stepper(int pin1, int pin2, int pin3, int pin4, StepMode mode)
{

    this->pin1 = pin1;
    this->pin2 = pin2;
    this->pin3 = pin3;
    this->pin4 = pin4;
    this->mode = mode;
}
void Stepper::initialize()
{
    gpio_init(pin1);
    gpio_init(pin2);
    gpio_init(pin3);
    gpio_init(pin4);
    gpio_set_dir(pin1, 1);
    gpio_set_dir(pin2, 1);
    gpio_set_dir(pin3, 1);
    gpio_set_dir(pin4, 1);
    gpio_put(pin1, 0);
    gpio_put(pin2, 0);
    gpio_put(pin3, 0);
    gpio_put(pin4, 0);
}
void Stepper::Stop()
{
    gpio_put(pin1, 0);
    gpio_put(pin2, 0);
    gpio_put(pin3, 0);
    gpio_put(pin4, 0);
}

void Stepper::halfStep(bool direction)
{
    if (direction)
    {
        stepNum -= 1;
        if (stepNum < 0)
        {
            stepNum = 7;
        }
    }

    else
    {
        stepNum += 1;
        if (stepNum > 7)
        {
            stepNum = 0;
        }
    }

    switch (stepNum)
    {
    case 0: // 1010
        gpio_put(pin1, 0);
        gpio_put(pin2, 1);
        gpio_put(pin3, 1);
        gpio_put(pin4, 1);
        break;
    case 1: // 0110
        gpio_put(pin1, 0);
        gpio_put(pin2, 0);
        gpio_put(pin3, 1);
        gpio_put(pin4, 1);
        break;
    case 2: // 0101
        gpio_put(pin1, 1);
        gpio_put(pin2, 0);
        gpio_put(pin3, 1);
        gpio_put(pin4, 1);
        break;
    case 3: // 1001
        gpio_put(pin1, 1);
        gpio_put(pin2, 0);
        gpio_put(pin3, 0);
        gpio_put(pin4, 1);
        break;
    case 4: // 1001
        gpio_put(pin1, 1);
        gpio_put(pin2, 1);
        gpio_put(pin3, 0);
        gpio_put(pin4, 1);
        break;
    case 5: // 1001
        gpio_put(pin1, 1);
        gpio_put(pin2, 1);
        gpio_put(pin3, 0);
        gpio_put(pin4, 0);
        break;
    case 6: // 1001
        gpio_put(pin1, 1);
        gpio_put(pin2, 1);
        gpio_put(pin3, 1);
        gpio_put(pin4, 0);
        break;
    case 7: // 1001
        gpio_put(pin1, 0);
        gpio_put(pin2, 1);
        gpio_put(pin3, 1);
        gpio_put(pin4, 0);
        break;
    }
}
void Stepper::fullStep(bool direction)
{
    if (direction)
    {
        stepNum -= 1;
        if (stepNum < 0)
        {
            stepNum = 3;
        }
    }

    else
    {
        stepNum += 1;
        if (stepNum > 3)
        {
            stepNum = 0;
        }
    }

    switch (stepNum)
    {
    case 0: // 1010
        gpio_put(pin1, 0);
        gpio_put(pin2, 0);
        gpio_put(pin3, 1);
        gpio_put(pin4, 1);
        break;
    case 1: // 0110
        gpio_put(pin1, 0);
        gpio_put(pin2, 1);
        gpio_put(pin3, 1);
        gpio_put(pin4, 0);
        break;
    case 2: // 0101
        gpio_put(pin1, 1);
        gpio_put(pin2, 1);
        gpio_put(pin3, 0);
        gpio_put(pin4, 0);
        break;
    case 3: // 1001
        gpio_put(pin1, 1);
        gpio_put(pin2, 0);
        gpio_put(pin3, 0);
        gpio_put(pin4, 1);
        break;
    }
}
void Stepper::turnLeft(bool count)
{
    if (count)
    {
        TotalSteps += 1;
        if (backwardSteps > 0)
        {
            backwardSteps = 0;
        }
        forwardSteps += 1;
    }

    if (mode == StepMode::halfStep)
    {
        halfStep(true);
    }
    else
    {
        fullStep(true);
    }
}
void Stepper::turnRight(bool count)
{
    if (count)
    {
        TotalSteps += 1;
        if (forwardSteps > 0)
        {
            forwardSteps = 0;
        }
        backwardSteps += 1;
    }
    if (mode == StepMode::halfStep)
    {
        halfStep(false);
    }
    else
    {
        fullStep(false);
    }
}

driver::driver(Stepper *left, Stepper *right)
{
    this->LeftMotor = left;
    this->RightMotor = right;
}
void driver::initialize()
{
    contTime = time_us_64();
    TurnTimer = time_us_64();
    DistanceUpdateTime = time_us_64();
    this->LeftMotor->initialize();
    this->RightMotor->initialize();
}
void driver::forward()
{
    this->LeftMotor->turnRight();
    this->RightMotor->turnLeft();
}
void driver::stop()
{

    this->LeftMotor->Stop();
    this->RightMotor->Stop();
}
void driver::turnRight()
{
    this->LeftMotor->turnLeft(false);
    this->RightMotor->turnLeft(false);
}
void driver::turnLeft()
{
    this->LeftMotor->turnRight(false);
    this->RightMotor->turnRight(false);
}
void driver::backward()
{
    this->LeftMotor->turnLeft();
    this->RightMotor->turnRight();
}
void driver::calculateDistance(double yaw)
{
    int direction = 1;
    unsigned long LeftSteps = this->LeftMotor->TotalSteps;

    if (this->LeftMotor->backwardSteps > 0)
    {

        direction = -1;
    }

    double revolution = 0;
    if (this->LeftMotor->mode == StepMode::halfStep)
    {
        revolution = 2048 * 2;
    }
    else
    {
        revolution = 2048;
    }

    unsigned long steps = LeftSteps - totalSteps;
    totalSteps = LeftSteps;
    // printf("Steps: %lu\n", steps);
    distance = wheel_circumference * ((steps) / revolution) * direction;
    distance /= 100;
    // printf("Distance: %f\n", distance);
    // printf("stepConst: %f\n", (double)((LeftSteps) / revolution));
    x += distance * cos(yaw * Pi / 180);
    y += distance * sin(yaw * Pi / 180);
    // printf("x: %f\n", (double)x);
    // printf("y: %f\n", (double)y);
}

bool driver::update()
{
    controllerYaw = ((initialYaw - *angle) * 0.8) + ((initialmagAngle - *magAngle) * 0.2);
    if (controllerYaw < 0)
    {
        controllerYaw = 360 + controllerYaw;
    }
    else if (controllerYaw > 360)
    {
        controllerYaw = controllerYaw - 360;
    }

    if (time_us_64() - DistanceUpdateTime > 1000000)
    {
        calculateDistance(controllerYaw);
        DistanceUpdateTime = time_us_64();
        // printf("x: %.3f, y: %.3f, yaw: %.3f\n", x, y, controllerYaw);
    }

    if (time_us_64() - contTime > 2000)
    {
        if (counterTimer <= 0)
        {
            direction = 's';
        }
        else
        {
            if (!continueTurning || direction == 'f' || direction == 'b')
                counterTimer -= 1;
        }
        contTime = time_us_64();
        switch (direction)
        {
        case 's':
            stop();
            calculateDistance(controllerYaw);
            break;
        case 'f':
            forward();
            break;
        case 'l':
            if (controllerOnce)
            {

                turningInitialYaw = *angle;
                controllerOnce = false;
            }
            if (isBetween(*angle, turningInitialYaw, 90) && continueTurning)
            {
                turnLeft();

                if (-TurnTimer + time_us_64() > 10000000)
                {
                    continueTurning = false;

                    controllerOnce = true;
                }
            }
            else
            {
                direction = 's';
                controllerOnce = true;
                printf("turned1 %f degrees\n", turningInitialYaw);
                printf("turned2 %f degrees\n", *angle);
            }

            break;
        case 'r':
            if (controllerOnce)
            {

                turningInitialYaw = *angle;
                controllerOnce = false;
            }

            if (isBetween(*angle, turningInitialYaw, 90) && continueTurning)
            {
                turnRight();

                if (-TurnTimer + time_us_64() > 10000000)
                {
                    continueTurning = false;

                    controllerOnce = true;
                }
            }
            else
            {
                direction = 's';
                controllerOnce = true;
                printf("turned1 %f degrees\n", turningInitialYaw);
                printf("turned2 %f degrees\n", *angle);
            }
            break;
        case 'b':
            backward();
            break;

        default:
            printf("default");
            break;
        }
    }
    if (direction == 's')
    {
        stop();
        calculateDistance(controllerYaw);
        isDone = true;
        return true;
    }
    isDone = false;
    return false;
}
void driver::setCommand(command cmd)
{
    currentCommand = cmd;
    counterTimer = currentCommand.counterTimer;
    continueTurning = currentCommand.continueTurning;
    TurnTimer = currentCommand.TurnTimer;
    direction = cmd.direction;
}
bool driver::isBetween(float input, float refValue, float offset)
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