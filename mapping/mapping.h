#include "pico/stdlib.h"
#include <stdio.h>
#include "../MPU6050/math3d/helper_3dmath.h"
using namespace std;

struct mapCom
{
    int rotate = -2;
    bool direction;
};
class mapping
{

public:
    unsigned int width = 25, height = 25;
    int posx = 12;
    int posy = 12;
    char map[50][50];
    float maxDistance = 15 * 1.5f;
    int direction;
    mapping(double *left, double *front, double *right);
    void CreateMap(unsigned int width, unsigned int height);
    void updatePosition(double x, double y);
    void updateMap();
    void drawMap();
    mapCom getPos();
    bool Goal();
    void putCharMap(char *buf,int size);
    char *getCharmap(int *size);
    bool leftBlocked, frontBlocked, rightBlocked;

private:
    double *left, *right, *front;
    const char u = 'u', f = 'f', o = 'o';
    const char n = 'n', e = 'e', s = 's', w = 'w';
    char orientation[4] = {'f', 'r', 'd', 'l'};
    char Sonar(char a);
    void rotate_to_left();
    void rotate_to_right();
    mapCom turn(char rotation_value);
    int atStart = 10;
};
