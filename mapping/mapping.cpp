#include "mapping.h"
mapping::mapping(double *left, double *front, double *right)
{
    this->left = left;
    this->front = front;
    this->right = right;
}
void mapping::CreateMap(unsigned int width, unsigned int height)
{
    this->width = width;
    this->height = height;
    for (int i = 0; i < this->width; i++)
    {
        for (int j = 0; j < this->height; j++)
        {
            this->map[i][j] = u;
        }
    }
    for (int i = 0; i < height; i++)
    {
        this->map[i][0] = o;
        this->map[i][width - 1] = o;
    }
    for (int j = 0; j < width; j++)
    {
        this->map[0][j] = o;
        this->map[height - 1][j] = o;
    }
    this->map[posx][posy] = 'S';
}
void mapping::updatePosition(double posx, double posy)
{
    this->posx = posx;
    this->posy = posy;
}
char mapping::Sonar(char orien)
{
    if (orien == 'f')
    {
        if (*front < maxDistance)
        {
            frontBlocked = true;
            return 'o';
        }

        else
        {
            frontBlocked = false;
            return 'f';
        }
    }
    else if (orien == 'r')
    {
        if (*right < maxDistance)
        {
            rightBlocked = true;
            return 'o';
        }
        else
        {
            rightBlocked = false;
            return 'f';
        }
    }
    else if (orien == 'l')
    {
        if (*left < maxDistance)
        {
            leftBlocked = true;
            return 'o';
        }
        else
        {
            leftBlocked = false;
            return 'f';
        }
    }
    else // todo yeri gelince f yapmalı
    {
        int pos = 0;
        for (int i = 0; i < 4; i++)
        {
            if (orientation[i] == 'd')
                pos = i;
        }
        // char orientation[4] = {'f', 'r', 'd', 'l'}; yukarı,sağ,aşşağı,sol
        switch (pos)
        {
        case 0:
            return this->map[posx - 1][posy] == 'u' ? this->map[posx - 1][posy] : 'f'; // save on map 'f' if position (posx-1,posy) is to be visited and 'o' if obstacle
            break;
        case 1:
            return this->map[posx][posy + 1] == 'u' ? this->map[posx][posy + 1] : 'f'; // save on map 'f' if position (posx,posy+1) is to be visited and 'o' if obstacle
            break;
        case 2:
            return this->map[posx + 1][posy] == 'u' ? this->map[posx + 1][posy] : 'f'; // save on map 'f' if position (posx,posy-1) is to be visited and 'o' if obstacle
            break;
        case 3:
            return this->map[posx][posy - 1] == 'u' ? this->map[posx][posy - 1] : 'f'; // save on map 'f' if position (posx,posy-1) is to be visited and 'o' if obstacle
            break;

        default:
            break;
        }
        return 'u';
    }
}
void mapping::updateMap()
{
    printf("%2.f,%2.f,%2.f\n", *left, *front, *right);
    bool cantfind = false;
    if (this->map[posx - 1][posy] == u) // see North
    {
        this->map[posx - 1][posy] = Sonar(orientation[0]); // save on map 'f' if position (posx-1,posy) is to be visited and 'o' if obstacle
        cantfind = true;
    }
    if (this->map[posx][posy + 1] == u) // see East
    {
        this->map[posx][posy + 1] = Sonar(orientation[1]); // save on map 'f' if position (posx,posy+1) is to be visited and 'o' if obstacle
        cantfind = true;                                   // PrintMap();
    }
    if (this->map[posx][posy - 1] == u) // see West
    {
        this->map[posx][posy - 1] = Sonar(orientation[3]); // save on map 'f' if position (posx,posy-1) is to be visited and 'o' if obstacle
        cantfind = true;                                   // PrintMap();
    }
    if (this->map[posx + 1][posy] == u) // see south
    {
        if (!cantfind)
        {
            if (map[posx + 1][posy] == u) // todo problem olabilecek bir yer
            {
                map[posx + 1][posy] = f;
            }
        }
        this->map[posx + 1][posy] = Sonar(orientation[2]);
        //  PrintMap();
    }
}
void mapping::drawMap()
{
    printf("Map of explored this->map:\n");
    // Serial.println("");
    for (int i = 0; i < height; i++)
    {
        printf("\n");
        for (int j = 0; j < width; j++)
        {
            char te = ' ';
            if (i == posx && j == posy)
            {
                printf("O ");
            }
            else if (this->map[i][j] == f || this->map[i][j] == 'S')
            {
                printf(te + " "); // free path
            }

            else if (this->map[i][j] == n || this->map[i][j] == s)
            {
                printf(te + " "); // free path
            }

            else if (this->map[i][j] == e || this->map[i][j] == w)
            {
                printf(te + " "); // free path
            }

            else if (this->map[i][j] == o)
            {
                printf("# "); // obstacle
            }

            else if (this->map[i][j] == u)
            {
                printf(". "); // obstacle
            }
        }
    }
    printf("\n");
}

mapCom mapping::getPos()
{
    VectorInt16 pos;
    mapCom com;
    if (map[posx - 1][posy] == f)
    {
        direction = 0;
        map[posx - 1][posy] = n;
        com = turn(orientation[0]);
        if (com.rotate < 2)
            posx--;

        printf("posx:%d posy:%d\n", posx, posy);
        printf("North\n");
    }

    else if (map[posx][posy + 1] == f)
    {
        direction = 3;
        map[posx][posy + 1] = e;
        com = turn(orientation[1]);
        if (com.rotate < 2)
            posy++;

        printf("posx:%d posy:%d\n", posx, posy);
        printf("East\n");
    }
    else if (map[posx + 1][posy] == f)
    {
        direction = 2;
        map[posx + 1][posy] = s;

        com = turn(orientation[2]);
        if (com.rotate < 2)
            posx++;
        printf("posx:%d posy:%d\n", posx, posy);
        printf("South\n");
    }
    else if (map[posx][posy - 1] == f)
    {
        direction = 1;
        map[posx][posy - 1] = w;
        com = turn(orientation[3]);
        if (com.rotate < 2)
            posy--;

        printf("posx:%d posy:%d\n", posx, posy);
        printf("West\n");
    }

    else if ((map[posx - 1][posy] != f) && (map[posx][posy + 1] != f) && (map[posx + 1][posy] != f) && (map[posx][posy - 1] != f))
    {
        // read value in the (posx,posy) position and move following the rules to come back
        if (map[posx][posy] == n)
        {
            direction = 2;
            posx++;                     // update robot position
            com = turn(orientation[2]); // turn robot 180°, turn orientation, go one step forward
        }

        else if (map[posx][posy] == e)
        {
            direction = 1;
            posy--;                     // update robot position
            com = turn(orientation[3]); // turn robot 90° anticlockwise, turn orientation, go one step forward
        }

        else if (map[posx][posy] == s)
        {
            direction = 0;
            posx--;                     // update robot position
            com = turn(orientation[0]); // go one step forward
        }

        else if (map[posx][posy] == w)
        {
            direction = 3;
            posy++;                     // update robot position
            com = turn(orientation[1]); // turn robot 90° clockwise, turn orientation, go one step forward
        }
    }

    return com;
}

void mapping::rotate_to_left() // left rotation
{

    char temp = orientation[0];
    for (int i = 0; i < 4; i++)
    {
        orientation[i] = orientation[i + 1];
    }
    orientation[3] = temp;
}

void mapping::rotate_to_right() // right rotation
{

    char temp = orientation[3];
    for (int i = 3; i >= 0; i--)
    {
        orientation[i] = orientation[i - 1];
    }
    orientation[0] = temp;
}
mapCom mapping::turn(char rotation_value)
{
    mapCom com;
    if (rotation_value == 'f')
    {
        com.rotate = -1;
        com.direction = 0;
        // GO(); // do not turn, just go
        // STOP();
        printf("go forward\n");
    }

    if (rotation_value == 'r') // turn right
    {
        com.rotate = 1;
        com.direction = 1;
        // TURN_RIGHT();
        // STOP();
        // GO();
        // STOP();
        rotate_to_right(); // update orientation state (array) of robot
        printf("turn right\n");
    }

    else if (rotation_value == 'd') // turn backwards
    {
        com.rotate = 2;
        com.direction = 0;
        // TURN_LEFT();
        // TURN_LEFT();
        // STOP();
        // GO();
        // STOP();
        rotate_to_left();
        rotate_to_left(); // update orientation state (array) of robot
        printf("turn backwards\n");
    }

    else if (rotation_value == 'l') // turn left
    {
        com.rotate = 1;
        com.direction = 0;
        // TURN_LEFT();
        // STOP();
        // GO();
        // STOP();
        rotate_to_left(); // update orientation state (array) of robot
        printf("turn left\n");
    }
    return com;
}

bool mapping::Goal()
{
    // loop to check if there are clear places close to unknown places, so that goal is not reached yet (returns 0)
    if (map[posx][posy] == 'S')
    {
        atStart -= 1;
    }
    else
    {
        atStart = 10;
    }
    if (atStart <= 0)
    {
        return 1;
    }
    for (int i = 1; i < height; i++) // N,M map length and width
    {
        for (int j = 1; j < width; j++)
        {
            if (map[i][j] == f || map[i][j] == 'S') // consider only clear positions and see if the neighbor positions are unexplored, i.e. there are others ways to explore
            {
                if ((map[i - 1][j] == u) || (map[i - 1][j] == n) || (map[i - 1][j] == e) || (map[i - 1][j] == s) || (map[i - 1][j] == w)) // North
                {
                    return 0;
                }

                else if ((map[i][j + 1] == u) || (map[i][j + 1] == n) || (map[i][j + 1] == e) || (map[i][j + 1] == s) || (map[i][j + 1] == w)) // East
                {
                    return 0;
                }

                else if ((map[i + 1][j] == u) || (map[i + 1][j] == n) || (map[i + 1][j] == e) || (map[i + 1][j] == s) || (map[i + 1][j] == w)) // South
                {
                    return 0;
                }

                else if ((map[i][j - 1] == u) || (map[i][j - 1] == n) || (map[i][j - 1] == e) || (map[i][j - 1] == s) || (map[i][j - 1] == w)) // West
                {
                    return 0;
                }
            }
        }
    }

    return 1;
}

char *mapping::getCharmap(int *size)
{
    *size = int(height * width * 3 / 8) + 2;
    char *map_char = new char[int(height * width * 3 / 8) + 2];
    int k = 0;
    int bye = 0;
    unsigned long databye;
    bool wrote = false;

    for (int i = 0; i < height; i++) // N,M map length and width
    {
        for (int j = 0; j < width; j++)
        {
            char t = map[i][j];

            if (bye == 7)
            {
                k += 3;
                for (int a = 0; a < 3; a++)
                {
                    map_char[k - a] = (databye & 0x000000FF);
                    databye = databye >> 8;
                }

                bye = 0;
                wrote = false;
                databye = 0;
            }

            switch (t)
            {
            case 'S':
                if (!wrote)
                    databye = 0;
                else
                    databye = (databye << 3);
                bye++;
                break;
            case 'n':
                if (!wrote)
                    databye = 1;
                else
                    databye = (databye << 3) | 1;
                bye++;
                break;

            case 'e':
                if (!wrote)
                    databye = 2;
                else
                    databye = (databye << 3) | 2;
                bye++;
                break;
            case 's':
                if (!wrote)
                    databye = 3;
                else
                    databye = (databye << 3) | 3;
                bye++;
                break;
            case 'w':
                if (!wrote)
                    databye = 4;
                else
                    databye = (databye << 3) | 4;
                bye++;
                break;

            case 'o':
                if (!wrote)
                    databye = 5;
                else
                    databye = (databye << 3) | 5;
                bye++;
                break;
            case 'u':
                if (!wrote)
                    databye = 6;
                else
                    databye = (databye << 3) | 6;
                bye++;
                break;
            case 'f':
                if (!wrote)
                    databye = 7;
                else
                    databye = (databye << 3) | 7;
                bye++;
                break;

            default:
                break;
            }
        }
    }
    return map_char;
}

void mapping::putCharMap(char *buf,int size)
{

    unsigned long *tempList = new unsigned long[size];
    int lonCount = 0;
    int k = 0;
    unsigned long byt = 0;
    for (int i = 0; i < size; i++)
    {
        byt = byt << 8 | buf[i];
        k++;
        if (k == 3)
        {
            k = 0;
            tempList[lonCount] = byt;
            lonCount++;
        }
    }
    // decode
    int x = 0;
    int y = 0;
    for (int i = 0; i < size; i++)
    {
        for (int j = 0; j < 8; j++)
        {
            map[x][y] = (int)(tempList[i] & 0x7);

            if (map[x][y] == 5)
            {
                map[x][y] = o;
            }
            else if (map[x][y] == 6)
            {

                map[x][y] = u;
            }
            else
            {
                map[x][y] = f;
            }

            tempList[i] = tempList[i] >> 3;
            x++;
            if (x == 25)
            {
                x = 0;
                y++;
            }
        }
    }
}