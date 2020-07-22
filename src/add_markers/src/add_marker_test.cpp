#include <stdio.h>
#include <cmath>

struct Position
{
    float x;
    float y;
    float w;
};

//marker positions
Position pickUp = {6.0, 3.0, 1.0};
Position dropOff = {0.0, 0.0, 1.0};
Position threshold = {0.3, 0.3, 0.01};

// areas & flags
bool atPickUp = false;
bool atDropOff = false;
bool pickUpDone = false;
bool dropOffDone = false;

bool checkArea(Position roboPos, Position target)
{
    if (((std::abs(target.x - roboPos.x) < threshold.x)) && ((std::abs(target.y - roboPos.y) < threshold.y)) && ((std::abs(target.w - roboPos.w) < threshold.w)))
    {
        printf("In the target area \n");
        return true;
    }
    return false;
}

//callback function
void odomCallback(Position *odomMsg)
{
    Position roboPos;
    roboPos.x = odomMsg->x;
    roboPos.y = odomMsg->y;
    roboPos.w = odomMsg->w;

    // checking the robot position if in pickup area
    checkArea(roboPos, pickUp) ? atPickUp = true : atPickUp = false;

    // checking the robot if in dropoff area
    checkArea(roboPos, dropOff) ? atDropOff = true : atDropOff = false;
}

int main(int argc, char **argv)
{
    Position target[4] = {{6.0, 3.0, 1.0}, {3.0, 3.0, 1.0}, {0.0, 3.0, 1.0}, {0.0, 0.0, 1.0}};
    
    odomCallback(&target[0]);
    printf("area: pickup = %s, dropoff = %s\n", atPickUp ? "true" : "false", atDropOff ? "true" : "false");
    
    odomCallback(&target[1]);
    printf("area: pickup = %s, dropoff = %s\n", atPickUp ? "true" : "false", atDropOff ? "true" : "false");

    odomCallback(&target[2]);
    printf("area: pickup = %s, dropoff = %s\n", atPickUp ? "true" : "false", atDropOff ? "true" : "false");

    odomCallback(&target[3]);
    printf("area: pickup = %s, dropoff = %s\n", atPickUp ? "true" : "false", atDropOff ? "true" : "false");
}