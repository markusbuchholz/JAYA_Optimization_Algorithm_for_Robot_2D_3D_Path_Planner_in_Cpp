// Markus Buchholz
// g++ jaya_robot3D.cpp -o t -I/usr/include/python3.8 -lpython3.8

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>
#include <numeric>

#include "matplotlibcpp.h"

namespace plt = matplotlibcpp;

//--------Path Planner--------------------------------------------------------------

float xmin = 0.0;
float xmax = 50.0;
float ymin = 0.0;
float ymax = 50.0;
float zmin = 0.0;
float zmax = 50.0;


float obsX = 25.0;
float obsY = 25.0;
float obsZ = 25.0;
float obsR = 3.0;

float goalX = 45.0;
float goalY = 45.0;
float goalZ = 45.0;

float startX = 2.0;
float startY = 2.0;
float startZ = 2.0;

float K1 = 0.18;            //  fitting parameter table 1
float K2 = 0.000000000000000001; // fitting parameter table 2

//----------------------------------------------------------------------------------

int EVOLUTIONS = 2;
int PARTICLES = 45;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
    float z;
};
//--------------------------------------------------------------------------------

float generateRandom()
{

    std::random_device engine;
    std::uniform_real_distribution<float> distribution(0.0, 1.0);
    return distribution(engine);
}

//--------------------------------------------------------------------------------
float valueGenerator(float low, float high)
{

    return low + generateRandom() * (high - low);
}

//--------------------------------------------------------------------------------

float euclid(Pos a, Pos b)
{

    return std::sqrt(std::pow(a.x - b.x, 2) + std::pow(a.y - b.y, 2) + std::pow(a.z - b.z, 2));
}
//--------------------------------------------------------------------------------

std::vector<Pos> initPosXYZ()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < PARTICLES; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax), valueGenerator(zmin, zmax)});
    }

    return pos;
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{

    std::vector<float> funcValue;
    Pos Obs{obsX, obsY, obsZ};
    Pos Goal{goalX, goalY, goalZ};

    for (auto &ii : pos)
    {

        funcValue.push_back(K1 * (1 / euclid(Obs, ii)) + K2 * euclid(Goal, ii));
    }

    return funcValue;
}

//--------------------------------------------------------------------------------
float func(Pos pos)
{
    Pos Obs{obsX, obsY, obsZ};
    Pos Goal{goalX, goalY, goalZ};

    return K1 * (1 / euclid(Obs, pos)) + K2 * euclid(Goal, pos);
}
//--------------------------------------------------------------------------------

Pos positionUpdateCheck(Pos actualPos)
{

    Pos Pnew = actualPos;

    if (Pnew.x < xmin)
    {
        Pnew.x = xmin;
    }

    if (Pnew.x > xmax)
    {
        Pnew.x = xmax;
    }

    if (Pnew.y < ymin)
    {
        Pnew.y = ymin;
    }

    if (Pnew.y > ymax)
    {
        Pnew.y = ymax;
    }


    if (Pnew.z < zmin)
    {
        Pnew.z = zmin;
    }

    if (Pnew.z > zmax)
    {
        Pnew.z = zmax;
    }


    return Pnew;
}

//-------------------------------------------------------------------------------
bool compareMin(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second < b.second;
}

//-------------------------------------------------------------------------------
bool compareMax(std::pair<Pos, float> a, std::pair<Pos, float> b)
{

    return a.second > b.second;
}

//-------------------------------------------------------------------------------

// min
std::tuple<Pos, float> findBestPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMin);

    return best[0];
}

//--------------------------------------------------------------------------------
// max
std::tuple<Pos, float> findWorstPosFuncValue(std::vector<Pos> positions, std::vector<float> func)
{

    std::vector<std::pair<Pos, float>> best;

    for (int ii = 0; ii < func.size(); ii++)
    {

        best.push_back(std::pair<Pos, float>(positions[ii], func[ii]));
    }

    std::sort(best.begin(), best.end(), compareMax);

    return best[0];
}

//--------------------------------------------------------------------------------
Pos updatePosition(Pos old, Pos best, Pos worst)
{

    Pos pos;

    pos.x = old.x + generateRandom() * (best.x - std::abs(old.x)) - generateRandom() * (worst.x - std::abs(old.x));
    pos.y = old.y + generateRandom() * (best.y - std::abs(old.y)) - generateRandom() * (worst.y - std::abs(old.y));
    pos.y = old.z + generateRandom() * (best.z - std::abs(old.z)) - generateRandom() * (worst.z - std::abs(old.z));

    return positionUpdateCheck(pos);
}

//--------------------------------------------------------------------------------

std::vector<Pos> runJAYA()
{

    std::vector<Pos> currentPositions = initPosXYZ();
    std::vector<float> currentFunctionValue = function(currentPositions);

    for (int ii = 0; ii < EVOLUTIONS; ii++)
    {

        std::tuple<Pos, float> bestPosValue = findBestPosFuncValue(currentPositions, currentFunctionValue); // min for minimilzation problem
        Pos bestPos = std::get<0>(bestPosValue);
        float bestFunctionValue = std::get<1>(bestPosValue);

        std::tuple<Pos, float> worstPosValue = findWorstPosFuncValue(currentPositions, currentFunctionValue); // max for minimilzation problem
        Pos worstPos = std::get<0>(worstPosValue);
        float worstFunctionValue = std::get<1>(worstPosValue);

        for (int jj = 0; jj < PARTICLES; jj++)
        {

            Pos newPos = updatePosition(currentPositions[jj], bestPos, worstPos);

            float newFunctionValue = func(newPos);

            if (newFunctionValue < currentFunctionValue[jj])
            {

                currentPositions[jj] = newPos;
                currentFunctionValue[jj] = newFunctionValue;
            }
        }
    }

    //     for (auto &ii : currentFunctionValue)
    //     {
    //         std::cout << ii << "\n";
    //     }
    return currentPositions;
}

//-------------------------------------------------------------------------------
std::tuple<std::vector<float>, std::vector<float>> gen_circle(float a, float b, float r)
{

    std::vector<float> xX;
    std::vector<float> yY;

    for (float dt = -M_PI; dt < M_PI; dt += 0.01)
    {

        xX.push_back(a + r * std::cos(dt));
        yY.push_back(b + r * std::sin(dt));
    }
    return std::make_tuple(xX, yY);
}

//-----------------------------------------------------------------------------------------

void plot2D(std::vector<float> xX, std::vector<float> yY)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());

    std::tuple<std::vector<float>, std::vector<float>> circle = gen_circle(obsX, obsY, obsR);

    std::vector<float> xObs = std::get<0>(circle);
    std::vector<float> yObs = std::get<1>(circle);

    plt::plot(xX, yY);
    plt::plot(xObs, yObs);
    plt::xlabel("X");
    plt::ylabel("Y");
    plt::show();
}
//---------------------------------------------------------------------

void plot3D(std::vector<float> xX, std::vector<float> yY,  std::vector<float> zZ)
{
    std::sort(xX.begin(), xX.end());
    std::sort(yY.begin(), yY.end());
    std::sort(zZ.begin(), zZ.end());


    plt::plot3(xX, yY, zZ);
    plt::xlabel("x");
    plt::ylabel("y");
    plt::set_zlabel("z");
    plt::show();
}

//--------------------------------------------------------------------------
int main()
{

    std::vector<Pos> path = runJAYA();

    std::vector<float> xX;
    std::vector<float> yY;
    std::vector<float> zZ;

    for (auto &ii : path)
    {
        xX.push_back(ii.x);
        yY.push_back(ii.y);
        zZ.push_back(ii.z);

        std::cout << ii.x << " ," << ii.y << " ," << ii.z <<"\n";
    }

    plot3D(xX, yY, zZ);
}