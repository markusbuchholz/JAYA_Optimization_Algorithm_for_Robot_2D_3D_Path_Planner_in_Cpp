// Markus Buchholz

#include <iostream>
#include <vector>
#include <tuple>
#include <algorithm>
#include <math.h>
#include <random>
#include <numeric>

//--------Path Planner--------------------------------------------------------------

float xmin = -5.0; // 0.0;
float xmax = 5.0;  // 50.0;
float ymin = -5.0; // 0.0;
float ymax = 5.0;  // 50.0;

float obsX = 25.0;
float obsY = 25.0;
float obsR = 5.0;

float goalX = 45.0;
float goalY = 45.0;

float startX = 2.0;
float startY = 2.0;

//----------------------------------------------------------------------------------
int EVOLUTIONS = 100;
int PARTICLES = 20;

//--------------------------------------------------------------------------------

struct Pos
{

    float x;
    float y;
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

std::vector<Pos> initPosXY()
{

    std::vector<Pos> pos;

    for (int ii = 0; ii < PARTICLES; ii++)
    {

        pos.push_back({valueGenerator(xmin, xmax), valueGenerator(ymin, ymax)});
    }

    return pos;
}

//--------------------------------------------------------------------------------

std::vector<float> function(std::vector<Pos> pos)
{
    std::vector<float> funcValue;

    for (auto &ii : pos)
    {

        funcValue.push_back(ii.x * ii.y);
    }

    return funcValue;
}

//--------------------------------------------------------------------------------

float func(Pos pos)
{

    return pos.x * pos.y;
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

    return positionUpdateCheck(pos);
}

//--------------------------------------------------------------------------------

void runJAYA()
{

    std::vector<Pos> currentPositions = initPosXY();
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

    for (auto &ii : currentFunctionValue)
    {
        std::cout << ii << "\n";
    }
}

//--------------------------------------------------------------------------------

int main()
{

    runJAYA();
}