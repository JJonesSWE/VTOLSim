#include "Utils.h"
#include <cmath>

#define TOLERANCE 0.00001

bool almostEqual( double a, double b )
{
    return std::abs( a - b ) < TOLERANCE;
}