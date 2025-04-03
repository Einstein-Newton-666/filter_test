#pragma once
#include "filter_test/extended_kalman.hpp"

class ArmorFilter
{
public:
    ArmorFilter(/* args */);

private:
    ExtendedKalman<10, 8>
};


