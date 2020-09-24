#include "T_STL.h"

void T_StlTest()
{
    iarray[20] = 50;
    int *ip = std::find(iarray, iarray + SIZE, 50);
    if (ip == iarray + SIZE)
        std::cout << "50 not found in array" << std::endl;
    else
        std::cout << *ip << " found in array" << std::endl;
}