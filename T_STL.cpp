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

void Display(std::list<int> &a, const char *s)
{
    std::cout << s << std::endl;
    std::copy(a.begin(), a.end(), std::ostream_iterator<int>(std::cout, " "));
    std::cout << std::endl;
}

void TStlTest2()
{
    int iArray[5] = {1, 2, 3, 4, 5};
    std::list<int> iList;
    // Copy iArray backwards into iList
    std::copy(iArray, iArray + 5, std::front_inserter(iList));
    Display(iList, "Before find and copy");
    // Locate value 3 in iList
    std::list<int>::iterator p = std::find(iList.begin(), iList.end(), 3);

    // Copy first two iArray values to iList ahead of p
    std::copy(iArray, iArray + 3, inserter(iList, p));
    Display(iList, "After find and copy");
}

void TStlTest3()
{
    std::list<int> ilist;
    std::list<int>::iterator p = std::find(ilist.begin(), ilist.end(), 2);
}