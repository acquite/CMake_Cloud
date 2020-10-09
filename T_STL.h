// http://net.pku.edu.cn/~yhf/UsingSTL.htm

#include <string>
#include <iterator>
#include <iostream>

#include <stdlib.h>  // Need random(), srandom()
#include <time.h>    // Need time()
#include <algorithm> // Need sort(), copy()
#include <vector>    // Need vector
#include <list>

#define SIZE 100
int iarray[SIZE];
void T_StlTest();

void Display(std::list<int> &a, const char *s);

void TStlTest2();

void TStlTest3();