#include <stdio.h>
#include <math.h> 

using namespace std;

void raiseTestingError(string message) {
  throw message.c_str();
}

bool isCloseTo(double a, double b, double epsilon) {
  return fabs(a-b) < epsilon ? true : false;
}
