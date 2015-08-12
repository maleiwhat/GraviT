#ifndef GVTWORKER_H
#define GVTWORKER_H

#include <iostream>
#include <string>
#include <mpi.h>
#include <pthread.h>
#include <vector>
#include <sstream>
#include <stack>
#include <string>

#include <boost/timer/timer.hpp>

#include "gvtState.h"
using namespace std;

namespace cvt
{

class Worker
{
public:
  Worker(const std::vector<std::string>* objFilenames) {
    this->objFilenames = objFilenames;
  }
  void Launch();
  StateLocal stateLocal;
  StateUniversal stateUniversal;
  MPIBuffer buffer;
  const std::vector<std::string>* objFilenames;
};

}
        // MPI_Comm globalComm;
//
// server
//
// int main(int argc, char** argv)
// {
// }

#endif
