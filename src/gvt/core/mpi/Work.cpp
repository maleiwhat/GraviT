#include <iostream>
#include "Work.h"
#include "Application.h"
#include "Message.h"
#include "MessageManager.h"

using namespace std;
using namespace gvt::core::mpi;

Work::Work(bool c) {
  className = "Work";
  communicates = c;
}

Work::Work(bool c, int n) {
  className = "Work";
  communicates = c;
  contents = smem::New(n);
}

Work::Work(bool c, SharedP s) {
  className = "Work";
  communicates = c;
  contents = s;
}

int Work::RegisterSubclass(Work *(*d)(SharedP)) { return Application::GetTheApplication()->RegisterWork(d); }

Work::~Work() {}

