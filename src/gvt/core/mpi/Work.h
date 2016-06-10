#pragma once

#include <mpi.h>
#include <iostream>
#include <memory>
#include <vector>
#include "Message.h"
#include "classes.h"
#include "smem.h"

using namespace std;

namespace gvt {
namespace core {
namespace mpi {

class Work {
 public:
  Work(bool c);
  Work(bool c, int n);
  Work(bool c, SharedP s);
  ~Work();

  void *get() { return contents->get(); }
  int get_size() { return contents->get_size(); }

  virtual void initialize(){};
  virtual bool Action() { return true; };
  virtual bool Action(MPI_Comm comm) { return true; };

  // TODO: temporary workaround. this should go away (hpark)
  virtual bool deferDeletingThis() { return false; }

  void Send(int i);
  void Broadcast(bool c, bool b);

  string getClassName() { return className; };
  int GetType() { return type; }

  SharedP get_pointer() { return contents; }

 protected:
  static int RegisterSubclass(Work *(*d)(SharedP));

  void SetCommunicates(bool c) { communicates = c; }
  bool Communicates() { return communicates; }

  int type;
  string className;
  bool communicates;
  SharedP contents;
};

#define WORK_CLASS_TYPE(ClassName) int ClassName::class_type = 0;

#define WORK_CLASS(ClassName, communicates)                                    \
 public:                                                                       \
  ClassName(size_t n = 0) : Work(communicates, n) {                            \
    className = string(#ClassName);                                            \
    type = ClassName::class_type;                                              \
    initialize();                                                              \
  }                                                                            \
                                                                               \
  ClassName(SharedP p) : Work(communicates, p) {                               \
    className = string(#ClassName);                                            \
    type = ClassName::class_type;                                              \
    initialize();                                                              \
  }                                                                            \
                                                                               \
 public:                                                                       \
  string getClassName() { return string(#ClassName); }                         \
  unsigned char *get_contents() { return contents->get(); }                    \
                                                                               \
  static void Register() {                                                     \
    ClassName::class_type = Work::RegisterSubclass(Deserialize);               \
  }                                                                            \
                                                                               \
  static Work *Deserialize(SharedP ptr) { return (Work *)new ClassName(ptr); } \
                                                                               \
 private:                                                                      \
  static int class_type;
}
}
}
