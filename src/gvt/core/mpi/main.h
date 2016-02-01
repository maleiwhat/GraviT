/*THIS FILE SHOULD BE REMOVED */

#include <iostream>
#include "Application.h"
#include "Work.h"

using namespace std;

class AllG : public Work {
  WORK_CLASS_HEADER(AllG)

public:
  static Work *Deserialize(size_t size, unsigned char *serialized) {
    if (size != 0) {
      std::cerr << "AllG deserializer call with size != 0 rank " << Application::GetApplication()->GetRank() << "\n";
      exit(1);
    }

    AllG *allg = new AllG;
    return (Work *)allg;
  }

  void Serialize(size_t &size, unsigned char *&serialized) {
    size = 0;
    serialized = NULL;
  }

  bool Action();
};

class Ping : public Work {
  WORK_CLASS_HEADER(Ping)

public:
  static Work *Deserialize(size_t size, unsigned char *serialized) {
    if (size != sizeof(int)) {
      std::cerr << "Test deserializer ctor with size != sizeof(int)\n";
      exit(1);
    }
    Ping *ping = new Ping;
    ping->SetValue(*(int *)serialized);
    return (Work *)ping;
  }

  int GetValue() { return value; }
  void SetValue(int v) { value = v; }

  void Serialize(size_t &size, unsigned char *&serialized) {
    size = sizeof(int);
    serialized = (unsigned char *)malloc(size);
    *(int *)serialized = value;
  }

  bool Action();

private:
  int value;
};
