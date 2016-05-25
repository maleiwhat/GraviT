#include "smem.h"

using namespace std;
using namespace gvt::core::mpi;

smem::~smem() {
  // cerr << "freed " << sz << " at " << (long)ptr << "\n";
  free(ptr);
}

smem::smem(int n) {
  if (n > 0)
    ptr = (unsigned char *)malloc(n);
  else
    ptr = NULL;
  sz = n;
  // cerr << "alloc " << sz << " at " << (long)ptr << "\n";
}
