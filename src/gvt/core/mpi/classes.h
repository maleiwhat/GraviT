#pragma once

#include <memory>
using namespace std;

namespace gvt {
namespace core {
namespace mpi {

class Message;
typedef shared_ptr<Message> MessageP;

class Work;
typedef shared_ptr<Work> WorkP;
}
}
}
