#ifndef GVT_RENDER_UNIT_PROFILER_H
#define GVT_RENDER_UNIT_PROFILER_H

#include <string>
#include <chrono>
#include <vector>

namespace gvt {
namespace render {
namespace unit {
namespace profiler {

using namespace std::chrono;

class Timer {
 public:
  Timer() {
    elapsed = 0.0;
    st = high_resolution_clock::now();
  }
  void Start() { st = high_resolution_clock::now(); }
  double Stop() {
    auto et = high_resolution_clock::now();
    double elapsed =
        std::chrono::duration<double, std::ratio<1, 1000> >(et - st).count();
    return elapsed;
  }
  double GetElapsed() const { return elapsed; }

 private:
  high_resolution_clock::time_point st;
  double elapsed;
};

class Profiler {
 public:
  enum Timers {
    ALL = 0,
    CAMRAYS,
    SCHEDULE,
    ADAPTER,
    TRACE,
    SHUFFLE,
    SEND,
    RECV,
    VOTE,
    COMPOSITE,
    WAIT,
    NUM_TIMERS
  };

  enum Counters {
    // rays
    PROCESSED_RAYS = 0,
    SENT_RAYS,
    RECVED_RAYS,
    // schedule
    VALID_SCHEDULE,
    INVALID_SCHEDULE,
    NUM_COUNTERS
  };

  Profiler();

  void Start(int timer);
  void Stop(int timer);

  void AddCounter(int type, int count);

 private:
  std::vector<Timer> timers;
  std::vector<double> times;
  std::vector<std::string> timerNames;

  std::vector<std::size_t> counters;
  std::vector<std::string> counterNames;
};

}  // namespace profiler
}  // namespace unit
}  // namespace render
}  // namespace gvt

#endif
