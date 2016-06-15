#include "gvt/render/unit/Profiler.h"

#include  <iostream>
#include  <fstream>

namespace gvt {
namespace render {
namespace unit {
namespace profiler {

Profiler::Profiler() {
  timers.resize(NUM_TIMERS);
  times.resize(NUM_TIMERS, 0.0);
  timerNames.resize(NUM_TIMERS);
  timerNames = {"TOTAL_TIME", "CAMERA_RAY", "SCHEDULE", "ADAPTER",
                "TRACE",      "SHUFFLE",    "SEND",     "RECV",
                "VOTE",       "COMPOSITE",  "WAIT",     "NOT_MEASURED"};

  counters.resize(NUM_COUNTERS, 0);
  counterNames.resize(NUM_COUNTERS);
  counterNames = {"PROCESS_RAY",    "SEND_RAY",         "RECV_RAY",
                  "VALID_SCHEDULE", "INVALID_SCHEDULE", "ADAPTER_HIT",
                  "ADAPTER_MISS"};
}

void Profiler::Start(int timer) { timers[timer].Start(); }

void Profiler::Stop(int timer) {
  Timer& t = timers[timer];
  times[timer] += t.Stop();
}

void Profiler::AddCounter(int type, int count) {
  counters[type] += count;
}

void Profiler::WriteToFile(const std::string &filename, int rank) {
  double sum = 0;
  for (std::size_t i = 0; i < times.size(); ++i) {
    if (i != TOTAL_TIME) sum += times[i];
  }
  times[NOT_MEASURED] = times[TOTAL_TIME] - sum;

  std::ofstream file;
  file.open(filename);
  file << "Rank "<< rank << "\n";
  file << *this;
  file.close();
}

std::ostream &operator<<(std::ostream &os, const Profiler &p) {
  // times
  for (std::size_t i = 0; i < p.times.size(); ++i) {
    double time = p.times[i];
    double ratio = time * 100.0 / p.times[Profiler::TOTAL_TIME];
    os << "Timer [" << p.timerNames[i] << "] : " << time << " ms (" << ratio
       << " %)\n";
    ;
  }
  // counters
  for (std::size_t i = 0; i < p.counters.size(); ++i) {
    os << "Counter [" << p.counterNames[i] << "] : " << p.counters[i] << "\n";
  }
  return os;
}

}  // namespace profiler
}  // namespace unit
}  // namespace render
}  // namespace gvt

