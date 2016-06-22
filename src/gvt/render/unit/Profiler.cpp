#include "gvt/render/unit/Profiler.h"

#include  <iostream>
#include  <fstream>

namespace gvt {
namespace render {
namespace unit {
namespace profiler {

Profiler::Profiler() {
#if GVT_USE_TIMING
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
#endif
}

void Profiler::Reset() {
#if GVT_USE_TIMING
  for (std::size_t i = 0; i < times.size(); ++i) times[i] = 0.0;
  for (std::size_t i = 0; i < counters.size(); ++i) counters[i] = 0;
#endif
}

void Profiler::Start(int timer) {
#if GVT_USE_TIMING
  timers[timer].Start();
#endif
}

void Profiler::Stop(int timer) {
#if GVT_USE_TIMING
  Timer& t = timers[timer];
  t.Stop();
  times[timer] += t.GetElapsed();
#endif
}

void Profiler::AddCounter(int type, int count) {
#if GVT_USE_TIMING
  counters[type] += count;
#endif
}

void Profiler::WriteToFile(const std::string &filename, int rank) {
#if GVT_USE_TIMING
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
#endif
}

#if GVT_USE_TIMING
std::ostream &operator<<(std::ostream &os, const Profiler &p) {
  // times
  for (std::size_t i = 0; i < p.times.size(); ++i) {
    double time = p.times[i];
    double ratio = time * 100.0 / p.times[Profiler::TOTAL_TIME];
#ifdef USE_CHRONO_TIMER
    os << "Timer [" << p.timerNames[i] << "] : " << time << " ms (" << ratio
       << " %)\n";
#else
    os << "Timer [" << p.timerNames[i] << "] : " << time << " s (" << ratio
       << " %)\n";
#endif
  }
  // counters
  for (std::size_t i = 0; i < p.counters.size(); ++i) {
    os << "Counter [" << p.counterNames[i] << "] : " << p.counters[i] << "\n";
  }
  return os;
}
#endif

}  // namespace profiler
}  // namespace unit
}  // namespace render
}  // namespace gvt

