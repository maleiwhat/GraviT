#include "gvt/render/unit/Profiler.h"

namespace gvt {
namespace render {
namespace unit {
namespace profiler {

Profiler::Profiler() {
  timers.resize(NUM_TIMERS);
  times.resize(NUM_TIMERS, 0.0);
  timerNames.resize(NUM_TIMERS);
  timerNames = {"ALL",  "CAMRAYS", "SCHEDULE", "ADAPTER",   "TRACE", "SHUFFLE",
                "SEND", "RECV",    "VOTE",     "COMPOSITE", "WAIT"};

  counters.resize(NUM_COUNTERS, 0);
  counterNames.resize(NUM_COUNTERS);
  counterNames = {"PROCESSED_RAYS", "SENT_RAYS", "RECVED_RAYS",
                  "VALID_SCHEDULE", "INVALID_SCHEDULE"};
}

inline void Profiler::Start(int timer) { timers[timer].Start(); }

inline void Profiler::Stop(int timer) {
  Timer& t = timers[timer];
  times[timer] += t.Stop();
}

inline void Profiler::AddCounter(int type, int count) {
  counters[type] += count;
}

}  // namespace profiler
}  // namespace unit
}  // namespace render
}  // namespace gvt

