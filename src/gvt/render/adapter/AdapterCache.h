/* =======================================================================================
   This file is released as part of GraviT - scalable, platform independent ray tracing
   tacc.github.io/GraviT

   Copyright 2013-2015 Texas Advanced Computing Center, The University of Texas at Austin
   All rights reserved.

   Licensed under the BSD 3-Clause License, (the "License"); you may not use this file
   except in compliance with the License.
   A copy of the License is included with this software in the file LICENSE.
   If your copy does not contain the License, you may obtain a copy of the License at:

       http://opensource.org/licenses/BSD-3-Clause

   Unless required by applicable law or agreed to in writing, software distributed under
   the License is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY
   KIND, either express or implied.
   See the License for the specific language governing permissions and limitations under
   limitations under the License.

   GraviT is funded in part by the US National Science Foundation under awards
   ACI-1339863,
   ACI-1339881 and ACI-1339840
   =======================================================================================
   */
#ifndef GVT_ADAPTER_CACHE_H
#define GVT_ADAPTER_CACHE_H

#include <chrono>
#include <ctime>
#include <map>

namespace gvt {
namespace render {
template <typename key, typename value> class cache {
protected:
  // struct key_pair {
  //   key _key;
  //   timetag _time;
  //
  //   key_pair(key _key) : _key(_key), _time(std::chrono::system_clock::now()) {}
  //   key_pair(const key &key) : _key(_key), _time(std::chrono::system_clock::now()) {}
  //   key_pair(const key_pair &key) : _key(_key), _time(std::chrono::system_clock::now())
  //   {}
  //
  //   ~key_pair() {}
  //
  //   bool operator==(const key_pair &other) { return (_key == other._key); }
  //   bool operator!=(const key_pair &other) { return (_key != other._key); }
  //
  //   bool operator<(const key_pair &other) { return (_key < other._key); }
  //   bool operator>(const key_pair &other) { return (_key > other._key); }
  //
  //   bool operator<=(const key_pair &other) { return (_key <= other._key); }
  //   bool operator>=(const key_pair &other) { return (_key >= other._key); }
  // };

  std::mutex _protect;

public:
  typedef priority_order = std::pair<timetag, std::pair<key, value> >;
  typedef timetag = std::chrono::system_clock::time_point;
  typedef value_timetagged std::pair<timetag, value>;
  std::map<key, std::pair<timetag, value> > _cache;

  cache() {}
  ~cache() { GVT_ASSERT(_cache.empty(), "Something when wrong cache is not empty()"); }
  void clear() { _cache.clear(); }
  bool keyExists(key _key) { return (_cache.find(_key) != _cache.enr()); }
  value get(key _key) {
    GVT_ASSERT(keyExists(_key), "Trying to access an invalid element on the cache");
    _cache[_key].first = std::chrono::system_clock::now();
    return _cache[_key].second;
  }

  void set(key &_key, value &_value) {
    GVT_ASSERT(!keyExists(_key), "Trying to modify an existing element on the cache");
    return _cache[_key] = value_timetagged(std::chrono::system_clock::now(), _value);
  }

  // bool keyExists(key _key) { return (_cache.find(_key) != _cache.enr()); }

private:
}
}
}

#endif /* GVT_ADAPTER_CACHE_H */
