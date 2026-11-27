#ifndef     RAMULATOR_BASE_REQUEST_H
#define     RAMULATOR_BASE_REQUEST_H

#include <vector>
#include <list>
#include <string>

#include "base/base.h"

namespace Ramulator {

struct Request {
  Addr_t    addr = -1;
  Addr_t    orig_addr = -1;   // WI-013: LLC-side address of a read redirected to a CoWs source row (restored on return)
  AddrVec_t addr_vec {};

  // Basic request id convention
  // 0 = Read, 1 = Write. The device spec defines all others
  struct Type {
    enum : int {
      Read = 0,
      Write,
    };
  };

  struct CacheStatus {
    enum : int {
      Unknown = -1,
      Miss = 0,
      Hit,
      HalfMiss,
      CowsMiss
    };
  };

  int type_id = -1;    // An identifier for the type of the request
  int source_id = -1;  // An identifier for where the request is coming from (e.g., which core)

  int cache_status = CacheStatus::Unknown;  // Whether the request is a hit or miss in the cache (Unknown if not applicable)

  int command = -1;          // The command that need to be issued to progress the request
  int final_command = -1;    // The final command that is needed to finish the request
  bool is_stat_updated = false; // Memory controller stats
  int row_class = -1;           // WI-005: row-buffer outcome at the request's first issued command: 0 hit, 1 miss (bank closed), 2 conflict
  bool is_cows_table = false;   // WI-019: a CoWs table read/write (cows_table_access: real); only the CoWs cache sets it
  bool cows_waited_wm = false;  // WI-019: a table read that sat in the read buffer while the controller was in write-drain mode
  // WI-022 (stats only): a table read's wait before its first command, by cause (controller ticks), and its first-command clock
  uint32_t cows_wait_wm = 0, cows_wait_busy = 0, cows_wait_notready = 0, cows_wait_ready = 0, cows_wait_nr_bank = 0;
  Clk_t cows_first_cmd = -1;
  bool refdiag_rank_in_refresh = false;   // WI-026 diagnostic (stats only): its rank was in all-bank refresh when it arrived

  Clk_t core2llc = -1;   // Clock cycle when the request arrives at LLC
  Clk_t arrive = -1;   // Clock cycle when the request arrive at the memory controller
  Clk_t depart = -1;   // Clock cycle when the request depart the memory controller
  Clk_t llc2mc = -1;   // Clock cycle when the request departs LLC to MC
  Clk_t mc2llc = -1;   // Clock cycle when the request arrives at LLC from MC
  Clk_t done = -1;     // Clock cycle when the request arrives at the core

  std::array<int, 4> scratchpad = { 0 };    // A scratchpad for the request

  std::function<void(Request&)> callback;

  void* m_payload = nullptr;    // Point to a generic payload

  Request(Addr_t addr, int type);
  Request(AddrVec_t addr_vec, int type);
  Request(Addr_t addr, int type, int source_id, std::function<void(Request&)> callback);
};


struct ReqBuffer {
  std::list<Request> buffer;
  size_t max_size = 32;


  using iterator = std::list<Request>::iterator;
  iterator begin() { return buffer.begin(); };
  iterator end() { return buffer.end(); };


  size_t size() const { return buffer.size(); }

  bool enqueue(const Request& request) {
    if (buffer.size() <= max_size) {
      buffer.push_back(request);
      return true;
    } else {
      return false;
    }
  }

  void remove(iterator it) {
    buffer.erase(it);
  }
};

}        // namespace Ramulator


#endif   // RAMULATOR_BASE_REQUEST_H
