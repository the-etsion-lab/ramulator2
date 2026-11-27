#ifndef     RAMULATOR_FRONTEND_PROCESSOR_SIMPLEO3_LLC_H
#define     RAMULATOR_FRONTEND_PROCESSOR_SIMPLEO3_LLC_H

#include <vector>
#include <list>
#include <unordered_map>
#include <iostream>
#include <fstream>

#include "base/clocked.h"
#include "base/debug.h"
#include "base/type.h"
#include "base/request.h"
#include "memory_system/memory_system.h"
#include "cows_cache.h"

namespace Ramulator {

DECLARE_DEBUG_FLAG(DSIMPLEO3LLC);
ENABLE_DEBUG_FLAG(DSIMPLEO3LLC);

class SimpleO3LLC : public Clocked<SimpleO3LLC> {
  friend class SimpleO3;
  struct Line {
    Addr_t addr = -1;
    Addr_t tag = -1;
    bool dirty = false;
    bool ready = false;   // Whether this line is ready (i.e., is still inflight?)
  };

  private:
    using CacheSet_t = std::list<Line>;   // LRU queue for the set. The head of the list is the least-recently-used way.
    std::unordered_map<int, CacheSet_t> m_cache_sets;

    using MSHREntry_t = std::pair<Addr_t, CacheSet_t::iterator>;
    using MSHR_t = std::vector<MSHREntry_t>;
    MSHR_t m_mshrs;
    std::unordered_map<Addr_t, std::vector<Request>> m_receive_requests;

    // Request that miss in the LLC with the clock cycle (current cycle + llc latency) that they
    // should be sent to the memory system
    std::list<std::pair<Clk_t, Request>> m_miss_list;

    // Request that hit in the LLC with the clock cycle (current cycle + llc latency) that they
    // should be sent back to the core (calls the callback)
    std::list<std::pair<Clk_t, Request>> m_hit_list;

    IMemorySystem* m_memory_system;

    Logger_t m_logger;


  public:
    int m_latency;
    CoWsCache* m_cows_cache;

    size_t m_size_bytes;
    size_t m_linesize_bytes;
    int m_associativity;
    int m_set_size;
    int m_num_mshrs;

    Addr_t m_index_mask;
    int m_index_offset;
    int m_tag_offset;


    int s_llc_read_access = 0;
    int s_llc_write_access = 0;
    int s_llc_read_misses = 0;
    int s_llc_write_misses = 0;
    int s_llc_eviction = 0;
    int s_llc_mshr_unavailable = 0;
    int s_llc_total_cows_latency = 0;
    // WI-004 D6/D7: an LLC miss is counted once, when the request is accepted (MSHR merge or MSHR allocation);
    // requests rejected for retry are counted separately (old llc_*_misses = new + retries, with CoWs off)
    uint64_t s_llc_mshr_allocs = 0;
    uint64_t s_llc_mshr_merges = 0;
    uint64_t s_llc_retry_no_line = 0;
    uint64_t s_llc_retry_cows_not_handled = 0;

    std::string m_stats_name = "llc";
    bool m_dump_raw_latency = true;   // WI-005 (D-28): write the raw per-request latency files (GBs at 1G)
    std::vector<std::tuple<Addr_t, Clk_t>> s_stats_dram_latency; // dram read latency for all requests that miss in the LLC (addr, latency) pairs
    std::vector<std::tuple<Addr_t, Clk_t>> s_stats_read_latencies; // total read latency for all requests
    std::vector<std::tuple<Addr_t, Clk_t, int>> s_stats_read_miss_latencies; // total read latency for read misses

    Clk_t s_last_mshr_change_clk = 0;
    std::vector<std::tuple<Clk_t, Clk_t, size_t>> s_stats_mshr_utilization; // mshr utilization over time

    Clk_t s_avg_total_read_lat_sum = 0;
    Clk_t s_avg_total_read_lat_cnt = 0;
    Clk_t s_avg_total_read_miss_lat_sum = 0;
    Clk_t s_avg_total_read_miss_lat_cnt = 0;
    Clk_t s_avg_dram_read_lat_sum = 0;
    Clk_t s_avg_dram_read_lat_cnt = 0;

    public:
    SimpleO3LLC(int latency, int size_bytes, int linesize_bytes, int associativity, int num_mshrs, CoWsCache* cows_cache);
    void connect_memory_system(IMemorySystem* memory_system) { m_memory_system = memory_system; };

    void tick();
    bool send(Request req);
    void receive(Request& req);

    void serialize(std::string serialization_filename);
    void deserialize(std::string serialization_filename);
    void dump_llc();

    // WI-010 carve-out
    int64_t m_carve_lines = 0;
    int m_carve_full_ways = 0;
    int64_t m_carve_rem_sets = 0;
    int64_t s_llc_carveout_lines = 0;
    int64_t s_llc_effective_bytes = 0;
    bool m_carve_page_aligned = false;   // WI-018: reduced sets taken in whole groups of m_carve_group_sets consecutive sets
    int64_t m_carve_group_sets = 0;
    void set_carveout(int64_t lines, const std::string& mode, const std::string& layout = "interleaved", int64_t group_sets = 0);
    int assoc_of(int set_index) const;
    int get_num_sets() const { return m_set_size; }
    int get_associativity() const { return m_associativity; }
    int get_size_bytes() const { return m_size_bytes; }

    int total_misses() {
      return s_llc_read_misses + s_llc_write_misses;
    }

    void count_llc_miss(const Request& req) {
      if (req.type_id == Request::Type::Read) {
        s_llc_read_misses++;
      } else if (req.type_id == Request::Type::Write) {
        s_llc_write_misses++;
      }
    }

    // called when simulation finished to dump stats
    void fini();
    void dump_dram_read_latency_samples();
    void dump_total_read_latency_samples();
    void dump_total_read_miss_latency_samples();
    void dump_mshr_utilization();

  private:
    int get_index(Addr_t addr)  { return (addr >> m_index_offset) & m_index_mask; };
    Addr_t get_tag(Addr_t addr) { return (addr >> m_tag_offset); };
    Addr_t align(Addr_t addr)   { return (addr & ~(m_linesize_bytes-1l)); };

    CacheSet_t& get_set(Addr_t addr);
    CacheSet_t::iterator allocate_line(CacheSet_t& set, Addr_t addr);
    bool need_eviction(const CacheSet_t& set, Addr_t addr);
    void evict_line(CacheSet_t& set, CacheSet_t::iterator victim_it);

    CacheSet_t::iterator check_set_hit(CacheSet_t& set, Addr_t addr);
    MSHR_t::iterator check_mshr_hit(Addr_t addr);
};

}        // namespace Ramulator


#endif   // RAMULATOR_FRONTEND_PROCESSOR_SIMPLEO3_LLC_H
