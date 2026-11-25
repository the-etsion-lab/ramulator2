#include <functional>

#include "base/utils.h"
#include "frontend/frontend.h"
#include "translation/translation.h"
#include "frontend/impl/processor/simpleO3/core.h"
#include "frontend/impl/processor/simpleO3/llc.h"
#include "memory_system/memory_system.h"
#include "dram/dram.h"
#include "addr_mapper/addr_mapper.h"


namespace Ramulator {

class SimpleO3 final : public IFrontEnd, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IFrontEnd, SimpleO3, "SimpleO3", "Simple timing model OoO processor frontend.")

  private:
    ITranslation*  m_translation;
    uint64_t m_cows_page_bytes = 0;            // WI-004 D10
    bool m_cows_check_page_geometry = true;    // WI-004 D10
    bool m_cows_geometry_checked = false;      // WI-004 D10
    std::string m_cows_carveout = "none";      // WI-010
    std::string m_cows_placement = "self";     // WI-013
    uint64_t m_cows_source_pool = 0;           // WI-013
    int m_llc_linesize_bytes = 64;             // WI-010

    int m_num_cores = -1;
    std::vector<SimpleO3Core*> m_cores;
    SimpleO3LLC* m_llc;
    CoWsCache* m_cows_cache;

    size_t m_num_expected_insts = 0;
    size_t m_warmup_percent = 0;
    size_t m_skip_insts = 0;     // WI-013
    size_t m_warmup_insts = 0;   // WI-013

    std::string serialization_filename;

  public:
    void init() override {
      m_clock_ratio = param<uint>("clock_ratio").required();

      // Core params
      std::vector<std::string> trace_list = param<std::vector<std::string>>("traces").desc("A list of traces.").required();
      m_num_cores = trace_list.size();

      int ipc   = param<int>("ipc").desc("IPC of the SimpleO3 core.").default_val(4);
      int depth = param<int>("inst_window_depth").desc("Instruction window size of the SimpleO3 core.").default_val(128);
      uint32_t lat_factor = param<uint32_t>("latency_factor").desc("Multiple all memory latencies by that factor.").default_val(1);
      set_latency_factor(lat_factor);

      // LLC params
      int llc_latency           = param<int>("llc_latency").desc("Aggregated latency of the LLC.").default_val(47);
      int llc_linesize_bytes    = param<int>("llc_linesize").desc("LLC cache line size in bytes.").default_val(64);
      m_llc_linesize_bytes = llc_linesize_bytes;
      int llc_associativity     = param<int>("llc_associativity").desc("LLC set associativity.").default_val(8);
      int llc_capacity_per_core = parse_capacity_str(param<std::string>("llc_capacity_per_core").desc("LLC capacity per core.").default_val("2MB"));
      int llc_num_mshr_per_core = param<int>("llc_num_mshr_per_core").desc("Number of LLC MSHR entries per core.").default_val(16);

      bool cows_accel_zero_page = param<bool>("cows_accel_zero_page").desc("Accelerate zero page copy").required();

      bool cows_enable = param<bool>("cows_enable").desc("Enable CoWs.").required();
      uint32_t dram_latency_on_translation = param<int>("cows_dram_latency_on_translation").desc("DRAM latency for COWs mapping translations.").required();
      // WI-004 D8: the CoWs cache lives in the LLC; a lookup is serial to the LLC tag miss. Default 50 = one LLC access
      // (PI ruling 2026-11-24; EXP-02 sweeps {0, 25, 50}).
      uint32_t cows_cache_access_latency = param<int>("cows_cache_access_latency").desc("No. of cycles to access the CoWs cache.").default_val(50);
      uint32_t llc2cows_ratio = param<uint32_t>("cows_cache2llc_ratio").desc("Ratio between the number of LLC cache lines and CoWs cache entries.").required();
      uint32_t cows_cache_assoc = param<uint32_t>("cows_cache_assoc").desc("Associativity of CoWs cache.").required();
      uint32_t dram_page_bytes = parse_capacity_str(param<std::string>("dram_page_bytes").desc("size of DRAM page.").required());
      m_cows_page_bytes = dram_page_bytes;
      m_cows_placement = param<std::string>("cows_source_placement")
        .desc("WI-013: where reads of unwritten blocks go: self (own row, legacy) | random | same_bank.").default_val("self");
      if (m_cows_placement != "self" && m_cows_placement != "random" && m_cows_placement != "same_bank") {
        throw ConfigurationError("cows_source_placement must be self, random or same_bank (got {})", m_cows_placement);
      }
      m_cows_source_pool = param<uint64_t>("cows_source_pool")
        .desc("WI-013: number of distinct source rows (required unless placement is self).").default_val(0);
      if (cows_enable && m_cows_placement != "self" && m_cows_source_pool == 0) {
        throw ConfigurationError("cows_source_pool must be given (> 0) with cows_source_placement {}", m_cows_placement);
      }
      m_cows_carveout = param<std::string>("cows_carveout")
        .desc("WI-010: charge the CoWs cache's LLC capacity: none | ways | lines.").default_val("none");
      if (m_cows_carveout != "none" && m_cows_carveout != "ways" && m_cows_carveout != "lines") {
        throw ConfigurationError("cows_carveout must be none, ways or lines (got {})", m_cows_carveout);
      }
      m_cows_check_page_geometry = param<bool>("cows_check_page_geometry")
        .desc("Require a CoWs page to be exactly one DRAM row in one bank (WI-004 D10).").default_val(true);
      uint32_t always_miss = param<bool>("cows_always_miss").desc("Always miss in CoWs cache.").required();
      std::string cows_replacement = param<std::string>("cows_replacement").desc("Replacement policy in CoWs cache.").required();
      CoWsCache::ReplPolicy *cows_policy = nullptr;
      if(cows_replacement == "LRU_nohit") {
        cows_policy = new CoWsCache::LRU_nohit();
      } else {
        throw ConfigurationError("Unknown CoWs replacement policy {}!", cows_replacement);
      }
      std::cerr<<"# cows_replacement: "<<cows_policy->name()<<std::endl;

      CoWsCache::CoWsStats cows_stats;
      cows_stats.stats_fname = param<std::string>("cows_stats_file").desc("Filename for to dump COWS stats.").default_val("");

      // Simulation parameters
      m_num_expected_insts = param<size_t>("num_expected_insts").desc("Number of instructions that the frontend should execute.").required();
      m_warmup_percent = param<size_t>("warmup_percent").desc("Percent of the run that is considered warmup.").required();
      // WI-013 region runner: skip the first skip_insts instructions of every trace functionally, then simulate num_expected_insts,
      // of which the first warmup_insts (if > 0, overriding warmup_percent) are warmup.
      m_skip_insts = param<size_t>("skip_insts").desc("Instructions skipped functionally at the start of each trace.").default_val(0);
      m_warmup_insts = param<size_t>("warmup_insts").desc("Absolute warmup instructions (0: use warmup_percent).").default_val(0);
      if (m_warmup_insts >= m_num_expected_insts && m_warmup_insts > 0) {
        throw ConfigurationError("warmup_insts ({}) must be < num_expected_insts ({})", m_warmup_insts, m_num_expected_insts);
      }

      // Create address translation module
      m_translation = create_child_ifce<ITranslation>();

      // Create CoWsCache
      m_cows_cache = nullptr;
      if(cows_enable) {
        uint32_t llc_nlines = llc_capacity_per_core * m_num_cores / llc_linesize_bytes;
        uint32_t cows_nlines = llc_nlines / llc2cows_ratio;
        std::cerr<<"# Creating CoWsCache with "<<cows_nlines<<" lines "
        "(llc_nlines="<<llc_nlines<<", ratio="<<llc2cows_ratio<<")"<<std::endl;

        // cows line number must be a multiple of its assoc
        //cows_nlines = cows_cache_assoc * ((cows_nlines + cows_cache_assoc - 1) / cows_cache_assoc);
        assert(cows_nlines % cows_cache_assoc == 0);
        assert(cows_nlines >= cows_cache_assoc);

        std::cerr<<"# LLC bytes="<<(llc_nlines*llc_linesize_bytes)<<" (or "<<llc_capacity_per_core*m_num_cores<<"), lines: "<<llc_nlines<<", COWS lines: "<<cows_nlines<<std::endl;

        m_cows_cache = new CoWsCache(cows_nlines,
                                    cows_cache_assoc,
                                    llc_linesize_bytes,
                                    dram_page_bytes,
                                    always_miss,
                                    cows_cache_access_latency,
                                    dram_latency_on_translation,
                                    cows_policy,
                                    cows_stats);

        std::cerr<<"# Finished creating CoWsCache"<<std::endl;
      }

      // Create the LLC
      bool dump_raw_latency = param<bool>("dump_raw_latency_samples")
        .desc("Write raw per-request latency files (llc.*-latency); the -cdf summaries are always written.").default_val(true);
      m_llc = new SimpleO3LLC(llc_latency, llc_capacity_per_core * m_num_cores, llc_linesize_bytes, llc_associativity, llc_num_mshr_per_core * m_num_cores, m_cows_cache);
      m_llc->m_dump_raw_latency = dump_raw_latency;
      // m_llc->deserialize(serialization_filename);
      // m_llc->serialize(serialization_filename);

      // Create the cores
      SimpleO3Core::Trace::dram_page_bytes = dram_page_bytes;
      for (int id = 0; id < m_num_cores; id++) {
        SimpleO3Core* core = new SimpleO3Core(id, ipc, depth, m_num_expected_insts, m_warmup_percent, trace_list[id], cows_accel_zero_page, m_translation, m_llc,
                                              m_warmup_insts, m_skip_insts);
        core->m_callback = [this](Request& req){return this->receive(req);} ;
        m_cores.push_back(core);
      }

      m_logger = Logging::create_logger("SimpleO3");

      // Register the stats
      register_stat(m_num_expected_insts, false).name("num_expected_insts");// do not reset this counter
      register_stat(m_llc->s_llc_eviction).name("llc_eviction");
      register_stat(m_llc->s_llc_read_access).name("llc_read_access");
      register_stat(m_llc->s_llc_write_access).name("llc_write_access");
      register_stat(m_llc->s_llc_read_misses).name("llc_read_misses");
      register_stat(m_llc->s_llc_write_misses).name("llc_write_misses");
      register_stat(m_llc->s_llc_mshr_unavailable).name("llc_mshr_unavailable");
      register_stat(m_llc->s_llc_total_cows_latency).name("llc_total_cows_latency");
      register_stat(m_llc->s_llc_mshr_allocs).name("llc_mshr_allocs");
      register_stat(m_llc->s_llc_carveout_lines, false).name("llc_carveout_lines");      // WI-010
      register_stat(m_llc->s_llc_effective_bytes, false).name("llc_effective_bytes");    // WI-010
      register_stat(m_llc->s_llc_mshr_merges).name("llc_mshr_merges");
      register_stat(m_llc->s_llc_retry_no_line).name("llc_retry_no_line");
      register_stat(m_llc->s_llc_retry_cows_not_handled).name("llc_retry_cows_not_handled");

      static auto tot_llc_accesses = ADDi(m_llc->s_llc_read_access, m_llc->s_llc_write_access);
      static auto tot_llc_misses = ADDi(m_llc->s_llc_read_misses, m_llc->s_llc_write_misses);
      register_stat(tot_llc_accesses, false).name("llc_total_access"); // do not reset compount counters
      register_stat(tot_llc_misses, false).name("llc_total_misses");
      static auto llc_missrate = DIV(tot_llc_misses, tot_llc_accesses);
      register_stat(llc_missrate, false).name("llc_missrate");

      if(m_cows_cache != nullptr) {
        register_stat(m_cows_cache->s_hits).name("cows_cache_hits");
        register_stat(m_cows_cache->s_misses).name("cows_cache_misses");
        register_stat(m_cows_cache->s_access).name("cows_cache_access");
        register_stat(m_cows_cache->s_cows_cycles).name("cows_access_cycles");
        register_stat(m_cows_cache->s_cows_cycles_self).name("cows_access_cycles_self");
        register_stat(m_cows_cache->s_access).name("cows_accesses");
        register_stat(m_cows_cache->s_accesses_on_llc_hit).name("cows_accesses_on_llc_hit");
        register_stat(m_cows_cache->s_accesses_on_llc_miss).name("cows_accesses_on_llc_miss");
        register_stat(m_cows_cache->s_accesses_on_llc_evict).name("cows_accesses_on_llc_evict");
        register_stat(m_cows_cache->s_misses_on_llc_hit).name("cows_misses_on_llc_hit");
        register_stat(m_cows_cache->s_misses_on_llc_miss).name("cows_misses_on_llc_miss");
        register_stat(m_cows_cache->s_misses_on_llc_evict).name("cows_misses_on_llc_evict");
        register_stat(m_cows_cache->s_half_misses_on_llc_hit).name("cows_half_misses_on_llc_hit");
        register_stat(m_cows_cache->s_half_misses_on_llc_miss).name("cows_half_misses_on_llc_miss");
        register_stat(m_cows_cache->s_half_misses_on_llc_evict).name("cows_half_misses_on_llc_evict");
        register_stat(m_cows_cache->s_miss_not_handled_on_llc_hit).name("cows_miss_not_handled_on_llc_hit");
        register_stat(m_cows_cache->s_miss_not_handled_on_llc_miss).name("cows_miss_not_handled_on_llc_miss");
        register_stat(m_cows_cache->s_miss_not_handled_on_llc_evict).name("cows_miss_not_handled_on_llc_evict");
        register_stat(m_cows_cache->s_lookups_write_alloc).name("cows_lookups_write_alloc");
        register_stat(m_cows_cache->s_bits_set).name("cows_bits_set");
        register_stat(m_cows_cache->s_table_reads).name("cows_table_reads");
        register_stat(m_cows_cache->s_table_writes).name("cows_table_writes");
        // WI-013
        m_cows_cache->m_source_pool = m_cows_source_pool;
        m_cows_cache->m_placement = m_cows_placement == "random" ? CoWsCache::Placement::RANDOM
                                  : m_cows_placement == "same_bank" ? CoWsCache::Placement::SAME_BANK : CoWsCache::Placement::SELF;
        register_stat(m_cows_cache->s_redirected_reads).name("cows_redirected_reads");
        register_stat(m_cows_cache->s_unwritten_block_misses).name("cows_unwritten_block_misses");
        register_stat(m_cows_cache->s_written_block_misses).name("cows_written_block_misses");
        register_stat(m_cows_cache->s_misses).name("cows_misses");
        register_stat(m_cows_cache->s_half_misses).name("cows_half_misses");
        register_stat(m_cows_cache->s_hits).name("cows_hits");
        register_stat(m_cows_cache->s_evicts).name("cows_evicts");
        static auto cows_missrate = DIV(m_cows_cache->s_misses, m_cows_cache->s_access);
        register_stat(cows_missrate, false).name("cows_missrate");
        register_stat(m_cows_cache->s_src_index_bits, false).name("cows_src_index_bits");
        register_stat(m_cows_cache->s_entry_bits, false).name("cows_cache_entry_bits");
        register_stat(m_cows_cache->s_cache_bytes, false).name("cows_cache_bytes");

        static auto cows2llc_miss_ratio = DIV(m_cows_cache->s_misses_on_llc_miss, tot_llc_misses);
        register_stat(cows2llc_miss_ratio, false).name("cows2llc_miss_ratio");

        static auto avg_dram_lat = DIV(m_llc->s_avg_dram_read_lat_sum, m_llc->s_avg_dram_read_lat_cnt);
        register_stat(avg_dram_lat, false).name("avg_dram_read_latency");

        static auto avg_total_lat = DIV(m_llc->s_avg_total_read_lat_sum, m_llc->s_avg_total_read_lat_cnt);
        register_stat(avg_total_lat, false).name("avg_total_read_latency");

        static auto avg_total_miss_lat = DIV(m_llc->s_avg_total_read_miss_lat_sum, m_llc->s_avg_total_read_miss_lat_cnt);
        register_stat(avg_total_miss_lat, false).name("avg_total_read_miss_latency");
      }

      std::vector<size_t*> tot_insts_post_warmup;
      std::vector<size_t*> tot_cycles_post_warmup;
      for (int core_id = 0; core_id < m_cores.size(); core_id++) {
        register_stat(m_cores[core_id]->s_insts_retired, false).name("insts_retired_core_{}", core_id);
        register_stat(m_cores[core_id]->s_cycles_recorded, false).name("cycles_recorded_core_{}", core_id);
        register_stat(m_cores[core_id]->s_cycles_recorded_post_warmup).name("cycles_recorded_post_warmup_core_{}", core_id);
        register_stat(m_cores[core_id]->s_total_cycles_recorded_post_warmup).name("total_cycles_recorded_post_warmup_core_{}", core_id);
        register_stat(m_cores[core_id]->s_insts_retired_post_warmup).name("insts_retired_post_warmup_core_{}", core_id);
        register_stat(m_cores[core_id]->s_total_insts_retired_post_warmup).name("total_insts_retired_post_warmup_core_{}", core_id);
        register_stat(m_cores[core_id]->s_mem_access_cycles).name("memory_access_cycles_recorded_core_{}", core_id);
        auto frac_cycles_post_warmup = new DIV(m_cores[core_id]->s_cycles_recorded_post_warmup, m_cores[core_id]->s_cycles_recorded);
        auto frac_insts_post_warmup = new DIV(m_cores[core_id]->s_insts_retired_post_warmup, m_num_expected_insts);
        register_stat(*frac_cycles_post_warmup, false).name("frac_cycles_post_warmup_{}", core_id);
        register_stat(*frac_insts_post_warmup, false).name("frac_insts_post_warmup_{}", core_id);

        auto core_ipc = new DIV(m_cores[core_id]->s_insts_retired_post_warmup, m_cores[core_id]->s_cycles_recorded_post_warmup);
        register_stat(*core_ipc, false).name("ipc_core_{}", core_id);


        tot_insts_post_warmup.push_back(&m_cores[core_id]->s_insts_retired_post_warmup);
        tot_cycles_post_warmup.push_back(&m_cores[core_id]->s_cycles_recorded_post_warmup);
      }
      auto tot_insts = new SUM(tot_insts_post_warmup);
      auto tot_cycles = new SUM(tot_cycles_post_warmup);
      auto tot_ipc = new DIV(*tot_insts, *tot_cycles);
      register_stat(*tot_ipc, false).name("ipc_total");

    }

    void tick() override {
      m_clk++;

      if (m_cows_cache != nullptr && !m_cows_geometry_checked) {
        check_cows_page_geometry(m_memory_system);
        m_cows_geometry_checked = true;
        apply_cows_carveout();
      }

      if(m_clk % 1000000 == 0) {
        m_logger->info("Processor Heartbeat {} cycles.", m_clk);
      }

      m_llc->tick();
      for (auto core : m_cores) {
        core->tick();
      }
    }

    void receive(Request& req) {
//      printf("SimpleO3: receive req. clk=%lu, followers=%lu, cache_status=%d, addr=0x%lx\n",
//             m_clk, m_llc->m_receive_requests[req.addr].size(), req.cache_status, req.addr);
      req.done = m_clk;
      m_llc->receive(req);

      // TODO: LLC latency for the core to receive the request?
      for (auto r : m_llc->m_receive_requests[req.addr]) {
        r.done = m_clk;

//        printf("SimpleO3: m_receive_requests. clk=%lu, cache_status=%d, addr=0x%lx\n", m_clk, r.cache_status, r.addr);

        // collect total read latency samples
        r.done = m_clk; // update done time for queued request copies
        auto total_read_latency = r.done - r.core2llc;
        m_llc->s_avg_total_read_lat_sum += total_read_latency;
        m_llc->s_avg_total_read_lat_cnt += 1;
        m_llc->s_stats_read_latencies.push_back(std::make_tuple(r.addr, total_read_latency));

        if(r.cache_status != Request::CacheStatus::Hit) {
          m_llc->s_avg_total_read_miss_lat_sum += total_read_latency;
          m_llc->s_avg_total_read_miss_lat_cnt += 1;
//          printf("LLC: total read latency sample. clk=%lu, cache_status=%d, addr=0x%lx, total_read_latency=%lu\n", m_clk, r.cache_status, r.addr, total_read_latency);
          m_llc->s_stats_read_miss_latencies.push_back(std::make_tuple(r.addr, total_read_latency, r.cache_status));
        }

        m_cores[r.source_id]->receive(r);
      }
      m_llc->m_receive_requests[req.addr].clear();
    };

    bool is_finished() override {
      for (auto core : m_cores) {
        if (!(core->reached_expected_num_insts)){
          return false;
        }
      }
      if(m_cows_cache != nullptr) {
        m_cows_cache->fini();
      }
      m_llc->fini();

      for (auto core : m_cores) {
        core->fini();
      }
      return true;
    }

#if 1
    bool is_warmup_finished() override {
      for (auto core : m_cores) {
        if (!(core->finished_warmup)){
          return false;
        }
      }

      return true;
    }
#endif

    void connect_memory_system(IMemorySystem* memory_system) override {
      m_llc->connect_memory_system(memory_system);
      m_memory_system = memory_system;   // WI-004 D10: geometry is checked at the first tick (the memory system's
                                         // address mapper is set up only after the front end is connected)
    };

    /**
     * WI-004 D9/D10: a CoWs page must be exactly one DRAM row in one bank under the address mapping (refresh can only be
     * skipped per physical row). Map the first and last 64B block of random pages and the first block of the next page:
     * the first two must share channel/rank/bankgroup/bank/row, the next page must not. Also sets the source-index width
     * (log2 rows per bank) used for the entry size.
     */
    /**
     * WI-010: charge the LLC capacity the CoWs cache occupies (needs the entry size, known after the geometry check).
     *  lines: an entry cannot straddle LLC lines and a lookup reads one CoWs set, so each CoWs set occupies
     *         ceil(assoc x entry_bits / 512) LLC lines: carve cows_sets x that many lines (exact up to line granularity).
     *  ways:  carve ceil(cows_bytes / way_bytes) full ways from every set (spec option; mostly rounding for small caches).
     */
    void apply_cows_carveout() {
      if (m_cows_carveout == "none") {
        return;
      }
      uint64_t entry_bits = m_cows_cache->s_entry_bits;
      if (entry_bits == 0) {
        throw ConfigurationError("cows_carveout needs the CoWs entry size (memory system without DRAM/address mapper?)");
      }
      uint64_t line_bits = 8ull * m_llc_linesize_bytes;
      uint64_t lines_per_cows_set = (m_cows_cache->get_assoc() * entry_bits + line_bits - 1) / line_bits;
      int64_t lines_physical = (int64_t)m_cows_cache->get_num_sets() * lines_per_cows_set;
      int64_t lines_bytes = (int64_t)((m_cows_cache->s_cache_bytes + m_llc_linesize_bytes - 1) / m_llc_linesize_bytes);
      int64_t lines = lines_physical;
      if (m_cows_carveout == "ways") {
        int64_t way_lines = m_llc->get_num_sets();
        int64_t ways = (lines_bytes + way_lines - 1) / way_lines;
        lines = ways * way_lines;
      }
      std::cerr << "# CoWs carve-out: cows bytes=" << m_cows_cache->s_cache_bytes << ", exact lines=" << lines_bytes
                << ", physical lines (" << m_cows_cache->get_num_sets() << " sets x " << lines_per_cows_set << ")=" << lines_physical
                << ", mode " << m_cows_carveout << " -> carve " << lines << " lines" << std::endl;
      m_llc->set_carveout(lines, m_cows_carveout);
    }

    void check_cows_page_geometry(IMemorySystem* memory_system) {
      IDRAM* dram = memory_system->get_dram();
      IAddrMapper* mapper = memory_system->get_addr_mapper();
      if (dram == nullptr || mapper == nullptr) {
        std::cerr << "# WARNING: memory system has no DRAM/address mapper; CoWs page geometry not checked" << std::endl;
        return;
      }
      int rows = dram->get_level_size("row");
      // WI-013: source geometry. total rows = channel x rank x bankgroup x bank x row; the row field position for same_bank is
      // found by probing the mapper (lowest address bit >= page bits that changes only the row, to row 1).
      uint64_t total_rows = 1;
      for (auto lvl : {"channel", "rank", "bankgroup", "bank", "row"}) {
        total_rows *= dram->get_level_size(lvl);
      }
      m_cows_cache->m_total_rows = total_rows;
      m_cows_cache->m_rows_per_bank = rows;
      {
        int row_level = dram->m_levels("row");
        auto mapv = [&](Addr_t a) { Request r(a, Request::Type::Read); mapper->apply(r); return r.addr_vec; };
        auto z = mapv(0);
        for (uint32_t b = calc_log2(m_cows_page_bytes); b < 56; b++) {
          auto v = mapv((Addr_t)1 << b);
          bool only_row = v[row_level] == 1;
          for (int l = 0; l < row_level && only_row; l++) only_row = (v[l] == z[l]);
          if (only_row) { m_cows_cache->m_row_shift = b; break; }
        }
        if (m_cows_placement == "same_bank" && m_cows_cache->m_row_shift == 0) {
          throw ConfigurationError("cows_source_placement same_bank: could not locate the row field in the address mapping");
        }
        if (m_cows_placement == "random") {   // linear 4KB-aligned addresses below total_rows x page must cover distinct rows
          auto a = mapv((Addr_t)(total_rows - 1) * m_cows_page_bytes), w = mapv((Addr_t)total_rows * m_cows_page_bytes);
          a.resize(row_level + 1); w.resize(row_level + 1); z.resize(row_level + 1);
          if (w != z) {
            throw ConfigurationError("cows_source_placement random: address space of {} rows does not wrap as expected", total_rows);
          }
        }
      }
      // D9 + WI-013: same-bank (and self = today's model) sources need log2(rows per bank) bits, random sources log2(total rows)
      uint32_t src_bits = (m_cows_placement == "random") ? calc_log2(total_rows) : calc_log2(rows);
      m_cows_cache->set_source_index_bits(src_bits);
      std::cerr << "# CoWs source placement " << m_cows_placement << ": pool=" << m_cows_cache->m_source_pool << " rows, total rows="
                << total_rows << ", rows/bank=" << rows << ", row field at bit " << m_cows_cache->m_row_shift
                << ", src index bits=" << src_bits << std::endl;
      if (!m_cows_check_page_geometry) {
        std::cerr << "# WARNING: cows_check_page_geometry=false; CoWs page geometry not checked" << std::endl;
        return;
      }
      int col_level = dram->m_levels("column");
      uint64_t page = m_cows_page_bytes;
      auto map = [&](Addr_t a) { Request r(a, Request::Type::Read); mapper->apply(r); r.addr_vec.resize(col_level); return r.addr_vec; };
      uint64_t x = 0x9E3779B97F4A7C15ull;   // fixed-seed xorshift: deterministic check
      for (int i = 0; i < 1024; i++) {
        x ^= x << 13; x ^= x >> 7; x ^= x << 17;
        Addr_t p = (Addr_t)((x % (1ull << 34)) & ~(page - 1));
        auto first = map(p), last = map(p + page - 64), next = map(p + page);
        if (first != last || first == next) {
          throw ConfigurationError("CoWs page ({} bytes) is not exactly one DRAM row in one bank under the address mapping "
                                   "(page 0x{:x}: first/last block {}, next page {}). Set dram_page_bytes to the row size.",
                                   page, p, first == last ? "same row" : "different rows", first == next ? "same row" : "different row");
        }
      }
      std::cerr << "# COWs page geometry OK: " << page << "-byte page == one row in one bank (" << rows << " rows/bank)" << std::endl;
    }

    int get_num_cores() override {
      return m_num_cores;
    };
};

}        // namespace Ramulator
