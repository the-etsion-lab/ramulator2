#include <algorithm>
#include <fstream>
#include <map>

#include "dram_controller/controller.h"
#include "memory_system/memory_system.h"
#include <cstdio>

namespace Ramulator {

    static const std::vector<Addr_t> debug_addrs = {0x128bad300ULL, 0x755d7e240ULL, 0x7d917e000ULL, 0x7d917e000ULL};
    inline bool is_debug_addr(Addr_t addr) {
      return std::find(debug_addrs.begin(), debug_addrs.end(), addr) != debug_addrs.end();
    };
    inline bool is_debug_addr(const ReqBuffer::iterator& req_it) {
      return is_debug_addr(req_it->addr);
    }
    inline bool is_debug_addr(const Request& req) {
      return is_debug_addr(req.addr);
    }

class GenericDRAMController final : public IDRAMController, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IDRAMController, GenericDRAMController, "Generic", "A generic DRAM controller.");
  private:
    std::deque<Request> pending;          // A queue for read requests that are about to finish (callback after RL)

    ReqBuffer m_active_buffer;            // Buffer for requests being served. This has the highest priority
    ReqBuffer m_priority_buffer;          // Buffer for high-priority requests (e.g., maintenance like refresh).
    ReqBuffer m_read_buffer;              // Read request buffer
    ReqBuffer m_write_buffer;             // Write request buffer

    int m_bank_addr_idx = -1;

    // WI-005: strict closed-row mode. Every RD/WR request is served with an auto-precharge command (RDA/WRA), so the
    // bank closes after every access and no request can hit an open row.
    bool m_auto_precharge = false;
    int m_cmd_RD = -1, m_cmd_WR = -1, m_cmd_RDA = -1, m_cmd_WRA = -1;

    float m_wr_low_watermark;
    float m_wr_high_watermark;
    bool  m_is_write_mode = false;

    size_t s_row_hits = 0;
    size_t s_row_misses = 0;
    size_t s_row_conflicts = 0;
    size_t s_read_row_hits = 0;
    size_t s_read_row_misses = 0;
    size_t s_read_row_conflicts = 0;
    size_t s_write_row_hits = 0;
    size_t s_write_row_misses = 0;
    size_t s_write_row_conflicts = 0;

    size_t m_num_cores = 0;
    std::vector<size_t> s_read_row_hits_per_core;
    std::vector<size_t> s_read_row_misses_per_core;
    std::vector<size_t> s_read_row_conflicts_per_core;

    size_t s_num_read_reqs = 0;
    size_t s_num_write_reqs = 0;
    size_t s_num_other_reqs = 0;
    size_t s_queue_len = 0;
    size_t s_read_queue_len = 0;
    size_t s_write_queue_len = 0;
    size_t s_priority_queue_len = 0;
    float s_queue_len_avg = 0;
    float s_read_queue_len_avg = 0;
    float s_write_queue_len_avg = 0;
    float s_priority_queue_len_avg = 0;

    size_t s_read_latency = 0;
    float s_avg_read_latency = 0;

    // WI-019: CoWs table requests. Active only when cows_table_priority is configured (real table access); otherwise nothing is
    // registered or changed (bit-exact).
    bool m_cows_table = false;          // cows_table_priority given
    bool m_cows_table_first = false;    // `first`: oldest issuable table read ahead of FR-FCFS in read mode
    uint64_t s_table_reads = 0, s_table_writes = 0, s_table_reads_forwarded = 0;
    uint64_t s_table_read_lat_sum = 0;
    std::map<uint64_t, uint64_t> m_table_read_lat_hist;
    float s_table_read_lat_mean = 0, s_table_read_lat_p50 = 0, s_table_read_lat_p90 = 0;
    uint64_t s_data_reads = 0, s_data_read_lat_sum = 0; float s_data_read_lat_mean = 0;   // non-table reads (DRAM-served)
    uint64_t s_table_row_rd[3] = {0, 0, 0}, s_table_row_wr[3] = {0, 0, 0};   // hit / miss / conflict at first command
    uint64_t s_table_reads_waited_wm = 0, s_table_wm_cycles = 0;   // review R6: table reads that sat through a write drain
    uint64_t s_table_first_picks = 0;   // `first`: table reads chosen ahead of FR-FCFS
    // WI-022 item 1 (stats only): per DRAM-served table read, histograms (sum / mean / p50 / p90, controller cycles) of
    //   wait    = arrival -> first command;  wait_wm = its write-mode ticks;  wait_rm = its read-mode ticks, of which
    //   wait_busy = ticks whose target bank holds ANOTHER open row (W5 "target bank busy");
    //   service = first command -> data (depart).
    // Read-mode ticks that are not bank-busy are split (sums only) into: first command not ready (timing: tRP/tRC recovery, tRRD/tFAW,
    // bus, refresh) vs ready but another request was issued (queue order / scheduling).
    struct LatHist {
      std::map<uint64_t, uint64_t> h; uint64_t sum = 0; float mean = 0, p50 = 0, p90 = 0;
      void add(uint64_t v) { h[v]++; sum += v; }
      void fin() {
        uint64_t n = 0; for (auto& [l, c] : h) n += c;
        mean = n ? (float)sum / n : 0;
        uint64_t acc = 0; bool f50 = false;
        for (auto& [l, c] : h) {
          acc += c;
          if (!f50 && acc * 2 >= n) { p50 = l; f50 = true; }
          if (acc * 10 >= n * 9) { p90 = l; break; }
        }
      }
    };
    LatHist m_tw, m_tw_wm, m_tw_rm, m_tw_busy, m_tsvc;
    uint64_t s_tw_notready = 0, s_tw_ready = 0, s_tw_busy_pre_ready = 0, s_tw_nr_bank = 0;
    uint64_t s_tw_nr_level[3] = {0, 0, 0};
    // PI 2026-11-28: cause of rank-level blocking, reconstructed from this channel's issued commands per rank (stats only).
    // First match: refresh (REFab/RFMab within nRFC1/nRFM1) > tFAW (4th-last ACT + nFAW) > tRRD_S (last ACT + nRRDS) >
    // write->read turnaround (RD blocked within nCCDS_WTR of the rank's last WR/WRA) > other (tCCD_S, sibling-rank, PREA).
    struct RankHist { Clk_t ref_end = -1; Clk_t act[4] = {-1, -1, -1, -1}; int act_i = 0; Clk_t last_wr = -1; };
    std::map<int, RankHist> m_rank_hist;
    int m_rank_idx = -1, m_cmd_act = -1, m_cmd_rd = -1;
    uint64_t s_tw_rank_cause[5] = {0, 0, 0, 0, 0};   // notready ticks blocked at level 0..2 (channel / rank / bankgroup), non-exclusive
    LatHist m_tw_occ;   // review H1: target bank occupied = bank_busy + notready blocked at the bank level
    // WI-021 `first_drain`: a ready table read may bypass a write drain, while (a) the write buffer is below kWriteBypassLimit
    // (= max_size; ReqBuffer::enqueue accepts up to max_size + 1, so this is a slightly conservative hard limit) and (b) fewer than
    // m_drain_bypass_max bypasses happened since the last issued write.
    bool m_cows_table_drain = false;
    int m_drain_bypass_max = 4;
    int m_consec_bypass = 0;
    bool m_pick_bypass = false;   // this tick's request was chosen by the bypass branch
    uint64_t s_drain_bypasses = 0, s_drain_bypass_pre = 0, s_drain_bypass_act = 0;   // review D2: bypasses whose command is PRE / ACT
    uint64_t s_drain_blocked_cap = 0, s_drain_blocked_k = 0;
    uint64_t s_drain_bypass_vetoed = 0;   // bypass picks vetoed by the 2.3 closing check (the tick issues nothing)
    uint64_t s_prio_blocked_ticks = 0, s_prio_blocked_demand_ticks = 0;   // WI-026 review D2: ticks lost to a not-ready priority head
    // WI-026 diagnostic (stats only): read-buffer pressure while some rank of this channel is in all-bank refresh (REFab issued
    // < nRFC1 ago, tracked from issued commands) vs not.
    std::vector<Clk_t> m_ref_end;   // per rank
    int m_cmd_refab = -1;
    uint64_t s_rd_ref_ticks = 0, s_rd_occ_ref = 0, s_rd_occ_noref = 0, s_rd_full_ref = 0, s_rd_full_noref = 0;
    uint64_t s_rd_on_ref_rank = 0;   // sum over ticks of reads in the read buffer that target a rank in refresh
    uint64_t s_rd_refused_ref = 0, s_rd_refused_noref = 0;
    // per DRAM-served read: accepted while ITS rank was in refresh (count, latency sum) vs not
    uint64_t s_rd_arr_ref_n = 0, s_rd_arr_ref_lat = 0, s_rd_arr_noref_n = 0, s_rd_arr_noref_lat = 0;
    uint64_t s_write_refusals = 0, s_write_buffer_hwm = 0, s_write_lat_sum = 0, s_writes_issued = 0;   // review D1, write latency

    std::string m_controller_stats_name;
    std::vector<std::tuple<Addr_t, uint64_t>> m_dram_latency_samples;
    std::vector<std::tuple<Addr_t, uint64_t>> m_total_latency_samples;

  private:

    void dump_dram_latency_samples()
    {
      std::string fname = m_controller_stats_name;
      fname += "_dram_latency_samples";
      auto of = std::ofstream(fname);
      of << "# Latency samples for dram controller only " + std::to_string(m_channel_id) + ". Format: addr\t\tlatency\n";
      for (const auto& sample : m_dram_latency_samples) {
        of << std::hex << "0x"<< std::get<0>(sample) << "\t\t\t" << std::dec << std::get<1>(sample) << "\n";
      }
      of.close();
    }

    void dump_total_latency_samples()
    {
      std::string fname = m_controller_stats_name;
      fname += "_total_latency_samples";
      auto of = std::ofstream(fname);
      of << "# Latency samples for total latency " + std::to_string(m_channel_id) + ". Format: addr\t\tlatency\n";
      for (const auto& sample : m_total_latency_samples) {
        of << std::hex << "0x"<< std::get<0>(sample) << "\t\t\t" << std::dec << std::get<1>(sample) << "\n";
      }
      of.close();
    }

  public:
    void init() override {
      m_wr_low_watermark =  param<float>("wr_low_watermark").desc("Threshold for switching back to read mode.").default_val(0.2f);
      m_wr_high_watermark = param<float>("wr_high_watermark").desc("Threshold for switching to write mode.").default_val(0.8f);

      auto str = param<std::string>("controller_stats_name_prefix").desc("Prefix for the DRAM controller related stats names.").default_val("");
      m_controller_stats_name = str;
      m_controller_stats_name += "_";
      m_dump_raw_latency = param<bool>("dump_raw_latency_samples")
        .desc("Write the raw per-request latency sample files (large).").default_val(true);

      {   // WI-019
        std::string tp = param<std::string>("cows_table_priority")
          .desc("WI-019: fcfs | first (table reads ahead of other reads in read mode); empty = no CoWs table requests").default_val("");
        if (tp != "" && tp != "fcfs" && tp != "first" && tp != "first_drain")
          throw ConfigurationError("cows_table_priority must be fcfs, first or first_drain (got {})", tp);
        m_cows_table = tp != "";
        m_cows_table_first = tp == "first" || tp == "first_drain";   // first_drain includes the read-mode priority
        m_cows_table_drain = tp == "first_drain";   // WI-021
        m_drain_bypass_max = param<int>("cows_drain_bypass_max")
          .desc("WI-021: max consecutive table-read bypasses of a write drain without an issued write").default_val(4);
      }
      m_scheduler = create_child_ifce<IScheduler>();
      m_refresh = create_child_ifce<IRefreshManager>();
      m_rowpolicy = create_child_ifce<IRowPolicy>();

      if (m_config["plugins"]) {
        YAML::Node plugin_configs = m_config["plugins"];
        for (YAML::iterator it = plugin_configs.begin(); it != plugin_configs.end(); ++it) {
          m_plugins.push_back(create_child_ifce<IControllerPlugin>(*it));
        }
      }
    };

    bool m_dump_raw_latency = true;   // WI-005 (D-28)

    void fini() {
      if (m_dump_raw_latency) {
        dump_dram_latency_samples();
        dump_total_latency_samples();
      }
      dump_read_latency_by_rowclass();
    }

    // WI-005: post-warmup histogram of controller read latency (arrive -> data, DRAM cycles) by row-buffer outcome, so the
    // non-hit (miss + conflict) mean/p95 can be reported (D-24). Forwarded reads (write-buffer hits) are excluded.
    std::map<Clk_t, std::array<uint64_t, 3>> m_read_lat_by_class;
    uint64_t s_read_lat_sum_by_class[3] = {0, 0, 0};
    uint64_t s_read_cnt_by_class[3] = {0, 0, 0};
    uint64_t s_redirect_rowclass[3] = {0, 0, 0};   // WI-013: row outcome of reads redirected to a CoWs source row

    void dump_read_latency_by_rowclass() {
      auto of = std::ofstream(m_controller_stats_name + "_read_latency_by_rowclass");
      of << "# post-warmup controller read latency (DRAM cycles) histogram, channel " << m_channel_id << ": latency hit miss conflict\n";
      for (const auto& [lat, c] : m_read_lat_by_class) {
        of << lat << " " << c[0] << " " << c[1] << " " << c[2] << "\n";
      }
    }

    void on_reset_stats() override {
      m_read_lat_by_class.clear();   // registered sums/counts are reset by the stats system
      m_table_read_lat_hist.clear();   // WI-019
      for (auto* x : {&m_tw, &m_tw_wm, &m_tw_rm, &m_tw_busy, &m_tw_occ, &m_tsvc}) x->h.clear();   // WI-022 (sums are registered)
    }

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = memory_system->get_ifce<IDRAM>();
      m_bank_addr_idx = m_dram->m_levels("bank");
      m_rank_idx = m_dram->m_levels("rank");
      m_cmd_act = m_dram->m_commands("ACT"); m_cmd_rd = m_dram->m_commands("RD");   // WI-022
      m_cmd_refab = m_dram->m_commands("REFab"); m_ref_end.assign(m_dram->get_level_size("rank"), -1);   // WI-026 diagnostic

      m_auto_precharge = param<bool>("auto_precharge")
        .desc("Strict closed-row: serve every read/write with RDA/WRA (auto-precharge).").default_val(false);
      if (m_auto_precharge) {
        for (auto c : {"RD", "WR", "RDA", "WRA"}) {
          if (!m_dram->m_commands.contains(c))
            throw ConfigurationError("auto_precharge requires DRAM command {} in {}!", c, m_dram->get_name());
        }
        m_cmd_RD = m_dram->m_commands("RD");   m_cmd_WR = m_dram->m_commands("WR");
        m_cmd_RDA = m_dram->m_commands("RDA"); m_cmd_WRA = m_dram->m_commands("WRA");
      }
      m_priority_buffer.max_size = 512*3 + 32;
      m_read_buffer.max_size = param<int>("read_buffer_size")   // WI-019 test (f): a tiny queue forces back-pressure
        .desc("Read buffer capacity (default 32 = the ReqBuffer default).").default_val(32);
      m_write_buffer.max_size = param<int>("write_buffer_size")   // WI-021 review D3: small-write-buffer stress test
        .desc("Write buffer capacity (default 32 = the ReqBuffer default).").default_val(32);

      m_controller_stats_name += std::to_string(m_channel_id);

      m_num_cores = frontend->get_num_cores();

      s_read_row_hits_per_core.resize(m_num_cores, 0);
      s_read_row_misses_per_core.resize(m_num_cores, 0);
      s_read_row_conflicts_per_core.resize(m_num_cores, 0);

      register_stat(s_row_hits).name("row_hits_{}", m_channel_id);
      register_stat(s_row_misses).name("row_misses_{}", m_channel_id);
      register_stat(s_row_conflicts).name("row_conflicts_{}", m_channel_id);
      register_stat(s_read_row_hits).name("read_row_hits_{}", m_channel_id);
      register_stat(s_read_row_misses).name("read_row_misses_{}", m_channel_id);
      register_stat(s_read_row_conflicts).name("read_row_conflicts_{}", m_channel_id);
      register_stat(s_write_row_hits).name("write_row_hits_{}", m_channel_id);
      register_stat(s_write_row_misses).name("write_row_misses_{}", m_channel_id);
      register_stat(s_write_row_conflicts).name("write_row_conflicts_{}", m_channel_id);

      for (size_t core_id = 0; core_id < m_num_cores; core_id++) {
        register_stat(s_read_row_hits_per_core[core_id]).name("read_row_hits_core_{}", core_id);
        register_stat(s_read_row_misses_per_core[core_id]).name("read_row_misses_core_{}", core_id);
        register_stat(s_read_row_conflicts_per_core[core_id]).name("read_row_conflicts_core_{}", core_id);
      }

      register_stat(s_num_read_reqs).name("num_read_reqs_{}", m_channel_id);
      register_stat(s_num_write_reqs).name("num_write_reqs_{}", m_channel_id);
      register_stat(s_num_other_reqs).name("num_other_reqs_{}", m_channel_id);
      register_stat(s_queue_len).name("queue_len_{}", m_channel_id);
      register_stat(s_read_queue_len).name("read_queue_len_{}", m_channel_id);
      register_stat(s_write_queue_len).name("write_queue_len_{}", m_channel_id);
      register_stat(s_priority_queue_len).name("priority_queue_len_{}", m_channel_id);
      register_stat(s_prio_blocked_ticks).name("priority_head_blocked_ticks_{}", m_channel_id);   // WI-026 review D2
      register_stat(s_prio_blocked_demand_ticks).name("priority_head_blocked_demand_ticks_{}", m_channel_id);
      register_stat(s_rd_ref_ticks).name("refdiag_ticks_rank_in_refresh_{}", m_channel_id);   // WI-026 diagnostic
      register_stat(s_rd_occ_ref).name("refdiag_rdbuf_occ_sum_refresh_{}", m_channel_id);
      register_stat(s_rd_occ_noref).name("refdiag_rdbuf_occ_sum_norefresh_{}", m_channel_id);
      register_stat(s_rd_full_ref).name("refdiag_rdbuf_full_ticks_refresh_{}", m_channel_id);
      register_stat(s_rd_full_noref).name("refdiag_rdbuf_full_ticks_norefresh_{}", m_channel_id);
      register_stat(s_rd_on_ref_rank).name("refdiag_rd_on_refreshing_rank_sum_{}", m_channel_id);
      register_stat(s_rd_refused_ref).name("refdiag_read_refusals_refresh_{}", m_channel_id);
      register_stat(s_rd_refused_noref).name("refdiag_read_refusals_norefresh_{}", m_channel_id);
      register_stat(s_rd_arr_ref_n).name("refdiag_reads_own_rank_refreshing_{}", m_channel_id);
      register_stat(s_rd_arr_ref_lat).name("refdiag_read_latency_sum_own_rank_refreshing_{}", m_channel_id);
      register_stat(s_rd_arr_noref_n).name("refdiag_reads_own_rank_not_refreshing_{}", m_channel_id);
      register_stat(s_rd_arr_noref_lat).name("refdiag_read_latency_sum_own_rank_not_refreshing_{}", m_channel_id);
      register_stat(s_queue_len_avg).name("queue_len_avg_{}", m_channel_id);
      register_stat(s_read_queue_len_avg).name("read_queue_len_avg_{}", m_channel_id);
      register_stat(s_write_queue_len_avg).name("write_queue_len_avg_{}", m_channel_id);
      register_stat(s_priority_queue_len_avg).name("priority_queue_len_avg_{}", m_channel_id);

      register_stat(s_read_latency).name("read_latency_{}", m_channel_id);
      const char* rc_names[3] = {"hit", "miss", "conflict"};
      for (int k = 0; k < 3; k++) {
        register_stat(s_read_lat_sum_by_class[k]).name("read_latency_sum_{}_{}", rc_names[k], m_channel_id);
        register_stat(s_read_cnt_by_class[k]).name("read_count_{}_{}", rc_names[k], m_channel_id);
        register_stat(s_redirect_rowclass[k]).name("redirected_read_{}_{}", rc_names[k], m_channel_id);
      }
      register_stat(s_avg_read_latency).name("avg_read_latency_{}", m_channel_id);
      if (m_cows_table) {   // WI-019
        register_stat(s_table_reads).name("table_reads_{}", m_channel_id);
        register_stat(s_table_writes).name("table_writes_{}", m_channel_id);
        register_stat(s_table_reads_forwarded).name("table_reads_forwarded_{}", m_channel_id);
        register_stat(s_table_read_lat_sum).name("table_read_latency_sum_{}", m_channel_id);
        register_stat(s_table_read_lat_mean).name("table_read_latency_mean_{}", m_channel_id);
        register_stat(s_table_read_lat_p50).name("table_read_latency_p50_{}", m_channel_id);
        register_stat(s_table_read_lat_p90).name("table_read_latency_p90_{}", m_channel_id);
        register_stat(s_data_reads).name("data_reads_{}", m_channel_id);
        register_stat(s_data_read_lat_sum).name("data_read_latency_sum_{}", m_channel_id);
        register_stat(s_data_read_lat_mean).name("data_read_latency_mean_{}", m_channel_id);
        const char* rc[3] = {"hit", "miss", "conflict"};
        for (int k = 0; k < 3; k++) {
          register_stat(s_table_row_rd[k]).name("table_read_row_{}_{}", rc[k], m_channel_id);
          register_stat(s_table_row_wr[k]).name("table_write_row_{}_{}", rc[k], m_channel_id);
        }
        register_stat(s_table_reads_waited_wm).name("table_reads_waited_write_mode_{}", m_channel_id);
        register_stat(s_table_wm_cycles).name("table_read_write_mode_cycles_{}", m_channel_id);
        register_stat(s_table_first_picks).name("table_first_picks_{}", m_channel_id);
        {   // WI-022 item 1
          std::pair<LatHist*, const char*> hs[] = {{&m_tw, "wait"}, {&m_tw_wm, "wait_wm"}, {&m_tw_rm, "wait_rm"},
                                                   {&m_tw_busy, "wait_bank_busy"}, {&m_tw_occ, "wait_bank_occupied"},
                                                   {&m_tsvc, "service"}};
          for (auto& [x, n] : hs) {
            register_stat(x->sum).name("table_read_{}_sum_{}", n, m_channel_id);
            register_stat(x->mean).name("table_read_{}_mean_{}", n, m_channel_id);
            register_stat(x->p50).name("table_read_{}_p50_{}", n, m_channel_id);
            register_stat(x->p90).name("table_read_{}_p90_{}", n, m_channel_id);
          }
          register_stat(s_tw_notready).name("table_read_wait_rm_notready_sum_{}", m_channel_id);
          register_stat(s_tw_ready).name("table_read_wait_rm_ready_sum_{}", m_channel_id);
          register_stat(s_tw_nr_bank).name("table_read_wait_rm_notready_bank_sum_{}", m_channel_id);
          register_stat(s_tw_busy_pre_ready).name("table_read_wait_bank_busy_pre_ready_sum_{}", m_channel_id);
          for (int l = 0; l < m_bank_addr_idx && l < 3; l++)
            register_stat(s_tw_nr_level[l]).name("table_read_wait_rm_notready_{}_ticks_{}", m_dram->m_levels(l), m_channel_id);
          const char* rc5[] = {"refresh", "faw", "rrd", "wtr", "other"};
          for (int k = 0; k < 5; k++)
            register_stat(s_tw_rank_cause[k]).name("table_read_wait_rm_notready_rank_{}_ticks_{}", rc5[k], m_channel_id);
        }
        register_stat(s_drain_bypasses).name("table_drain_bypasses_{}", m_channel_id);   // WI-021
        register_stat(s_drain_bypass_pre).name("table_drain_bypass_pre_{}", m_channel_id);
        register_stat(s_drain_bypass_act).name("table_drain_bypass_act_{}", m_channel_id);
        register_stat(s_drain_blocked_cap).name("table_drain_blocked_cap_{}", m_channel_id);
        register_stat(s_drain_blocked_k).name("table_drain_blocked_k_{}", m_channel_id);
        register_stat(s_drain_bypass_vetoed).name("table_drain_bypass_vetoed_{}", m_channel_id);
        register_stat(s_write_refusals).name("write_refusals_{}", m_channel_id);
        register_stat(s_write_buffer_hwm).name("write_buffer_hwm_{}", m_channel_id);
        register_stat(s_write_lat_sum).name("write_latency_sum_{}", m_channel_id);
        register_stat(s_writes_issued).name("writes_issued_{}", m_channel_id);
      }
    };

    bool send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);
      if (m_auto_precharge) {
        if      (req.final_command == m_cmd_RD) req.final_command = m_cmd_RDA;
        else if (req.final_command == m_cmd_WR) req.final_command = m_cmd_WRA;
      }

//      printf("DRAM Controller: Received request. clk=%lu, addr=0x%lx, type=%d, cache_status=%d\n", m_clk, req.addr, req.type_id, req.cache_status);

      switch (req.type_id) {
        case Request::Type::Read: {
          s_num_read_reqs++;
          break;
        }
        case Request::Type::Write: {
          s_num_write_reqs++;
          break;
        }
        default: {
          s_num_other_reqs++;
          break;
        }
      }

      // Forward existing write requests to incoming read requests
      if (req.type_id == Request::Type::Read) {
        auto compare_addr = [req](const Request& wreq) {
          return wreq.addr == req.addr;
        };
        if (std::find_if(m_write_buffer.begin(), m_write_buffer.end(), compare_addr) != m_write_buffer.end()) {
          req.arrive = m_clk; // Yoav bugfix: we should set the arrive time for forwarded requests as well

          // The request will depart at the next cycle
          req.depart = m_clk + 1;

          pending.push_back(req);
          if (m_cows_table && req.is_cows_table) s_table_reads++;   // WI-019: accepted (forwarded) table read
          return true;
        }
      }

      // Else, enqueue them to corresponding buffer based on request type id
      bool is_success = false;
      req.arrive = m_clk;
      if        (req.type_id == Request::Type::Read) {
        { int rk = req.addr_vec[m_rank_idx];   // WI-026 diagnostic: was this read's own rank in refresh on arrival?
          req.refdiag_rank_in_refresh = rk >= 0 && m_clk < m_ref_end[rk]; }
        is_success = m_read_buffer.enqueue(req);
      } else if (req.type_id == Request::Type::Write) {
        is_success = m_write_buffer.enqueue(req);
      } else {
        throw std::runtime_error("Invalid request type!");
      }
      if (!is_success) {
        // We could not enqueue the request
        req.arrive = -1;
        if (m_cows_table && req.type_id == Request::Type::Write) s_write_refusals++;   // WI-021 review D1
        if (req.type_id == Request::Type::Read) {   // WI-026 diagnostic
          bool ref = false; for (Clk_t e : m_ref_end) ref |= m_clk < e;
          if (ref) s_rd_refused_ref++; else s_rd_refused_noref++;
        }
        return false;
      }
      if (m_cows_table && req.type_id == Request::Type::Write)   // WI-021: write-buffer high-water mark
        s_write_buffer_hwm = std::max<uint64_t>(s_write_buffer_hwm, m_write_buffer.size());
      if (m_cows_table && req.is_cows_table) {   // WI-019: count accepted table requests only (refusals are retried by the CoWs side)
        if (req.type_id == Request::Type::Read) s_table_reads++; else s_table_writes++;
      }

      return true;
    };

    bool other_active_request_in_bank(const ReqBuffer::iterator& req_it) override {
      for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
        if (&*_it == &*req_it)
          continue;
        bool is_matching = true;
        for (int i = 0; i < m_bank_addr_idx + 1; i++) {
          if (_it->addr_vec[i] != req_it->addr_vec[i] && _it->addr_vec[i] != -1 && req_it->addr_vec[i] != -1) {
            is_matching = false;
            break;
          }
        }
        if (is_matching)
          return true;
      }
      return false;
    }

    int pending_requests_to_row(const AddrVec_t& addr_vec, const Request* exclude) override {   // WI-017
      int row_idx = m_dram->m_levels("row");
      auto match = [&](const Request& r) {
        if (&r == exclude) return false;
        for (int i = 0; i <= row_idx; i++)
          if (r.addr_vec[i] != addr_vec[i] && r.addr_vec[i] != -1 && addr_vec[i] != -1) return false;
        return true;
      };
      int m = 0;
      for (auto& r : m_read_buffer)  if (match(r)) { m |= PENDING_READ; break; }
      for (auto& r : m_write_buffer) if (match(r)) { m |= PENDING_WRITE; break; }
      for (auto& r : m_active_buffer)
        if (match(r)) m |= (r.type_id == Request::Type::Write) ? PENDING_ACTIVE_WRITE : PENDING_ACTIVE_READ;
      return m;
    }

    bool priority_send(Request& req) override {
      req.final_command = m_dram->m_request_translations(req.type_id);

      bool is_success = false;
      is_success = m_priority_buffer.enqueue(req);
      return is_success;
    }

    void tick() override {
      m_clk++;

      // Update statistics
      s_queue_len += m_read_buffer.size() + m_write_buffer.size() + m_priority_buffer.size() + pending.size();
      s_read_queue_len += m_read_buffer.size() + pending.size();
      s_write_queue_len += m_write_buffer.size();
      s_priority_queue_len += m_priority_buffer.size();

      {   // WI-026 diagnostic (stats only)
        bool ref = false;
        for (Clk_t e : m_ref_end) ref |= m_clk < e;
        if (ref) {
          s_rd_ref_ticks++; s_rd_occ_ref += m_read_buffer.size();
          if (m_read_buffer.size() > m_read_buffer.max_size) s_rd_full_ref++;   // full = capacity max_size + 1 (ReqBuffer::enqueue)
          for (auto& r : m_read_buffer) { int rk = r.addr_vec[m_rank_idx]; if (rk >= 0 && m_clk < m_ref_end[rk]) s_rd_on_ref_rank++; }
        } else {
          s_rd_occ_noref += m_read_buffer.size();
          if (m_read_buffer.size() > m_read_buffer.max_size) s_rd_full_noref++;
        }
      }
      if (m_cows_table && m_is_write_mode) {   // WI-019 review R6: table reads waiting through a write drain
        for (auto& r : m_read_buffer) if (r.is_cows_table) { r.cows_waited_wm = true; s_table_wm_cycles++; }
      }
      if (m_cows_table) {   // WI-022 item 1: classify this tick for every table read still waiting for its first command
        for (auto& r : m_read_buffer) {
          if (!r.is_cows_table || r.is_stat_updated) continue;
          if (m_is_write_mode) { r.cows_wait_wm++; continue; }
          int pc = m_dram->get_preq_command(r.final_command, r.addr_vec);
          bool ready = m_dram->check_ready(pc, r.addr_vec);
          if (m_dram->check_node_open(r.final_command, r.addr_vec) && !m_dram->check_rowbuffer_hit(r.final_command, r.addr_vec)) {
            r.cows_wait_busy++;   // review H2: is the PRE itself timing-blocked (tRAS/tRTP/tWR) or ready but not issued?
            if (ready) s_tw_busy_pre_ready++;
          } else if (!ready) {   // review H1: blocked at the target bank (tRP/tRC recovery) vs above it (tRRD/tFAW/tCCD/bus)
            r.cows_wait_notready++;
            int bm = m_dram->check_ready_block_mask(pc, r.addr_vec);
            if (bm & (1 << m_bank_addr_idx)) r.cows_wait_nr_bank++;
            for (int l = 0; l < m_bank_addr_idx && l < 3; l++) if (bm & (1 << l)) s_tw_nr_level[l]++;   // non-exclusive
            if (bm & (1 << m_rank_idx)) s_tw_rank_cause[rank_cause(pc, r.addr_vec[m_rank_idx])]++;
          } else
            r.cows_wait_ready++;
        }
      }
      // 1. Serve completed reads
      serve_completed_reads();

      m_refresh->tick();

      // 2. Try to find a request to serve.
      ReqBuffer::iterator req_it;
      ReqBuffer* buffer = nullptr;
      bool request_found = schedule_request(req_it, buffer);

      // 2.1 Take row policy action
      m_rowpolicy->update(request_found, req_it);

      // 3. Update all plugins
      for (auto plugin : m_plugins) {
        plugin->update(request_found, req_it);
      }

      if (request_found && m_cows_table) {   // WI-021: bypass / write accounting at issue
        if (m_pick_bypass) {   // chosen by the bypass branch and not vetoed by the closing check
          s_drain_bypasses++; m_consec_bypass++;
          const auto& meta = m_dram->m_command_meta(req_it->command);
          if (meta.is_closing) s_drain_bypass_pre++;
          if (meta.is_opening) s_drain_bypass_act++;
        }
        if (req_it->type_id == Request::Type::Write && m_dram->m_command_meta(req_it->command).is_accessing) {
          m_consec_bypass = 0;
          if (req_it->command == req_it->final_command) { s_write_lat_sum += m_clk - req_it->arrive; s_writes_issued++; }
        }
      }
      // 4. Finally, issue the commands to serve the request
      if (request_found) {
        // If we find a real request to serve
        if (req_it->is_stat_updated == false) {
          update_request_stats(req_it);
        }
        m_dram->issue_command(req_it->command, req_it->addr_vec);
        if (m_cows_table) note_rank_cmd(req_it->command, req_it->addr_vec);   // WI-022 (stats only)
        if (req_it->command == m_cmd_refab && req_it->addr_vec[m_rank_idx] >= 0)   // WI-026 diagnostic
          m_ref_end[req_it->addr_vec[m_rank_idx]] = m_clk + m_dram->m_timing_vals("nRFC1");

        // If we are issuing the last command, set depart clock cycle and move the request to the pending queue
        if (req_it->command == req_it->final_command) {
          if (req_it->type_id == Request::Type::Read) {
            req_it->depart = req_it->arrive + latmul(m_clk - req_it->arrive + m_dram->m_read_latency);
            pending.push_back(*req_it);
          } else if (req_it->type_id == Request::Type::Write) {
            // TODO: Add code to update statistics
          }
          buffer->remove(req_it);
        } else {
          if (m_dram->m_command_meta(req_it->command).is_opening) {
            m_active_buffer.enqueue(*req_it);
            buffer->remove(req_it);
          }
        }

      }

    };


  private:
    /**
     * @brief    Helper function to check if a request is hitting an open row
     * @details
     *
     */
    bool is_row_hit(ReqBuffer::iterator& req)
    {
        return m_dram->check_rowbuffer_hit(req->final_command, req->addr_vec);
    }
    /**
     * @brief    Helper function to check if a request is opening a row
     * @details
     *
    */
    bool is_row_open(ReqBuffer::iterator& req)
    {
        return m_dram->check_node_open(req->final_command, req->addr_vec);
    }

    /**
     * @brief
     * @details
     *
     */
    // WI-022 (stats only): per-rank command history for the rank-level cause split
    void note_rank_cmd(int cmd, const AddrVec_t& av) {
      int rk = av[m_rank_idx];
      if (rk < 0) return;
      auto& h = m_rank_hist[rk];
      const auto& c = m_dram->m_commands;
      if (cmd == c("REFab")) h.ref_end = std::max<Clk_t>(h.ref_end, m_clk + m_dram->m_timing_vals("nRFC1"));
      else if (cmd == c("RFMab")) h.ref_end = std::max<Clk_t>(h.ref_end, m_clk + m_dram->m_timing_vals("nRFM1"));
      else if (cmd == m_cmd_act) { h.act[h.act_i] = m_clk; h.act_i = (h.act_i + 1) & 3; }
      else if (cmd == c("WR") || cmd == c("WRA")) h.last_wr = m_clk;
    }
    int rank_cause(int pc, int rk) {   // 0 refresh, 1 tFAW, 2 tRRD_S, 3 write->read turnaround, 4 other
      auto it = m_rank_hist.find(rk);
      if (it == m_rank_hist.end()) return 4;
      const auto& h = it->second;
      if (m_clk < h.ref_end) return 0;
      if (pc == m_cmd_act) {
        Clk_t oldest = h.act[h.act_i];   // the 4th-last ACT (ring buffer of 4)
        if (oldest >= 0 && m_clk < oldest + m_dram->m_timing_vals("nFAW")) return 1;
        Clk_t last = h.act[(h.act_i + 3) & 3];
        if (last >= 0 && m_clk < last + m_dram->m_timing_vals("nRRDS")) return 2;
      } else if (pc == m_cmd_rd || pc == m_dram->m_commands("RDA")) {
        if (h.last_wr >= 0 && m_clk < h.last_wr + m_dram->m_timing_vals("nCCDS_WTR")) return 3;
      }
      return 4;
    }

    void update_request_stats(ReqBuffer::iterator& req)
    {
      req->is_stat_updated = true;
      if (m_cows_table && req->is_cows_table) req->cows_first_cmd = m_clk;   // WI-022 item 1

      if (m_cows_table && req->is_cows_table) {   // WI-019: row-policy outcome of table requests (also counted below)
        int k = is_row_hit(req) ? 0 : (is_row_open(req) ? 2 : 1);
        if (req->type_id == Request::Type::Read) s_table_row_rd[k]++; else s_table_row_wr[k]++;
      }
      if (req->type_id == Request::Type::Read)
      {
        if (is_row_hit(req)) {
          s_read_row_hits++;
          s_row_hits++;
          req->row_class = 0;
          if (req->orig_addr != -1) s_redirect_rowclass[0]++;   // WI-013: read redirected to a CoWs source row
          if (req->source_id != -1)
            s_read_row_hits_per_core[req->source_id]++;
        } else if (is_row_open(req)) {
          s_read_row_conflicts++;
          s_row_conflicts++;
          req->row_class = 2;
          if (req->orig_addr != -1) s_redirect_rowclass[2]++;   // WI-013: read redirected to a CoWs source row
          if (req->source_id != -1)
            s_read_row_conflicts_per_core[req->source_id]++;
        } else {
          s_read_row_misses++;
          s_row_misses++;
          req->row_class = 1;
          if (req->orig_addr != -1) s_redirect_rowclass[1]++;   // WI-013: read redirected to a CoWs source row
          if (req->source_id != -1)
            s_read_row_misses_per_core[req->source_id]++;
        }
      }
      else if (req->type_id == Request::Type::Write)
      {
        if (is_row_hit(req)) {
          s_write_row_hits++;
          s_row_hits++;
        } else if (is_row_open(req)) {
          s_write_row_conflicts++;
          s_row_conflicts++;
        } else {
          s_write_row_misses++;
          s_row_misses++;
        }
      }
    }

    /**
     * @brief    Helper function to serve the completed read requests
     * @details
     * This function is called at the beginning of the tick() function.
     * It checks the pending queue to see if the top request has received data from DRAM.
     * If so, it finishes this request by calling its callback and poping it from the pending queue.
     */
    void serve_completed_reads() {
      if (pending.size()) {
        // Check the first pending request
        auto& req = pending[0];
        if (req.depart <= m_clk) {
          // Request received data from dram
          if (m_cows_table && req.is_cows_table && req.depart - req.arrive <= 1) s_table_reads_forwarded++;   // WI-019
          if (req.depart - req.arrive > 1) {
            // Check if this requests accesses the DRAM or is being forwarded.
            // TODO add the stats back
            if(is_debug_addr(req)) {
              std::fprintf(stdout, "DEBUG req complete clk=%llu (arrive=%llu, depart=%llu)\n",
                          static_cast<unsigned long long>(req.depart-req.arrive),
                          static_cast<unsigned long long>(req.arrive),
                          static_cast<unsigned long long>(req.depart));
            }
            m_dram_latency_samples.push_back(std::make_tuple(req.addr, req.depart - req.arrive));
            m_total_latency_samples.push_back(std::make_tuple(req.addr, req.depart - req.core2llc));
            s_read_latency += req.depart - req.arrive;
            if (req.type_id == Request::Type::Read) {   // WI-026 diagnostic
              if (req.refdiag_rank_in_refresh) { s_rd_arr_ref_n++; s_rd_arr_ref_lat += req.depart - req.arrive; }
              else { s_rd_arr_noref_n++; s_rd_arr_noref_lat += req.depart - req.arrive; }
            }
            if (m_cows_table) {   // WI-019: table vs ordinary (data) read latency
              uint64_t lat = req.depart - req.arrive;
              if (req.is_cows_table) {
                s_table_read_lat_sum += lat; m_table_read_lat_hist[lat]++;
                if (req.cows_waited_wm) s_table_reads_waited_wm++;
                // WI-022 item 1: wait / service split at the first command; the per-tick classes cover the wait
                m_tw.add(req.cows_first_cmd - req.arrive); m_tsvc.add(req.depart - req.cows_first_cmd);
                m_tw_wm.add(req.cows_wait_wm); m_tw_rm.add(req.cows_wait_busy + req.cows_wait_notready + req.cows_wait_ready);
                m_tw_busy.add(req.cows_wait_busy); m_tw_occ.add(req.cows_wait_busy + req.cows_wait_nr_bank);
                s_tw_notready += req.cows_wait_notready; s_tw_ready += req.cows_wait_ready; s_tw_nr_bank += req.cows_wait_nr_bank;
              } else {
                s_data_reads++; s_data_read_lat_sum += lat;
              }
            }
            if (req.row_class >= 0) {
              Clk_t lat = req.depart - req.arrive;
              m_read_lat_by_class[lat][req.row_class]++;
              s_read_lat_sum_by_class[req.row_class] += lat;
              s_read_cnt_by_class[req.row_class]++;
            }
          }

//          printf("DRAM Controller: request complete. clk=%lu, addr=0x%lx, type=%d, cache_status=%d\n", m_clk, req.addr, req.type_id, req.cache_status);

          if (req.callback) {
            // If the request comes from outside (e.g., processor), call its callback
            req.callback(req);
          }
          // Finally, remove this request from the pending queue
          pending.pop_front();
        }
      };
    };


    /**
     * @brief    Checks if we need to switch to write mode
     *
     */
    void set_write_mode() {
      if (!m_is_write_mode) {
        if ((m_write_buffer.size() > m_wr_high_watermark * m_write_buffer.max_size) || m_read_buffer.size() == 0) {
          m_is_write_mode = true;
        }
      } else {
        if ((m_write_buffer.size() < m_wr_low_watermark * m_write_buffer.max_size) && m_read_buffer.size() != 0) {
          m_is_write_mode = false;
        }
      }
    };


    /**
     * @brief    Helper function to find a request to schedule from the buffers.
     *
     */
    bool schedule_request(ReqBuffer::iterator& req_it, ReqBuffer*& req_buffer) {
      bool request_found = false;
      m_pick_bypass = false;   // WI-021
      // 2.1    First, check the act buffer to serve requests that are already activating (avoid useless ACTs)
      if (req_it= m_scheduler->get_best_request(m_active_buffer); req_it != m_active_buffer.end()) {
        if (m_dram->check_ready(req_it->command, req_it->addr_vec)) {
          request_found = true;
          req_buffer = &m_active_buffer;
        }
      }

      // 2.2    If no requests can be scheduled from the act buffer, check the rest of the buffers
      if (!request_found) {
        // 2.2.1    We first check the priority buffer to prioritize e.g., maintenance requests
        if (m_priority_buffer.size() != 0) {
          req_buffer = &m_priority_buffer;
          req_it = m_priority_buffer.begin();
          req_it->command = m_dram->get_preq_command(req_it->final_command, req_it->addr_vec);

          request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
          if (!request_found & m_priority_buffer.size() != 0) {
            // WI-026 review D2 (stats only): the not-ready priority head (REFab / its PREA) blocks the whole channel this tick
            s_prio_blocked_ticks++;
            if (m_read_buffer.size() + m_write_buffer.size() != 0) s_prio_blocked_demand_ticks++;
            return false;
          }
        }

        // 2.2.1    If no request to be scheduled in the priority buffer, check the read and write buffers.
        if (!request_found) {
          // Query the write policy to decide which buffer to serve
          set_write_mode();
          auto& buffer = m_is_write_mode ? m_write_buffer : m_read_buffer;
          if (m_cows_table_drain && m_is_write_mode) {   // WI-021 `first_drain`: a ready table read may bypass the drain
            for (auto it = m_read_buffer.begin(); it != m_read_buffer.end(); it++) {
              if (!it->is_cows_table) continue;
              it->command = m_dram->get_preq_command(it->final_command, it->addr_vec);
              if (!m_dram->check_ready(it->command, it->addr_vec)) continue;
              const size_t kWriteBypassLimit = m_write_buffer.max_size;   // rule (a); capacity is max_size + 1
              if (m_write_buffer.size() >= kWriteBypassLimit) { s_drain_blocked_cap++; break; }
              if (m_consec_bypass >= m_drain_bypass_max) { s_drain_blocked_k++; break; }   // rule (b)
              req_it = it; request_found = true; req_buffer = &m_read_buffer; m_pick_bypass = true;
              break;
            }
          }
          if (!request_found && m_cows_table_first && !m_is_write_mode) {   // WI-019 `first`: the oldest table read whose NEXT command is ready
            for (auto it = m_read_buffer.begin(); it != m_read_buffer.end(); it++) {
              if (!it->is_cows_table) continue;
              it->command = m_dram->get_preq_command(it->final_command, it->addr_vec);
              if (m_dram->check_ready(it->command, it->addr_vec)) {
                req_it = it; request_found = true; req_buffer = &m_read_buffer; s_table_first_picks++;
                break;
              }
            }
          }
          if (!request_found) {
            if (req_it = m_scheduler->get_best_request(buffer); req_it != buffer.end()) {
              request_found = m_dram->check_ready(req_it->command, req_it->addr_vec);
              req_buffer = &buffer;
            }
          }
        }
      }

      // 2.3 If we find a request to schedule, we need to check if it will close an opened row in the active buffer.
      if (request_found) {
        if (m_dram->m_command_meta(req_it->command).is_closing) {
          auto& rowgroup = req_it->addr_vec;
          for (auto _it = m_active_buffer.begin(); _it != m_active_buffer.end(); _it++) {
            // WI-005: an RDA/WRA scheduled from the active buffer closes the row its own request opened; it must not
            // block itself. (Without auto_precharge no closing command is ever scheduled from the active buffer.)
            if (req_buffer == &m_active_buffer && _it == req_it) {
              continue;
            }
            auto& _it_rowgroup = _it->addr_vec;
            bool is_matching = true;
            for (int i = 0; i < m_bank_addr_idx + 1 ; i++) {
              if (_it_rowgroup[i] != rowgroup[i] && _it_rowgroup[i] != -1 && rowgroup[i] != -1) {
                is_matching = false;
                break;
              }
            }
            if (is_matching) {
              // WI-005: in strict mode a second request cannot have opened the same bank (every access closes it)
              if (m_auto_precharge && req_buffer == &m_active_buffer)
                throw std::runtime_error("auto_precharge: two active-buffer requests share a bank (impossible in strict mode)");
              request_found = false;
              if (m_pick_bypass) s_drain_bypass_vetoed++;   // WI-021 review note 1
              break;
            }
          }
        }
      }

      return request_found;
    }

    void finalize() override {
      s_avg_read_latency = (float) s_read_latency / (float) s_num_read_reqs;
      if (m_cows_table) {   // WI-019: table-read latency mean / p50 / p90 (DRAM-served table reads), ordinary-read mean
        uint64_t n = 0; for (auto& [l, c] : m_table_read_lat_hist) n += c;
        s_table_read_lat_mean = n ? (float)s_table_read_lat_sum / n : 0;
        uint64_t acc = 0; bool p50 = false;
        for (auto& [l, c] : m_table_read_lat_hist) {
          acc += c;
          if (!p50 && acc * 2 >= n) { s_table_read_lat_p50 = l; p50 = true; }
          if (acc * 10 >= n * 9) { s_table_read_lat_p90 = l; break; }
        }
        s_data_read_lat_mean = s_data_reads ? (float)s_data_read_lat_sum / s_data_reads : 0;
        for (auto* x : {&m_tw, &m_tw_wm, &m_tw_rm, &m_tw_busy, &m_tw_occ, &m_tsvc}) x->fin();   // WI-022 item 1
      }

      s_queue_len_avg = (float) s_queue_len / (float) m_clk;
      s_read_queue_len_avg = (float) s_read_queue_len / (float) m_clk;
      s_write_queue_len_avg = (float) s_write_queue_len / (float) m_clk;
      s_priority_queue_len_avg = (float) s_priority_queue_len / (float) m_clk;

      fini();

      return;
    }

};

}   // namespace Ramulator
