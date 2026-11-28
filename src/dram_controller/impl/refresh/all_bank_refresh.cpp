#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/refresh.h"

namespace Ramulator {

class AllBankRefresh : public IRefreshManager, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRefreshManager, AllBankRefresh, "AllBank", "All-Bank Refresh scheme.")
  private:
    Clk_t m_clk = 0;
    IDRAM* m_dram;
    IDRAMController* m_ctrl;

    int m_dram_org_levels = -1;
    int m_num_ranks = -1;

    int m_nrefi = -1;
    int m_ref_req_id = -1;
    Clk_t m_next_refresh_cycle = -1;

    // WI-026: `stagger: rank` gives rank r of channel c its first REFab at nREFI * (1 + (r*C + c) / (C*R)) (integer floor), then
    // every nREFI, so the C*R ranks refresh evenly spread over the interval. Rank-major order (review D1): the ranks of one channel are
    // nREFI/R apart, so (with nRFC1 < nREFI/R) they are never in refresh at the same time. `none` (default) = stock lockstep behaviour.
    bool m_stagger = false;
    bool m_dimm = false;      // WI-028: same phases in every channel, the R ranks of a channel nREFI/R apart
    bool m_enabled = true;    // WI-028
    std::vector<Clk_t> m_next_rank;
    std::vector<uint64_t> s_refab_sent;     // per rank, whole run (not reset at warmup)
    std::vector<uint64_t> s_first_refresh;  // per rank
    uint64_t s_clk = 0;                     // controller cycles of the whole run (the stock count is floor(clk / nREFI))

  public:
    void init() override { 
      m_ctrl = cast_parent<IDRAMController>();
      std::string s = param<std::string>("stagger").desc("WI-026: none (stock, all ranks at once) | rank | dimm (WI-028)").default_val("none");
      if (s != "none" && s != "rank" && s != "dimm") throw ConfigurationError("AllBank stagger must be none, rank or dimm (got {})", s);
      m_stagger = s != "none"; m_dimm = s == "dimm";
      // WI-028: `enabled: false` issues no REFab at all (the no-refresh row of the bracket; no other refresh manager exists)
      m_enabled = param<bool>("enabled").desc("WI-028: false = issue no refresh at all (diagnostic bracket)").default_val(true);
    };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_dram = m_ctrl->m_dram;

      m_dram_org_levels = m_dram->m_levels.size();
      m_num_ranks = m_dram->get_level_size("rank");

      m_nrefi = m_dram->m_timing_vals("nREFI");
      m_ref_req_id = m_dram->m_requests("all-bank-refresh");

      m_next_refresh_cycle = m_nrefi;
      if (m_stagger) {   // WI-026
        int C = m_dram->get_level_size("channel"), c = m_ctrl->m_channel_id;
        register_stat(s_clk, false).name("refresh_clk_ch{}", c);
        m_next_rank.resize(m_num_ranks); s_refab_sent.assign(m_num_ranks, 0); s_first_refresh.assign(m_num_ranks, 0);
        for (int r = 0; r < m_num_ranks; r++) {
          m_next_rank[r] = m_dimm ? m_nrefi + (Clk_t)((int64_t)m_nrefi * r / m_num_ranks)   // WI-028 dimm: channel-independent
                                  : m_nrefi + (Clk_t)((int64_t)m_nrefi * (r * C + c) / ((int64_t)C * m_num_ranks));   // review D1: rank-major
          s_first_refresh[r] = m_next_rank[r];
          register_stat(s_refab_sent[r], false).name("refab_sent_ch{}_rank{}", c, r);
          register_stat(s_first_refresh[r], false).name("first_refresh_cycle_ch{}_rank{}", c, r);
        }
      }
    };

    void tick() {
      m_clk++;
      if (!m_enabled) return;   // WI-028: no refresh

      if (m_stagger) {   // WI-026: per-rank schedule
        s_clk = m_clk;
        for (int r = 0; r < m_num_ranks; r++) {
          if (m_clk != m_next_rank[r]) continue;
          m_next_rank[r] += m_nrefi;
          std::vector<int> addr_vec(m_dram_org_levels, -1);
          addr_vec[0] = m_ctrl->m_channel_id;
          addr_vec[1] = r;
          Request req(addr_vec, m_ref_req_id);
          if (!m_ctrl->priority_send(req)) throw std::runtime_error("Failed to send refresh!");
          s_refab_sent[r]++;
        }
        return;
      }

      if (m_clk == m_next_refresh_cycle) {
        m_next_refresh_cycle += m_nrefi;
        for (int r = 0; r < m_num_ranks; r++) {
          std::vector<int> addr_vec(m_dram_org_levels, -1);
          addr_vec[0] = m_ctrl->m_channel_id;
          addr_vec[1] = r;
          Request req(addr_vec, m_ref_req_id);

          bool is_success = m_ctrl->priority_send(req);
          if (!is_success) {
            throw std::runtime_error("Failed to send refresh!");
          }
        }
      }
    };

};

}       // namespace Ramulator
