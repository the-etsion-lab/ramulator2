#include <vector>
#include <deque>
#include <map>

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

    // WI-030 (D-50) `postpone: true` (requires stagger: rank): rank r accrues one REFab obligation at each `rank` phase; an outstanding
    // obligation is sent as soon as the rank has no queued request and no rank of the channel (incl. itself) has a REFab queued or in
    // progress (T = 0); when 4 are outstanding and a 5th accrues, one is forced at once (stock priority path). No pull-in.
    static constexpr int kMaxOutstanding = 4;
    bool m_postpone = false;
    struct Sent { Clk_t accrual, send; bool forced; };
    std::vector<int> m_out;                          // outstanding obligations per rank
    std::vector<std::deque<Clk_t>> m_accruals;       // accrual clocks of the outstanding obligations
    std::vector<std::deque<Sent>> m_sent;            // sent, not yet issued
    std::vector<uint64_t> s_accrued, s_sent_n, s_issued, s_forced, s_opp, s_zero_req_issue, s_zero_req_send;   // whole run
    std::vector<std::vector<uint64_t>> s_out_ticks;  // post-warmup ticks with k outstanding (k = 0..4) per rank
    uint64_t s_delay_send_sum = 0, s_delay_issue_sum = 0, s_delay_n = 0;   // post-warmup, per channel
    float s_delay_issue_p90 = 0, s_delay_issue_max = 0, s_delay_send_p90 = 0, s_delay_send_max = 0;
    std::map<Clk_t, uint64_t> m_hist_issue, m_hist_send;

    void send_refab(int r, bool forced) {   // postpone mode: send the oldest outstanding obligation of rank r
      bool zero = !m_ctrl->rank_has_requests(r);   // before the REFab itself is enqueued
      std::vector<int> addr_vec(m_dram_org_levels, -1);
      addr_vec[0] = m_ctrl->m_channel_id;
      addr_vec[1] = r;
      Request req(addr_vec, m_ref_req_id);
      if (!m_ctrl->priority_send(req)) throw std::runtime_error("Failed to send refresh!");
      s_refab_sent[r]++; s_zero_req_send[r] += zero;
      m_sent[r].push_back({m_accruals[r].front(), m_clk, forced}); m_accruals[r].pop_front();
      m_out[r]--; s_sent_n[r]++; (forced ? s_forced : s_opp)[r]++;
      if (m_out[r] < 0) throw std::runtime_error("WI-030 invariant: outstanding obligations < 0");
    }

  public:
    void init() override { 
      m_ctrl = cast_parent<IDRAMController>();
      std::string s = param<std::string>("stagger").desc("WI-026: none (stock, all ranks at once) | rank | dimm (WI-028)").default_val("none");
      if (s != "none" && s != "rank" && s != "dimm") throw ConfigurationError("AllBank stagger must be none, rank or dimm (got {})", s);
      m_stagger = s != "none"; m_dimm = s == "dimm";
      // WI-028: `enabled: false` issues no REFab at all (the no-refresh row of the bracket; no other refresh manager exists)
      m_enabled = param<bool>("enabled").desc("WI-028: false = issue no refresh at all (diagnostic bracket)").default_val(true);
      m_postpone = param<bool>("postpone").desc("WI-030 (D-50): opportunistic postponement, forced at 4 outstanding (needs stagger: rank)").default_val(false);
      if (m_postpone && s != "rank") throw ConfigurationError("AllBank postpone: true requires stagger: rank (got {})", s);
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
        if (m_postpone) {   // WI-030
          int R = m_num_ranks;
          m_out.assign(R, 0); m_accruals.resize(R); m_sent.resize(R);
          for (auto* v : {&s_accrued, &s_sent_n, &s_issued, &s_forced, &s_opp, &s_zero_req_issue, &s_zero_req_send}) v->assign(R, 0);
          s_out_ticks.assign(R, std::vector<uint64_t>(kMaxOutstanding + 1, 0));
          for (int r = 0; r < R; r++) {
            register_stat(s_accrued[r], false).name("postpone_accrued_ch{}_rank{}", c, r);
            register_stat(s_sent_n[r], false).name("postpone_sent_ch{}_rank{}", c, r);
            register_stat(s_issued[r], false).name("postpone_issued_ch{}_rank{}", c, r);
            register_stat(s_opp[r], false).name("postpone_refab_opportunistic_ch{}_rank{}", c, r);
            register_stat(s_forced[r], false).name("postpone_refab_forced_ch{}_rank{}", c, r);
            register_stat(s_zero_req_send[r], false).name("postpone_refab_zero_requests_at_send_ch{}_rank{}", c, r);
            register_stat(s_zero_req_issue[r], false).name("postpone_refab_zero_requests_at_issue_ch{}_rank{}", c, r);
            for (int k = 0; k <= kMaxOutstanding; k++)
              register_stat(s_out_ticks[r][k]).name("postpone_outstanding_{}_ticks_ch{}_rank{}", k, c, r);
          }
          register_stat(s_delay_n).name("postpone_refab_issued_post_warmup_ch{}", c);
          register_stat(s_delay_send_sum).name("postpone_delay_accrual_to_send_sum_ch{}", c);
          register_stat(s_delay_issue_sum).name("postpone_delay_accrual_to_issue_sum_ch{}", c);
          register_stat(s_delay_send_p90).name("postpone_delay_accrual_to_send_p90_ch{}", c);
          register_stat(s_delay_send_max).name("postpone_delay_accrual_to_send_max_ch{}", c);
          register_stat(s_delay_issue_p90).name("postpone_delay_accrual_to_issue_p90_ch{}", c);
          register_stat(s_delay_issue_max).name("postpone_delay_accrual_to_issue_max_ch{}", c);
        }
      }
    };

    void refab_issued(int rank, Clk_t clk, bool no_requests) override {   // WI-030
      if (!m_postpone) return;
      if (m_sent[rank].empty()) throw std::runtime_error("WI-030 invariant: REFab issued that the manager did not send");
      Sent s = m_sent[rank].front(); m_sent[rank].pop_front();
      s_issued[rank]++; s_zero_req_issue[rank] += no_requests;
      Clk_t ds = s.send - s.accrual, di = clk - s.accrual;
      s_delay_n++; s_delay_send_sum += ds; s_delay_issue_sum += di; m_hist_send[ds]++; m_hist_issue[di]++;
    }

    void on_reset_stats() override { m_hist_send.clear(); m_hist_issue.clear(); }   // WI-030 (sums are registered)

    void finalize() override {
      if (!m_postpone) return;
      for (int r = 0; r < m_num_ranks; r++) {   // invariants (whole run)
        if ((int64_t)s_sent_n[r] != (int64_t)s_accrued[r] - m_out[r])
          throw std::runtime_error("WI-030 invariant: sent != accrued - outstanding");
        if (s_issued[r] != s_sent_n[r] - m_sent[r].size()) throw std::runtime_error("WI-030 invariant: issued != sent - queued");
      }
      auto pct = [](const std::map<Clk_t, uint64_t>& h, float& p90, float& mx) {
        uint64_t n = 0; for (auto& [k, v] : h) n += v;
        uint64_t acc = 0; p90 = 0; mx = 0;
        for (auto& [k, v] : h) { acc += v; if (p90 == 0 && acc * 10 >= n * 9) p90 = k; mx = k; }
      };
      pct(m_hist_send, s_delay_send_p90, s_delay_send_max); pct(m_hist_issue, s_delay_issue_p90, s_delay_issue_max);
    }

    void tick() {
      m_clk++;
      if (!m_enabled) return;   // WI-028: no refresh

      if (m_postpone) {   // WI-030 (D-50)
        s_clk = m_clk;
        for (int r = 0; r < m_num_ranks; r++) {
          if (m_clk != m_next_rank[r]) continue;   // accrual at the `rank` phase
          m_next_rank[r] += m_nrefi;
          if (m_out[r] == kMaxOutstanding) send_refab(r, true);   // forcing: a 5th would accrue
          m_out[r]++; m_accruals[r].push_back(m_clk); s_accrued[r]++;
          if (m_out[r] > kMaxOutstanding) throw std::runtime_error("WI-030 invariant: outstanding obligations > 4");
        }
        bool busy = false;   // one refreshing rank per channel (unless forced)
        for (int r = 0; r < m_num_ranks && !busy; r++) busy = m_ctrl->rank_refresh_busy(r);
        if (!busy) {
          for (int r = 0; r < m_num_ranks; r++) {   // opportunistic (T = 0): at most one REFab per channel per tick
            if (m_out[r] > 0 && !m_ctrl->rank_has_requests(r)) { send_refab(r, false); break; }
          }
        }
        for (int r = 0; r < m_num_ranks; r++) s_out_ticks[r][m_out[r]]++;
        return;
      }

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
