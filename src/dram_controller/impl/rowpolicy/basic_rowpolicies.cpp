#include <vector>

#include "base/base.h"
#include "dram_controller/controller.h"
#include "dram_controller/scheduler.h"
#include "dram_controller/rowpolicy.h"

namespace Ramulator {

class OpenRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, OpenRowPolicy, "OpenRowPolicy", "Open Row Policy.")
  private:
    
  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override { };

    void update(bool request_found, ReqBuffer::iterator& req_it) override { 
      // OpenRowPolicy does not need to take any actions
    };


};

class ClosedRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, ClosedRowPolicy, "ClosedRowPolicy", "Close Row Policy.")
  private:
    IDRAM* m_dram;
    
    int m_PRE_req_id = -1;
    
    int m_cap = -1;
    
    int m_rank_level = -1;
    int m_bankgroup_level = -1;
    int m_bank_level = -1;
    int m_row_level = -1;
    int m_num_ranks = -1;
    int m_num_bankgroups = -1;
    int m_num_banks = -1;

    int s_num_close_reqs = 0;

    // WI-005 (Q-02): close the row with the auto-precharge variant of the cap-th access instead of a priority PRE
    bool m_auto_precharge = false;
    int m_cmd_RD = -1, m_cmd_WR = -1, m_cmd_RDA = -1, m_cmd_WRA = -1;
    int s_num_close_autoprecharge = 0;  // cap-th access converted to RDA/WRA
    int s_num_close_fallback = 0;       // could not convert (RDA/WRA not ready, or another active request uses the bank)

    std::vector<uint64_t> m_col_accesses;

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;

      m_cap = param<int>("cap").default_val(10000000); // TODO

      m_rank_level = m_dram->m_levels("rank");
      m_bankgroup_level = m_dram->m_levels("bankgroup");
      m_bank_level = m_dram->m_levels("bank");
      m_row_level = m_dram->m_levels("row");

      m_PRE_req_id = m_dram->m_requests("close-row");

      m_num_ranks = m_dram->get_level_size("rank");
      m_num_bankgroups = m_dram->get_level_size("bankgroup");
      m_num_banks = m_dram->get_level_size("bank");
      
      m_col_accesses.resize(m_num_banks * m_num_bankgroups * m_num_ranks, 0);

      register_stat(s_num_close_reqs).name("num_close_reqs");

      m_auto_precharge = param<bool>("auto_precharge")
        .desc("Close the row with RDA/WRA on the cap-th access instead of a priority PRE.").default_val(false);
      if (m_auto_precharge) {
        for (auto c : {"RD", "WR", "RDA", "WRA"}) {
          if (!m_dram->m_commands.contains(c))
            throw ConfigurationError("ClosedRowPolicy auto_precharge requires DRAM command {}!", c);
        }
        m_cmd_RD = m_dram->m_commands("RD");   m_cmd_WR = m_dram->m_commands("WR");
        m_cmd_RDA = m_dram->m_commands("RDA"); m_cmd_WRA = m_dram->m_commands("WRA");
        register_stat(s_num_close_autoprecharge).name("num_close_autoprecharge");
        register_stat(s_num_close_fallback).name("num_close_fallback");
      }
    };

    void update(bool request_found, ReqBuffer::iterator& req_it) override {

      if (!request_found)
        return;

      if (m_dram->m_command_meta(req_it->command).is_closing ||
          m_dram->m_command_meta(req_it->command).is_refreshing)  // PRE or REF 
      {  

        if (req_it->addr_vec[m_bankgroup_level] == -1 && req_it->addr_vec[m_bank_level] == -1) {  // all bank closes
          for (int b = 0; b < m_num_banks; b++) {
            for (int bg = 0; bg < m_num_bankgroups; bg++) {
              int rank_id = req_it->addr_vec[m_rank_level];
              int flat_bank_id = b + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
              m_col_accesses[flat_bank_id] = 0;
            }
          }
        } else if (req_it->addr_vec[m_bankgroup_level] == -1) {  // same bank closes
          for (int bg = 0; bg < m_num_bankgroups; bg++) {
            int bank_id = req_it->addr_vec[m_bank_level];
            int rank_id = req_it->addr_vec[m_rank_level];
            int flat_bank_id = bank_id + bg * m_num_banks + rank_id * m_num_banks * m_num_bankgroups;
            m_col_accesses[flat_bank_id] = 0;
          }
        } else {  // single bank closes  (PRE, VRR, RDA, WRA)
          int flat_bank_id = req_it->addr_vec[m_bank_level] + 
                             req_it->addr_vec[m_bankgroup_level] * m_num_banks + 
                             req_it->addr_vec[m_rank_level] * m_num_banks * m_num_bankgroups;

          m_col_accesses[flat_bank_id] = 0;
        }
      } else if (m_dram->m_command_meta(req_it->command).is_accessing)  // RD or WR
      {
        int flat_bank_id = req_it->addr_vec[m_bank_level] + 
                           req_it->addr_vec[m_bankgroup_level] * m_num_banks + 
                           req_it->addr_vec[m_rank_level] * m_num_banks * m_num_bankgroups;
        
        m_col_accesses[flat_bank_id]++;

        if (m_col_accesses[flat_bank_id] >= m_cap && m_auto_precharge &&
            (req_it->command == m_cmd_RD || req_it->command == m_cmd_WR)) {
          int ap_cmd = (req_it->command == m_cmd_RD) ? m_cmd_RDA : m_cmd_WRA;
          if (m_dram->check_ready(ap_cmd, req_it->addr_vec) && !m_ctrl->other_active_request_in_bank(req_it)) {
            // issue this access as RDA/WRA: the bank closes with it, no priority PRE
            req_it->command = ap_cmd;
            req_it->final_command = ap_cmd;
            m_col_accesses[flat_bank_id] = 0;
            s_num_close_reqs++;
            s_num_close_autoprecharge++;
            return;
          }
          s_num_close_fallback++;
        }
        if (m_col_accesses[flat_bank_id] >= m_cap) {
          Request req(req_it->addr_vec, m_PRE_req_id);
          m_ctrl->priority_send(req);
          m_col_accesses[flat_bank_id] = 0;
          s_num_close_reqs++;
        }
      }
    };
};

// WI-017 (P4): gem5-style close-adaptive row policy. After each RD/WR, close the row with that access (RDA/WRA) if no other
// queued request targets the same row (scope: read+write+active buffers, or gem5 = the served request's type only), or at the
// cap-th access since the ACT even with hits pending. If RDA/WRA is not legal (e.g. WR->RDA to the bank) or another active-buffer
// request still uses the bank, the access is issued as is and the bank is marked close-pending: on every later tick a PRE is
// priority-sent only when it is legal and the row still has no pending request in scope (never a blocking, not-yet-ready priority
// PRE; WI-017 ruling 3 amended, review K1). Limitations: the decision is taken at schedule time; requests still in flight from
// the LLC are invisible (as in gem5).
class CloseAdaptiveRowPolicy : public IRowPolicy, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IRowPolicy, CloseAdaptiveRowPolicy, "CloseAdaptiveRowPolicy",
                                    "Close-adaptive row policy (gem5 close_adaptive + cap).")
  private:
    IDRAM* m_dram;
    int m_cap = -1;
    bool m_gem5_scope = false;
    int m_rank_level = -1, m_bankgroup_level = -1, m_bank_level = -1;
    int m_num_ranks = -1, m_num_bankgroups = -1, m_num_banks = -1;
    int m_PRE_req_id = -1;
    int m_cmd_RD = -1, m_cmd_WR = -1, m_cmd_RDA = -1, m_cmd_WRA = -1;
    uint64_t m_tick = 0;

    struct BankState {
      uint64_t accesses = 0;          // RD/WR(/RDA/WRA) since the last ACT (reset at the actual close, review K4)
      bool close_pending = false;     // fallback: close with a PRE as soon as it is legal and no request in scope is pending
      bool pending_is_write = false;  // type of the access that requested the close (gem5 scope)
      bool pending_cap = false;       // the close was requested by the cap (for the stats)
      AddrVec_t addr;                 // address of that access (bank + row)
      bool pre_sent = false;          // the fallback PRE is in the priority buffer
      uint64_t pre_sent_tick = 0;
    };
    std::vector<BankState> m_banks;

    uint64_t s_accesses = 0, s_acts = 0;
    uint64_t s_close_nohit_ap = 0, s_close_cap_ap = 0;     // closed with RDA/WRA
    uint64_t s_close_nohit_pre = 0, s_close_cap_pre = 0;   // closed with a fallback PRE
    uint64_t s_fallback_marked = 0, s_fallback_cancelled = 0;
    uint64_t s_keep_open = 0, s_keep_open_wronly = 0;      // wronly: only WRITE-buffer requests kept a read's row open (review K3)
    uint64_t s_fallback_pre_wait_cycles = 0;               // sum over fallback PREs of (issue - enqueue - 1) ticks (review K1)
    uint64_t s_max_accesses_per_act = 0;
    uint64_t s_other_closes = 0;                           // closes by PRE/PREA/REF of other origin (conflicts, refresh)
    uint64_t s_over_cap_fallback = 0;   // accesses beyond the cap while the bank was close-pending (hit served before the PRE was legal)
    uint64_t s_over_cap_other = 0;      // accesses beyond the cap otherwise (must stay 0)
    uint64_t s_stale_pre = 0;           // our close-row PRE issued after the bank's mark was cleared (a REF/PREA/conflict PRE closed
                                        // it first): it closes a re-opened row or hits a closed bank (review, should be ~0)

    int flat_bank(const AddrVec_t& a) const {
      return a[m_bank_level] + a[m_bankgroup_level] * m_num_banks + a[m_rank_level] * m_num_banks * m_num_bankgroups;
    }

    // a request in scope (other than `self`) targets the row of `addr`; *wronly = only write-buffer requests while serving a read
    bool pending_hit(const AddrVec_t& addr, const Request* self, bool is_write, bool* wronly) const {
      int m = m_ctrl->pending_requests_to_row(addr, self);
      using C = IDRAMController;
      int scope = m_gem5_scope ? (is_write ? (C::PENDING_WRITE | C::PENDING_ACTIVE_WRITE) : (C::PENDING_READ | C::PENDING_ACTIVE_READ))
                               : (C::PENDING_READ | C::PENDING_WRITE | C::PENDING_ACTIVE_READ | C::PENDING_ACTIVE_WRITE);
      if (wronly) *wronly = !is_write && (m & scope) == C::PENDING_WRITE;
      return (m & scope) != 0;
    }

    void closed(BankState& b) {
      s_max_accesses_per_act = std::max<uint64_t>(s_max_accesses_per_act, b.accesses);
      b.accesses = 0;
    }

    void reset_bank(int fb, bool counted_close) {   // the bank's row closed by a command we did not convert
      BankState& b = m_banks[fb];
      if (b.close_pending && b.pre_sent) {          // our fallback PRE (or a PREA/REF that closed the bank first)
        s_fallback_pre_wait_cycles += m_tick - b.pre_sent_tick - 1;   // 0 = issued at the first tick after it was enqueued
        if (b.pending_cap) s_close_cap_pre++; else s_close_nohit_pre++;
      } else if (b.close_pending) {
        s_fallback_cancelled++;                     // closed by someone else before our PRE was legal
        if (counted_close) s_other_closes++;
      } else if (counted_close && b.accesses > 0) {
        s_other_closes++;
      }
      b.close_pending = false; b.pre_sent = false;
      closed(b);
    }

  public:
    void init() override { };

    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) override {
      m_ctrl = cast_parent<IDRAMController>();
      m_dram = m_ctrl->m_dram;
      m_cap = param<int>("cap").desc("Close at the cap-th access since the ACT, even with hits pending (N).").required();
      if (m_cap < 1) throw ConfigurationError("CloseAdaptiveRowPolicy: cap must be >= 1 (got {})", m_cap);
      std::string scope = param<std::string>("pending_scope")
        .desc("all: read+write+active buffers keep a row open; gem5: only the served request's type").default_val("all");
      if (scope != "all" && scope != "gem5") throw ConfigurationError("CloseAdaptiveRowPolicy: pending_scope must be all or gem5");
      m_gem5_scope = scope == "gem5";
      for (auto c : {"RD", "WR", "RDA", "WRA"})
        if (!m_dram->m_commands.contains(c)) throw ConfigurationError("CloseAdaptiveRowPolicy requires DRAM command {}!", c);
      m_cmd_RD = m_dram->m_commands("RD");   m_cmd_WR = m_dram->m_commands("WR");
      m_cmd_RDA = m_dram->m_commands("RDA"); m_cmd_WRA = m_dram->m_commands("WRA");
      m_rank_level = m_dram->m_levels("rank");
      m_bankgroup_level = m_dram->m_levels("bankgroup");
      m_bank_level = m_dram->m_levels("bank");
      m_num_ranks = m_dram->get_level_size("rank");
      m_num_bankgroups = m_dram->get_level_size("bankgroup");
      m_num_banks = m_dram->get_level_size("bank");
      m_PRE_req_id = m_dram->m_requests("close-row");
      m_banks.resize(m_num_banks * m_num_bankgroups * m_num_ranks);

      register_stat(s_accesses).name("ca_accesses");
      register_stat(s_acts).name("ca_acts");
      register_stat(s_close_nohit_ap).name("ca_close_nohit_ap");
      register_stat(s_close_cap_ap).name("ca_close_cap_ap");
      register_stat(s_close_nohit_pre).name("ca_close_nohit_pre");
      register_stat(s_close_cap_pre).name("ca_close_cap_pre");
      register_stat(s_fallback_marked).name("ca_fallback_marked");
      register_stat(s_fallback_cancelled).name("ca_fallback_cancelled");
      register_stat(s_keep_open).name("ca_keep_open");
      register_stat(s_keep_open_wronly).name("ca_keep_open_wronly");
      register_stat(s_fallback_pre_wait_cycles).name("ca_fallback_pre_wait_cycles");
      register_stat(s_max_accesses_per_act).name("ca_max_accesses_per_act");
      register_stat(s_other_closes).name("ca_other_closes");
      register_stat(s_over_cap_fallback).name("ca_over_cap_fallback");
      register_stat(s_over_cap_other).name("ca_over_cap_other");
      register_stat(s_stale_pre).name("ca_stale_pre");
    };

    void update(bool request_found, ReqBuffer::iterator& req_it) override {
      m_tick++;

      // A. the command issued this tick (it is issued right after this call)
      int issued_bank = -1;
      if (request_found) {
        const auto& meta = m_dram->m_command_meta(req_it->command);
        auto& a = req_it->addr_vec;
        if (meta.is_closing || meta.is_refreshing) {
          if (req_it->type_id == m_PRE_req_id && a[m_bankgroup_level] != -1 && a[m_bank_level] != -1) {   // our fallback PRE
            const BankState& b = m_banks[flat_bank(a)];
            if (!(b.close_pending && b.pre_sent)) s_stale_pre++;
          }
          if (a[m_bankgroup_level] == -1 && a[m_bank_level] == -1) {          // all-bank close (PREA/REFab)
            for (int fb = 0; fb < m_num_banks * m_num_bankgroups; fb++)
              reset_bank(fb + a[m_rank_level] * m_num_banks * m_num_bankgroups, true);
          } else if (a[m_bankgroup_level] == -1) {                            // same-bank close
            for (int bg = 0; bg < m_num_bankgroups; bg++)
              reset_bank(a[m_bank_level] + bg * m_num_banks + a[m_rank_level] * m_num_banks * m_num_bankgroups, true);
          } else {
            reset_bank(flat_bank(a), true);
          }
        } else if (meta.is_opening) {
          s_acts++;
          m_banks[flat_bank(a)].accesses = 0;
        } else if (meta.is_accessing) {
          int fb = flat_bank(a);
          BankState& b = m_banks[fb];
          issued_bank = fb;
          if ((int)b.accesses >= m_cap) { if (b.close_pending) s_over_cap_fallback++; else s_over_cap_other++; }
          b.close_pending = false; b.pre_sent = false;   // a new access to the row decides again
          s_accesses++;
          b.accesses++;
          bool is_write = req_it->command == m_cmd_WR;
          bool wronly = false;
          bool hit = pending_hit(a, &*req_it, is_write, &wronly);
          bool cap = (int)b.accesses >= m_cap;
          if (hit && !cap) {
            s_keep_open++;
            if (wronly) s_keep_open_wronly++;
          } else {
            int ap = is_write ? m_cmd_WRA : m_cmd_RDA;
            if ((req_it->command == m_cmd_RD || req_it->command == m_cmd_WR) &&
                m_dram->check_ready(ap, a) && !m_ctrl->other_active_request_in_bank(req_it)) {
              req_it->command = ap;
              req_it->final_command = ap;
              if (cap) s_close_cap_ap++; else s_close_nohit_ap++;
              closed(b);
            } else {                                  // fallback: close with a PRE once it is legal (part B)
              b.close_pending = true; b.pending_is_write = is_write; b.pending_cap = cap; b.addr = a;
              s_fallback_marked++;
            }
          }
        }
      }

      // B. close-pending banks (every tick, also without a request): send the PRE only when it is legal now
      for (int fb = 0; fb < (int)m_banks.size(); fb++) {
        BankState& b = m_banks[fb];
        if (!b.close_pending || b.pre_sent || fb == issued_bank) continue;
        if (!m_dram->check_rowbuffer_hit(m_cmd_RD, b.addr)) {   // closed meanwhile (conflict PRE / refresh)
          b.close_pending = false; s_fallback_cancelled++; continue;
        }
        if (!b.pending_cap && pending_hit(b.addr, nullptr, b.pending_is_write, nullptr)) continue;   // a hit arrived: keep open
        // the scheduler blocks a closing command while an active-buffer request uses the bank (whatever the scope): wait for it,
        // so the PRE never sits not-ready at the head of the priority buffer
        if (m_ctrl->pending_requests_to_row(b.addr, nullptr) &
            (IDRAMController::PENDING_ACTIVE_READ | IDRAMController::PENDING_ACTIVE_WRITE)) continue;
        if (m_dram->check_ready(m_dram->m_request_translations(m_PRE_req_id), b.addr)) {
          Request pre(b.addr, m_PRE_req_id);
          if (m_ctrl->priority_send(pre)) { b.pre_sent = true; b.pre_sent_tick = m_tick; }
        }
      }
    };
};

}       // namespace Ramulator
