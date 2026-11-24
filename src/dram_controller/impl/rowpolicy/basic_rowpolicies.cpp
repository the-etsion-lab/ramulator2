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

}       // namespace Ramulator
