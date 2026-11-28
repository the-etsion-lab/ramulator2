#ifndef RAMULATOR_CONTROLLER_CONTROLLER_H
#define RAMULATOR_CONTROLLER_CONTROLLER_H

#include <vector>
#include <deque>

#include <spdlog/spdlog.h>
#include <yaml-cpp/yaml.h>

#include "base/base.h"
#include "dram/dram.h"
#include "dram_controller/scheduler.h"
#include "dram_controller/plugin.h"
#include "dram_controller/refresh.h"
#include "dram_controller/rowpolicy.h"


namespace Ramulator {

class IDRAMController : public Clocked<IDRAMController> {
  RAMULATOR_REGISTER_INTERFACE(IDRAMController, "Controller", "Memory Controller Interface");

  public:
    IDRAM*  m_dram = nullptr;          
    IScheduler*   m_scheduler = nullptr;
    IRefreshManager*   m_refresh = nullptr;
    IRowPolicy*   m_rowpolicy = nullptr;
    std::vector<IControllerPlugin*> m_plugins;

    int m_channel_id = -1;
  public:
    /**
     * @brief       Send a request to the memory controller.
     * 
     * @param    req        The request to be enqueued.
     * @return   true       Successful.
     * @return   false      Failed (e.g., buffer full).
     */
    virtual bool send(Request& req) = 0;

    /**
     * @brief       Send a high-priority request to the memory controller.
     * 
     */
    virtual bool priority_send(Request& req) = 0;

    /**
     * @brief       WI-005: true if a request in the active buffer other than *req_it targets the same bank as *req_it,
     *              i.e. closing that bank now would close a row another request has opened and still needs.
     */
    virtual bool other_active_request_in_bank(const ReqBuffer::iterator& req_it) { return false; };

    /**
     * @brief       WI-017: which buffers hold a request (other than *exclude) to the same channel/rank/bankgroup/bank/ROW as
     *              addr_vec. Bit mask of PENDING_* (0 = none). Default 0: only GenericDRAMController implements it.
     */
    static constexpr int PENDING_READ = 1, PENDING_WRITE = 2, PENDING_ACTIVE_READ = 4, PENDING_ACTIVE_WRITE = 8;
    virtual int pending_requests_to_row(const AddrVec_t& addr_vec, const Request* exclude) { return 0; };

    /**
     * @brief       WI-030: true if a request to `rank` is queued in the read, write, priority or active buffer (reads already issued
     *              and waiting for data do not count). Default false: only GenericDRAMController implements it.
     */
    virtual bool rank_has_requests(int rank) { return false; };
    /**
     * @brief       WI-030: true if a REFab of `rank` is queued (sent, not yet issued) or in progress (issued < nRFC1 ago).
     */
    virtual bool rank_refresh_busy(int rank) { return false; };

    /**
     * @brief       Ticks the memory controller.
     * 
     */
    virtual void tick() = 0;
   
};

}       // namespace Ramulator

#endif  // RAMULATOR_CONTROLLER_CONTROLLER_H