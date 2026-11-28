#ifndef     RAMULATOR_CONTROLLER_REFRESH_H
#define     RAMULATOR_CONTROLLER_REFRESH_H

#include <vector>
#include <string>

#include "base/base.h"


namespace Ramulator {

class IRefreshManager {
  RAMULATOR_REGISTER_INTERFACE(IRefreshManager, "RefreshManager", "Refresh Manager Interface.");

  public:
    virtual void tick() = 0;
    /**
     * @brief   WI-030: the controller reports every issued REFab (rank, issue clock, whether no request to that rank was queued in
     *          the read / write / priority / active buffers at issue). Default: ignored.
     */
    virtual void refab_issued(int rank, Clk_t clk, bool no_requests) {};
};

}        // namespace Ramulator


#endif   // RAMULATOR_CONTROLLER_REFRESH_H