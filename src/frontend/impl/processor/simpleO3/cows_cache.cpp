//#define DEBUG_COWS

#ifdef NDEBUG
#undef NDEBUG
#endif

#include <iostream>
#include <fstream>
#include <sstream>
#include <iomanip>

#include "cows_cache.h"

#define PRINT_MISS_OVER_LLC 0

namespace Ramulator {

// initialize variables to illegal values, so we can flush out bugs
uint64_t CoWsCache::AddrParser::s_dram_page_bytes;
uint64_t CoWsCache::AddrParser::s_bytes_per_line;
uint32_t CoWsCache::AddrParser::s_nsets;
uint64_t CoWsCache::Line::bytes_per_line = 0;

static std::string addr_to_binary(Addr_t addr) {
    std::string result;
    int bits = sizeof(addr) * 8;
    for(int i = bits - 1; i >= 0; i--) {
        if(i!=0 && (i%8)==0)
            result += ' ';
        result += ((addr >> i) & 1) ? '1' : '0';
    }
    return result;
}

std::string CoWsCache::AddrParser::toString() const
{
    std::ostringstream os;

    os<<"AddrParser:\t"<<"dram_page_bytes="<<s_dram_page_bytes<<", page_bits="<<getDramPageBits()<<", page_mask="<<std::hex<<getDramPageAddrMask()<<std::dec<<std::endl;
    os<<"AddrParser:\t"<<"nsets="<<s_nsets<<", set_bit="<<getSetBits()<<", set_mask="<<std::hex<<getSetMask()<<std::dec<<std::endl;
    os<<"AddrParser:\t"<<"s_bytes_per_line="<<s_bytes_per_line<<std::endl;;
    os<<"AddrParser:\t\t"<<"addr     ="<<std::hex<<m_addr<<std::dec<<std::endl;
    os<<"AddrParser:\t\t"<<"page_num ="<<std::hex<<getPageNum()<<std::dec<<std::endl;
    os<<"AddrParser:\t\t"<<"tag      ="<<std::hex<<getTag()<<std::dec<<std::endl;
    os<<"AddrParser:\t\t"<<"set_idx  ="<<std::hex<<getSetID()<<std::dec<<std::endl;
    Addr_t page_num_bits = getPageNum() << getDramPageBits();
    Addr_t set_bits = getSetID() << getDramPageBits();
    Addr_t tag_bits = getTag() << (getDramPageBits() + getSetBits());
    os<<"AddrParser:\t\t"<<"addr_bits   ="<<addr_to_binary(m_addr)<<std::endl;
    os<<"AddrParser:\t\t"<<"pagenum_bits="<<addr_to_binary(page_num_bits)<<std::endl;
    os<<"AddrParser:\t\t"<<"tag_bits    ="<<addr_to_binary(tag_bits)<<std::endl;
    os<<"AddrParser:\t\t"<<"set_bits    ="<<addr_to_binary(set_bits)<<std::endl;

    return os.str();
}

std::string
CoWsCache::Line::toString() const
{
    std::ostringstream oss;

    uint64_t lo=0;
    uint64_t hi=0;

    for(uint64_t i=0; i<64; i++)
        lo |= (block_map[i]) ? 0x1 : 0x0;

    for(uint64_t i=64; i<127; i++)
        hi |= (block_map[i]) ? 0x1 : 0x0;

    oss<<"page_phys_addr=0x"<<std::hex<<page_phys_addr
        <<", "<<"valid="<<valid
        <<", map=0x"
        << std::setfill('0') << std::setw(8) << std::right << std::hex << hi << "_"
        << std::setfill('0') << std::setw(8) << std::right << std::hex << lo;

    return oss.str();
}

std::tuple<bool,bool,bool> CoWsCache::LRU_nohit::llc_hit(CacheSet_t& set, Line* page, uint32_t block_idx, bool is_write) const
{
    // on writes we access the mapping in the cows cache to update the block map, so we need it.

    // miss? get the page's cows entry from memory
    bool miss = false;
    bool miss_not_handled = false;
    bool need_evict = false;
    auto tag = page->getTag();
    auto it = find_in_set(set, tag);

    if(it != set.end()) {
        // HIT. move the accessed line to MRU (tail of list)
        move_to_mru(set, it);
    }
    else {
        // MISS. we access the mapping the cows cache (and insert it if it's not there)
        miss = true;
        std::tie(it, need_evict) = alloc_line(set, page);
        if(it == set.end()) {
            // no victim available (e.g., all lines are pending). Cannot handle the miss.
            miss_not_handled = true;
        }
    }

    // a write and block bit not set? set it and mark the entry dirty
    if(!miss_not_handled && is_write && !page->getBlockID(block_idx)) {
        page->setBlockID(block_idx, true);
        page->setDirty(true);
    }

    return {miss, need_evict, miss_not_handled};
}

std::tuple<bool,bool,bool> CoWsCache::LRU_nohit::llc_miss(CacheSet_t& set, Line* page, uint32_t block_idx, bool is_write) const
{
    auto tag = page->getTag();
    bool miss = false;
    bool miss_not_handled = false;
    bool need_evict = false;
    // on a miss we access the mapping the cows cache (and insert it if it's not there)
    auto it = find_in_set(set, tag);
    if(it != set.end()) {
        // HIT. move the accessed line to MRU (tail of list)
        move_to_mru(set, it);
    }
    else {
        miss = true;

        std::tie(it, need_evict) = alloc_line(set, page);
        if(it == set.end()) {
            // no victim available (e.g., all lines are pending). Cannot handle the miss.
            miss_not_handled = true;
        }
    }

    // we fetched the page rename data into the cache
    // on a write miss we need to make sure the the block is now renamed and set it dirty
    // on a read miss, no bits need updating.
    if(!miss_not_handled && is_write && !page->getBlockID(block_idx)) {
        page->setBlockID(block_idx, true);
        page->setDirty(true);
    }

    return {miss, need_evict, miss_not_handled};
}

std::tuple<bool,bool,bool> CoWsCache::LRU_nohit::llc_evict(CacheSet_t& set, Line* page, uint32_t block_idx, bool is_write) const
{
    auto tag = page->getTag();
    bool miss = false;
    bool miss_not_handled = false;
    bool need_evict = false;
    auto it = find_in_set(set, tag);
    if(it != set.end()) {
        // HIT. move the accessed line to MRU (tail of list)
        move_to_mru(set, it);
    }
    else {
        // MISS. we access the mapping the cows cache (and insert it if it's not there)
        miss = true;
        std::tie(it, need_evict) = alloc_line(set, page);
        if(it == set.end()) {
            // no victim available (e.g., all lines are pending). Cannot handle the miss.
            miss_not_handled = true;
        }
    }

    // WI-004 D2: a dirty LLC writeback is the moment the block reaches the (new) page in DRAM: set its bit.
    // Done even if the entry could not be installed (C1): the entry is fetched and updated in the table anyway.
    if(is_write && !page->getBlockID(block_idx)) {
        page->setBlockID(block_idx, true);
        page->setDirty(true);
        m_cows_cache->s_bits_set++;
    }

    return {miss, need_evict, miss_not_handled};
}

CoWsCache::CoWsCache(uint32_t nlines,
                     uint32_t assoc,
                     uint32_t cache_line_bytes,
                     uint32_t dram_page_bytes,
                     bool always_miss,
                     uint32_t access_latency,
                     uint32_t dram_latency_on_translation,
                     CoWsCache::ReplPolicy* policy,
                     const CoWsStats& stats)
              : m_nlines(nlines),
                m_nsets(nlines/assoc),
                m_assoc(assoc),
                m_cache_line_bytes(cache_line_bytes),
                m_dram_page_bytes(dram_page_bytes),
                m_always_miss(always_miss),
                m_access_latency(access_latency),
                m_stats(stats),
                m_dram_latency_on_translation(dram_latency_on_translation),
                m_dram_page_bit_offset(CoWsCache::AddrParser::getDramPageBits()),
                m_dram_page_mask(CoWsCache::AddrParser::getDramPageAddrMask()),
                m_lines_in_llc(0),
                m_stats_cows2llc_valid(),
                m_stats_cows_miss_clk(),
                m_stats_set_access(m_nsets),
                m_stats_set_miss(m_nsets),
                m_perfect_cache(),
                m_cache_sets(m_nsets, CacheSet_t()),
                m_policy(policy)
{
    assert(is_power_of_2(nlines));
    assert(is_power_of_2(assoc));
    assert(is_power_of_2(m_cache_line_bytes));
    assert(is_power_of_2(m_dram_page_bytes));
    assert(is_power_of_2(m_nsets));

    // set global cache line parameters
    CoWsCache::Line::bytes_per_line = cache_line_bytes;
    CoWsCache::AddrParser::s_dram_page_bytes = dram_page_bytes;
    CoWsCache::AddrParser::s_bytes_per_line = cache_line_bytes;
    CoWsCache::AddrParser::s_nsets = m_nsets;

    m_policy->setCowsCache(this);

    std::cerr<<"# COWs cache: dram: "
             <<", dram page bytes="<<dram_page_bytes
             <<", dram_page_offset_bits="<<CoWsCache::AddrParser::getDramPageBits()
             <<", dram_page_offset_mask=0x"<<std::hex<<CoWsCache::AddrParser::getDramPageOffsetMask()<<std::dec
             <<", dram_page_addr_mask=0x"<<std::hex<<CoWsCache::AddrParser::getDramPageAddrMask()<<std::dec
             <<std::endl;

    std::cerr<<"# COWs cache: dimensions: "
             <<"cache line bytes="<<cache_line_bytes
            <<", nlines="<<m_nlines
            <<", sets="<<m_nsets
            <<", m_assoc="<<m_assoc
            <<", tag bits="<<Line::getTagBits()
            <<", data bits="<<CoWsCache::AddrParser::getNumBlocksPerPage()
             <<std::endl;

    auto cows_cache_tag_bytes = (m_nsets * m_assoc * Line::getTagBits())/8;
    auto cows_cache_data_bytes = (m_nsets * m_assoc * CoWsCache::AddrParser::getNumBlocksPerPage())/8; // 1 bit per block
    std::cerr<<"# COWs cache: bytes="
            <<"tag store="<<cows_cache_tag_bytes
            <<", data store="<<cows_cache_data_bytes
            <<", total="<<(cows_cache_tag_bytes + cows_cache_data_bytes)
            <<std::endl;
}

void CoWsCache::set_source_index_bits(uint32_t src_index_bits) {
    s_src_index_bits = src_index_bits;
    s_entry_bits = Line::getTagBits() + src_index_bits + 1 + AddrParser::getNumBlocksPerPage();
    s_cache_bytes = ((uint64_t)m_nsets * m_assoc * s_entry_bits + 7) / 8;
    if (m_table_line_fetch) {   // WI-015 T1 (PI): tagless table entries, cows_table_entries_per_line per 64B table line
        std::cerr << "# COWs table-line fetch: " << m_entries_per_line << " entries per 64B table line (cache entry " << s_entry_bits
                  << " b incl. tag; table entry tagless)" << std::endl;
    }
    std::cerr<<"# COWs cache: entry bits="<<s_entry_bits
             <<" (tag+valid+dirty="<<Line::getTagBits()<<", src row index="<<src_index_bits<<", src valid=1"
             <<", bitmap="<<AddrParser::getNumBlocksPerPage()<<"), entries="<<(uint64_t)m_nsets * m_assoc
             <<", bytes="<<s_cache_bytes<<std::endl;
}

void CoWsCache::maybe_redirect(Request& req, bool is_fill) {
    AddrParser ap(req.addr);
    auto it = m_perfect_cache.find(ap.getPageAddr());
    if (it == m_perfect_cache.end()) {
        return;   // no entry (cannot happen on the LLC-miss path: the lookup creates it)
    }
    if (it->second.getBlockID(AddrParser::getBlockIdx(req.addr))) {
        s_written_block_misses++;
        return;   // written block: it lives in the page's own row
    }
    if (m_renamed_fraction < 1.0 &&
        (double)(mix64((uint64_t)ap.getPageNum() ^ 0xF4AC7105ull) >> 11) * 0x1.0p-53 >= m_renamed_fraction) {
        s_not_renamed_unwritten_misses++;
        return;   // page not renamed: its data lives in its own row
    }
    s_unwritten_block_misses++;
    if (m_placement == Placement::SELF) {
        return;   // today's model: the data is read from the own row
    }
    // Pool index by first-touch order (pool = #pages -> every page has its own source; pool = #pages/4 -> 4 pages per source),
    // source row = a fixed permutation of the pool index over all rows (random) or over the bank's rows (same_bank).
    uint64_t pool_idx = it->second.touch_order % m_source_pool;
    Addr_t src_page;
    uint64_t row_id;
    if (m_placement == Placement::RANDOM) {
        row_id = permute(pool_idx, std::countr_zero(m_total_rows));
        src_page = (Addr_t)(row_id << AddrParser::getDramPageBits());   // 4KB-aligned linear address = one row anywhere
    } else {   // SAME_BANK: keep channel/rank/bankgroup/bank, replace the row field
        // Sources must be distinct only within a bank: per-bank first-touch order modulo min(pool, rows per bank), so pool >= #pages
        // never shares (also when pool > rows per bank). (Pool-based sharing, e.g. footprint/4, is a random-placement sensitivity.)
        // The bank is identified by the address bits between the page and the row field (channel/rank/bankgroup/bank under the
        // row-on-top mappings the startup probe accepts).
        Line& pl = it->second;
        if (pl.bank_touch_order < 0) {
            uint64_t bank = (uint64_t)ap.getPageNum() & ((1ull << (m_row_shift - AddrParser::getDramPageBits())) - 1);
            pl.bank_touch_order = (int64_t)m_bank_touch[bank]++;
        }
        uint64_t per_bank_pool = std::min<uint64_t>(m_rows_per_bank, m_source_pool);
        uint64_t row = permute((uint64_t)pl.bank_touch_order % per_bank_pool, std::countr_zero(m_rows_per_bank));
        Addr_t row_mask = (Addr_t)(m_rows_per_bank - 1) << m_row_shift;
        src_page = (ap.getPageAddr() & ~row_mask) | (Addr_t)(row << m_row_shift);
        row_id = (uint64_t)src_page >> AddrParser::getDramPageBits();
    }
    if (m_used_source_rows.insert(row_id).second) {
        s_distinct_source_rows++;
    }
    req.orig_addr = req.addr;
    req.addr = src_page | (req.addr & AddrParser::getDramPageOffsetMask());
    s_redirected_reads++;
    if (is_fill) {
        s_redirected_fills++;
    }
}

void CoWsCache::fetch_line_siblings(Addr_t page_num, Clk_t ready_clk, bool on_evict) {
    // WI-015: the table read returned the whole 64B line = entries of the aligned pages [base, base + entries_per_line)
    // (base = page >> log2(epf) << log2(epf)); install the others in their own sets with the same ready time. Rename-independent.
    s_line_fetches++;
    Addr_t base = page_num & ~(Addr_t)(m_entries_per_line - 1);
    for(Addr_t q = base; q < base + (Addr_t)m_entries_per_line; q++) {
        if(q == page_num) continue;
        auto [pm, sib] = perfect_cache_lookup(q << AddrParser::getDramPageBits());
        auto& sset = m_cache_sets[(uint32_t)(q & AddrParser::getSetMask())];
        uint64_t writes_before = s_table_writes;
        int r = m_policy->install(sset, sib);
        s_line_sibling_victim_writes += s_table_writes - writes_before;   // T2: a dirty victim was written back to the table
        if(r == 0) {
            sib->setWhenReady(ready_clk);
            sib->setPending(true);
            m_pending_lines.push_back(sib);
            sib->prefetched = true;
            sib->prefetched_on_evict = on_evict;
            s_line_siblings_installed++;
        } else if(r == 1) {
            s_line_siblings_present++;
        } else {
            s_line_siblings_dropped++;
        }
    }
}

void CoWsCache::check_pending_lines(Clk_t clk) {
    for(auto it = m_pending_lines.begin(); it != m_pending_lines.end(); ) {
        auto* line = *it;
        if(line->checkIfReady(clk)) {
            // this line is ready now, we can remove it from the pending list
            (*it)->setPending(false);
            it = m_pending_lines.erase(it);
        }
        else {
            // this line is still not ready, keep it in the pending list
            ++it;
        }
    }
}
std::tuple<bool, CoWsCache::Line*> CoWsCache::perfect_cache_lookup(Addr_t page_addr)
{
    CoWsCache::Line* line = nullptr;

    bool miss = false;

    auto it = m_perfect_cache.find(page_addr);
    if(it == m_perfect_cache.end()) {
        // insert new line and set mapping
        Line& newline = m_perfect_cache[page_addr];
        assert(newline.getNumBlocks() == 0); // make sure it's a fresh line.

        newline.setPageAddr(page_addr);
        newline.setValid(true);
        newline.touch_order = m_next_touch++;   // WI-013
        s_tracked_pages++;

        line = &newline;
        miss = true;
    }
    else {
        line = &it->second;
        miss=false;
    }
    return {miss, line};
}

// this function is here as a placeholder for collecting statistics and calling the ReplPolicy hit method
std::tuple<bool, uint32_t, bool> CoWsCache::llc_hit(Addr_t baddr, bool is_write, Clk_t clk, int total_llc_misses)
{
    // on read hit we don't need to access the cows cache at all.
    if(!is_write) {
        return {true, 0, false};
    }

    AddrParser ap(baddr);
    auto page_addr = ap.getPageAddr();
    auto page_num = ap.getPageNum();
    auto set_idx = ap.getSetID();
    uint32_t block_idx = CoWsCache::AddrParser::getBlockIdx(baddr);

    // get page data from main db
    auto [perf_miss, page] = perfect_cache_lookup(page_addr);

    // miss latency is at least a cows cache access latency
    uint32_t latency = m_access_latency;

    //
    // access cows cache
    //
    // tell the replacement policy we have an llc write hit
    auto& set = m_cache_sets[set_idx];   // WI-004 D1: the real set, not a copy

    auto [cows_cache_miss, cows_cache_evicted, miss_not_handled] = m_policy->llc_hit(set, page, block_idx, is_write);
    if(miss_not_handled) {
        s_miss_not_handled++;
        s_miss_not_handled_on_llc_hit++;
        // the miss cannot be handled (e.g., all lines in the set are pending). For simplicity we treat it as a miss and retry later.
        return {true, 0, true};
    }
    s_cows_cycles_self += m_access_latency;   // WI-004 C2: handled lookups only
    if(!cows_cache_miss && page->prefetched) {   // WI-015: the sibling fetched with a table line saved a table read
        page->prefetched = false;
        s_prefetch_useful++;
        if(page->prefetched_on_evict) s_prefetch_useful_evictpath++;   // T5
    }
    if(m_always_miss) {
        cows_cache_miss = true;
    }

    if(PRINT_MISS_OVER_LLC && s_misses > total_llc_misses) {
        printf("llc_hit[clk=%lu]: baddr=0x%lx, is_write=%d (s_misses=%lu, llc_misses=%d)\t[perf/miss=%d/%d, s_access=%lu, on_llc_miss=%lu, on_llc_hit=%lu, on_llc_evict=%lu]\n",
                clk, baddr, (int)is_write, s_misses, total_llc_misses, perf_miss, cows_cache_miss, s_access, s_misses_on_llc_miss, s_misses_on_llc_hit, s_misses_on_llc_evict);
    }

    // gather cows stats
    ++s_access;
    ++s_accesses_on_llc_hit;
    m_stats_set_access[set_idx]++;

    // hit-under-miss?
    if(!cows_cache_miss && !page->checkIfReady(clk)) {   // WI-004 C2: ready iff clk >= ready_clk

        cows_cache_miss = true;
        assert(page->getWhenReady() > clk);
        latency = std::max<Clk_t>(m_access_latency, page->getWhenReady() - clk);   // WI-004 C2

        s_half_misses_on_llc_hit++;
        s_half_misses++;
    }
    // real miss?
    else if(cows_cache_miss) {
        ++s_misses;
        ++s_misses_on_llc_hit;
        m_stats_set_miss[set_idx]++;

        track_misses(clk, total_llc_misses);

        // miss latency is at least a cows cache access latency
        latency += get_dram_latency_on_translation(page_num);
        s_table_reads++;   // WI-004 D3

        page->setWhenReady(clk + latency);
        page->setPending(true);
        m_pending_lines.push_back(page);
        if(m_table_line_fetch) {   // WI-015
            fetch_line_siblings(page_num, clk + latency, false);
        }
    }
    else {
        ++s_hits;
    }

    s_cows_cycles += latency;

    return {cows_cache_miss, latency, false};
}

std::tuple<bool, uint32_t, bool> CoWsCache::llc_miss(Addr_t baddr, bool is_write, Clk_t clk, int total_llc_misses)
{
    AddrParser ap(baddr);
    auto page_addr = ap.getPageAddr();
    auto page_num = ap.getPageNum();
    auto set_idx = ap.getSetID();
    uint32_t block_idx = CoWsCache::AddrParser::getBlockIdx(baddr);

    // miss latency is at least a cows cache access latency
    uint32_t latency = m_access_latency;

    // get page data from main db
    auto [perf_miss, page] = perfect_cache_lookup(page_addr);

    //
    // now update the real cache we had a miss
    //
    auto& set = m_cache_sets[set_idx];

    // WI-004 D2: an LLC write miss is a write-allocate *fill* (a read of the block); the bit is set only when the dirty
    // line is written back (llc_evict). is_write is used for stats only.
    auto [cows_cache_miss, cows_cache_evicted, miss_not_handled] = m_policy->llc_miss(set, page, block_idx, false);
    if(is_write && !miss_not_handled) {
        s_lookups_write_alloc++;
    }
    if(miss_not_handled) {
        s_miss_not_handled++;
        s_miss_not_handled_on_llc_miss++;
        // the miss cannot be handled (e.g., all lines in the set are pending). For simplicity we treat it as a miss and retry later.
        return {true, 0, true}; // treat it as a miss, but return the latency of just accessing the cows cache. The caller should retry this access later.
    }
    s_cows_cycles_self += m_access_latency;   // WI-004 C2: handled lookups only
    if(!cows_cache_miss && page->prefetched) {   // WI-015: the sibling fetched with a table line saved a table read
        page->prefetched = false;
        s_prefetch_useful++;
        if(page->prefetched_on_evict) s_prefetch_useful_evictpath++;   // T5
    }
    if(m_always_miss) {
        cows_cache_miss = true;
    }

    if(PRINT_MISS_OVER_LLC && s_misses > total_llc_misses) {
        printf("llc_miss[clk=%lu]: baddr=0x%lx, is_write=%d (s_misses=%lu, llc_misses=%d)\t[perf/miss=%d/%d, s_access=%lu, on_llc_miss=%lu, on_llc_hit=%lu, on_llc_evict=%lu]\n",
                clk, baddr, (int)is_write, s_misses, total_llc_misses,  perf_miss, cows_cache_miss, s_access, s_misses_on_llc_miss, s_misses_on_llc_hit, s_misses_on_llc_evict);
    }

    ++s_access;
    ++s_accesses_on_llc_miss;
    m_stats_set_access[set_idx]++;

    // hit-under-miss?
    if(!cows_cache_miss && !page->checkIfReady(clk)) {   // WI-004 C2: ready iff clk >= ready_clk
        cows_cache_miss = true;
        if(page->getWhenReady() <= clk) {
            fprintf(stderr, "Error: page should not be ready yet. clk=%lu, when_ready=%lu\n", clk, page->getWhenReady());
        }
        assert(page->getWhenReady() > clk);
        latency = std::max<Clk_t>(m_access_latency, page->getWhenReady() - clk);   // WI-004 C2

        s_half_misses_on_llc_miss++;
        s_half_misses++;
    }
    // real miss?
    else if(cows_cache_miss) {
        ++s_misses;
        ++s_misses_on_llc_miss;
        m_stats_set_miss[set_idx]++;

        track_misses(clk, total_llc_misses);

        latency += get_dram_latency_on_translation(page_num);
        s_table_reads++;   // WI-004 D3

        page->setWhenReady(clk + latency);
        page->setPending(true);
        m_pending_lines.push_back(page);
        if(m_table_line_fetch) {   // WI-015
            fetch_line_siblings(page_num, clk + latency, false);
        }
    }
    else {
        ++s_hits;
    }

    s_cows_cycles += latency;

    return {cows_cache_miss, latency, false};
}

void foo_break() { printf("breaking here\n"); }

std::tuple<bool, uint32_t, bool> CoWsCache::llc_evict(Addr_t baddr, bool evict_dirty, Clk_t clk, int total_llc_misses)
{
    AddrParser ap(baddr);
    auto page_addr = ap.getPageAddr();
    auto page_num = ap.getPageNum();
    auto set_idx = ap.getSetID();
    uint32_t block_idx = CoWsCache::AddrParser::getBlockIdx(baddr);

    uint32_t latency = 0;

    auto [perf_miss, page] = perfect_cache_lookup(page_addr);

    //
    // now update the real cache we had a miss
    //
    latency += m_access_latency;

    auto& set = m_cache_sets[set_idx];   // WI-004 D1: the real set, not a copy

    auto [cows_cache_miss, cows_cache_evicted, miss_not_handled] = m_policy->llc_evict(set, page, block_idx, evict_dirty);
    if(miss_not_handled) {
        s_miss_not_handled++;
        s_miss_not_handled_on_llc_evict++;
        // WI-004 C1: a writeback cannot be retried (the LLC victim is gone). The entry is fetched from the table and
        // updated there without being installed (bit/dirty already set by the policy): charge the table latency.
        latency += get_dram_latency_on_translation(page_num);
        s_cows_cycles += latency;
        s_table_reads++;
        if(page->isDirty()) {   // updated in place in the table: one table write, the entry is clean afterwards
            page->setDirty(false);
            s_table_writes++;
        }
        return {true, latency, true};
    }
    s_cows_cycles_self += m_access_latency;   // WI-004 C2: handled lookups only
    if(!cows_cache_miss && page->prefetched) {   // WI-015: the sibling fetched with a table line saved a table read
        page->prefetched = false;
        s_prefetch_useful++;
        if(page->prefetched_on_evict) s_prefetch_useful_evictpath++;   // T5
    }
    if(m_always_miss) {
        cows_cache_miss = true;
    }

    if(PRINT_MISS_OVER_LLC && s_misses > total_llc_misses) {
        printf("llc_evict[clk=%lu]: baddr=0x%lx, is_write=%d (s_misses=%lu, llc_misses=%d)\t[perf/miss=%d/%d, s_access=%lu, on_llc_miss=%lu, on_llc_hit=%lu, on_llc_evict=%lu]\n",
                clk, baddr, (int)false, s_misses, total_llc_misses, perf_miss, cows_cache_miss, s_access, s_misses_on_llc_miss, s_misses_on_llc_hit, s_misses_on_llc_evict);
    }

    ++s_access;
    ++s_accesses_on_llc_evict;
    m_stats_set_access[set_idx]++;

    // hit-under-miss?
    if(!cows_cache_miss && !page->checkIfReady(clk)) {   // WI-004 C2: ready iff clk >= ready_clk
        cows_cache_miss = true;
        assert(page->getWhenReady() > clk);
        latency = std::max<Clk_t>(m_access_latency, page->getWhenReady() - clk);   // WI-004 C2

        s_half_misses_on_llc_evict++;
        s_half_misses++;
    }
    // real miss?
    else if(cows_cache_miss) {
        ++s_misses;
        ++s_misses_on_llc_evict;
        m_stats_set_miss[set_idx]++;

        track_misses(clk, total_llc_misses);

        latency += get_dram_latency_on_translation(page_num);
        s_table_reads++;   // WI-004 D3

        page->setWhenReady(clk + latency);
        page->setPending(true);
        m_pending_lines.push_back(page);
        if(m_table_line_fetch) {   // WI-015
            fetch_line_siblings(page_num, clk + latency, true);
        }
    }
    else {
        ++s_hits;
    }

    s_cows_cycles += latency;

    return {cows_cache_miss, latency, false};
}

void CoWsCache::fini()
{
    static char outbuf[1024];

    if(m_stats.stats_fname.size() == 0) // no stats
        return;

    // dump stat: fraction on entries in LLC that also exist in CoWs cache
    {
        auto of = std::ofstream(m_stats.stats_fname + ".frac-of-entries");

        std::cout<<"# Dumping CoWs stats"<<std::endl;
        of<<"# CoWs stats"<<std::endl;
        for(auto it : m_stats_cows2llc_valid) {
            auto cows = it.first;
            auto llc = it.second;
            float ratio = (1.0*cows) / llc;

            snprintf(outbuf, 1024, "%12u%12u%12.3f", cows, llc, ratio);
            of<<outbuf<<std::endl;
        }
    }

    // dump stat: cycles between misses
    {
        std::map<Clk_t, size_t> diff_histogram;
        size_t diff_count = 0;
        auto of = std::ofstream(m_stats.stats_fname + ".cycles-between-misses");

        std::cout<<"# Dumping CoWs stats"<<std::endl;
        of<<"# CoWs stats"<<std::endl;
        Clk_t prev_clk = 0;
        for(auto it : m_stats_cows_miss_clk) {
            auto clk = it;
            auto diff = clk - prev_clk;
            prev_clk = clk;

            diff_histogram[diff]++;
            diff_count++;

            snprintf(outbuf, 1024, "%12ld%12ld", clk, diff);
            of<<outbuf<<std::endl;
        }
        of.close();

        of = std::ofstream(m_stats.stats_fname + ".cycles-between-misses-cdf");
        size_t cumulative_count = 0;
        for (const auto& [latency, count] : diff_histogram) {
            cumulative_count += count;

            const double pdf = diff_count == 0 ? 0.0 : static_cast<double>(count) / diff_count;
            const double cdf = diff_count == 0 ? 0.0 : static_cast<double>(cumulative_count) / diff_count;

            of << latency << "\t\t\t" << pdf << "\t\t\t" << cdf << std::endl;
        }
        of.close();
    }

    // dump stat: set popularity
    {
        auto of = std::ofstream(m_stats.stats_fname + ".set-popularity");

        std::cout<<"# Dumping CoWs stats"<<std::endl;
        of<<"# CoWs stats"<<std::endl;
        of<<"# CoWs access: "<<s_access<<std::endl;
        of<<"# CoWs hits: "<<s_hits<<std::endl;
        of<<"# CoWs miss: "<<s_misses<<std::endl;
        of<<"# SetID   Accesses[set]     Misses[set]"<<std::endl;
        for(uint32_t set=0; set<m_nsets; set++) {
            snprintf(outbuf, 1024, "%12u%12lu%12lu", set, m_stats_set_access[set], m_stats_set_miss[set]);
            of<<outbuf<<std::endl;
        }
    }
}

}; // namespace
