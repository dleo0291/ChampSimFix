/*
 *    Copyright 2023 The ChampSim Contributors
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#ifndef VMEM_H
#define VMEM_H

#include <cstdint>
#include <map>
#include <deque>
#include <vector>

#include "champsim_constants.h"

class MEMORY_CONTROLLER;

// reserve 1MB or one page of space
inline constexpr auto VMEM_RESERVE_CAPACITY = std::max<uint64_t>(PAGE_SIZE, 1ull << 20);

inline constexpr std::size_t PTE_BYTES = 8;

class BuddyAllocator
{
  struct alloc_table_entry // each entry in our allocated vector will have 4 variables
  {
    uint64_t start_frame; //start of the frame
    uint64_t size; // size of allocation
    uint64_t start_page; //  where tha allocation starts on the page
    uint64_t last_access; /// cycle value to keep track of when it was accessed
  };

  public: 
  std::vector<uint64_t> free_frame_table;
  std::vector<alloc_table_entry> allocated_frame_table;


  // here we have all of our existing functions defined in the cc file
  BuddyAllocator(uint64_t dram_size,uint64_t frame_size);

  uint64_t get_free_frame(uint64_t pref_frame, bool try_match);

  std::pair<bool,uint64_t> can_merge(uint64_t page);

  uint64_t ppage_allocate(uint64_t cycle, uint64_t vaddr);

  uint64_t merging(uint64_t entry_1, uint64_t cycle);

  uint64_t deallocation(uint64_t index, uint64_t cycle);

  //std::size_t available_ppages() const;
};

class VirtualMemory
{
private:
  std::map<std::pair<uint32_t, uint64_t>, uint64_t> vpage_to_ppage_map;
  std::map<std::tuple<uint32_t, uint64_t, uint32_t>, uint64_t> page_table;

  //for randomization
  std::deque<uint64_t> ppage_free_list;


  //two vector to keep track of our free frames and allocated frames
  std::vector<uint64_t> free_table;
  std::vector<uint64_t> allocated_table;

  uint64_t next_pte_page = 0;

  uint64_t next_ppage;
  uint64_t last_ppage;

  uint64_t ppage_front(uint64_t cycle, uint64_t vaddr) const;
  void ppage_pop(uint64_t cycle, uint64_t vaddr);

  MEMORY_CONTROLLER& dram;

public:

  BuddyAllocator BA; // constructs the buddy allocator

  static uint64_t virtual_seed;
  uint64_t pmem_size;
  const uint64_t minor_fault_penalty;
  const std::size_t pt_levels;
  const uint64_t pte_page_size; // Size of a PTE page


  // capacity and pg_size are measured in bytes, and capacity must be a multiple of pg_size
  VirtualMemory(uint64_t pg_size, std::size_t page_table_levels, uint64_t minor_penalty, MEMORY_CONTROLLER& dram);
  uint64_t shamt(std::size_t level) const;
  uint64_t get_offset(uint64_t vaddr, std::size_t level) const;
  std::size_t available_ppages() const;
  std::pair<uint64_t, uint64_t> va_to_pa(uint32_t cpu_num, uint64_t vaddr);
  std::pair<uint64_t, uint64_t> get_pte_pa(uint32_t cpu_num, uint64_t vaddr, std::size_t level);
  static void set_virtual_seed(uint64_t v_seed);

  void shuffle_pages();
  void populate_pages();
};

#endif
