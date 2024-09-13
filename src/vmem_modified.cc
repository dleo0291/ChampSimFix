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

#include "vmem.h"

#include <cassert>

#include "champsim.h"
#include "champsim_constants.h"
#include "dram_controller.h"
#include <fmt/core.h>

#include <vector>
#include <iostream>
#include <chrono>

uint64_t VirtualMemory::virtual_seed = 0;

void VirtualMemory::set_virtual_seed(uint64_t v_seed)
{
  virtual_seed = v_seed;
}


// constructor for the buddy allocater class
BuddyAllocator::BuddyAllocator(uint64_t dram_size, uint64_t frame_size)
{

  uint64_t num_of_frames = dram_size / frame_size; // calculating number of frams that will populate the list
  
  for (uint64_t i = 0; i <= num_of_frames; i++) // filling up our list (equal in size to our real physical space)
  {
    free_frame_table.push_back(i);
  }  
  
}

// looking for an available frame with a preferred frame passed through (will normaly be the very first frame available)
uint64_t BuddyAllocator::get_free_frame(uint64_t pref_frame){

  for(int i = 0; i <= free_frame_table.size(); i++){ // here we are searching for the preferred frame
    if (pref_frame == free_frame_table[i]){
      return(pref_frame); // in the case that the perferred frame is found we will return it
    }
  
    else{
      return (free_frame_table[0]); // In the case that it is not found we will return the first available frame 
    }
  }
}

uint64_t BuddyAllocator::can_merge(uint64_t page){
  for (uint64_t i = 0; i < allocated_frame_table.size(); i++) // iterating through our allocated table vector
  {
    if (page == allocated_frame_table[i].start_page + allocated_frame_table[i].size + 1){ // if we find a match then we incriment the size of the allocation by 1
      return(allocated_frame_table[i].size + 1); // returning the size variable of our allocation
    }   
  }

  return(0);
}

uint64_t BuddyAllocator::merging(uint64_t pref_frame, uint64_t cycle){

  for (uint64_t i = 0; i < allocated_frame_table.size(); i++) //searching through the allocated table
  {
    if (pref_frame == allocated_frame_table[i].start_page + allocated_frame_table[i].size + 1) // finding the mergable entry
    {
      allocated_frame_table[i].size += 1; // incrimenting the size of the entry within the vector by one
      
      allocated_frame_table[i].last_access = cycle; //updating the current last accessed to the current cycle 

      return (0);
    }
    
  }
  
}

 uint64_t BuddyAllocator::ppage_allocate(uint64_t cycle, uint64_t vaddr){

  uint64_t pref_frame = can_merge(vaddr>>12); // shifitng our value by 12 bits

  uint64_t real_frame = get_free_frame(pref_frame); 

  if (pref_frame == real_frame){ //checking if the pref_frame and real_frame are the same so that we can merge 

    merging(pref_frame, cycle);

    free_frame_table.erase(free_frame_table.begin() + pref_frame);
  }

  else{ 

  allocated_frame_table.push_back({real_frame, 1, vaddr>>12, cycle}); // new entry made if we can not merge the two allocations

  free_frame_table.erase(free_frame_table.begin()); // removing the free frame from the free frame vector
  }

  return(real_frame << 12);
 }

 uint64_t BuddyAllocator::deallocation(uint64_t cycle, uint64_t index){

  if (cycle > 10000000) // here we check the cycle count
  {
    for (uint64_t i = 0; i < free_frame_table.size(); i++)// need to go through the free frame list and find the first frame number thats bigger than the allocated frame num
    {
      if (free_frame_table[i] > allocated_frame_table[index].start_frame)
      {
      
        free_frame_table.insert(free_frame_table.begin() + i , allocated_frame_table[index].start_frame + allocated_frame_table[index].size); // addding the frames back into the free_table _List_impl
        

      } 
      
    }

    allocated_frame_table.erase(allocated_frame_table.begin() + index); // this will remove the mapping as well as the entry within our allocated table
  }
  
 }

//constructor, what is called whren vmem starts up
VirtualMemory::VirtualMemory(uint64_t page_table_page_size, std::size_t page_table_levels, uint64_t minor_penalty, MEMORY_CONTROLLER& _dram)
    : next_ppage(VMEM_RESERVE_CAPACITY), last_ppage(1ull << (LOG2_PAGE_SIZE + champsim::lg2(page_table_page_size / PTE_BYTES) * page_table_levels)),
      minor_fault_penalty(minor_penalty), pt_levels(page_table_levels), pte_page_size(page_table_page_size),pmem_size(_dram.size()), dram(_dram),
      BA(_dram.size(),4096) // calling the Buddy Allocator constructor here
{
  assert(page_table_page_size > 1024);
  assert(page_table_page_size == (1ull << champsim::lg2(page_table_page_size)));
  assert(last_ppage > VMEM_RESERVE_CAPACITY);

  auto required_bits = champsim::lg2(last_ppage);
  if (required_bits > 64)
    fmt::print("WARNING: virtual memory configuration would require {} bits of addressing.\n", required_bits); // LCOV_EXCL_LINE
  if (required_bits > champsim::lg2(dram.size()))
    fmt::print("WARNING: physical memory size is smaller than virtual memory size; Virtual address space will be aliased.\n"); // LCOV_EXCL_LINE

  // populate_pages(); populated the free table and create alloc_table. Replaced with BuddyAllocator Consturctor 

}

// randomizes pages in the page list
void VirtualMemory::shuffle_pages()
{
  if(virtual_seed != 0)
  {
    std::mt19937_64 rng(virtual_seed);
    std::shuffle(ppage_free_list.begin(),ppage_free_list.end(),rng);
    fmt::print("Shuffled {} physical pages with seed {}\n",ppage_free_list.size(),virtual_seed);
  }
}

// refilling the page list if it's empty and reshuffles the pages 
void VirtualMemory::populate_pages()
{
  ppage_free_list.resize((pmem_size - VMEM_RESERVE_CAPACITY)/PAGE_SIZE);
  uint64_t base_address = VMEM_RESERVE_CAPACITY;
  for(auto it = ppage_free_list.begin(); it != ppage_free_list.end(); it++)
  {
    *(it) = base_address;
    base_address += PAGE_SIZE;
  }
  fmt::print("Created {} new physical pages\n",ppage_free_list.size());
}
uint64_t VirtualMemory::shamt(std::size_t level) const { return LOG2_PAGE_SIZE + champsim::lg2(pte_page_size / PTE_BYTES) * (level - 1); }

uint64_t VirtualMemory::get_offset(uint64_t vaddr, std::size_t level) const
{
  return (vaddr >> shamt(level)) & champsim::bitmask(champsim::lg2(pte_page_size / PTE_BYTES));
}

uint64_t VirtualMemory::ppage_front(uint64_t cycle, uint64_t vaddr) const //edited to pass in cycle and vaddr
{
  assert(available_ppages() > 0);
  return ppage_free_list.front();
  // checking if pages are availbe and returns the first free frame found
}

void VirtualMemory::ppage_pop(uint64_t cycle, uint64_t vaddr)
{ 
  //removes the frame from the free frame list
  ppage_free_list.pop_front();
  if(available_ppages() == 0)
  {
    populate_pages();
    shuffle_pages();
  }
  // in the case we run out physcial memory we will repop the enitre free frame list
}

// std::size_t VirtualMemory::available_ppages() const { return ppage_free_list.size(); } returning number of free frames available  
std::size_t VirtualMemory::available_ppages() const { return free_table.size(); }

std::pair<uint64_t, uint64_t> VirtualMemory::va_to_pa(uint32_t cpu_num, uint64_t vaddr)
{

    bool faulty;
  uint64_t ppage;
  if (vpage_to_ppage_map.find({cpu_num, vaddr >> LOG2_PAGE_SIZE}) != vpage_to_ppage_map.end())
  {
    faulty = 0;
    fmt::print("fault results",faulty);
  }

  else{
    faulty = 1;
    fmt::print("fault results",faulty);
  }
  // checking if there is an existing entry. If not then we are creating a new entry using vmem

  //checking if there is an existing allocation 
  // ppage will be created or existing frame, fault 1 if it missed 0 if already defined 

  // auto [ppage, fault] = vpage_to_ppage_map.insert({{cpu_num, vaddr >> LOG2_PAGE_SIZE}, ppage_front(dram.current_cycle, vaddr)}); 

  //log2_page_size is the number of bits
   
  if (faulty) {// if there isn't a current page tabe entry
    ppage_pop(dram.current_cycle, vaddr); 

     // here we are allocating using the buddy system

     // make sure allocation is actuall happening and check if fault is correctly working
  
    vpage_to_ppage_map[{cpu_num,vaddr}] = BA.ppage_allocate(dram.current_cycle, vaddr);
  }
  ppage = vpage_to_ppage_map[{cpu_num,vaddr}];

  auto paddr = champsim::splice_bits(ppage, vaddr, LOG2_PAGE_SIZE);

  if constexpr (champsim::debug_print) {
    fmt::print("[VMEM] {} paddr: {:x} vaddr: {:x} fault: {}\n", __func__, paddr, vaddr, faulty);
  }

  return {paddr, faulty ? minor_fault_penalty : 0};
} 


std::pair<uint64_t, uint64_t> VirtualMemory::get_pte_pa(uint32_t cpu_num, uint64_t vaddr, std::size_t level)
{

  if (next_pte_page == 0) { // if we're waiting on an allocation then we will assume there is none and alllocate

    // next_pte_page = ppage_front(dram.current_cycle, vaddr); 
    // ppage_pop(dram.current_cycle, vaddr);

    next_pte_page = BA.ppage_allocate(dram.current_cycle, vaddr); 
  }

  std::tuple key{cpu_num, vaddr >> shamt(level), level};
  auto [ppage, fault] = page_table.insert({key, next_pte_page});
  // do similar thing to what was done in vadd_pp
  //have two variables fault and ppage (the actual physical page)


  if (fault) { // if there is a miss then we will allocate a new frame to the page
    next_pte_page += pte_page_size;
    if (!(next_pte_page % PAGE_SIZE)) {
      
      // next_pte_page = ppage_front(dram.current_cycle, vaddr);
      // ppage_pop(dram.current_cycle, vaddr);

      // next_pte_page = BA.ppage_allocate(dram.current_cycle, vaddr);

      vpage_to_ppage_map[{cpu_num,vaddr}] = BA.ppage_allocate(dram.current_cycle, vaddr);
    }
  }

  auto offset = get_offset(vaddr, level);
  auto paddr = champsim::splice_bits(ppage->second, offset * PTE_BYTES, champsim::lg2(pte_page_size));


  if constexpr (champsim::debug_print) {
    fmt::print("[VMEM] {} paddr: {:x} vaddr: {:x} pt_page_offset: {} translation_level: {} fault: {}\n", __func__, paddr, vaddr, offset, level, fault);
  }

  return {paddr, fault ? minor_fault_penalty : 0};
}
