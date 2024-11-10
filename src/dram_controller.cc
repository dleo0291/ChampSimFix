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

#include "dram_controller.h"

#include <algorithm>
#include <cfenv>
#include <cmath>
#include <utility> // for move
#include <fmt/core.h>

#include "champsim_constants.h"
#include "deadlock.h"
#include "instruction.h"
#include "util/bits.h" // for lg2, bitmask
#include "util/span.h"

uint64_t cycles(double time, int io_freq)
{
  std::fesetround(FE_UPWARD);
  auto result = std::lrint(time * io_freq);
  return result < 0 ? 0 : static_cast<uint64_t>(result);
}

MEMORY_CONTROLLER::MEMORY_CONTROLLER(double freq_scale, int io_freq, double t_rp, double t_rcd, double t_cas, double turnaround,
                                     std::vector<channel_type*>&& ul)
    : champsim::operable(freq_scale), queues(std::move(ul))
{
  #ifdef RAMULATOR

  //this line can be used to read in the config as a file (this might be easier and more intuitive for users familiar with Ramulator)
  //the full file path should be included, otherwise Ramulator looks in the current working directory (BAD)
  config = Ramulator::Config::parse_config_file("ramulator.yaml", {});


  //create our frontend (us) and the memory system (ramulator)
  ramulator2_frontend = Ramulator::Factory::create_frontend(config);
  ramulator2_memorysystem = Ramulator::Factory::create_memory_system(config);

  //connect the two. we can use this connection to get some more information from ramulator
  ramulator2_frontend->connect_memory_system(ramulator2_memorysystem);
  ramulator2_memorysystem->connect_frontend(ramulator2_frontend);

  //correct clock scale for ramulator2 frequency. Looks like this may point to an inaccuracy in our own model:
  //although the data bus is running at freq f, the memory controller runs at half this (f/2). This is where "DDR" gets its name
  CLOCK_SCALE = ((ramulator2_memorysystem->get_tCK() / (1000.0/double(DRAM_IO_FREQ)))*(CLOCK_SCALE+1.0)) - 1.0;

  #else
  for (std::size_t i{0}; i < DRAM_CHANNELS; ++i) {
    channels.emplace_back(io_freq, t_rp, t_rcd, t_cas, turnaround, DRAM_ROWS, DRAM_COLUMNS, DRAM_RANKS, DRAM_BANKS,this);
  }
  #endif
}

DRAM_CHANNEL::DRAM_CHANNEL(int io_freq, double t_rp, double t_rcd, double t_cas, double turnaround, std::size_t rows, std::size_t columns, std::size_t ranks,
                           std::size_t banks, MEMORY_CONTROLLER* mc)
    : champsim::operable(1.0), tRP(cycles(t_rp / 1000, io_freq)), tRCD(cycles(t_rcd / 1000, io_freq)), tCAS(cycles(t_cas / 1000, io_freq)),
      DRAM_DBUS_TURN_AROUND_TIME(cycles(turnaround / 1000, io_freq)), DRAM_DBUS_RETURN_TIME(cycles(std::ceil(BLOCK_SIZE) / std::ceil(DRAM_CHANNEL_WIDTH), 1)),
      ROWS(rows), COLUMNS(columns), RANKS(ranks), BANKS(banks), MC(mc)
{
}

long MEMORY_CONTROLLER::operate()
{
  long progress{0};

  initiate_requests();

  #ifdef RAMULATOR
  //tick ramulator.
  //we will assume no deadlock, since there are no other ways to measure progress
  ramulator2_memorysystem->tick();
  progress = 1;
  #else
  for (auto& channel : channels) {
    progress += channel._operate();
  }
  #endif

  
  return progress;
}

long DRAM_CHANNEL::operate()
{
  long progress{0};

  if (warmup) {
    for (auto& entry : RQ) {
      if (entry.has_value()) {
        response_type response{entry->address, entry->v_address, entry->data, entry->pf_metadata, entry->instr_depend_on_me};
        for (auto* ret : entry.value().to_return) {
          ret->push_back(response);
        }

        ++progress;
        entry.reset();
      }
    }

    for (auto& entry : WQ) {
      if (entry.has_value()) {
        ++progress;
      }
      entry.reset();
    }
  }

  check_collision();
  progress += finish_dbus_request();
  swap_write_mode();
  progress += schedule_refresh();
  progress += populate_dbus();
  progress += service_packet(schedule_packet());

  //added for rowhammer
  HC.log_cycle();

  if(HC.is_start_cycle(current_cycle))
  {
    refresh_row = HC.get_target_row() + (DRAM_ROWS/(1<<13));
  }

  if (current_cycle % HC.cycles_per_heartbeat == 0) {
    // print heartbeat
    double cum_hit_rate = (rb_hits / double(rb_hits + rb_miss));
    double hit_rate = ((rb_hits - last_rb_hits) / double((rb_hits-last_rb_hits) + (rb_miss - last_rb_miss)));

    double throughput = (((bank_util - last_bank_util)/double(HC.cycles_per_heartbeat)) * (DRAM_IO_FREQ*1e6)) / double(1<<30);
    double cum_throughput = (((bank_util)/double(current_cycle)) * (DRAM_IO_FREQ*1e6)) / double(1<<30);
    fmt::print("Heartbeat DRAM {} Throughput: {:.3f}GB/s Cumulative Throughput: {:.3f}GB/s Row Buffer Hit Rate: {:.3f} Cumulative Row Buffer Hit Rate: {:.3f}\n",HC.channel_num,throughput,cum_throughput,hit_rate,cum_hit_rate);
    last_bank_util = bank_util;
    last_rb_hits = rb_hits;
    last_rb_miss = rb_miss;
  }

  return progress;
}

long DRAM_CHANNEL::finish_dbus_request()
{
  long progress{0};

  if (active_request != std::end(bank_request) && active_request->event_cycle <= current_cycle) {
    response_type response{active_request->pkt->value().address, active_request->pkt->value().v_address, active_request->pkt->value().data,
                           active_request->pkt->value().pf_metadata, active_request->pkt->value().instr_depend_on_me};
    for (auto* ret : active_request->pkt->value().to_return) {
      ret->push_back(response);
    }

    active_request->valid = false;

    active_request->pkt->reset();
    active_request = std::end(bank_request);
    ++progress;

    bank_util += BLOCK_SIZE;
  }

  return progress;
}

long DRAM_CHANNEL::schedule_refresh()
{
  long progress = {0};
  //check if we reached refresh cycle
  bool schedule_refresh = (current_cycle % uint64_t((DRAM_IO_FREQ * 1e6 * 0.064) / (DRAM_ROWS/(double)8))) == 1;

  //if so, record stats
  if(schedule_refresh)
  {
    refresh_row = (refresh_row + (DRAM_ROWS/(1<<13))) % DRAM_ROWS;
    sim_stats.refresh_cycles++;

    //added for Rowhammer
    //HC.log_refresh(refresh_row,current_cycle);
  }

  //go through each bank, and handle refreshes
  for (auto it = std::begin(bank_request); it != std::end(bank_request); ++it)
  {
    //refresh is now needed for this bank
    if(schedule_refresh)
    {
      it->need_refresh = true;
    }
    //refresh is being scheduled for this bank
    if(it->need_refresh && !it->valid)
    {
      it->event_cycle = current_cycle + tCAS + tRCD;
      it->need_refresh = false;
      it->under_refresh = true;
    }
    //refresh is done for this bank
    else if(it->under_refresh && it->event_cycle <= current_cycle)
    {
      it->under_refresh = false;
      it->open_row.reset();
      
      //log the refresh activations
      auto address_dec = std::distance(bank_request.begin(),it);
      auto op_rank = address_dec / DRAM_BANKS;
      auto op_bank = address_dec % DRAM_BANKS;
      auto op_row = refresh_row;
      auto channel = std::distance(MC->channels.data(), this);
      for(int i = 0; i < (DRAM_ROWS/(1<<13)); i++)
        HC.log_charge(Address(channel, op_bank, op_rank,op_row+i),0,0,RH_REFRESH,false,current_cycle,false);

      progress++;
    }
  }
  return(progress);
}

void DRAM_CHANNEL::swap_write_mode()
{
    // Check queue occupancy
  auto wq_occu = static_cast<std::size_t>(std::count_if(std::begin(WQ), std::end(WQ), [](const auto& x) { return x.has_value(); }));
  auto rq_occu = static_cast<std::size_t>(std::count_if(std::begin(RQ), std::end(RQ), [](const auto& x) { return x.has_value(); }));

  // Change modes if the queues are unbalanced
  if ((!write_mode && (wq_occu >= DRAM_WRITE_HIGH_WM || (rq_occu == 0 && wq_occu > 0)))
      || (write_mode && (wq_occu == 0 || (rq_occu > 0 && wq_occu < DRAM_WRITE_LOW_WM)))) {
    // Reset scheduled requests
    for (auto* it = std::begin(bank_request); it != std::end(bank_request); ++it) {
      // Leave active request on the data bus
      if (it != active_request && it->valid) {
        // Leave rows charged
        if (it->event_cycle < (current_cycle + tCAS)) {

          //log charge
          /*if(it->open_row.has_value())
          {
            auto address_dec = std::distance(bank_request.begin(),it);
            auto op_rank = address_dec / DRAM_BANKS;
            auto op_bank = address_dec % DRAM_BANKS;
            auto op_row = it->open_row.value();

            auto channel = std::distance(MC->channels.data(), this);
            HC.log_charge(Address(channel, op_bank,
                                op_rank,op_row),write_mode ? RH_WRITE : RH_READ,false,current_cycle,false);

          }*/

          //we are closing row
          //it->open_row.reset();

        }

        // This bank is ready for another DRAM request
        it->valid = false;
        it->pkt->value().scheduled = false;
        it->pkt->value().event_cycle = current_cycle;
      }
    }

    // Add data bus turn-around time
    if (active_request != std::end(bank_request)) {
      dbus_cycle_available = active_request->event_cycle + DRAM_DBUS_TURN_AROUND_TIME; // After ongoing finish
    } else {
      dbus_cycle_available = current_cycle + DRAM_DBUS_TURN_AROUND_TIME;
    }

    // Invert the mode
    write_mode = !write_mode;
  }
}

// Look for requests to put on the bus
long DRAM_CHANNEL::populate_dbus()
{
 long progress{0};

  auto* iter_next_process = std::min_element(std::begin(bank_request), std::end(bank_request),
                                             [](const auto& lhs, const auto& rhs) { return !rhs.valid || (lhs.valid && lhs.event_cycle < rhs.event_cycle); });
  if (iter_next_process->valid && iter_next_process->event_cycle <= current_cycle) {
    if (active_request == std::end(bank_request) && dbus_cycle_available <= current_cycle) {
      // Bus is available
      // Put this request on the data bus
      active_request = iter_next_process;
      active_request->event_cycle = current_cycle + DRAM_DBUS_RETURN_TIME;

      if (iter_next_process->row_buffer_hit) {
        rb_hits++;
        if (write_mode) {
          ++sim_stats.WQ_ROW_BUFFER_HIT;
        } else {
          ++sim_stats.RQ_ROW_BUFFER_HIT;
        }
      } else if (write_mode) {
        rb_miss++;
        ++sim_stats.WQ_ROW_BUFFER_MISS;
        HC.log_charge(Address(get_channel(active_request->pkt->value().address), get_bank(active_request->pkt->value().address),
                              get_rank(active_request->pkt->value().address),get_row(active_request->pkt->value().address)),active_request->pkt->value().address,active_request->pkt->value().v_address,write_mode ? RH_WRITE : RH_READ,active_request->pkt->value().type == access_type::PREFETCH,current_cycle,active_request->pkt->value().write_back);
      } else {
        rb_miss++;
        ++sim_stats.RQ_ROW_BUFFER_MISS;
        HC.log_charge(Address(get_channel(active_request->pkt->value().address), get_bank(active_request->pkt->value().address),
                              get_rank(active_request->pkt->value().address),get_row(active_request->pkt->value().address)),active_request->pkt->value().address,active_request->pkt->value().v_address,write_mode ? RH_WRITE : RH_READ,active_request->pkt->value().type == access_type::PREFETCH,current_cycle,active_request->pkt->value().write_back);
      }

      //open the row
      active_request->open_row = get_row(active_request->pkt->value().address);

      ++progress;
    } else {
      // Bus is congested
      if (active_request != std::end(bank_request)) {
        sim_stats.dbus_cycle_congested += (active_request->event_cycle - current_cycle);
      } else {
        sim_stats.dbus_cycle_congested += (dbus_cycle_available - current_cycle);
      }
      ++sim_stats.dbus_count_congested;
    }
  }

  return progress;
}

// Look for queued packets that have not been scheduled
DRAM_CHANNEL::queue_type::iterator DRAM_CHANNEL::schedule_packet()
{
  // Look for queued packets that have not been scheduled
  // prioritize packets that are ready to execute, bank is free
  auto next_schedule = [this](const auto& lhs, const auto& rhs) {
    if (!(rhs.has_value() && !rhs.value().scheduled))
    return(true);
    if (!(lhs.has_value() && !lhs.value().scheduled))
    return(false);

    auto lop_idx = this->get_rank(lhs.value().address)*this->BANKS + this->get_bank(lhs.value().address);
    auto rop_idx = this->get_rank(rhs.value().address)*this->BANKS + this->get_bank(rhs.value().address);
    auto rready = !this->bank_request[rop_idx].valid && !this->bank_request[rop_idx].under_refresh;
    auto lready = !this->bank_request[lop_idx].valid && !this->bank_request[lop_idx].under_refresh;
    return !(rready ^ lready) ? lhs.value().event_cycle < rhs.value().event_cycle : lready;
    //return lhs.value().event_cycle < rhs.value().event_cycle;
  };
  queue_type::iterator iter_next_schedule;
  if (write_mode) {
    iter_next_schedule = std::min_element(std::begin(WQ), std::end(WQ), next_schedule);
  } else {
    iter_next_schedule = std::min_element(std::begin(RQ), std::end(RQ), next_schedule);
  }
  return(iter_next_schedule);
}

long DRAM_CHANNEL::service_packet(DRAM_CHANNEL::queue_type::iterator pkt)
{
  long progress{0};
  if (pkt->has_value() && pkt->value().event_cycle <= current_cycle && !pkt->value().scheduled) {
    auto op_rank = get_rank(pkt->value().address);
    auto op_bank = get_bank(pkt->value().address);
    auto op_row = get_row(pkt->value().address);

    auto op_idx = op_rank * DRAM_BANKS + op_bank;

    if (!bank_request[op_idx].valid && !bank_request[op_idx].under_refresh) {
      bool row_buffer_hit = (bank_request[op_idx].open_row.has_value() && bank_request[op_idx].open_row.value() == op_row);

      // this bank is now busy
      uint64_t row_charge_delay = bank_request[op_idx].open_row.has_value() ? tRP + tRCD : tRCD;


      bank_request[op_idx] = {true,row_buffer_hit,false,false,bank_request[op_idx].open_row, current_cycle + tCAS + (row_buffer_hit ? 0 : row_charge_delay),pkt};
      pkt->value().scheduled = true;
      pkt->value().event_cycle = std::numeric_limits<uint64_t>::max();

      ++progress;
    }
  }

  return progress;
}

void MEMORY_CONTROLLER::initialize()
{
  #ifdef RAMULATOR
  //ramulator will print this information out upon startup. We might be able to derive size somehow
  fmt::print("Refer to Ramulator configuration for Off-chip DRAM Size and Configuration\n");
  YAML::Emitter em;
  em << config;
  fmt::print("{}\n",em.c_str());
  #else
  long long int dram_size = DRAM_CHANNELS * DRAM_RANKS * DRAM_BANKS * DRAM_ROWS * DRAM_COLUMNS * BLOCK_SIZE / 1024 / 1024; // in MiB
  fmt::print("Off-chip DRAM Size: ");
  if (dram_size > 1024) {
    fmt::print("{} GiB", dram_size / 1024);
  } else {
    fmt::print("{} MiB", dram_size);
  }
  fmt::print(" Channels: {} Width: {}-bit Data Race: {} MT/s\n", DRAM_CHANNELS, 8 * DRAM_CHANNEL_WIDTH, DRAM_IO_FREQ);
  #endif
}

void DRAM_CHANNEL::initialize() {}

void MEMORY_CONTROLLER::begin_phase()
{
  std::size_t chan_idx = 0;
  for (auto& chan : channels) {
    DRAM_CHANNEL::stats_type new_stats;
    new_stats.name = "Channel " + std::to_string(chan_idx++);
    chan.sim_stats = new_stats;
    chan.warmup = warmup;
  }

  for (auto* ul : queues) {
    channel_type::stats_type ul_new_roi_stats;
    channel_type::stats_type ul_new_sim_stats;
    ul->roi_stats = ul_new_roi_stats;
    ul->sim_stats = ul_new_sim_stats;
  }
}

void DRAM_CHANNEL::begin_phase() {}

void MEMORY_CONTROLLER::end_phase(unsigned cpu)
{
  #ifdef RAMULATOR
  //this happens to also print stats. We should probably disable the first phase printout and reset stats?
  if(!warmup)
  {
    ramulator2_frontend->finalize();
    ramulator2_memorysystem->finalize();
  }
  #endif

  for (auto& chan : channels) {
    chan.end_phase(cpu);
  }
}

void DRAM_CHANNEL::end_phase(unsigned /*cpu*/) 
{ 
  roi_stats = sim_stats;
  HC.print_file(); 
}


void DRAM_CHANNEL::check_collision()
{
  for (auto wq_it = std::begin(WQ); wq_it != std::end(WQ); ++wq_it) {
    if (wq_it->has_value() && !wq_it->value().forward_checked) {
      auto checker = [addr = wq_it->value().address, offset = LOG2_BLOCK_SIZE](const auto& pkt) {
        return pkt.has_value() && (pkt->address >> offset) == (addr >> offset);
      };
      if (auto found = std::find_if(std::begin(WQ), wq_it, checker); found != wq_it) { // Forward check
        wq_it->reset();
      } else if (found = std::find_if(std::next(wq_it), std::end(WQ), checker); found != std::end(WQ)) { // Backward check
        wq_it->reset();
      } else {
        wq_it->value().forward_checked = true;
      }
    }
  }

  for (auto rq_it = std::begin(RQ); rq_it != std::end(RQ); ++rq_it) {
    if (rq_it->has_value() && !rq_it->value().forward_checked) {
      auto checker = [addr = rq_it->value().address, offset = LOG2_BLOCK_SIZE](const auto& pkt) {
        return pkt.has_value() && (pkt->address >> offset) == (addr >> offset);
      };
      if (auto wq_it = std::find_if(std::begin(WQ), std::end(WQ), checker); wq_it != std::end(WQ)) {
        response_type response{rq_it->value().address, rq_it->value().v_address, rq_it->value().data, rq_it->value().pf_metadata,
                               rq_it->value().instr_depend_on_me};
        response.data = wq_it->value().data;
        for (auto ret : rq_it->value().to_return)
          ret->push_back(response);

        rq_it->reset();
      } else if (auto found = std::find_if(std::begin(RQ), rq_it, checker); found != rq_it) {
        auto instr_copy = std::move(found->value().instr_depend_on_me);
        auto ret_copy = std::move(found->value().to_return);

        std::set_union(std::begin(instr_copy), std::end(instr_copy), std::begin(rq_it->value().instr_depend_on_me), std::end(rq_it->value().instr_depend_on_me),
                       std::back_inserter(found->value().instr_depend_on_me), ooo_model_instr::program_order);
        std::set_union(std::begin(ret_copy), std::end(ret_copy), std::begin(rq_it->value().to_return), std::end(rq_it->value().to_return),
                       std::back_inserter(found->value().to_return));

        rq_it->reset();
      } else if (found = std::find_if(std::next(rq_it), std::end(RQ), checker); found != std::end(RQ)) {
        auto instr_copy = std::move(found->value().instr_depend_on_me);
        auto ret_copy = std::move(found->value().to_return);

        std::set_union(std::begin(instr_copy), std::end(instr_copy), std::begin(rq_it->value().instr_depend_on_me), std::end(rq_it->value().instr_depend_on_me),
                       std::back_inserter(found->value().instr_depend_on_me), ooo_model_instr::program_order);
        std::set_union(std::begin(ret_copy), std::end(ret_copy), std::begin(rq_it->value().to_return), std::end(rq_it->value().to_return),
                       std::back_inserter(found->value().to_return));

        rq_it->reset();
      } else {
        rq_it->value().forward_checked = true;
      }
    }
  }
}

void MEMORY_CONTROLLER::initiate_requests()
{
  // Initiate read requests
  for (auto* ul : queues) {
    for (auto q : {std::ref(ul->RQ), std::ref(ul->PQ)}) {
      auto [begin, end] = champsim::get_span_p(std::cbegin(q.get()), std::cend(q.get()), [ul, this](const auto& pkt) { return this->add_rq(pkt, ul); });
      q.get().erase(begin, end);
    }

    // Initiate write requests
    auto [wq_begin, wq_end] = champsim::get_span_p(std::cbegin(ul->WQ), std::cend(ul->WQ), [this](const auto& pkt) { return this->add_wq(pkt); });
    ul->WQ.erase(wq_begin, wq_end);
  }
}

DRAM_CHANNEL::request_type::request_type(const typename champsim::channel::request_type& req)
  :   pf_metadata(req.pf_metadata), address(req.address), v_address(req.v_address), data(req.data), instr_id(req.instr_id), instr_depend_on_me(req.instr_depend_on_me), write_back(req.write_back)
{
  asid[0] = req.asid[0];
  asid[1] = req.asid[1];
  type = req.type;
  //fmt::print("added address: {}\n",address);
}

#ifdef RAMULATOR
void MEMORY_CONTROLLER::return_packet_rq_rr(Ramulator::Request& req, DRAM_CHANNEL::request_type pkt)
{
  response_type response{pkt.address, pkt.v_address, pkt.data,
                        pkt.pf_metadata, pkt.instr_depend_on_me};

  for (auto* ret : pkt.to_return) {
    ret->push_back(response);
  }

  HammerCounter::processed_packets += 1;
  return;
};
#endif

bool MEMORY_CONTROLLER::add_rq(const request_type& packet, champsim::channel* ul)
{
  #ifdef RAMULATOR
  //return handler, to make sure packet responses get delivered
  //if packet needs response, we need to track its data to return later
  if(!warmup)
  {
    //if not warmup
    if(packet.response_requested)
    {
      DRAM_CHANNEL::request_type pkt = DRAM_CHANNEL::request_type{packet};
      pkt.to_return = {&ul->returned};
      bool success = ramulator2_frontend->receive_external_requests(int(Ramulator::Request::Type::Read), int64_t(packet.address), packet.type == access_type::PREFETCH ? 1 : 0, [=](Ramulator::Request& req) {return_packet_rq_rr(req,pkt);});
      return(success);
    }
    else
    {
      //otherwise feed to ramulator directly with no response requested
      bool success = (ramulator2_frontend->receive_external_requests(int(Ramulator::Request::Type::Read), int64_t(packet.address), packet.type == access_type::PREFETCH ? 1 : 0,[this](Ramulator::Request& req){}));
      HammerCounter::processed_packets += success ? 1 : 0;
      return(success);
    }
  }
  else
  {
    //if warmup, just return true and send necessary responses
    if(packet.response_requested)
    {
        response_type response{packet.address, packet.v_address, packet.data,
                              packet.pf_metadata, packet.instr_depend_on_me};
        for (auto* ret : {&ul->returned}) {
          ret->push_back(response);
        }
    }
    return(true);
  }
    #else
    auto& channel = channels[dram_get_channel(packet.address)];

    // Find empty slot
    if (auto rq_it = std::find_if_not(std::begin(channel.RQ), std::end(channel.RQ), [](const auto& pkt) { return pkt.has_value(); });
        rq_it != std::end(channel.RQ)) {
      *rq_it = DRAM_CHANNEL::request_type{packet};
      rq_it->value().forward_checked = false;
      rq_it->value().event_cycle = current_cycle;
      if (packet.response_requested) {
        rq_it->value().to_return = {&ul->returned};
      }

      return true;
    }

    return false;
  #endif
}

bool MEMORY_CONTROLLER::add_wq(const request_type& packet)
{

  #ifdef RAMULATOR
  //if ramulator, feed directly. Since its a write, no response is needed
  if(!warmup)
  {
    bool success = (ramulator2_frontend->receive_external_requests(int(Ramulator::Request::Type::Write), int64_t(packet.address), 0, [this](Ramulator::Request& req){}));
    HammerCounter::processed_packets += success ? 1 : 0;
    return(success);
  }
  return(true);
  #else
  auto& channel = channels[dram_get_channel(packet.address)];

  // search for the empty index
  if (auto wq_it = std::find_if_not(std::begin(channel.WQ), std::end(channel.WQ), [](const auto& pkt) { return pkt.has_value(); });
      wq_it != std::end(channel.WQ)) {
    *wq_it = DRAM_CHANNEL::request_type{packet};
    wq_it->value().forward_checked = false;
    wq_it->value().event_cycle = current_cycle;

    return true;
  }

  ++channel.sim_stats.WQ_FULL;
  return false;
  #endif
}

/*
 * | row address | rank index | column address | bank index | channel | block offset |
 */



//this needs to be changed. We need to find a way to dynamically map as to support different state-of-the-art mappings
//page interleaving
/*
   | row address | rank index | channel index | remaining column bit(s) |  bank index | column address (sum to 9 with block offset) | block offset |
 */

unsigned long MEMORY_CONTROLLER::dram_get_channel(uint64_t address) const
{
  int shift = LOG2_BLOCK_SIZE;
  return (address >> shift) & champsim::bitmask(champsim::lg2(DRAM_CHANNELS));

  //int shift = LOG2_BLOCK_SIZE + champsim::lg2(DRAM_COLUMNS) + champsim::lg2(DRAM_BANKS);
  //return (address >> shift) & champsim::bitmask(champsim::lg2(DRAM_CHANNELS));
}

unsigned long MEMORY_CONTROLLER::dram_get_bank(uint64_t address) const { return channels.at(dram_get_channel(address)).get_bank(address); }

unsigned long MEMORY_CONTROLLER::dram_get_column(uint64_t address) const { return channels.at(dram_get_channel(address)).get_column(address); }

unsigned long MEMORY_CONTROLLER::dram_get_rank(uint64_t address) const { return channels.at(dram_get_channel(address)).get_rank(address); }

unsigned long MEMORY_CONTROLLER::dram_get_row(uint64_t address) const { return channels.at(dram_get_channel(address)).get_row(address); }

unsigned long DRAM_CHANNEL::get_channel(uint64_t address) const
{
  return(MC->dram_get_channel(address));
}
unsigned long DRAM_CHANNEL::get_bank(uint64_t address) const
{
  //int shift = std::min(int(LOG2_BLOCK_SIZE + champsim::lg2(COLUMNS)),int(LOG2_PAGE_SIZE) - int(champsim::lg2(BANKS)));
  //uint64_t bank_address = ((address >> shift)) & champsim::bitmask(champsim::lg2(BANKS));
  //return bank_address;

  auto shift = LOG2_BLOCK_SIZE + champsim::lg2(DRAM_CHANNELS);
  return((address >> shift) & champsim::bitmask(champsim::lg2(BANKS)));
}

unsigned long DRAM_CHANNEL::get_column(uint64_t address) const
{
  /*auto shift1 = LOG2_BLOCK_SIZE;
  auto secondary_bits = std::max(int(champsim::lg2(COLUMNS)) + int(LOG2_BLOCK_SIZE) - int(LOG2_PAGE_SIZE) + int(champsim::lg2(BANKS)),0);
  auto shift2 = LOG2_BLOCK_SIZE + champsim::lg2(BANKS) + champsim::lg2(COLUMNS) - secondary_bits;
  auto part_1 = (address >> shift1) & champsim::bitmask(champsim::lg2(COLUMNS) - secondary_bits);
  auto part_2 = (address >> shift2) & champsim::bitmask(secondary_bits);
  return (part_1 | (part_2 << (champsim::lg2(COLUMNS) - secondary_bits)));*/

  auto shift = LOG2_BLOCK_SIZE + champsim::lg2(DRAM_CHANNELS) + champsim::lg2(DRAM_BANKS);
  return((address >> shift) & champsim::bitmask(champsim::lg2(COLUMNS)));
}

unsigned long DRAM_CHANNEL::get_rank(uint64_t address) const
{
  auto shift = champsim::lg2(BANKS) + champsim::lg2(COLUMNS) + champsim::lg2(DRAM_CHANNELS) + LOG2_BLOCK_SIZE;
  return (address >> shift) & champsim::bitmask(champsim::lg2(RANKS));
}

unsigned long DRAM_CHANNEL::get_row(uint64_t address) const
{
  auto shift = champsim::lg2(RANKS) + champsim::lg2(BANKS) + champsim::lg2(COLUMNS) + champsim::lg2(DRAM_CHANNELS) + LOG2_BLOCK_SIZE;
  return (address >> shift) & champsim::bitmask(champsim::lg2(ROWS));
}

std::size_t MEMORY_CONTROLLER::size() const { return DRAM_CHANNELS * DRAM_RANKS * DRAM_BANKS * DRAM_ROWS * DRAM_COLUMNS * BLOCK_SIZE; }

// LCOV_EXCL_START Exclude the following function from LCOV
void MEMORY_CONTROLLER::print_deadlock()
{
  int j = 0;
  for (auto& chan : channels) {
    fmt::print("DRAM Channel {}\n", j++);
    chan.print_deadlock();
  }
}

void DRAM_CHANNEL::print_deadlock()
{
  std::string_view q_writer{"instr_id: {} address: {:#x} v_addr: {:#x} type: {} translated: {}"};
  auto q_entry_pack = [](const auto& entry) {
    return std::tuple{entry->address, entry->v_address};
  };

  //champsim::range_print_deadlock(RQ, "RQ", q_writer, q_entry_pack);
  //champsim::range_print_deadlock(WQ, "WQ", q_writer, q_entry_pack);
}
// LCOV_EXCL_STOP

#ifdef RAMULATOR
namespace Ramulator{
 class RoRaCoBaBgCh final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RoRaCoBaBgCh, "RoRaCoBaBgCh", "Applies a RoRaCoBaBgCh mapping to the address. (Default ChampSim)");

  public:
    IDRAM* m_dram = nullptr;
    int m_num_levels = -1;
    std::vector<int> m_addr_bits;
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;
   
    static std::vector<std::string> previous; // Static variable to store across `apply` runs
    static int total_differing_bits; // Cumulative count of differing bits
    static int total_bits_compared; // Cumulative count of bits compared

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
      }
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);
      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }
      m_col_bits_idx = m_num_levels - 1;
    }

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);
      Addr_t addr = req.addr >> m_tx_offset;

      req.addr_vec[m_dram->m_levels("channel")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);

      if (m_dram->m_organization.count.size() > 5)
        req.addr_vec[m_dram->m_levels("bankgroup")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]);

      req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]);
      req.addr_vec[m_dram->m_levels("column")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("column")]);
      req.addr_vec[m_dram->m_levels("rank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);
      req.addr_vec[m_dram->m_levels("row")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);

      std::vector<int> bit_limits_og = {0, 0, 2, 2, 15, 10};
      std::vector<std::string> og;
     
      if (req.addr_vec.size() != bit_limits_og.size()) {
        std::cerr << "Error: req.addr_vec size does not match bit_limits size." << std::endl;
        return;
      }

      for (size_t i = 0; i < req.addr_vec.size(); ++i) {
        int value = req.addr_vec[i];
        int bit_limit = bit_limits_og[i];
        if (bit_limit == 0) {
          og.push_back("--");
        } else {
          int limited_value = value & ((1 << bit_limit) - 1);
          std::string binary_representation = std::bitset<32>(limited_value).to_string();
          std::string limited_binary = binary_representation.substr(32 - bit_limit);
          og.push_back(limited_binary);
        }
      }

      // XOR `previous` and `og`, and count differing bits
      if (!previous.empty() && previous.size() == og.size()) {
        for (size_t i = 0; i < og.size(); ++i) {
          if (previous[i] != "--" && og[i] != "--") { // Ignore "--" entries
            std::bitset<32> prev_bits(previous[i]);
            std::bitset<32> og_bits(og[i]);
            std::bitset<32> xor_result = prev_bits ^ og_bits; // XOR operation
            int differing_bits = xor_result.count(); // Count the 1s
            total_differing_bits += differing_bits; // Update cumulative count
            total_bits_compared += bit_limits_og[i]; // Update total bits compared

            // std::cout << "XOR Result for position " << i << ": " << xor_result << " (" << differing_bits << " differing bits)\n";
          }
        }

        std::cout << "Total cumulative differing bits across all calls: " << total_differing_bits << "\n";
        std::cout << "Total cumulative bits compared across all calls: " << total_bits_compared << "\n";
       
        double power_com;
power_com = static_cast<double>(total_differing_bits) / total_bits_compared;
std::cout << std::fixed << std::setprecision(4);
//std::cout << power_com << std::endl;

std::cout << "Total power consumption: " << power_com*100 << "%" << "\n";
      }
     
      /*
      std::cout << "Binary representation of previous with bit limit:" << std::endl;
      for (const auto& binary_value : previous) {
        std::cout << binary_value << "\n";
      }
      */
     
      previous = og; // Save the current `og` in `previous`

      /*
      std::cout << "Binary representation of current with bit limit:" << std::endl;
      for (const auto& binary_value : og) {
        std::cout << binary_value << "\n";
      }
      */
    }
  };

  // Define the static variables
  std::vector<std::string> RoRaCoBaBgCh::previous;
  int RoRaCoBaBgCh::total_differing_bits = 0; // Initialize cumulative count to 0
  int RoRaCoBaBgCh::total_bits_compared = 0; // Initialize cumulative count of bits compared to 0

class PBPI_Mapping final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, PBPI_Mapping, "PBPI_Mapping", "Applies a PBPI Mapping to the address. (Alternate ChampSim)");

  public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?
    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    // store the previous address vector
    std::vector<Addr_t> m_prev_addr_vec;

    // make a vector to store power consumption rates
    std::vector<double> power_consumption_rates;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;
      m_num_levels = count.size();
      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
        //std::cout << "This is the number of bits in level [" << level << "]: " << m_addr_bits[level] << std::endl;
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits for ChRaBaRoCo and RoBaRaCoCh
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;

      //tot power
      std::vector<Addr_t> tot_power; 

      // initialize the previous address vector with the same size
      m_prev_addr_vec.assign(m_num_levels, 0);
    }
    
    // initialize bit counter
    int bit_counter = 0;
    int num_bits_pc = 0;

    void apply(Request& req) override {
      req.addr_vec.resize(m_num_levels, -1);

      // initialize xor result to hold power consumption for each level
      Addr_t xor_result_power = 0;

      // initialize power_cons to hold the bit changes for each level
      std::vector<Addr_t> power_vector;

      //retrieve the number of bits for the level currently in
      int num_bits = m_addr_bits.size();

      Addr_t col1_bits = 12 - m_tx_offset - m_addr_bits[m_dram->m_levels("bankgroup")] - m_addr_bits[m_dram->m_levels("bank")] - m_addr_bits[m_dram->m_levels("channel")];
      //std::cout << "The number of col1_bits [" << col1_bits << "]." << std::endl;
      Addr_t col2_bits = m_addr_bits[m_dram->m_levels("column")] - col1_bits;
      //std::cout << "The number of col2_bits [" << col2_bits << "]." << std::endl;
      Addr_t addr = req.addr >> m_tx_offset;
      //std::cout << "The address is: " << addr << std::endl;
      Addr_t xor_bits = req.addr >> 17;
      //std::cout << "The xor_bits being used: " << xor_bits << std::endl;

      //channel
      req.addr_vec[m_dram->m_levels("channel")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);
      //std::cout << "The channel address is: " << req.addr_vec[m_dram->m_levels("channel")] << std::endl;
      //col 1
      req.addr_vec[m_dram->m_levels("column")] = slice_lower_bits(addr, col1_bits);
      //std::cout << "The column address is: " << req.addr_vec[m_dram->m_levels("column")] << std::endl;
      //bank group and bank
      if(m_dram->m_organization.count.size() > 5)
      {
        int bankgroup_val = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]) ^ xor_bits;
        req.addr_vec[m_dram->m_levels("bankgroup")] = slice_lower_bits(bankgroup_val, m_addr_bits[m_dram->m_levels("bankgroup")]);
        //std::cout << "After count > 5, bankgroup size is: " << req.addr_vec[m_dram->m_levels("bankgroup")] << std::endl;

        int bank_val = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]) ^ (xor_bits >> m_addr_bits[m_dram->m_levels("bankgroup")]);
        req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(bank_val,m_addr_bits[m_dram->m_levels("bank")]);
        //std::cout << "After count > 5, bank size is: " << req.addr_vec[m_dram->m_levels("bank")] << std::endl;
      }
      else
      {
        int bank_val = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]) ^ xor_bits;
        req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(bank_val, m_addr_bits[m_dram->m_levels("bank")]);
        //std::cout << "The bankgroup size is: " << req.addr_vec[m_dram->m_levels("bank")] << std::endl;
      }
      //col 2
      req.addr_vec[m_dram->m_levels("column")] += slice_lower_bits(addr, col2_bits) << col1_bits;
      //std::cout << "The column bits is: " << req.addr_vec[m_dram->m_levels("column")] << std::endl;
      //rank
      req.addr_vec[m_dram->m_levels("rank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);
      //std::cout << "The rank bits is: " << req.addr_vec[m_dram->m_levels("rank")] << std::endl;
      //row
      req.addr_vec[m_dram->m_levels("row")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);
      //std::cout << "The row address is: " << req.addr_vec[m_dram->m_levels("row")] << std::endl;
      //std::cout << std::endl;

      // calculate bit changes for power consumption
      for (size_t i = 0; i < m_num_levels; ++i) {
        // uncomment for analysis [NOT FOR LONG RUNS]
        //std::cout << "The current address for level [" << i << "]: " << req.addr_vec[i] << std::endl;
        //std::cout << "The prev address for level [" << i << "]: " << m_prev_addr_vec[i] << std::endl;
        xor_result_power |= (req.addr_vec[i] ^ m_prev_addr_vec[i]);
        //std::cout << "The xor result: " << xor_result_power << std::endl;
      }
      //std::cout << std::endl;
      // store xor_result into power_vector
      power_vector.push_back(xor_result_power);


      // power consumption -----------------------------------------------------------------------------
        
      // convert to binary
      std::bitset<64> binary_representation(xor_result_power);

      // count the number of 1s in the xor result 
      int bit_transitions = std::bitset<64>(xor_result_power).count();

      //update the total bit counter for transistions and total bits
      bit_counter += bit_transitions;
      num_bits_pc += m_addr_bits.size();

      // Cast to float for proper decimal division
      double power_consumption_rate = (static_cast<double>(bit_counter) / static_cast<double>(num_bits_pc)) * 100;

      // uncomment this for analysis [NOT FOR LONGER RUNS]
      //std::cout << "The power consumption rate: " << std::dec << power_consumption_rate << "%" << std::endl << std::endl;  
      power_consumption_rates.push_back(power_consumption_rate);

      m_prev_addr_vec.assign(req.addr_vec.begin(), req.addr_vec.end());

      writePowerConsumptionRatesToFile("power_consumption_rates_PBPI.txt");
    }

    void writePowerConsumptionRatesToFile(const std::string& filename) const {
        std::ofstream outFile(filename);  // Open the file for writing

        if (outFile.is_open()) {
            for (size_t i = 0; i < power_consumption_rates.size(); ++i) {
                outFile << "Rate for instruction " << i << ": " << power_consumption_rates[i] << "%" << std::endl;
            }
            outFile.close();  // Close the file
        } else {
            std::cerr << "Unable to open file " << filename << std::endl;
        }
    }
    
  };

   class RASL final : public IAddrMapper, public Implementation {
    RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, RASL, "RASL", "Applies a RASL Mapping to the address. Yanez's Scheme.");
    // We will try to increase randomization without using too much power

    public:
    IDRAM* m_dram = nullptr;

    int m_num_levels = -1;          // How many levels in the hierarchy?
    std::vector<int> m_addr_bits;   // How many address bits for each level in the hierarchy?

    Addr_t m_tx_offset = -1;

    int m_col_bits_idx = -1;
    int m_row_bits_idx = -1;

    // store the previous address vector
    std::vector<Addr_t> m_prev_addr_vec;

    // make a vector to store power consumption rates
    std::vector<double> power_consumption_rates;

    void init() override { };
    void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
      m_dram = memory_system->get_ifce<IDRAM>();

      // Populate m_addr_bits vector with the number of address bits for each level in the hierachy
      const auto& count = m_dram->m_organization.count;

      // count the num of levels in our hierarchy
      m_num_levels = count.size();
       //std::cout << "The number of levels in our hierarchy: " << m_num_levels << std::endl;

      m_addr_bits.resize(m_num_levels);
      for (size_t level = 0; level < m_addr_bits.size(); level++) {
        m_addr_bits[level] = calc_log2(count[level]);
        //std::cout << "This is the number of bits in [" << level << "]: " << m_addr_bits[level] << std::endl; 
      }

      // Last (Column) address have the granularity of the prefetch size
      m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

      int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
      m_tx_offset = calc_log2(tx_bytes);

      // Determine where are the row and col bits
      try {
        m_row_bits_idx = m_dram->m_levels("row");
      } catch (const std::out_of_range& r) {
        throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
      }

      // Assume column is always the last level
      m_col_bits_idx = m_num_levels - 1;

      //tot power
      std::vector<Addr_t> tot_power; 

      // initialize the previous address vector with the same size
      m_prev_addr_vec.assign(m_num_levels, 0);
    }

    // initialize bit counter
    int bit_counter = 0;
    int num_bits_pc = 0;
    
    void apply(Request& req) override {
      // initialize addr_vec and resize to match the number of levels in the DRAM hierarchy
      req.addr_vec.resize(m_num_levels, -1);

      //shift the original address to the right by offset bits.
      Addr_t addr = req.addr >> m_tx_offset;

      //channel
      req.addr_vec[m_dram->m_levels("channel")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("channel")]);
      //std::cout << "This is the current address of the channel: 0x" << std::hex << req.addr_vec[m_dram->m_levels("channel")] << std::endl;

      //bank group
      if(m_dram->m_organization.count.size() > 5)
      req.addr_vec[m_dram->m_levels("bankgroup")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bankgroup")]);
      //std::cout << "This is the current address of the bankgroup: 0x" << std::hex << req.addr_vec[m_dram->m_levels("bankgroup")] << std::endl;

      //bank
      req.addr_vec[m_dram->m_levels("bank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("bank")]);
      //std::cout << "This is the current address of the bank: 0x" << std::hex << req.addr_vec[m_dram->m_levels("bank")] << std::endl;

      //column
      req.addr_vec[m_dram->m_levels("column")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("column")]);
      //std::cout << "This is the current address of the column: 0x" << std::hex << req.addr_vec[m_dram->m_levels("column")] << std::endl;

      //rank
      req.addr_vec[m_dram->m_levels("rank")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("rank")]);
      //std::cout << "This is the current address of the rank: 0x" << std::hex << req.addr_vec[m_dram->m_levels("rank")] << std::endl;

      //row
      req.addr_vec[m_dram->m_levels("row")] = slice_lower_bits(addr, m_addr_bits[m_dram->m_levels("row")]);
      //std::cout << "This is the current address of the row: 0x" << std::hex << req.addr_vec[m_dram->m_levels("row")] << std::endl;

      //std::cout << std::endl;

      // initialize xor result to hold power consumption for each level
      Addr_t xor_result_power = 0;

      // initialize power_cons to hold the bit changes for each level
      std::vector<Addr_t> power_vector;

      // Generate random address bits for each level (iterate each level)
      for(size_t level = 0; level < m_num_levels; level++){

        //retrieve the number of bits for the level currently in
        int num_bits = m_addr_bits[level];
      
        //initialize rasl_addr to hold the randomized address bits for current level
        Addr_t rasl_addr = 0;

        // loop each bit of the address level currently in to extract
        for(int bit = 0;bit < num_bits; bit++){
          //extract the bit at 'bit position, then shift addr right by that 'bit'
          // bitwise 1 is to isolate the single bit at that position
          Addr_t extracted_bit = (req.addr_vec[level] >> bit) & 1;
          //place the extracted bit in a new, shuffled position
          //add 3 bits to the current bit position and check if new position is within available bits for current level
          int new_position = (bit + 3) % num_bits;
          //place the extracted bit in rasl_addr. Shift the extracted bit left by new_position and bitwise OR with
          //rasl_addr to combine with previous bits
          rasl_addr |= (extracted_bit << new_position);
        }
        // power consumption result for each level
        // uncomment this for analysis [NOT FOR LONGER RUNS]
        //std::cout << "This is the previous address for level [" << level << "] : " << m_prev_addr_vec[level] << std::endl;
        //std::cout << "This is the current address for level [" << level << "] : " << rasl_addr << std::endl;
        xor_result_power = m_prev_addr_vec[level] ^ rasl_addr;
        //std::cout << "This is the xor result of level [" << level << "] : " << xor_result_power << std::endl;
        //std::cout << std::endl;

        // store xor_result into power_vector
        power_vector.push_back(xor_result_power);
    
        //store the result of RASL to the corresponding level
        req.addr_vec[level] = rasl_addr;

        //prepare 'addr' for the next level by shifting out the bits we've just proccessed
        addr >> num_bits;

        // update m_prev_addr_vec with current RASL for next comparison
        m_prev_addr_vec[level] = req.addr_vec[level];

        // power consumption -----------------------------------------------------------------------------
        int bit_one_counter = 0;
        
        // convert to binary
        std::bitset<64> binary_representation(xor_result_power);

        // check if binary representation is correct
        std::string binary_str = binary_representation.to_string().substr(64-num_bits);

        // count the total number of 1 bits
        for (char bit : binary_str){
          if (bit == '1'){
            bit_one_counter += 1;
          }
        }

        bit_counter = bit_one_counter + bit_counter;
        num_bits_pc = num_bits + num_bits_pc;

      }
      // Cast to float for proper decimal division
      double power_consumption_rate = (static_cast<double>(bit_counter) / static_cast<double>(num_bits_pc)) * 100;

      // uncomment this for analysis [NOT FOR LONGER RUNS]
      //std::cout << "The power consumption rate: " << std::dec << power_consumption_rate << "%" << std::endl << std::endl;  
      power_consumption_rates.push_back(power_consumption_rate);

      writePowerConsumptionRatesToFile("power_consumption_rates_rasl.txt");
    }

    void writePowerConsumptionRatesToFile(const std::string& filename) const {
        std::ofstream outFile(filename);  // Open the file for writing

        if (outFile.is_open()) {
            for (size_t i = 0; i < power_consumption_rates.size(); ++i) {
                outFile << "Rate for instruction " << i << ": " << power_consumption_rates[i] << "%" << std::endl;
            }
            outFile.close();  // Close the file
        } else {
            std::cerr << "Unable to open file " << filename << std::endl;
        }
    }
  };
	
    class MINE final : public IAddrMapper, public Implementation {
  RAMULATOR_REGISTER_IMPLEMENTATION(IAddrMapper, MINE, "MINE", "Applies a My method mapping to the address.");

public:
  IDRAM* m_dram = nullptr;
  int m_num_levels = -1;
  std::vector<int> m_addr_bits;
  Addr_t m_tx_offset = -1;
  int m_col_bits_idx = -1;
  int m_row_bits_idx = -1;

  static std::vector<std::string> previous;
  static int total_differing_bits;
  static int total_bits_compared;

  void init() override { };
 
  void setup(IFrontEnd* frontend, IMemorySystem* memory_system) {
    m_dram = memory_system->get_ifce<IDRAM>();
    const auto& count = m_dram->m_organization.count;
    m_num_levels = count.size();
    m_addr_bits.resize(m_num_levels);

    for (size_t level = 0; level < m_addr_bits.size(); level++) {
      m_addr_bits[level] = calc_log2(count[level]);
    }
    m_addr_bits[m_num_levels - 1] -= calc_log2(m_dram->m_internal_prefetch_size);

    int tx_bytes = m_dram->m_internal_prefetch_size * m_dram->m_channel_width / 8;
    m_tx_offset = calc_log2(tx_bytes);

    try {
      m_row_bits_idx = m_dram->m_levels("row");
    } catch (const std::out_of_range& r) {
      throw std::runtime_error(fmt::format("Organization \"row\" not found in the spec, cannot use linear mapping!"));
    }

    m_col_bits_idx = m_num_levels - 1;
  }

  void apply(Request& req) override {
    req.addr_vec.resize(m_num_levels, -1);
    Addr_t addr = req.addr >> m_tx_offset;

    req.addr_vec[m_col_bits_idx] = slice_lower_bits(addr, 2);
    for (int lvl = 0; lvl < m_row_bits_idx; lvl++) {
      req.addr_vec[lvl] = slice_lower_bits(addr, m_addr_bits[lvl]);
    }

    req.addr_vec[m_row_bits_idx] = addr;

    // Caesar cipher shift------------------------------------------------------------------------------------------------
    int caesar_shift_1 = 15;
    if (m_row_bits_idx >= 0 && m_row_bits_idx < m_num_levels) {
      int total_row_space = 1 << m_addr_bits[m_row_bits_idx];
      req.addr_vec[m_row_bits_idx] = (req.addr_vec[m_row_bits_idx] + caesar_shift_1) % total_row_space;
    }

    // Store previous binary representation---------------------------------------------------------------------------------
    std::vector<int> bit_limits = {0, 0, 2, 2, 15, 10};
    std::vector<std::string> og;

    for (size_t i = 0; i < req.addr_vec.size(); ++i) {
      int value = req.addr_vec[i];
      int bit_limit = bit_limits[i];
      if (bit_limit == 0) {
        og.push_back("--");
      } else {
        int limited_value = value & ((1 << bit_limit) - 1);
        og.push_back(std::bitset<32>(limited_value).to_string().substr(32 - bit_limit));
      }
    }

    // Compare to previous and calculate power consumption---------------------------------------------------------------------
    if (!previous.empty() && previous.size() == og.size()) {
      for (size_t i = 0; i < og.size(); ++i) {
        if (previous[i] != "--" && og[i] != "--") {
          std::bitset<32> prev_bits(previous[i]);
          std::bitset<32> og_bits(og[i]);
          int differing_bits = (prev_bits ^ og_bits).count();
          total_differing_bits += differing_bits;
          total_bits_compared += bit_limits[i];
        }
      }
     
      std::cout << "Start of new address: \n";
      std::cout << "Total cumulative differing bits across all calls: " << total_differing_bits << "\n";
      std::cout << "Total cumulative bits compared across all calls: " << total_bits_compared << "\n";
     
      double power_com = static_cast<double>(total_differing_bits) / total_bits_compared;
      std::cout << "Total power consumption: " << power_com * 100 << "%" << "\n";
    }
    /*
    std::cout << "Binary representation of previous with bit limit:" << std::endl;
    for (const auto& binary_value : previous) {
      std::cout << binary_value << "\n";
    }
    */
   
    previous = og;
   
    /*
    std::cout << "Binary representation of current with bit limit:" << std::endl;
      for (const auto& binary_value : og) {
        std::cout << binary_value << "\n";
      }
    */
  }
};

std::vector<std::string> MINE::previous;
int MINE::total_differing_bits = 0;
int MINE::total_bits_compared = 0;

}
#endif