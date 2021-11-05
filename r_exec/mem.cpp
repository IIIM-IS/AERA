//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_exec
//_/_/
//_/_/ Eric Nivel
//_/_/ Center for Analysis and Design of Intelligent Agents
//_/_/   Reykjavik University, Menntavegur 1, 101 Reykjavik, Iceland
//_/_/   http://cadia.ru.is
//_/_/ Copyright(c)2012
//_/_/
//_/_/ This software was developed by the above copyright holder as part of
//_/_/ the HUMANOBS EU research project, in collaboration with the
//_/_/ following parties:
//_/_/
//_/_/ Autonomous Systems Laboratory
//_/_/   Technical University of Madrid, Spain
//_/_/   http://www.aslab.org/
//_/_/
//_/_/ Communicative Machines
//_/_/   Edinburgh, United Kingdom
//_/_/   http://www.cmlabs.com/
//_/_/
//_/_/ Istituto Dalle Molle di Studi sull'Intelligenza Artificiale
//_/_/   University of Lugano and SUPSI, Switzerland
//_/_/   http://www.idsia.ch/
//_/_/
//_/_/ Institute of Cognitive Sciences and Technologies
//_/_/   Consiglio Nazionale delle Ricerche, Italy
//_/_/   http://www.istc.cnr.it/
//_/_/
//_/_/ Dipartimento di Ingegneria Informatica
//_/_/   University of Palermo, Italy
//_/_/   http://roboticslab.dinfo.unipa.it/index.php/Main/HomePage
//_/_/
//_/_/
//_/_/ --- HUMANOBS Open-Source BSD License, with CADIA Clause v 1.0 ---
//_/_/
//_/_/ Redistribution and use in source and binary forms, with or without
//_/_/ modification, is permitted provided that the following conditions
//_/_/ are met:
//_/_/
//_/_/ - Redistributions of source code must retain the above copyright
//_/_/ and collaboration notice, this list of conditions and the
//_/_/ following disclaimer.
//_/_/
//_/_/ - Redistributions in binary form must reproduce the above copyright
//_/_/ notice, this list of conditions and the following
//_/_/ disclaimer in the documentation and/or other materials provided
//_/_/ with the distribution.
//_/_/
//_/_/ - Neither the name of its copyright holders nor the names of its
//_/_/ contributors may be used to endorse or promote products
//_/_/ derived from this software without specific prior written permission.
//_/_/
//_/_/ - CADIA Clause: The license granted in and to the software under this
//_/_/ agreement is a limited-use license. The software may not be used in
//_/_/ furtherance of:
//_/_/ (i) intentionally causing bodily injury or severe emotional distress
//_/_/ to any person;
//_/_/ (ii) invading the personal privacy or violating the human rights of
//_/_/ any person; or
//_/_/ (iii) committing or preparing for any act of war.
//_/_/
//_/_/ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
//_/_/ "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
//_/_/ LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
//_/_/ A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
//_/_/ OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
//_/_/ SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
//_/_/ LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
//_/_/ DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
//_/_/ THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
//_/_/ (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
//_/_/ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//_/_/
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include <algorithm>
#include <deque>
#include "mem.h"
#include "mdl_controller.h"
#include "model_base.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;

namespace r_exec {

_Mem::_Mem() : r_code::Mem(), 
  state_(NOT_STARTED), 
  deleted_(false),
  base_period_(50000),
  reduction_core_count_(0),
  time_core_count_(0),
  mdl_inertia_sr_thr_(0.9),
  mdl_inertia_cnt_thr_(6),
  tpx_dsr_thr_(0.1),
  min_sim_time_horizon_(0),
  max_sim_time_horizon_(0),
  sim_time_horizon_factor_(0.3),
  tpx_time_horizon_(5000000),
  perf_sampling_period_(250000),
  float_tolerance_(0.00001),
  time_tolerance_(10000),
  primary_thz_(seconds(3600000)),
  secondary_thz_(seconds(7200000)),
  debug_(true),
  ntf_mk_res_(1),
  goal_pred_success_res_(1000),
  keep_invalidated_objects_(false),
  probe_level_(2),
  enable_assumptions_(true),
  reduction_cores_(0),
  time_cores_(0),
  reduction_job_count_(0),
  time_job_count_(0),
  time_job_avg_latency_(0),
  _time_job_avg_latency_(0),
  core_count_(0),
  stop_sem_(0),
  stdin_(0),
  stdout_(0),
  self_(0),
  default_runtime_output_stream_(&std::cout)
{

  new ModelBase();
  objects_.reserve(1024);
  for (uint32 i = 0; i < RUNTIME_OUTPUT_STREAM_COUNT; ++i)
    runtime_output_streams_[i] = NULL;
}

_Mem::~_Mem() {

  for (uint32 i = 0; i < RUNTIME_OUTPUT_STREAM_COUNT; ++i)
    if (runtime_output_streams_[i] != NULL)
      delete runtime_output_streams_[i];
}

void _Mem::init(microseconds base_period,
  uint32 reduction_core_count,
  uint32 time_core_count,
  float32 mdl_inertia_sr_thr,
  uint32 mdl_inertia_cnt_thr,
  float32 tpx_dsr_thr,
  microseconds min_sim_time_horizon,
  microseconds max_sim_time_horizon,
  float32 sim_time_horizon_factor,
  microseconds tpx_time_horizon,
  microseconds perf_sampling_period,
  float32 float_tolerance,
  microseconds time_tolerance,
  microseconds primary_thz,
  microseconds secondary_thz,
  bool debug,
  uint32 ntf_mk_res,
  uint32 goal_pred_success_res,
  uint32 probe_level,
  uint32 traces,
  bool enable_assumptions,
  bool keep_invalidated_objects) {

  base_period_ = base_period;

  reduction_core_count_ = reduction_core_count;
  time_core_count_ = time_core_count;

  mdl_inertia_sr_thr_ = mdl_inertia_sr_thr;
  mdl_inertia_cnt_thr_ = mdl_inertia_cnt_thr;
  tpx_dsr_thr_ = tpx_dsr_thr;
  min_sim_time_horizon_ = min_sim_time_horizon;
  max_sim_time_horizon_ = max_sim_time_horizon;
  sim_time_horizon_factor_ = sim_time_horizon_factor;
  tpx_time_horizon_ = tpx_time_horizon;
  perf_sampling_period_ = perf_sampling_period;
  float_tolerance_ = float_tolerance;
  time_tolerance_ = time_tolerance;
  primary_thz_ = primary_thz;
  secondary_thz_ = secondary_thz;

  debug_ = debug;
  if (debug)
    ntf_mk_res_ = ntf_mk_res;
  else
    ntf_mk_res_ = 1;
  goal_pred_success_res_ = goal_pred_success_res;

  probe_level_ = probe_level;
  enable_assumptions_ = enable_assumptions;
  keep_invalidated_objects_ = keep_invalidated_objects;

  reduction_job_count_ = time_job_count_ = 0;
  reduction_job_avg_latency_ = _reduction_job_avg_latency_ = microseconds(0);
  time_job_avg_latency_ = _time_job_avg_latency_ = microseconds(0);

  uint32 mask = 1;
  for (uint32 i = 0; i < RUNTIME_OUTPUT_STREAM_COUNT; ++i) {

    if (traces & mask)
      // NULL means Output() will use defaultDebugStream_ . 
      runtime_output_streams_[i] = NULL;
    else
      runtime_output_streams_[i] = new NullOStream();
    mask <<= 1;
  }
}

std::ostream &_Mem::Output(TraceLevel l) {

  return (_Mem::Get()->runtime_output_streams_[l] == NULL ? 
    *_Mem::Get()->default_runtime_output_stream_ : *(_Mem::Get()->runtime_output_streams_[l]));
}

// This is declared at the r_exec namespace level in overlay.h, so that all headers
// don't need to include mem.h.
std::ostream &_Mem_Output(TraceLevel l) { return _Mem::Output(l); }

void _Mem::reset() {

  uint32 i;
  for (i = 0; i < reduction_core_count_; ++i)
    delete reduction_cores_[i];
  delete[] reduction_cores_;
  for (i = 0; i < time_core_count_; ++i)
    delete time_cores_[i];
  delete[] time_cores_;

  delete reduction_job_queue_;
  delete time_job_queue_;

  delete stop_sem_;
}

////////////////////////////////////////////////////////////////

Code *_Mem::get_root() const {

  return root_;
}

Code *_Mem::get_stdin() const {

  return stdin_;
}

Code *_Mem::get_stdout() const {

  return stdout_;
}

Code *_Mem::get_self() const {

  return self_;
}

////////////////////////////////////////////////////////////////

_Mem::State _Mem::check_state() {

  State s;
  stateCS_.enter();
  s = state_;
  stateCS_.leave();

  return s;
}

void _Mem::start_core() {

  core_countCS_.enter();
  if (++core_count_ == 1)
    stop_sem_->acquire();
  core_countCS_.leave();
}

void _Mem::shutdown_core() {

  core_countCS_.enter();
  if (--core_count_ == 0)
    stop_sem_->release();
  core_countCS_.leave();
}

////////////////////////////////////////////////////////////////

void _Mem::store(Code *object) {

  int32 location;
  objects_.push_back(object, location);
  object->set_strorage_index(location);
}

bool _Mem::load(vector<r_code::Code *> *objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid) { // no cov at init time.

  uint32 i;
  reduction_cores_ = new ReductionCore *[reduction_core_count_];
  for (i = 0; i < reduction_core_count_; ++i)
    reduction_cores_[i] = new ReductionCore();
  time_cores_ = new TimeCore *[time_core_count_];
  for (i = 0; i < time_core_count_; ++i)
    time_cores_[i] = new TimeCore();

  Utils::SetReferenceValues(base_period_, float_tolerance_, time_tolerance_);

  // load root (always comes first).
  root_ = (Group *)(*objects)[0];
  store((Code *)root_);
  initial_groups_.push_back(root_);

  // Get the highest existing OID.
  uint32 highest_oid = 0;
  for (uint32 i = 0; i < objects->size(); ++i)
    highest_oid = max(highest_oid, (*objects)[i]->get_oid());
  set_last_oid(max(highest_oid, objects->size() - 1));

  for (uint32 i = 1; i < objects->size(); ++i) { // skip root as it has no initial views.

    Code *object = (*objects)[i];
    store(object);

    if (object->get_oid() == stdin_oid)
      stdin_ = (Group *)(*objects)[i];
    else if (object->get_oid() == stdout_oid)
      stdout_ = (Group *)(*objects)[i];
    else if (object->get_oid() == self_oid)
      self_ = (*objects)[i];

    switch (object->code(0).getDescriptor()) {
    case Atom::MODEL:
      if (Utils::has_reference(&object->code(0), HLP_FWD_GUARDS)) {
        cerr << "ERROR: Illegal referenced object in forward guards of model OID " << object->get_oid() << endl;
        return false;
      }
      if (Utils::has_reference(&object->code(0), HLP_BWD_GUARDS)) {
        cerr << "ERROR: Illegal referenced object in backward guards of model OID " << object->get_oid() << endl;
        return false;
      }
      unpack_hlp(object);
      //object->add_reference(NULL); // classifier.
      ModelBase::Get()->load(object);
      break;
    case Atom::COMPOSITE_STATE:
      if (Utils::has_reference(&object->code(0), HLP_FWD_GUARDS)) {
        cerr << "ERROR: Illegal referenced object in forward guards of cst OID " << object->get_oid() << endl;
        return false;
      }
      if (Utils::has_reference(&object->code(0), HLP_BWD_GUARDS)) {
        cerr << "ERROR: Illegal referenced object in backward guards of cst OID " << object->get_oid() << endl;
        return false;
      }
      unpack_hlp(object);
      break;
    case Atom::INSTANTIATED_PROGRAM: // refine the opcode depending on the inputs and the program type.
      if (object->get_reference(0)->code(0).asOpcode() == Opcodes::Pgm) {

        if (object->get_reference(0)->code(object->get_reference(0)->code(PGM_INPUTS).asIndex()).getAtomCount() == 0)
          object->code(0) = Atom::InstantiatedInputLessProgram(object->code(0).asOpcode(), object->code(0).getAtomCount());
      } else
        object->code(0) = Atom::InstantiatedAntiProgram(object->code(0).asOpcode(), object->code(0).getAtomCount());
      break;
    }

    unordered_set<_View *, _View::Hash, _View::Equal>::const_iterator v;
    for (v = object->views_.begin(); v != object->views_.end(); ++v) {

      // init hosts' member_set.
      View *view = (View *)*v;
      view->set_object(object);
      Group *host = view->get_host();

      if (!host->load(view, object))
        return false;
      if (host == stdin_ && view->get_sync() == View::SYNC_AXIOM &&
          (view->object_->code(0).asOpcode() == Opcodes::Fact ||
           view->object_->code(0).asOpcode() == Opcodes::AntiFact))
        // This is an axiom in the stdin group, so save for matches_axiom().
        axiom_values_.push_back(view->object_->get_reference(0));
    }

    if (object->code(0).getDescriptor() == Atom::GROUP)
      initial_groups_.push_back((Group *)object); // convenience to create initial update jobs - see start().
  }

  return true;
}

void _Mem::init_timings(Timestamp now, const r_code::list<P<Code>>& objects) {

  auto time_tolerance = Utils::GetTimeTolerance() * 2;
  r_code::list<P<Code> >::const_iterator o;
  for (o = objects.begin(); o != objects.end(); ++o) {

    uint16 opcode = (*o)->code(0).asOpcode();
    if (opcode == Opcodes::Fact || opcode == Opcodes::AntiFact) {

      // Use these as offsets from now.
      auto after = Utils::GetTimestamp<Code>(*o, FACT_AFTER).time_since_epoch();
      auto before = Utils::GetTimestamp<Code>(*o, FACT_BEFORE).time_since_epoch();

      if (after < Utils_MaxTime - now)
        Utils::SetTimestamp<Code>(*o, FACT_AFTER, after + now);
      if (before < Utils_MaxTime - now - time_tolerance)
        Utils::SetTimestamp<Code>(*o, FACT_BEFORE, before + now + time_tolerance);
      else
        Utils::SetTimestamp<Code>(*o, FACT_BEFORE, Utils_MaxTime);
    }
    else if (opcode == Opcodes::IMdl || opcode == Opcodes::ICst) {
      // Look for timestamps in the template values and exposed values, and add now.
      // TODO: There should be a more general mechanism to locate timestamps in facts, imdls, and elsewhere.
      for (int pass = 1; pass <= 2; ++pass) {
        auto set_index = (pass == 1 ? (*o)->code(I_HLP_TPL_ARGS).asIndex()
                                    : (*o)->code(I_HLP_EXPOSED_ARGS).asIndex());
        auto set_count = (*o)->code(set_index).getAtomCount();
        for (int i = 1; i <= set_count; ++i) {
          auto index = set_index + i;
          if ((*o)->code(set_index + i).getDescriptor() == Atom::I_PTR) {
            auto timestamp_index = (*o)->code(set_index + i).asIndex();
            if ((*o)->code(timestamp_index).getDescriptor() == Atom::TIMESTAMP) {
              auto timestamp = Utils::GetTimestamp<Code>(*o, set_index + i).time_since_epoch();
              Utils::SetTimestampStruct(*o, timestamp_index, timestamp + now);
            }
          }
        }
      }
    }
  }
}

Timestamp _Mem::start() {

  if (state_ != STOPPED && state_ != NOT_STARTED)
    return Timestamp(seconds(0));

  core_count_ = 0;
  stop_sem_ = new Semaphore(1, 1);

  time_job_queue_ = new PipeNN<P<TimeJob>, 1024>();
  reduction_job_queue_ = new PipeNN<P<_ReductionJob>, 1024>();

  vector<std::pair<View *, Group *> > initial_reduction_jobs;

  uint32 i;
  auto now = Now();
  Utils::SetTimeReference(now);
  ModelBase::Get()->set_thz(secondary_thz_);
  init_timings(now, objects_);

  for (i = 0; i < initial_groups_.size(); ++i) {

    Group *g = initial_groups_[i];
    bool c_active = g->get_c_act() > g->get_c_act_thr();
    bool c_salient = g->get_c_sln() > g->get_c_sln_thr();

    FOR_ALL_VIEWS_BEGIN(g, v)
      Utils::SetTimestamp<View>(v->second, VIEW_IJT, now); // init injection time for the view.
    FOR_ALL_VIEWS_END

      if (c_active) {

        unordered_map<uint32, P<View> >::const_iterator v;

        // build signaling jobs for active input-less overlays.
        for (v = g->input_less_ipgm_views_.begin(); v != g->input_less_ipgm_views_.end(); ++v) {

          if (v->second->controller_ != NULL && v->second->controller_->is_activated()) {
            // The time scope is stored as a timestamp, but it is actually a duration.
            P<TimeJob> j = new InputLessPGMSignalingJob(v->second, now + Utils::GetTimestamp<Code>(v->second->object_, IPGM_TSC).time_since_epoch());
            time_job_queue_->push(j);
          }
        }

        // build signaling jobs for active anti-pgm overlays.
        for (v = g->anti_ipgm_views_.begin(); v != g->anti_ipgm_views_.end(); ++v) {

          if (v->second->controller_ != NULL && v->second->controller_->is_activated()) {
            // The time scope is stored as a timestamp, but it is actually a duration.
            P<TimeJob> j = new AntiPGMSignalingJob(v->second, now + Utils::GetTimestamp<Code>(v->second->object_, IPGM_TSC).time_since_epoch());
            time_job_queue_->push(j);
          }
        }
      }

    if (c_salient) {

      // build reduction jobs for each salient view and each active overlay - regardless of the view's sync mode.
      FOR_ALL_VIEWS_BEGIN(g, v)

        if (v->second->get_sln() > g->get_sln_thr()) { // salient view.

          g->newly_salient_views_.insert(v->second);
          initial_reduction_jobs.push_back(std::pair<View *, Group *>(v->second, g));
        }
      FOR_ALL_VIEWS_END
    }

    if (g->get_upr() > 0) { // inject the next update job for the group.

      P<TimeJob> j = new UpdateJob(g, g->get_next_upr_time(now));
      time_job_queue_->push(j);
    }
  }

  initial_groups_.clear();

  state_ = RUNNING;

  P<TimeJob> j = new PerfSamplingJob(now + perf_sampling_period_, perf_sampling_period_);
  time_job_queue_->push(j);

  for (i = 0; i < reduction_core_count_; ++i)
    reduction_cores_[i]->start(ReductionCore::Run);
  for (i = 0; i < time_core_count_; ++i)
    time_cores_[i]->start(TimeCore::Run);

  for (uint32 i = 0; i < initial_reduction_jobs.size(); ++i)
    initial_reduction_jobs[i].second->inject_reduction_jobs(initial_reduction_jobs[i].first);

  return now;
}

void _Mem::run_in_diagnostic_time(milliseconds run_time) {
  if (!(reduction_core_count_ == 0 && time_core_count_ == 0))
    // This should only be called if there are no running core threads.
    return;

  // The maximum number of reduction jobs to run before trying a time job.
  // Average job time is 10us. 10000 jobs is 100000us, which is the sampling period.
  // Assume 8 threads (on 4 cores), so allow 10000 * 8 = 80000 jobs per cycle.
  const size_t max_reduction_jobs_per_cycle = 80000;
  size_t n_reduction_jobs_this_sampling_period = 0;
  vector<P<_ReductionJob>> reduction_job_queue;
  // Use a deque so we can efficiently remove from the front.
  std::deque<P<TimeJob>> ordered_time_job_queue;

  auto tick_time = Now();
  on_diagnostic_time_tick();
  auto end_time = Now() + run_time;

  // Loop until the runTimeMilliseconds expires.
  while (true) {
    if (state_ == STOPPED)
      break;

    // Reduction jobs can add more reduction jobs, so make a few passes.
    for (int pass_number = 1; pass_number <= 100; ++pass_number) {
      // Transfer all reduction jobs to a local queue and run only these.
      // Below, we only run one time job, so any extra jobs that these reduction
      // jobs add will be run on the next pass after running the time job.
      while (true) {
        P<_ReductionJob> reduction_job = pop_reduction_job(false);
        if (reduction_job == NULL)
          // No more reduction jobs.
          break;
        reduction_job_queue.push_back(reduction_job);
      }

      size_t n_jobs_to_run = min(reduction_job_queue.size(), max_reduction_jobs_per_cycle);
      if (n_jobs_to_run == 0)
        break;
      for (size_t i = 0; i < n_jobs_to_run; ++i) {
        reduction_job_queue[i]->update(Now());
        reduction_job_queue[i] = NULL;
      }
      n_reduction_jobs_this_sampling_period += n_jobs_to_run;

      if (reduction_job_queue.size() > max_reduction_jobs_per_cycle)
        // There are remaining jobs to be run. Shift them to the front.
        reduction_job_queue.erase(
          reduction_job_queue.begin(), reduction_job_queue.begin() + max_reduction_jobs_per_cycle);
      else
        reduction_job_queue.clear();

      if (n_reduction_jobs_this_sampling_period >= max_reduction_jobs_per_cycle)
        // We have hit the limit of reduction jobs this sampling period.
        break;
    }

    // Transfer all time jobs to ordered_time_job_queue,
    // sorted on target_time_.
    while (true) {
      P<TimeJob> time_job = pop_time_job(false);
      if (time_job == NULL)
        // No more time jobs.
        break;

      ordered_time_job_queue.insert(
        upper_bound(ordered_time_job_queue.begin(),
          ordered_time_job_queue.end(), time_job, time_job_compare_),
        time_job);
    }

    if (Now() >= end_time)
      // Finished.
      break;

    // The entry at the front is the earliest.
    if (ordered_time_job_queue.size() == 0 ||
      ordered_time_job_queue.front()->target_time_ >=
        tick_time + get_sampling_period()) {
      // There is no time job before the next tick time, so tick.
      tick_time += get_sampling_period();
      // Increase the diagnostic time to the tick time.
      diagnostic_time_now_ = tick_time;
      // We are beginning a new sampling period.
      n_reduction_jobs_this_sampling_period = 0;
      on_diagnostic_time_tick();

      // Loop again in case on_diagnostic_time_tick() added a reduction job,
      // or a reduction job will add more time jobs.
      continue;
    }

    if (ordered_time_job_queue.size() == 0)
      // No time jobs. Loop again in case a reduction job will add one.
      continue;

    if (ordered_time_job_queue.front()->target_time_ > Now())
      // Increase the diagnostic time to the job's target time.
      diagnostic_time_now_ = ordered_time_job_queue.front()->target_time_;

    // Only process one job in case it adds more jobs.
    P<TimeJob> time_job = ordered_time_job_queue.front();
    ordered_time_job_queue.erase(ordered_time_job_queue.begin());

    if (!time_job->is_alive()) {
      time_job = NULL;
      continue;
    }

    Timestamp next_target(seconds(0));
    if (!time_job->update(next_target)) {
      // update() says to stop running.
      time_job = NULL;
      break;
    }

    if (next_target.time_since_epoch().count() != 0) {
      // The job wants to run again, so re-insert into the queue.
      time_job->target_time_ = next_target;
      ordered_time_job_queue.insert(
        upper_bound(ordered_time_job_queue.begin(),
          ordered_time_job_queue.end(), time_job, time_job_compare_),
        time_job);
    }
    else
      time_job = NULL;
  }
}

Timestamp _Mem::diagnostic_time_now_ = Timestamp(microseconds(1));

Timestamp _Mem::get_diagnostic_time_now() { return diagnostic_time_now_; }

void _Mem::on_diagnostic_time_tick() {}

void _Mem::stop() {

  stateCS_.enter();
  if (state_ != RUNNING) {

    stateCS_.leave();
    return;
  }

  uint32 i;
  P<_ReductionJob> r;
  for (i = 0; i < reduction_core_count_; ++i)
    reduction_job_queue_->push(r = new ShutdownReductionCore());
  P<TimeJob> t;
  for (i = 0; i < time_core_count_; ++i)
    time_job_queue_->push(t = new ShutdownTimeCore());

  state_ = STOPPED;
  stateCS_.leave();

  for (i = 0; i < time_core_count_; ++i)
    Thread::Wait(time_cores_[i]);

  for (i = 0; i < reduction_core_count_; ++i)
    Thread::Wait(reduction_cores_[i]);

  stop_sem_->acquire(); // wait for delegates.

  reset();
}

////////////////////////////////////////////////////////////////

P<_ReductionJob> _Mem::pop_reduction_job(bool waitForItem) {

  if (state_ == STOPPED)
    return NULL;
  return reduction_job_queue_->pop(waitForItem);
}

void _Mem::push_reduction_job(_ReductionJob *j) {

  if (state_ == STOPPED)
    return;
  j->ijt_ = Now();
  P<_ReductionJob> _j = j;
  reduction_job_queue_->push(_j);
}

P<TimeJob> _Mem::pop_time_job(bool waitForItem) {

  if (state_ == STOPPED)
    return NULL;
  return time_job_queue_->pop(waitForItem);
}

void _Mem::push_time_job(TimeJob *j) {

  if (state_ == STOPPED)
    return;
  P<TimeJob> _j = j;
  time_job_queue_->push(_j);
}

////////////////////////////////////////////////////////////////

void _Mem::eject(View *view, uint16 node_id) {
}

r_code::Code* _Mem::eject(Code *command) {
  // This is only for debugging
  /*
  uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;
  if (function == r_exec::GetOpcode("speak")) {
      std::cout << "Speak" << std::endl;
      //command->trace();
  }
  */
  return NULL;
}

////////////////////////////////////////////////////////////////

void _Mem::inject_copy(View *view, Group *destination) {

  View *copied_view = new View(view, destination); // ctrl values are morphed.
  inject_existing_object(copied_view, view->object_, destination);
}

void _Mem::inject_existing_object(View *view, Code *object, Group *host) {

  view->set_object(object); // the object already exists (content-wise): have the view point to the existing one.
  host->inject_existing_object(view);
}

void _Mem::inject_null_program(Controller *c, Group *group, microseconds time_to_live, bool take_past_inputs) {

  auto now = Now();

  Code *null_pgm = new LObject();
  null_pgm->code(0) = Atom::NullProgram(take_past_inputs);

  uint32 res = Utils::GetResilience(now, time_to_live, group->get_upr() * Utils::GetBasePeriod().count());

  View *view = new View(View::SYNC_ONCE, now, 0, res, group, NULL, null_pgm, 1);
  view->controller_ = c;

  c->set_view(view);

  inject(view);
}

void _Mem::inject_new_object(View *view) {

  Group *host = view->get_host();
  //uint64 t0,t1,t2;
  switch (view->object_->code(0).getDescriptor()) {
  case Atom::GROUP:
    bind(view);

    host->inject_group(view);
    break;
  default:
    //t0=Now();
    bind(view);
    //t1=Now();
    host->inject_new_object(view);
    //t2=Now();
    //timings_report.push_back(t2-t0);
    break;
  }
}

void _Mem::inject_from_io_device(View *view) {
  // Inject first to set the OID.
  inject(view, true);
  // For UNDEFINED_OID, assume that InjectionJob::update will log it.
  if (view->object_->get_oid() != UNDEFINED_OID)
    // The view injection time may be different than now, so log it too.
    OUTPUT_LINE(IO_DEVICE_INJ_EJT, Utils::RelativeTime(Now()) << " I/O device inject " <<
      view->object_->get_oid() << ", ijt " << Utils::RelativeTime(view->get_ijt()));
}

View* _Mem::inject_marker_value_from_io_device(
  Code* obj, Code* prop, Atom val, Timestamp after, Timestamp before,
  View::SyncMode sync_mode, Code* group) 
{
  if (!obj || !prop)
    // We don't expect this, but sanity check.
    return NULL;

  Code *object = new LObject(this);
  object->code(0) = Atom::Marker(GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0); // obj
  object->code(2) = Atom::RPointer(1); // prop
  object->code(3) = val;
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);

  return inject_fact_from_io_device(object, after, before, sync_mode, group);
}

View* _Mem::inject_marker_value_from_io_device(
  Code* obj, Code* prop, Code* val, Timestamp after, Timestamp before,
  View::SyncMode sync_mode, Code* group)
{
  if (!obj || !prop)
    // We don't expect this, but sanity check.
    return NULL;

  Code *object = new LObject(this);
  object->code(0) = Atom::Marker(GetOpcode("mk.val"), 4); // Caveat: arity does not include the opcode.
  object->code(1) = Atom::RPointer(0); // obj
  object->code(2) = Atom::RPointer(1); // prop
  object->code(3) = Atom::RPointer(2); // val
  object->code(4) = Atom::Float(1); // psln_thr.

  object->set_reference(0, obj);
  object->set_reference(1, prop);
  object->set_reference(2, val);

  return inject_fact_from_io_device(object, after, before, sync_mode, group);
}

View* _Mem::inject_fact_from_io_device(
  Code* object, Timestamp after, Timestamp before, View::SyncMode sync_mode,
  Code* group)
{
  // Build a fact.
  Code* fact = new Fact(object, after, before, 1, 1);

  // Build a view for the fact.
  View *view = new View(sync_mode, after, 1, 1, group, NULL, fact);

  // Inject the view.
  inject_from_io_device(view);
  return view;
}

void _Mem::inject(View *view, bool is_from_io_device) {

  if (view->object_->is_invalidated())
    return;

  Group *host = view->get_host();

  if (host->is_invalidated())
    return;

  auto now = Now();
  auto ijt = view->get_ijt();

  if (view->object_->is_registered()) { // existing object.

    if (ijt <= now)
      inject_existing_object(view, view->object_, host);
    else {

      P<TimeJob> j = new EInjectionJob(view, ijt);
      time_job_queue_->push(j);
    }
  } else { // new object.

    if (ijt <= now) {
      inject_new_object(view);

      if (view->object_->code(0).asOpcode() == Opcodes::Fact) {
        Goal *goal = ((_Fact *)view->object_)->get_goal();
        if (goal && goal->is_drive())
          // Log the injection of a drive, presumably from a program, possibly delayed by a TimeJob.
          // The view injection time may be different than now, so log it too.
          // In general, the problem is how to relate the reduction output event to the injection event
          // which could be delayed.
          OUTPUT_LINE(MDL_IN, Utils::RelativeTime(Now()) << " -> drive " <<
            view->object_->get_oid() << ", ijt " << Utils::RelativeTime(view->get_ijt()));
      }
    }
    else {

      P<TimeJob> j = new InjectionJob(view, ijt, is_from_io_device);
      time_job_queue_->push(j);
    }
  }
}

void _Mem::inject_async(View *view) {

  if (view->object_->is_invalidated())
    return;

  Group *host = view->get_host();

  if (host->is_invalidated())
    return;

  auto now = Now();
  auto ijt = view->get_ijt();

  if (ijt <= now) {

    P<_ReductionJob> j = new AsyncInjectionJob(view);
    reduction_job_queue_->push(j);
  } else {

    if (view->object_->is_registered()) { // existing object.

      P<TimeJob> j = new EInjectionJob(view, ijt);
      time_job_queue_->push(j);
    } else {

      P<TimeJob> j = new InjectionJob(view, ijt, false);
      time_job_queue_->push(j);
    }
  }
}

void _Mem::inject_hlps(vector<View *> views, Group *destination) {

  vector<View *>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view)
    bind(*view);

  destination->inject_hlps(views);
}

void _Mem::inject_notification(View *view, bool lock) { // no notification for notifications; no cov.
                                                            // notifications are ephemeral: they are not held by the marker sets of the object they refer to; this implies no propagation of saliency changes trough notifications.
  Group *host = view->get_host();

  bind(view);

  host->inject_notification(view, lock);
}

////////////////////////////////////////////////////////////////

void _Mem::register_reduction_job_latency(microseconds latency) {

  reduction_jobCS_.enter();
  ++reduction_job_count_;
  reduction_job_avg_latency_ += latency;
  reduction_jobCS_.leave();
}
void _Mem::register_time_job_latency(microseconds latency) {

  time_jobCS_.enter();
  ++time_job_count_;
  time_job_avg_latency_ += latency;
  time_jobCS_.leave();
}

void _Mem::inject_perf_stats() {

  reduction_jobCS_.enter();
  time_jobCS_.enter();

  microseconds d_reduction_job_avg_latency;
  if (reduction_job_count_ > 0) {

    reduction_job_avg_latency_ /= reduction_job_count_;
    d_reduction_job_avg_latency = reduction_job_avg_latency_ - _reduction_job_avg_latency_;
  } else
    reduction_job_avg_latency_ = d_reduction_job_avg_latency = microseconds(0);

  microseconds d_time_job_avg_latency;
  if (time_job_count_ > 0) {

    time_job_avg_latency_ /= time_job_count_;
    d_time_job_avg_latency = time_job_avg_latency_ - _time_job_avg_latency_;
  } else
    time_job_avg_latency_ = d_time_job_avg_latency = microseconds(0);

  Code *perf = new Perf(reduction_job_avg_latency_, d_reduction_job_avg_latency, time_job_avg_latency_, d_time_job_avg_latency);

  // reset stats.
  reduction_job_count_ = time_job_count_ = 0;
  _reduction_job_avg_latency_ = reduction_job_avg_latency_;
  _time_job_avg_latency_ = time_job_avg_latency_;

  time_jobCS_.leave();
  reduction_jobCS_.leave();

  // inject f->perf in stdin.
  auto now = Now();
  Code *f_perf = new Fact(perf, now, now + perf_sampling_period_, 1, 1);
  View *view = new View(View::SYNC_ONCE, now, 1, 1, stdin_, NULL, f_perf); // sync front, sln=1, res=1.
  inject(view);
}

////////////////////////////////////////////////////////////////

void _Mem::propagate_sln(Code *object, float32 change, float32 source_sln_thr) {

  // apply morphed change to views.
  // loops are prevented within one call, but not accross several upr:
  // - feedback can happen, i.e. m:(mk o1 o2); o1.vw.g propag -> o1 propag ->m propag -> o2 propag o2.vw.g, next upr in g, o2 propag -> m propag -> o1 propag -> o1,vw.g: loop spreading accross several upr.
  // - to avoid this, have the psln_thr set to 1 in o2: this is applicaton-dependent.
  object->acq_views();

  if (object->views_.size() == 0) {

    object->invalidate();
    object->rel_views();
    return;
  }

  unordered_set<_View *, _View::Hash, _View::Equal>::const_iterator it;
  for (it = object->views_.begin(); it != object->views_.end(); ++it) {

    float32 morphed_sln_change = View::MorphChange(change, source_sln_thr, ((View*)*it)->get_host()->get_sln_thr());
    if (morphed_sln_change != 0)
      ((View*)*it)->get_host()->pending_operations_.push_back(new Group::Mod(((View*)*it)->get_oid(), VIEW_SLN, morphed_sln_change));
  }
  object->rel_views();
}

void _Mem::unpack_hlp(Code *hlp) { // produces a new object (featuring a set of pattern objects instread of a set of embedded pattern expressions) and add it as a hidden reference to the original (still packed) hlp.

  Code *unpacked_hlp = new LObject(); // will not be transmitted nor decompiled.

  for (uint16 i = 0; i < hlp->code_size(); ++i)
    unpacked_hlp->code(i) = hlp->code(i);

  uint16 pattern_set_index = hlp->code(HLP_OBJS).asIndex();
  uint16 pattern_count = hlp->code(pattern_set_index).getAtomCount();
  for (uint16 i = 1; i <= pattern_count; ++i) { // init the new references with the facts; turn the exisitng i-ptrs into r-ptrs.

    Code *fact = unpack_fact(hlp, hlp->code(pattern_set_index + i).asIndex());
    unpacked_hlp->add_reference(fact);
    unpacked_hlp->code(pattern_set_index + i) = Atom::RPointer(unpacked_hlp->references_size() - 1);
  }

  uint16 group_set_index = hlp->code(HLP_OUT_GRPS).asIndex();
  uint16 group_count = hlp->code(group_set_index++).getAtomCount();
  for (uint16 i = 0; i < group_count; ++i) { // append the out_groups to the new references; adjust the exisitng r-ptrs.

    unpacked_hlp->add_reference(hlp->get_reference(hlp->code(group_set_index + i).asIndex()));
    unpacked_hlp->code(group_set_index + i) = Atom::RPointer(unpacked_hlp->references_size() - 1);
  }

  uint16 invalid_point = pattern_set_index + pattern_count + 1; // index of what is after set of the patterns.
  uint16 valid_point = hlp->code(HLP_FWD_GUARDS).asIndex(); // index of the first atom that does not belong to the patterns.
  uint16 invalid_zone_length = valid_point - invalid_point;
  for (uint16 i = valid_point; i < hlp->code_size(); ++i) { // shift the valid code upward; adjust i-ptrs.

    Atom h_atom = hlp->code(i);
    switch (h_atom.getDescriptor()) {
    case Atom::I_PTR:
      unpacked_hlp->code(i - invalid_zone_length) = Atom::IPointer(h_atom.asIndex() - invalid_zone_length);
      break;
    case Atom::ASSIGN_PTR:
      unpacked_hlp->code(i - invalid_zone_length) = Atom::AssignmentPointer(h_atom.asAssignmentIndex(), h_atom.asIndex() - invalid_zone_length);
      break;
    default:
      unpacked_hlp->code(i - invalid_zone_length) = h_atom;
      break;
    }
  }

  // adjust set indices.
  unpacked_hlp->code(HLP_FWD_GUARDS) = Atom::IPointer(hlp->code(HLP_FWD_GUARDS).asIndex() - invalid_zone_length);
  unpacked_hlp->code(HLP_BWD_GUARDS) = Atom::IPointer(hlp->code(HLP_BWD_GUARDS).asIndex() - invalid_zone_length);
  unpacked_hlp->code(HLP_OUT_GRPS) = Atom::IPointer(hlp->code(HLP_OUT_GRPS).asIndex() - invalid_zone_length);

  uint16 unpacked_code_length = hlp->code_size() - invalid_zone_length;
  unpacked_hlp->resize_code(unpacked_code_length);
  hlp->add_reference(unpacked_hlp);
}

Code *_Mem::unpack_fact(Code *hlp, uint16 fact_index) {

  Code *fact = new LObject();
  Code *fact_object;
  uint16 fact_size = hlp->code(fact_index).getAtomCount() + 1;
  for (uint16 i = 0; i < fact_size; ++i) {

    Atom h_atom = hlp->code(fact_index + i);
    switch (h_atom.getDescriptor()) {
    case Atom::I_PTR:
      fact->code(i) = Atom::RPointer(fact->references_size());
      fact_object = unpack_fact_object(hlp, h_atom.asIndex());
      fact->add_reference(fact_object);
      break;
    case Atom::R_PTR: // case of a reference to an exisitng object.
      fact->code(i) = Atom::RPointer(fact->references_size());
      fact->add_reference(hlp->get_reference(h_atom.asIndex()));
      break;
    default:
      fact->code(i) = h_atom;
      break;
    }
  }

  return fact;
}

Code *_Mem::unpack_fact_object(Code *hlp, uint16 fact_object_index) {

  Code *fact_object = new LObject();
  _unpack_code(hlp, fact_object_index, fact_object, fact_object_index);
  return fact_object;
}

void _Mem::_unpack_code(Code *hlp, uint16 fact_object_index, Code *fact_object, uint16 read_index) {

  Atom h_atom = hlp->code(read_index);
  uint16 code_size = h_atom.getAtomCount() + 1;
  uint16 write_index = read_index - fact_object_index;
  for (uint16 i = 0; i < code_size; ++i) {

    switch (h_atom.getDescriptor()) {
    case Atom::R_PTR:
      fact_object->code(write_index + i) = Atom::RPointer(fact_object->references_size());
      fact_object->add_reference(hlp->get_reference(h_atom.asIndex()));
      break;
    case Atom::I_PTR:
      fact_object->code(write_index + i) = Atom::IPointer(h_atom.asIndex() - fact_object_index);
      _unpack_code(hlp, fact_object_index, fact_object, h_atom.asIndex());
      break;
    default:
      fact_object->code(write_index + i) = h_atom;
      break;
    }

    h_atom = hlp->code(read_index + i + 1);
  }
}

void _Mem::pack_hlp(Code *hlp) const { // produces a new object where a set of pattern objects is transformed into a packed set of pattern code.

  Code *unpacked_hlp = clone(hlp);

  vector<Atom> trailing_code; // copy of the original code (the latter will be overwritten by packed facts).
  uint16 trailing_code_index = hlp->code(HLP_FWD_GUARDS).asIndex();
  for (uint16 i = trailing_code_index; i < hlp->code_size(); ++i)
    trailing_code.push_back(hlp->code(i));

  uint16 group_set_index = hlp->code(HLP_OUT_GRPS).asIndex();
  uint16 group_count = hlp->code(group_set_index).getAtomCount();

  vector<P<Code> > references;

  uint16 pattern_set_index = hlp->code(HLP_OBJS).asIndex();
  uint16 pattern_count = hlp->code(pattern_set_index).getAtomCount();
  uint16 insertion_point = pattern_set_index + pattern_count + 1; // point from where compacted code is to be inserted.
  uint16 extent_index = insertion_point;
  for (uint16 i = 0; i < pattern_count; ++i) {

    Code *pattern_object = hlp->get_reference(i);
    hlp->code(pattern_set_index + i + 1) = Atom::IPointer(extent_index);
    pack_fact(pattern_object, hlp, extent_index, &references);
  }

  uint16 inserted_zone_length = extent_index - insertion_point;

  for (uint16 i = 0; i < trailing_code.size(); ++i) { // shift the trailing code downward; adjust i-ptrs.

    Atom t_atom = trailing_code[i];
    switch (t_atom.getDescriptor()) {
    case Atom::I_PTR:
      hlp->code(i + extent_index) = Atom::IPointer(t_atom.asIndex() + inserted_zone_length);
      break;
    case Atom::ASSIGN_PTR:
      hlp->code(i + extent_index) = Atom::AssignmentPointer(t_atom.asAssignmentIndex(), t_atom.asIndex() + inserted_zone_length);
      break;
    default:
      hlp->code(i + extent_index) = t_atom;
      break;
    }
  }

  // adjust set indices.
  hlp->code(HLP_FWD_GUARDS) = Atom::IPointer(hlp->code(HLP_FWD_GUARDS).asIndex() + inserted_zone_length);
  hlp->code(HLP_BWD_GUARDS) = Atom::IPointer(hlp->code(HLP_BWD_GUARDS).asIndex() + inserted_zone_length);
  hlp->code(HLP_OUT_GRPS) = Atom::IPointer(hlp->code(HLP_OUT_GRPS).asIndex() + inserted_zone_length);

  group_set_index += inserted_zone_length;
  for (uint16 i = 1; i <= group_count; ++i) { // append the out_groups to the new references; adjust the exisitng r-ptrs.

    references.push_back(hlp->get_reference(hlp->code(group_set_index + i).asIndex()));
    hlp->code(group_set_index + i) = Atom::RPointer(references.size() - 1);
  }

  hlp->set_references(references);

  hlp->add_reference(unpacked_hlp); // hidden reference.
}

void _Mem::pack_fact(Code *fact, Code *hlp, uint16 &write_index, vector<P<Code> > *references) {

  uint16 extent_index = write_index + fact->code_size();
  for (uint16 i = 0; i < fact->code_size(); ++i) {

    Atom p_atom = fact->code(i);
    switch (p_atom.getDescriptor()) {
    case Atom::R_PTR: // transform into a i_ptr and pack the pointed object.
      hlp->code(write_index) = Atom::IPointer(extent_index);
      pack_fact_object(fact->get_reference(p_atom.asIndex()), hlp, extent_index, references);
      ++write_index;
      break;
    default:
      hlp->code(write_index) = p_atom;
      ++write_index;
      break;
    }
  }
  write_index = extent_index;
}

void _Mem::pack_fact_object(Code *fact_object, Code *hlp, uint16 &write_index, vector<P<Code> > *references) {

  uint16 extent_index = write_index + fact_object->code_size();
  uint16 offset = write_index;
  for (uint16 i = 0; i < fact_object->code_size(); ++i) {

    Atom p_atom = fact_object->code(i);
    switch (p_atom.getDescriptor()) {
    case Atom::R_PTR: { // append this reference to the hlp's if not already there.
      Code *reference = fact_object->get_reference(p_atom.asIndex());
      bool found = false;
      for (uint16 i = 0; i < references->size(); ++i) {

        if ((*references)[i] == reference) {

          hlp->code(write_index) = Atom::RPointer(i);
          found = true;
          break;
        }
      }
      if (!found) {

        hlp->code(write_index) = Atom::RPointer(references->size());
        references->push_back(reference);
      }
      ++write_index;
      break;
    }case Atom::I_PTR: // offset the ptr by write_index. PB HERE.
      hlp->code(write_index) = Atom::IPointer(offset + p_atom.asIndex());
      ++write_index;
      break;
    default:
      hlp->code(write_index) = p_atom;
      ++write_index;
      break;
    }
  }
}

Code *_Mem::clone(Code *original) const { // shallow copy; oid not copied.

  Code *_clone = build_object(original->code(0));
  uint16 opcode = original->code(0).asOpcode();
  if (opcode == Opcodes::Ont || opcode == Opcodes::Ent)
    return original;

  for (uint16 i = 0; i < original->code_size(); ++i)
    _clone->code(i) = original->code(i);
  for (uint16 i = 0; i < original->references_size(); ++i)
    _clone->add_reference(original->get_reference(i));
  return _clone;
}

bool _Mem::matches_axiom(Code* obj)
{
  for (auto axiom = axiom_values_.begin(); axiom != axiom_values_.end(); ++axiom) {
    if (_Fact::MatchObject(obj, *axiom))
      return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////

r_comp::Image *_Mem::get_models() {

  r_comp::Image *image = new r_comp::Image();
  image->timestamp_ = Now();

  r_code::list<P<Code> > models;
  ModelBase::Get()->get_models(models); // protected by ModelBase.
  image->add_objects(models);

  return image;
}

////////////////////////////////////////////////////////////////

MemStatic::MemStatic() : _Mem(), last_oid_(-1) {
}

MemStatic::~MemStatic() {
}

void MemStatic::bind(View *view) {

  Code *object = view->object_;
  object->views_.insert(view);
  objectsCS_.enter();
  object->set_oid(++last_oid_);
  if (object->code(0).getDescriptor() == Atom::NULL_PROGRAM) {

    objectsCS_.leave();
    return;
  }
  int32 location;
  objects_.push_back(object, location);
  object->set_strorage_index(location);
  objectsCS_.leave();
}
void MemStatic::set_last_oid(int32 oid) {

  last_oid_ = oid;
}

void MemStatic::delete_object(r_code::Code *object) { // called only if the object is registered, i.e. has a valid storage index.

  if (deleted_)
    return;

  // keep_invalidated_objects_ is true if settings.xml has
  // get_objects="yes_with_invalidated", in which case don't erase.
  if (!keep_invalidated_objects_) {
    objectsCS_.enter();
    objects_.erase(object->get_storage_index());
    objectsCS_.leave();
  }
}

r_comp::Image *MemStatic::get_objects(bool include_invalidated) {

  r_comp::Image *image = new r_comp::Image();
  image->timestamp_ = Now();

  objectsCS_.enter();
  image->add_objects(objects_, include_invalidated);
  objectsCS_.leave();

  return image;
}

////////////////////////////////////////////////////////////////

MemVolatile::MemVolatile() : _Mem(), last_oid_(-1) {
}

MemVolatile::~MemVolatile() {
}

uint32 MemVolatile::get_oid() {

  return Atomic::Increment32(&last_oid_);
}

void MemVolatile::set_last_oid(int32 oid) {

  last_oid_ = oid;
}

void MemVolatile::bind(View *view) {

  Code *object = view->object_;
  object->views_.insert(view);
  object->set_oid(get_oid());
}
}
