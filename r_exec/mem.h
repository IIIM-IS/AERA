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

#ifndef mem_h
#define mem_h

#include <sstream>
#include "reduction_core.h"
#include "time_core.h"
#include "pgm_overlay.h"
#include "binding_map.h"
#include "dll.h"

#include <list>

#include "../r_code/list.h"
#include "../r_comp/segments.h"

#include "../submodules/CoreLibrary/CoreLibrary/pipe.h"


namespace r_exec {

/** The rMem.
 * Maintains 2 pipes of jobs (injection, update, etc.). each job is processed asynchronously by instances of ReductionCore and TimeCore.
 * Pipes and threads are created at starting time and deleted at stopping time.
 * Groups and IPGMControllers are cleared up when only held by jobs;
 * - when a group is not projected anywhere anymore, it is invalidated (it releases all its views) and when a job attempts an update, the latter is cancelled.
 * - when a reduction core attempts to perform a reduction for an ipgm-controller that is not projected anywhere anymore, the reduction is cancelled.
 * In addition:
 * - when an object is scheduled for injection and the target group does not exist anymore (or is invalidated), the injection is cancelled.
 * - when an object is scheduled for propagation of sln changes and has no view anymore, the operation is cancelled.
 */
class r_exec_dll _Mem :
  public r_code::Mem {
public:
  typedef enum {
    NOT_STARTED = 0,
    RUNNING = 1,
    STOPPED = 2
  }State;
protected:
  // Parameters::Init.
  std::chrono::microseconds base_period_;
  uint32 reduction_core_count_;
  uint32 time_core_count_;

  // Parameters::System.
  float32 mdl_inertia_sr_thr_;
  uint32 mdl_inertia_cnt_thr_;
  float32 tpx_dsr_thr_;
  std::chrono::microseconds min_sim_time_horizon_;
  std::chrono::microseconds max_sim_time_horizon_;
  float32 sim_time_horizon_factor_;
  std::chrono::microseconds tpx_time_horizon_;
  std::chrono::microseconds perf_sampling_period_;
  float32 float_tolerance_;
  std::chrono::microseconds time_tolerance_;
  std::chrono::microseconds primary_thz_;
  std::chrono::microseconds secondary_thz_;

  // Parameters::Debug.
  bool debug_;
  uint32 ntf_mk_res_;
  uint32 goal_pred_success_res_;
  bool keep_invalidated_objects_;

  // Parameters::Run.
  uint32 probe_level_;
  bool enable_assumptions_;

  PipeNN<P<_ReductionJob>, 1024> *reduction_job_queue_;
  PipeNN<P<TimeJob>, 1024> *time_job_queue_;
  ReductionCore **reduction_cores_;
  TimeCore **time_cores_;
  TimeJob::Compare time_job_compare_;

  // Performance stats.
  uint32 reduction_job_count_;
  std::chrono::microseconds reduction_job_avg_latency_; // latency: popping time.-pushing time; the lower the better.
  std::chrono::microseconds _reduction_job_avg_latency_; // previous value.
  uint32 time_job_count_;
  std::chrono::microseconds time_job_avg_latency_; // latency: deadline-the time the job is popped from the pipe; if <0, not registered (as it is too late for action); the higher the better.
  std::chrono::microseconds _time_job_avg_latency_; // previous value.

  CriticalSection time_jobCS_;
  CriticalSection reduction_jobCS_;

  uint32 core_count_;
  CriticalSection core_countCS_;

  State state_;
  CriticalSection stateCS_;
  Semaphore *stop_sem_; // blocks the rMem until all cores terminate.

  r_code::list<P<r_code::Code> > objects_; // store objects in order of injection: holds the initial objects (and dynamically created ones if MemStatic is used).

  P<Group> root_; // holds everything.
  r_code::Code *stdin_;
  r_code::Code *stdout_;
  r_code::Code *self_;

  void reset(); // clear the content of the mem.

  std::vector<Group *> initial_groups_; // convenience; cleared after start();

  void store(r_code::Code *object);
  virtual void set_last_oid(int32 oid) = 0;
  virtual void bind(View *view) = 0;

  bool deleted_;

  static const uint32 RUNTIME_OUTPUT_STREAM_COUNT = 10;
  std::ostream *runtime_output_streams_[RUNTIME_OUTPUT_STREAM_COUNT];
  // Use default_runtime_output_stream_ if runtime_output_streams_[i] is NULL. For no output,
  // runtime_output_streams_[i] will be set to new NullOStream().
  std::ostream *default_runtime_output_stream_;

  std::vector<P<r_code::Code> > axiom_values_;

  _Mem();

  static void _unpack_code(r_code::Code *hlp, uint16 fact_object_index, r_code::Code *fact_object, uint16 read_index);
public:
  static _Mem *Get() { return (_Mem *)Mem::Get(); }

  typedef enum {
    STDIN = 0,
    STDOUT = 1
  }STDGroupID;

  virtual ~_Mem();

  void init(std::chrono::microseconds base_period,
    uint32 reduction_core_count,
    uint32 time_core_count,
    float32 mdl_inertia_sr_thr,
    uint32 mdl_inertia_cnt_thr,
    float32 tpx_dsr_thr,
    std::chrono::microseconds min_sim_time_horizon,
    std::chrono::microseconds max_sim_time_horizon,
    float32 sim_time_horizon_factor,
    std::chrono::microseconds tpx_time_horizon,
    std::chrono::microseconds perf_sampling_period,
    float32 float_tolerance,
    std::chrono::microseconds time_tolerance,
    std::chrono::microseconds primary_thz,
    std::chrono::microseconds secondary_thz,
    bool debug,
    uint32 ntf_mk_res,
    uint32 goal_pred_success_res,
    uint32 probe_level,
    uint32 traces,
    bool enable_assumptions,
    bool keep_invalidated_objects);

  /**
   * Get the sampling period, which is 2 * base_period from settings.xml. This should
   * match sampling_period in user.classes.replicode.
   */
  std::chrono::microseconds get_sampling_period() const { return 2 * base_period_; }
  uint64 get_probe_level() const { return probe_level_; }
  bool get_enable_assumptions() const { return enable_assumptions_; }
  uint32 get_reduction_core_count() const { return reduction_core_count_; }
  uint32 get_time_core_count() const { return time_core_count_; }
  float32 get_mdl_inertia_sr_thr() const { return mdl_inertia_sr_thr_; }
  uint32 get_mdl_inertia_cnt_thr() const { return mdl_inertia_cnt_thr_; }
  float32 get_tpx_dsr_thr() const { return tpx_dsr_thr_; }
  std::chrono::microseconds get_min_sim_time_horizon() const { return min_sim_time_horizon_; }
  std::chrono::microseconds get_max_sim_time_horizon() const { return max_sim_time_horizon_; }
  std::chrono::microseconds get_sim_time_horizon(std::chrono::microseconds horizon) const { 
    return std::chrono::microseconds((int64)(horizon.count() * sim_time_horizon_factor_)); 
  }
  std::chrono::microseconds get_sim_time_horizon(Timestamp::duration horizon) const { 
    return get_sim_time_horizon(std::chrono::duration_cast<std::chrono::microseconds>(horizon));
  }
  std::chrono::microseconds get_tpx_time_horizon() const { return tpx_time_horizon_; }
  std::chrono::microseconds get_primary_thz() const { return primary_thz_; }
  std::chrono::microseconds get_secondary_thz() const { return secondary_thz_; }

  bool get_debug() const { return debug_; }
  uint32 get_ntf_mk_res() const { return ntf_mk_res_; }
  uint32 get_goal_pred_success_res(Group *host, Timestamp now, Timestamp::duration time_to_live) const {

    if (debug_)
      return goal_pred_success_res_;
    if (time_to_live.count() == 0)
      return 1;
    return r_code::Utils::GetResilience(now, time_to_live, host->get_upr());
  }

  r_code::Code *get_root() const;
  r_code::Code *get_stdin() const;
  r_code::Code *get_stdout() const;
  r_code::Code *get_self() const;

  State check_state(); // called by delegates after waiting in case stop() is called in the meantime.
  void start_core(); // called upon creation of a delegate.
  void shutdown_core(); // called upon completion of a delegate's task.

  virtual bool load(std::vector<r_code::Code *> *objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid); // call before start; no mod/set/eje will be executed (only inj);
                                                                                                                  // return false on error.
  Timestamp start(); // return the starting time.

  /**
   * When reduction and core count == 0, start() does not start any core threads,
   * so call this instead of Thread::Sleep(run_time) to run in the current
   * thread using "diagnostic time".As opposed to real time which uses Time::Get,
   * diagnostic time uses get_diagnostic_time_now() which simply returns diagnostic_time_now_.
   * (The main() function should call r_exec::Init where time_base is get_diagnostic_time_now.)
   * So, run_in_diagnostic_time updates diagnostic_time_now_ based on the next time job(which always
   * runs on time). This way, the return value of Now() does not move with real time, but moves
   * step-by-step when diagnostic_time_now_ is updated, making it possible to set break points
   * and diagnose the program.
   * \param run_time The number of milliseconds (in diagnostic time) to run for.
   */
  void run_in_diagnostic_time(std::chrono::milliseconds run_time);
  static Timestamp diagnostic_time_now_;
  static Timestamp get_diagnostic_time_now();
  // Tell an inheriting class (with inject) when the time is changed.
  virtual void on_diagnostic_time_tick();
  void stop(); // after stop() the content is cleared and one has to call load() and start() again.

  // Internal core processing ////////////////////////////////////////////////////////////////

  P<_ReductionJob> pop_reduction_job(bool waitForItem = true);
  void push_reduction_job(_ReductionJob *j);
  P<TimeJob> pop_time_job(bool waitForItem = true);
  void push_time_job(TimeJob *j);

  /**
   * This is called from the I/O device to call the normal inject(view), then 
   * log the injection event.
   * \param view The View for inject(view).
   */
  void inject_from_io_device(View *view);

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [sync_mode after 1 1 group nil]
   * where val is a simple Atom.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The Atom value for the mk.val, such as Atom::Float(1).
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param sync_mode The view sync mode, such as View::SYNC_PERIODIC.
   * \param group The group of the view, such as get_stdin().
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, Atom val, Timestamp after, Timestamp before,
    View::SyncMode sync_mode, r_code::Code* group);

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [sync_mode after 1 1 stdin nil]
   * where val is a simple Atom.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The Atom value for the mk.val, such as Atom::Float(1).
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param sync_mode The view sync mode, such as View::SYNC_PERIODIC.
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, Atom val, Timestamp after, Timestamp before,
    View::SyncMode sync_mode)
  {
    return inject_marker_value_from_io_device(obj, prop, val, after, before, sync_mode, get_stdin());
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 group nil]
   * where val is a simple Atom.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The Atom value for the mk.val, such as Atom::Float(1).
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param group The group of the view, such as get_stdin().
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, Atom val, Timestamp after, Timestamp before, r_code::Code* group)
  {
    return inject_marker_value_from_io_device(
      obj, prop, val, after, before, View::SYNC_PERIODIC, group);
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 stdin nil]
   * where val is a simple Atom.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The Atom value for the mk.val, such as Atom::Float(1).
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, Atom val, Timestamp after, Timestamp before)
  {
    return inject_marker_value_from_io_device(
      obj, prop, val, after, before, View::SYNC_PERIODIC, get_stdin());
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [sync_mode after 1 1 group nil]
   * where val is a referenced Code object.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The value for the mk.val, which is an existing Code object. This will
   * add it to the references of the mkval.
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param sync_mode The view sync mode, such as View::SYNC_PERIODIC.
   * \param group The group of the view, such as get_stdin().
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, r_code::Code* val, Timestamp after, Timestamp before,
    View::SyncMode sync_mode, r_code::Code* group);

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [sync_mode after 1 1 stdin nil]
   * where val is a referenced Code object.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The value for the mk.val, which is an existing Code object. This will
   * add it to the references of the mkval.
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param sync_mode The view sync mode, such as View::SYNC_PERIODIC.
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, r_code::Code* val, Timestamp after, Timestamp before,
    View::SyncMode sync_mode)
  {
    return inject_marker_value_from_io_device(obj, prop, val, after, before, sync_mode, get_stdin());
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 group nil]
   * where val is a referenced Code object.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The value for the mk.val, which is an existing Code object. This will
   * add it to the references of the mkval.
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param group The group of the view, such as get_stdin().
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, r_code::Code* val, Timestamp after, Timestamp before, r_code::Code* group)
  {
    return inject_marker_value_from_io_device(
      obj, prop, val, after, before, View::SYNC_PERIODIC, group);
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 stdin nil]
   * where val is a referenced Code object.
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param obj The object for the mk.val.
   * \param prop The property for the mk.val.
   * \param val The value for the mk.val, which is an existing Code object. This will
   * add it to the references of the mkval.
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \return The created View.
   */
  View* inject_marker_value_from_io_device(
    r_code::Code* obj, r_code::Code* prop, r_code::Code* val, Timestamp after, Timestamp before)
  {
    return inject_marker_value_from_io_device(
      obj, prop, val, after, before, View::SYNC_PERIODIC, get_stdin());
  }

  /**
   * Inject (fact object after before 1 1)
   * [sync_mode after 1 1 group nil]
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param object The fact's object, which you must allocated and construct. (For mk.val, use one
   * of the methods for injectMarkerValue...)
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param sync_mode The view sync mode, such as View::SYNC_PERIODIC.
   * \param group The group of the view, such as get_stdin().
   * \return The created View.
   */
  View* inject_fact_from_io_device(
    r_code::Code* object, Timestamp after, Timestamp before, View::SyncMode sync_mode,
    r_code::Code* group);

  /**
   * Inject (fact object after before 1 1)
   * [SYNC_PERIODIC after 1 1 group nil]
   * This is called from the I/O device to call the normal inject(view), then
   * log the injection event.
   * \param object The fact's object, which you must allocated and construct. (For mk.val, use one
   * of the methods for injectMarkerValue...)
   * \param after The start of the fact time interval.
   * \param before The end of the fact time interval.
   * \param group The group of the view, such as get_stdin().
   * \return The created View.
   */
  View* inject_fact_from_io_device(r_code::Code* object, Timestamp after, Timestamp before, r_code::Code* group) {
    return inject_fact_from_io_device(
      object, after, before, View::SYNC_PERIODIC, group);
  }

  // Called upon successful reduction.
  /**
   * Call bind which assigns an OID to the view's object and inject the view. 
   * However, if the view's injection time is later than now, then create
   * an InjectionJob which will call bind and inject the view at the injection time.
   * \param view The View to inject.
   * \param is_from_io_device True if this is called from inject_from_io_device().
   * This is only needed to pass to the InjectionJob so that it will log the 
   * I/O device inject.
   */
  void inject(View *view, bool is_from_io_device = false);
  void inject_async(View *view);
  void inject_new_object(View *view);
  void inject_existing_object(View *view, r_code::Code *object, Group *host);
  void inject_null_program(Controller *c, Group *group, std::chrono::microseconds time_to_live, bool take_past_inputs); // build a view v (ijt=now, act=1, sln=0, res according to time_to_live in the group), attach c to v, inject v in the group.
  void inject_hlps(std::vector<View *> views, Group *destination);
  void inject_notification(View *view, bool lock);
  virtual r_code::Code *check_existence(r_code::Code *object) = 0; // returns the existing object if any, or object otherwise: in the latter case, packing may occur.

  void propagate_sln(r_code::Code *object, float32 change, float32 source_sln_thr);

  // Called by groups.
  void inject_copy(View *view, Group *destination); // for cov; NB: no cov for groups, r-groups, models, pgm or notifications.

  // Called by cores.
  void register_reduction_job_latency(std::chrono::microseconds latency);
  void register_reduction_job_latency(Timestamp::duration latency) {
    register_reduction_job_latency(std::chrono::duration_cast<std::chrono::microseconds>(latency));
  };
  void register_time_job_latency(std::chrono::microseconds latency);
  void register_time_job_latency(Timestamp::duration latency) {
    register_time_job_latency(std::chrono::duration_cast<std::chrono::microseconds>(latency)); 
  };
  void inject_perf_stats();

  // rMem to rMem.
  // The view must contain the destination group (either stdin or stdout) as its grp member.
  // To be redefined by object transport aware subcalsses.
  virtual void eject(View *view, uint16 node_id);

  /**
   * This is called by the program controller to eject a command from rMem to the 
   * I/O device. This method should be redefined by object transport-aware subclasses.
   * \param command The command from the Replicode (cmd ...).
   * \return The given command if it is executed as-is, or a new command object of the command
   * that is actually executed. The program controller will make a fact from the command and
   * inject it as the efferent copy. However, if the command is not executed, then return NULL
   * and the program controller will put an anti-fact of the command in the mk.rdx reduction.
   */
  virtual r_code::Code* eject(r_code::Code *command);

  virtual r_code::Code *_build_object(Atom head) const = 0;
  virtual r_code::Code *build_object(Atom head) const = 0;

  // unpacking of high-level patterns: upon loading or reception.
  static void unpack_hlp(r_code::Code *hlp);
  static r_code::Code *unpack_fact(r_code::Code *hlp, uint16 fact_index);
  static r_code::Code *unpack_fact_object(r_code::Code *hlp, uint16 fact_object_index);

  // packing of high-level patterns: upon dynamic generation or transmission.
  void pack_hlp(r_code::Code *hlp) const;
  static void pack_fact(r_code::Code *fact, r_code::Code *hlp, uint16 &write_index, std::vector<P<r_code::Code> > *references);
  static void pack_fact_object(r_code::Code *fact_object, r_code::Code *hlp, uint16 &write_index, std::vector<P<r_code::Code> > *references);

  r_code::Code *clone(r_code::Code *original) const; // shallow copy.

  // External device I/O ////////////////////////////////////////////////////////////////
  virtual r_comp::Image *get_objects(bool include_invalidated = false) = 0; // create an image; fill with all objects; call only when stopped.
  r_comp::Image *get_models(); // create an image; fill with all models; call only when stopped.

  /**
   * Check if an object matches the value of a fact in the seed whose view is SYNC_AXIOM.
   * \param obj A fact's value such as (mk.val b essence ball 1).
   * \return True if the object matches an axiom.
   */
  bool matches_axiom(r_code::Code* obj);

  /**
   * This is called on starting the executive to adjust the timestamps of all the initial
   * user-supplied facts by adding now. Therefore, if a the initial fact is defined with a
   * timestamp of 100us, it is changed to now + 100us.
   * \param now The value to add to the timestamps of all the initial facts.
   * \param objects he list of objects to search for facts.
   */
  static void init_timings(Timestamp now, const r_code::list<P<r_code::Code>>& objects);

  //std::vector<uint64> timings_report; // debug facility.

  /**
   * Set the runtime output stream which Output returns when a trace level is NULL.
   * \param default_runtime_output_stream The stream.
   */
  void set_default_runtime_output_stream(std::ostream *default_runtime_output_stream) {
    default_runtime_output_stream_ = default_runtime_output_stream;
  }

  /**
   * Get the output stream for the trace level based on (_Mem::Get()->runtime_output_streams_[l].
   * If (_Mem::Get()->runtime_output_streams_[l] is NULL, use default_runtime_output_stream_.
   * \param l The TraceLevel.
   * \return A reference to the output stream, which may be a NullOStream if the bit in 
   * settings.xml "trace_levels" was zero.
   */
  static std::ostream &Output(TraceLevel l);
};

// _Mem that stores the objects as long as they are not invalidated.
class r_exec_dll MemStatic :
  public _Mem {
private:
  CriticalSection objectsCS_; // protects last_oid_ and objects_.
  uint32 last_oid_;
  void bind(View *view); // assigns an oid, stores view->object in objects if needed.
  void set_last_oid(int32 oid);
protected:
  MemStatic();
public:
  virtual ~MemStatic();

  void delete_object(r_code::Code *object); // erase the object from objects if needed.
  // return an image containing valid objects, or all objects if include_invalidated.
  r_comp::Image *get_objects(bool include_invalidated = false);
};

// _Mem that does not store objects.
class r_exec_dll MemVolatile :
  public _Mem {
private:
  volatile int32 last_oid_;
  uint32 get_oid();
  void bind(View *view); // assigns an oid (atomic operation).
  void set_last_oid(int32 oid);
protected:
  MemVolatile();
public:
  virtual ~MemVolatile();

  void delete_object(r_code::Code *object) {}

  r_comp::Image *get_objects(bool include_invalidated = false) { return NULL; }
};

/** O is the class of the objects held by the rMem (except groups and notifications):
 *    r_exec::LObject if non distributed, or
 *    RObject (see the integration project) when network-aware.
 * Notification objects and groups are instances of r_exec::LObject (they are not network-aware).
 * Objects are built at reduction time as r_exec:LObjects and packed into instances of O when O is network-aware.
 * S is the super-class.
 */
template<class O, class S> class MemExec :
  public S {
public:
  MemExec();
  virtual ~MemExec();

  // Called at load time.
  r_code::Code *build_object(r_code::SysObject *source) const;

  // Called at runtime.
  r_code::Code *_build_object(Atom head) const;
  r_code::Code *build_object(Atom head) const;

  // Executive device functions ////////////////////////////////////////////////////////

  r_code::Code *check_existence(r_code::Code *object);

  // Called by the communication device (I/O).
  void inject(O *object, View *view);
};

}

#include "mem.tpl.cpp"

#endif
