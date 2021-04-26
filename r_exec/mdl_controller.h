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

#ifndef mdl_controller_h
#define mdl_controller_h

#include "hlp_overlay.h"
#include "hlp_controller.h"
#include "p_monitor.h"
#include "factory.h"


namespace r_exec {

class MDLOverlay :
  public HLPOverlay {
protected:
  MDLOverlay(Controller *c, const HLPBindingMap *bindngs);
public:
  ~MDLOverlay();

  void load_patterns();

  virtual bool reduce(_Fact *input, Fact *f_p_f_imdl, MDLController *req_controller) = 0;
};

class PrimaryMDLOverlay :
  public MDLOverlay {
protected:
  bool check_simulated_chaining(HLPBindingMap *bm, Fact *f_imdl, Pred *prediction);
public:
  PrimaryMDLOverlay(Controller *c, const HLPBindingMap *bindngs);
  ~PrimaryMDLOverlay();

  bool reduce(_Fact *input, Fact *f_p_f_imdl, MDLController *req_controller);
};

class SecondaryMDLOverlay :
  public MDLOverlay {
public:
  SecondaryMDLOverlay(Controller *c, const HLPBindingMap *bindngs);
  ~SecondaryMDLOverlay();

  bool reduce(_Fact *input, Fact *f_p_f_imdl, MDLController *req_controller);
};

class MDLController;
class Requirements {
public:
  std::vector<P<MDLController> > controllers;
  P<_Fact> f_imdl; // f1 as in f0->pred->f1->imdl.
  bool chaining_was_allowed;
};
typedef std::pair<Requirements, Requirements> RequirementsPair; // first: wr, second: sr.

// Requirements don't monitor their predictions: they don't inject any; instead, they store a f->imdl in the controlled model controllers (both primary and secondary), thus, no success injected for the productions of requirements.
// Models controlled by requirements maintain for each prediction they make, a list of all the controllers of the requirements having allowed/inhibited said prediction.
// P-monitors (associated to non-requirement models) propagate the outcome to the controllers associated with the prediction they monitor.
//
// Predictions and goals are injected in the primary group only.
// Simulations are injected in the primary group only; no mk.rdx.
//
// Each time a prediction is made by a non-req model, a f->imdl is injected in both primary and secondary groups. If the input was a prediction, f->pred->f->imdl is injected instead.
// f->imdl can also be tagged as simulations.
//
// Successes and failures are injected only in output groups.
//
// If forward chaining is inhibited (by strong reqs with better cfd than weak reqs), predictions are still produced, but not injected (no mk.rdx): this is to allow the rating of the requirements.
class MDLController :
  public HLPController {
protected:
  class RequirementEntry : // use for requirements.
    public PredictedEvidenceEntry {
  public:
    P<MDLController> controller_; // of the requirement.
    bool chaining_was_allowed_;

    RequirementEntry();
    RequirementEntry(_Fact *f_p_f_imdl, MDLController *c, bool chaining_was_allowed); // f_imdl is f0 as in f0->pred->f1->imdl.

    bool is_out_of_range(Timestamp now) const { return (before_<now || after_>now); }
  };

  class RequirementCache {
  public:
    CriticalSection CS;
    r_code::list<RequirementEntry> positive_evidences;
    r_code::list<RequirementEntry> negative_evidences;
  };

  RequirementCache requirements_;
  RequirementCache simulated_requirements_;

  void _store_requirement(r_code::list<RequirementEntry> *cache, RequirementEntry &e);

  CriticalSection p_monitorsCS_;
  r_code::list<P<PMonitor> > p_monitors_;

  P<r_code::Code> lhs_;
  P<r_code::Code> rhs_;

  static const uint32 LHSController = 0;
  static const uint32 RHSController = 1;

  typedef enum {
    NOT_A_REQUIREMENT = 0,
    WEAK_REQUIREMENT = 1,
    STRONG_REQUIREMENT = 2
  } RequirementType;

  RequirementType is_requirement_;
  bool is_reuse_;
  bool is_cmd_;

  float32 get_cfd() const;

  CriticalSection active_requirementsCS_;
  UNORDERED_MAP<P<_Fact>, RequirementsPair, r_code::PHash<_Fact> > active_requirements_; // P<_Fact>: f1 as in f0->pred->f1->imdl; requirements having allowed the production of prediction; first: wr, second: sr.

  template<class C> void reduce_cache(Fact *f_p_f_imdl, MDLController *controller) { // fwd; controller is the controller of the requirement which produced f_p_f_imdl.

    BatchReductionJob<C, Fact, MDLController> *j = new BatchReductionJob<C, Fact, MDLController>((C *)this, f_p_f_imdl, controller);
    _Mem::Get()->pushReductionJob(j);
  }

  template<class E> void reduce_cache(Cache<E> *cache, Fact *f_p_f_imdl, MDLController *controller) {

    cache->CS.enter();
    auto now = Now();
    r_code::list<E>::const_iterator _e;
    for (_e = cache->evidences.begin(); _e != cache->evidences.end();) {

      if ((*_e).is_too_old(now)) // garbage collection.
        _e = cache->evidences.erase(_e);
      else if ((*_e).evidence_->get_before() <= now)
        // Skip the evidence if its time interval ends at exactly now. This is a little
        // more strict than is_too_old() which only checks get_before() < now.
        ++_e;
      else {

        PrimaryMDLOverlay o(this, bindings_);
        o.reduce((*_e).evidence_, f_p_f_imdl, controller);
        ++_e;
      }
    }
    cache->CS.leave();
  }

  bool monitor_predictions(_Fact *input);

  MDLController(r_code::View *view);
public:
  static MDLController *New(View *view, bool &inject_in_secondary_group);

  void add_monitor(PMonitor *m);
  void remove_monitor(PMonitor *m);

  _Fact *get_lhs() const { return lhs_; }
  _Fact *get_rhs() const { return rhs_; }
  Fact *get_f_ihlp(HLPBindingMap *bindings, bool wr_enabled) const {
    return bindings->build_f_ihlp(getObject(), Opcodes::IMdl, wr_enabled);
  }

  virtual void store_requirement(_Fact *f_p_f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation) = 0;
  ChainingStatus retrieve_imdl_fwd(HLPBindingMap *bm, Fact *f_imdl, RequirementsPair &r_p, Fact *&ground, MDLController *req_controller, bool &wr_enabled); // checks the requirement instances during fwd; r_p: all wrs in first, all srs in second.
  ChainingStatus retrieve_imdl_bwd(HLPBindingMap *bm, Fact *f_imdl, Fact *&ground); // checks the requirement instances during bwd; ground is set to the best weak requirement if chaining allowed, NULL otherwise.
  ChainingStatus retrieve_simulated_imdl_fwd(HLPBindingMap *bm, Fact *f_imdl, Controller *root);
  ChainingStatus retrieve_simulated_imdl_bwd(HLPBindingMap *bm, Fact *f_imdl, Controller *root, Fact *&ground);

  virtual void predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground) = 0;
  virtual void register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures) = 0;
  virtual void register_req_outcome(Fact *f_pred, bool success, bool rate_failures) = 0;

  void add_requirement_to_rhs();
  void remove_requirement_from_rhs();

  bool is_requirement() const { return (is_requirement_ != NOT_A_REQUIREMENT); }
  bool is_reuse() const { return is_reuse_; }
  bool is_cmd() const { return is_cmd_; }

  void register_requirement(_Fact *f_pred, RequirementsPair &r_p);

  /**
   * Get the last two values from the imdl template values, assuming that they are the timings
   * from the prerequisite model.
   * \param imdl The imdl object.
   * \param after Set this to the after timestamp (if this returns true).
   * \param before Set this to the before timestamp (if this returns true).
   * \param (optional) after_ts_index If not NULL, set this to the index in imdl Code array of the after time stamp structure.
   * \param (optional)  before_ts_index If not NULL, set this to the index in imdl Code array of the before time stamp structure.
   * \return True for success, otherwise false if there are not enough template parameters,
   * or if the values some time other than a timestamp (such as an unbound variable).
   */
  static bool get_imdl_template_timings(
    r_code::Code* imdl, Timestamp& after, Timestamp& before, uint16* after_ts_index = NULL, uint16* before_ts_index = NULL);
};

class PMDLController :
  public MDLController {
protected:
  CriticalSection g_monitorsCS_;
  r_code::list<P<_GMonitor> > g_monitors_;
  r_code::list<P<_GMonitor> > r_monitors_;

  virtual uint32 get_rdx_out_group_count() const { return get_out_group_count(); }
  void inject_goal(HLPBindingMap *bm, Fact *goal, Fact *f_imdl) const;
  void inject_simulation(Fact *simulation) const;

  bool monitor_goals(_Fact *input);

  std::chrono::microseconds get_sim_thz(Timestamp now, Timestamp deadline) const;

  PMDLController(r_code::View *view);
public:
  void add_g_monitor(_GMonitor *m);
  void remove_g_monitor(_GMonitor *m);
  void add_r_monitor(_GMonitor *m);
  void remove_r_monitor(_GMonitor *m);

  virtual void register_goal_outcome(Fact *goal, bool success, _Fact *evidence) const = 0;
  void register_predicted_goal_outcome(Fact *goal, HLPBindingMap *bm, Fact *f_imdl, bool success, bool injected_goal);
  virtual void register_simulated_goal_outcome(Fact *goal, bool success, _Fact *evidence) const = 0;
};

// See g_monitor.h: controllers and monitors work closely together.
//
// Min sthz is the time allowed for simulated predictions to flow upward.
// Max sthz sets the responsiveness of the model, i.e. limits the time waiting for simulation results, i.e. limits the latency of decision making.
// Simulation is synchronous, i.e. is performed within the enveloppe of sthz, recursively.

// Drives are not monitored (since they are not produced by models): they are injected periodically by user-defined pgms.
// Drives are not observable: they cannot be predicted to succeed or fail.
// Rhs is a drive; the model is an axiom: no rating and lives in the primary group only.
// The model does not predict.
// There is exactly one top-level model for each drive: hence no simulation during backward chaining.
// Top-level models cannot have requirements.
//
// Backward chaining: inputs are drives.
// if lhs is in the fact cache, stop.
// if lhs is in the prediction cache, spawn a g-monitor (will be ready to catch a counter-prediction, invalidate the goal and trigger the re-issuing of a new goal).
// else commit to the sub-goal; this will trigger the simulation of sub-sub-goals; N.B.: commands are not simulated, commands with unbound values are not injected.
class TopLevelMDLController :
  public PMDLController {
private:
  uint32 get_rdx_out_group_count() const { return get_out_group_count() - 1; } // so that rdx are not injected in the drives host.

  void abduce(HLPBindingMap *bm, Fact *super_goal, float32 confidence);
  void abduce_lhs(HLPBindingMap *bm, Fact *super_goal, _Fact *sub_goal_target, Fact *f_imdl, _Fact *evidence);

  void register_drive_outcome(Fact *goal, bool success) const;

  void check_last_match_time(bool match) {}
public:
  TopLevelMDLController(r_code::View *view);

  void take_input(r_exec::View *input);
  void reduce(r_exec::View *input);

  void store_requirement(_Fact *f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation); // never called.

  void predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground);
  void register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures);
  void register_goal_outcome(Fact *goal, bool success, _Fact *evidence) const;
  void register_simulated_goal_outcome(Fact *goal, bool success, _Fact *evidence) const;
  void register_req_outcome(Fact *f_pred, bool success, bool rate_failures);
};

class SecondaryMDLController;

// Backward chaining: inputs are goals, actual or simulated.
// Actual goals:
// if lhs is in the fact cache, stop.
// if lhs is in the prediction cache, spawn a g-monitor (will be ready to catch a counter-prediction, invalidate the goal and re-issue a new goal).
// else
// if (before-now)*percentage<min sthz, commit sub-goal on lhs.
// else
// if chaining is allowed, simulate the lhs and spawn a g-monitor with sthz=min((before-now)*percentage,max sthz)-min sthz.
// else, simulate f->imdl and spawn a g-monitor with sthz=min((before-now)*percentage,max sthz)/2-min sthz.
// Simulated goals:
// if lhs is in the fact cache, .
// if lhs is in the prediction cache,
// else:
// if sthz/2>min thz, simulate the lhs and spawn a g-monitor with sthz/2-min sthz.
// else predict rhs (cfd=1) and stop.
// Commands with unbound values are not injected.
class PrimaryMDLController :
  public PMDLController {
private:
  SecondaryMDLController *secondary_;

  CriticalSection codeCS_;
  CriticalSection last_match_timeCS_;

  CriticalSection assumptionsCS_;
  r_code::list<P<r_code::Code> > assumptions_; // produced by the model; garbage collection at reduce() time..

  void rate_model(bool success);
  void kill_views(); // force res in both primary/secondary to 0.
  void check_last_match_time(bool match); // activate secondary controller if no match after primary_thz;

  void abduce_lhs(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim, Fact *ground, bool set_before);
  void abduce_imdl(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim);

  /**
   * \param forwardSimulation If forwardSimulation is NULL, then pass sim to predict_simulated_evidence, otherwise pass forwardSimulation.
   */
  void abduce_simulated_lhs(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim, Fact *ground, Sim* forwardSimulation);
  void abduce_simulated_imdl(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim);
  void predict_simulated_lhs(HLPBindingMap *bm, bool opposite, float32 confidence, Sim *sim);
  Fact* predict_simulated_evidence(_Fact *evidence, Sim *sim);
  void assume(_Fact *input);
  void assume_lhs(HLPBindingMap *bm, bool opposite, _Fact *input, float32 confidence);

  bool abduction_allowed(HLPBindingMap *bm);
  /**
   * Get the last two values from the template parameters, assuming that they are the timings
   * from the prerequisite model. This evaluates the backward guards if needed.
   * \param bm The binding map, which is not changed.
   * \param after Set this to the after timestamp (if this returns true).
   * \param before Set this to the before timestamp (if this returns true).
   * \return True for success, otherwise false if there are not enough template parameters,
   * or cannot evaluate the backward guards, or if the values are not timestamps.
   */
  bool get_template_timings(HLPBindingMap *bm, Timestamp& after, Timestamp& before);
public:
  PrimaryMDLController(r_code::View *view);

  void set_secondary(SecondaryMDLController *secondary);

  void take_input(r_exec::View *input);
  void reduce(r_exec::View *input);
  void reduce_batch(Fact *f_p_f_imdl, MDLController *controller);

  void store_requirement(_Fact *f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation);

  void predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground);
  bool inject_prediction(Fact *prediction, Fact *f_imdl, float32 confidence, Timestamp::duration time_to_live, r_code::Code *mk_rdx) const; // here, resilience=time to live, in us; returns true if the prediction has actually been injected.

  void register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures);
  void register_req_outcome(Fact *f_pred, bool success, bool rate_failures);

  void register_goal_outcome(Fact *goal, bool success, _Fact *evidence) const;
  void register_simulated_goal_outcome(Fact *goal, bool success, _Fact *evidence) const;

  bool check_imdl(Fact *goal, HLPBindingMap *bm);

  /**
   * \param forwardSimulation If forwardSimulation is NULL, then use the goal->get_goal()->get_sim() when calling abduce_simulated_lhs. 
   * Otherwise, use the given forwardSimulation, which is carried throughout forward simulation.
   */
  bool check_simulated_imdl(Fact *goal, HLPBindingMap *bm, Controller *root, Sim* forwardSimulation);

  /**
   * \param allow_simulation (optional) If false then treat the super_goal Sim object as if it has mode ROOT and
   * time horizon zero, forcing an actual abduced LHS (not simulated). If true or omitted, then process with the
   * super_goal's given Sim object.
   */
  void abduce(HLPBindingMap *bm, Fact *super_goal, bool opposite, float32 confidence, bool allow_simulation = true);

  /**
   * Make a binding map from the super_goal, then call abduce() with allow_simulation false to inject the LHS as
   * an actual goal.
   * \param super_goal A goal of the model RHS.
   * \param opposite See abduce().
   * \param confidence See abduce().
   */
  void abduce_no_simulation(Fact *super_goal, bool opposite, float32 confidence);

  void debug(View *input);
};

// No backward chaining.
// Rating happens only upon the success of predictions.
// Requirements are stroed whetwehr they come from a primary or a secondary controller.
// Positive requirements are stored into the rhs controller, both kinds (secondary or primary: the latter case is necessary for rating the model).
class SecondaryMDLController :
  public MDLController {
private:
  PrimaryMDLController *primary_;

  CriticalSection codeCS_;
  CriticalSection last_match_timeCS_;

  void rate_model(); // record successes only.
  void kill_views(); // force res in both primary/secondary to 0.
  void check_last_match_time(bool match); // kill if no match after secondary_thz;
public:
  SecondaryMDLController(r_code::View *view);

  void set_primary(PrimaryMDLController *primary);

  void take_input(r_exec::View *input);
  void reduce(r_exec::View *input);
  void reduce_batch(Fact *f_p_f_imdl, MDLController *controller);

  void store_requirement(_Fact *f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation);

  void predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground);
  void register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures);
  void register_req_outcome(Fact *f_pred, bool success, bool rate_failures);
};
}


#endif
