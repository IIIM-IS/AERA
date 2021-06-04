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

#ifndef g_monitor_h
#define g_monitor_h

#include "monitor.h"


namespace r_exec {

class PMDLController;
class PrimaryMDLController;

class _GMonitor :
  public Monitor {
protected:
  Timestamp deadline_; // of the goal.
  Timestamp sim_thz_timestamp_;
  _Fact *goal_target_; // convenience; f1->object.
  P<Fact> f_imdl_;
  SimMode sim_mode_;

  uint32 volatile simulating_; // 32 bits alignment.

  // The first in the pair is the (fact (pred (fact (success...)))) or
  // (fact (pred (|fact (success...)))) where the success's object is the (fact (goal ...)).
  typedef std::list<std::pair<P<_Fact>, P<Sim> > > SolutionList;
  
  /* From the (fact (pred (fact (success...)))) of a solution, get the goal from
   * the success object's (fact (goal ...)).
   */
  static Goal* get_solution_goal(_Fact* f_pred_f_success) {
    Pred* pred = f_pred_f_success->get_pred();
    if (!pred)
      return NULL;
    Success* success = pred->get_target()->get_success();
    if (!success)
      return NULL;
    return success->get_object()->get_goal();
  }

  class SimOutcomes {
  public:
    SolutionList mandatory_solutions;
    SolutionList optional_solutions;
  };

  // Simulated predictions of any goal success resulting from the simulation of the monitored goal.
  SimOutcomes sim_successes_;
  SimOutcomes sim_failures_;

  /**
   * Store the outcome of any goal affected by the simulation of the monitored goal. If
   * (fact (pred (fact ...))) then store in sim_successes_. Otherwise if
   * (fact (pred (|fact ...))) then store in sim_failures_.
   * \param f_pred_f_success The (fact (pred (fact (success...)))) or
   * (fact (pred (|fact (success...)))) where the success's object is the affected (fact (goal ...)).
   * \param sim The Sim object (for the relevant controller) from the prediction.
   */
  void store_simulated_outcome(_Fact *f_pred_f_success, Sim *sim);
  void invalidate_sim_outcomes();

  _GMonitor(PMDLController *controller,
    BindingMap *bindings,
    Timestamp deadline,
    Timestamp sim_thz_timestamp,
    Fact *goal,
    Fact *f_imdl); // goal is f0->g->f1->object.
public:
  virtual bool signal(bool is_simulation) { return false; }
};

/**
 * Monitors goals (other than requirements).
 * Use for SIM_ROOT.
 * Is aware of any predicted evidence for the goal target: if at construction time such an evidence is known, the goal is not injected.
 * Reporting a success or failure to the controller invalidates the goal; reporting a predicted success also does.
 * Reporting a predicted failure injects the goal if it has not been already, invalidates it otherwise (a new goal will be injected).
 * The monitor still runs after reporting a predicted success.
 * The monitor does not run anymore if the goal is invalidated (case of a predicted success, followed by a predicted failure).
 * Wait for the time horizon; in the meantime:
 * actual inputs:
 * If an input is an evidence for the target, report a success.
 * If an input is a counter-evidence of the target, report a failure.
 * If an input is a predicted evidence for the target, report a predicted success.
 * If an input is a predicted counter-evidence for the target, report a predicted failure.
 * If there is a predicted evidence for the target that becomes invalidated, report a predicted failure.
 * Simulated predictions: catch only those that are a simulation for the monitored goal.
 * If an input is an evidence of the goal target, simulate a prediction of the goal success.
 * If an input is an evidence of the goal target, simulate a prediction of the goal failure.
 * Store any simulated prediction of success/failure for any goal.
 * At the time horizon:
 * Simulation mode:
 *   Commit to the appropriate solutions for the goal.
 *   Mode become actual.
 *   Time horizon becomes the goal deadline.
 * Actual mode:
 *   If the goal is not invalidated, report a failure.
 */
class GMonitor :
  public _GMonitor {
protected:
  _Fact *volatile predicted_evidence_; // f0->pred->f1->object; 32 bits alignment.
  bool injected_goal_;

  void commit();
public:
  GMonitor(PMDLController *controller,
    BindingMap *bindings,
    Timestamp deadline,
    Timestamp sim_thz_timestamp,
    Fact *goal,
    Fact *f_imdl,
    _Fact *predicted_evidence); // goal is f0->g->f1->object.

  virtual bool reduce(_Fact *input); // returning true will remove the monitor form the controller.
  virtual void update(Timestamp &next_target);
};

/**
 * Monitors actual requirements.
 * Use for SIM_ROOT.
 * target==f_imdl; this means we need to fullfill some requirements:
 * Wait until the deadline of the goal, in the meantime:
 * Each time the monitor is signalled (i.e. a new pred->f_imdl has been produced), check if chaining is allowed:
 * If no, do nothing.
 * If yes: assert success and abort: the model will bind its rhs with the bm retrieved from the pred->f_imdl; this will
 * Kill the monitor and a new one will be built for the bound rhs sub-goal.
 * At the deadline, assert failure.
 */
class RMonitor :
  public GMonitor {
public:
  RMonitor(PrimaryMDLController *controller,
    BindingMap *bindings,
    Timestamp deadline,
    Timestamp sim_thz_timestamp,
    Fact *goal,
    Fact *f_imdl);

  bool reduce(_Fact *input);
  void update(Timestamp &next_target);
  bool signal(bool is_simulation);
};

// Monitors simulated goals.
class SGMonitor :
  public _GMonitor {
protected:
  void commit();
public:
  SGMonitor(PrimaryMDLController *controller,
    BindingMap *bindings,
    Timestamp sim_thz_timestamp,
    Fact *goal,
    Fact *f_imdl); // goal is f0->g->f1->object.

  bool reduce(_Fact *input);
  void update(Timestamp &next_target);
};

// Monitors simulated requirements.
// Use for SIM_OPTIONAL and SIM_MANDATORY.
class SRMonitor :
  public SGMonitor {
public:
  SRMonitor(PrimaryMDLController *controller,
    BindingMap *bindings,
    Timestamp sim_thz_timestamp,
    Fact *goal,
    Fact *f_imdl);

  bool reduce(_Fact *input);
  void update(Timestamp &next_target);

  bool signal(bool is_simulation);
};

// Case A: target==actual goal and target!=f_imdl: simulations have been produced for all sub-goals.
// Wait until the STHZ, in the meantime:
// if input==goal target, assert success and abort: invalidate goal (this will invalidate the related simulations).
// if input==|goal target, assert failure and abort: the super-goal will probably fail and so on, until some drives fail, which will trigger their re-injection.
// if input==pred goal target, do nothing.
// if input==pred |goal target, do nothing.
// if input==pred success/failure of any other goal and sim->super-goal==goal, store the simulation for decision at STHZ.
// if input==pred goal target and sim->super-goal==goal, store the simulation for decision at STHZ.
// if input==pred |goal target and sim->super-goal==goal, store the simulation for decision at STHZ.
// if input==pred goal target and sim->super-goal!=goal, predict success for the goal.
// if input==pred |goal target and sim->super-goal!=goal, predict failure for the goal.
// At STHZ, choose the best simulations if any, and commit to their sub-goals; kill the predictions for the discarded simulations.
//
// Case B: target==f_imdl; this means we need to fullfill some requirements: simulations have been produced for all the sub-goals of f_imdl.
// Wait until the STHZ, in the meantime:
// if input==pred success/failure of any goal and sim->super-goal==goal, store the simulation for decision at STHZ.
// At STHZ, choose the best simulations if any, and commit to their sub-goals; kill the predictions for the discarded simulations.
//
// Case C: target==simulated goal and target!=f_imdl: simulations have been produced for all sub-goals.
// Wait until the STHZ, in the meantime:
// if input==goal target, predict success for the goal and abort: invalidate goal.
// if input==|goal target, predict failure for the goal and abort: invalidate goal.
// if input==pred success/failure of any other goal and sim->super-goal==goal, store the simulation for decision at STHZ.
// if input==pred goal target and sim->super-goal==goal, store the simulation for decision at STHZ.
// if input==pred |goal target and sim->super-goal==goal, store the simulation for decision at STHZ.
// if input==pred goal target and sim->super-goal!=goal, predict success for the goal.
// if input==pred |goal target and sim->super-goal!=goal, predict failure for the goal.
//
// Case D: target==simulated f_imdl; this means we need to fullfill some requirements: simulations have been produced for all the sub-goals of f_imdl.
// Wait until the STHZ, in the meantime:
// if input==pred success/failure of any goal and sim->super-goal==goal, store the simulation for decision at STHZ.
// At STHZ, choose the best simulations if any, and commit to their sub-goals; kill the predictions for the discarded simulations.
}


#endif
