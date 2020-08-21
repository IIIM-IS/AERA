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

#include "g_monitor.h"
#include "mem.h"
#include "mdl_controller.h"
#include "factory.h"

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

_GMonitor::_GMonitor(PMDLController *controller,
  BindingMap *bindings,
  Timestamp deadline,
  Timestamp sim_thz_timestamp,
  Fact *goal,
  Fact *f_imdl) : Monitor(controller,
    bindings,
    goal),
  deadline_(deadline),
  sim_thz_timestamp_(sim_thz_timestamp),
  f_imdl_(f_imdl) { // goal is f0->g->f1->object.

  simulating_ = (sim_thz_timestamp > Now());
  sim_mode_ = goal->get_goal()->get_sim()->get_mode();
  goal_target_ = target_->get_goal()->get_target(); // f1.
}

void _GMonitor::store_simulated_outcome(Goal *affected_goal, Sim *sim, bool success) { // outcome is f0 as in f0->pred->f1->success.

  if (success) {

    switch (sim->get_mode()) {
    case SIM_MANDATORY:
      sim_successes_.mandatory_solutions.push_back(std::pair<P<Goal>, P<Sim> >(affected_goal, sim));
      break;
    case SIM_OPTIONAL:
      sim_successes_.optional_solutions.push_back(std::pair<P<Goal>, P<Sim> >(affected_goal, sim));
      break;
    default:
      break;
    }
  } else {

    switch (sim->get_mode()) {
    case SIM_MANDATORY:
      sim_failures_.mandatory_solutions.push_back(std::pair<P<Goal>, P<Sim> >(affected_goal, sim));
      break;
    case SIM_OPTIONAL:
      sim_failures_.optional_solutions.push_back(std::pair<P<Goal>, P<Sim> >(affected_goal, sim));
      break;
    default:
      break;
    }
  }
}

void _GMonitor::invalidate_sim_outcomes() {

  SolutionList::const_iterator solution;

  for (solution = sim_failures_.mandatory_solutions.begin(); solution != sim_failures_.mandatory_solutions.end(); ++solution)
    (*solution).second->invalidate();

  for (solution = sim_failures_.optional_solutions.begin(); solution != sim_failures_.optional_solutions.end(); ++solution)
    (*solution).second->invalidate();

  for (solution = sim_successes_.mandatory_solutions.begin(); solution != sim_successes_.mandatory_solutions.end(); ++solution)
    (*solution).second->invalidate();

  for (solution = sim_successes_.optional_solutions.begin(); solution != sim_successes_.optional_solutions.end(); ++solution)
    (*solution).second->invalidate();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GMonitor::GMonitor(PMDLController *controller,
  BindingMap *bindings,
  Timestamp deadline,
  Timestamp sim_thz_timestamp,
  Fact *goal,
  Fact *f_imdl,
  _Fact *predicted_evidence) : _GMonitor(controller,
    bindings,
    deadline,
    sim_thz_timestamp,
    goal,
    f_imdl),
  predicted_evidence_(predicted_evidence) { // goal is f0->g->f1->object.

  injected_goal_ = (predicted_evidence == NULL);
  MonitoringJob<GMonitor> *j = new MonitoringJob<GMonitor>(this, simulating_ ? sim_thz_timestamp : deadline);
  _Mem::Get()->pushTimeJob(j);
}

void GMonitor::commit() { // the purpose is to invalidate damaging simulations; if anything remains, commit to all mandatory simulations and to the best optional one.

  Goal *monitored_goal = target_->get_goal();

  auto now = Now();

  SolutionList::const_iterator solution;

  for (solution = sim_failures_.mandatory_solutions.begin(); solution != sim_failures_.mandatory_solutions.end(); ++solution) { // check if any mandatory solution could result in the failure of more important a goal.

    if ((*solution).second->is_invalidated())
      continue;
    if ((*solution).first->get_strength(now) > monitored_goal->get_strength(now)) { // cave in.

      invalidate_sim_outcomes(); // this stops any further propagation of the goal simulation.
      return;
    }
  }

  for (solution = sim_failures_.optional_solutions.begin(); solution != sim_failures_.optional_solutions.end(); ++solution) { // check if any optional solutions could result in the failure of more important a goal; invalidate the culprits.

    if ((*solution).second->is_invalidated())
      continue;
    if ((*solution).first->get_strength(now) > monitored_goal->get_strength(now))
      (*solution).second->invalidate();
  }

  Sim *best_sol = NULL;
  for (solution = sim_successes_.optional_solutions.begin(); solution != sim_successes_.optional_solutions.end(); ++solution) { // find the best optional solution left.

    if ((*solution).second->is_invalidated())
      continue;
    if (!best_sol)
      best_sol = (*solution).second;
    else {

      float32 s = (*solution).second->solution_cfd_ / duration_cast<microseconds>((*solution).second->solution_before_ - now).count();
      float32 _s = best_sol->solution_cfd_ / duration_cast<microseconds>(best_sol->solution_before_ - now).count();
      if (s > _s)
        best_sol = (*solution).second;
    }
  }

  invalidate_sim_outcomes(); // this stops any further propagation of the goal simulation.

  if (best_sol) {

    ((PrimaryMDLController *)best_sol->solution_controller_)->abduce(bindings_, best_sol->super_goal_, best_sol->opposite_, goal_target_->get_cfd());

    for (solution = sim_successes_.mandatory_solutions.begin(); solution != sim_successes_.mandatory_solutions.end(); ++solution) // commit to all mandatory solutions.
      ((PrimaryMDLController *)(*solution).second->solution_controller_)->abduce(bindings_, (*solution).second->super_goal_, (*solution).second->opposite_, goal_target_->get_cfd());
  }
}

bool GMonitor::reduce(_Fact *input) { // executed by a reduction core; invalidation check performed in Monitor::is_alive().

  if (target_->is_invalidated())
    return true;

  if (!injected_goal_) {

    if (predicted_evidence_ && predicted_evidence_->is_invalidated()) { // the predicted evidence was wrong.

      ((PMDLController *)controller_)->register_predicted_goal_outcome(target_, bindings_, f_imdl_, false, injected_goal_); // report a predicted failure; this will inject the goal.
      predicted_evidence_ = NULL;
      injected_goal_ = true;
      return false;
    }
  }

  Pred *prediction = input->get_pred();
  if (prediction) { // input is f0->pred->f1->object.

    _Fact *_input = prediction->get_target(); // _input is f1->obj.
    if (simulating_) { // injected_goal==true.
      // Debug: It was passing target. Is it right to pass controller_?
      Sim *sim = prediction->get_simulation(controller_);
      if (sim) {

        Code *outcome = _input->get_reference(0);
        if (outcome->code(0).asOpcode() == Opcodes::Success) { // _input is f1->success or |f1->success.

          _Fact *f_success = (_Fact *)outcome->get_reference(outcome->code(SUCCESS_OBJ).asIndex());
          Goal *affected_goal = f_success->get_goal();
          if (affected_goal) {

            store_simulated_outcome(affected_goal, sim, _input->is_fact());
            return false;
          }
        } else { // report the simulated outcome: this will inject a simulated prediction of the outcome, to allow any g-monitor deciding on this ground.

          switch (_input->is_evidence(goal_target_)) {
          case MATCH_SUCCESS_POSITIVE:
            ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, true, input); // report a simulated success.
            return false;
          case MATCH_SUCCESS_NEGATIVE:
            ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, false, input); // report a simulated failure.
            return false;
          case MATCH_FAILURE:
            return false;
          }
        }
      } else // during simulation (SIM_ROOT) if the prediction is actual, positive and comes true, we'll eventually catch an actual evidence; otherwise (positive that does not come true or negative), keep simulating: in any case ignore it.
        return false;
    } else {

      switch (_input->is_evidence(goal_target_)) {
      case MATCH_SUCCESS_POSITIVE:
        if (injected_goal_)
          ((PMDLController *)controller_)->register_predicted_goal_outcome(target_, bindings_, f_imdl_, true, true); // report a predicted success.
        if (predicted_evidence_ && _input->get_cfd() > predicted_evidence_->get_pred()->get_target()->get_cfd()) // bias toward cfd instead of age.
          predicted_evidence_ = input;
        return false;
      case MATCH_SUCCESS_NEGATIVE:
        ((PMDLController *)controller_)->register_predicted_goal_outcome(target_, bindings_, f_imdl_, false, injected_goal_); // report a predicted failure; this may invalidate the target.
        predicted_evidence_ = NULL;
        injected_goal_ = true;
        return target_->is_invalidated();
      case MATCH_FAILURE:
        return false;
      }
    }
  } else { // input is an actual fact.

    Goal *g = target_->get_goal();
    if (g->ground_invalidated(input)) { // invalidate the goal and abduce from the super-goal.

      target_->invalidate();
      ((PrimaryMDLController *)controller_)->abduce(bindings_, g->get_sim()->super_goal_, g->get_sim()->opposite_, goal_target_->get_cfd());
      return true;
    }

    switch (input->is_evidence(goal_target_)) {
    case MATCH_SUCCESS_POSITIVE:
      ((PMDLController *)controller_)->register_goal_outcome(target_, true, input); // report a success.
      return true;
    case MATCH_SUCCESS_NEGATIVE:
      ((PMDLController *)controller_)->register_goal_outcome(target_, false, input); // report a failure.
      return true;
    case MATCH_FAILURE:
      return false;
    }
  }
}

void GMonitor::update(Timestamp &next_target) { // executed by a time core.

  if (target_->is_invalidated()) {

    ((PMDLController *)controller_)->remove_g_monitor(this);
    next_target = Timestamp(seconds(0));
  } else if (simulating_) {

    simulating_ = 0;
    commit();
    next_target = deadline_;
  } else {

    ((PMDLController *)controller_)->register_goal_outcome(target_, false, NULL);
    ((PMDLController *)controller_)->remove_g_monitor(this);
    next_target = Timestamp(seconds(0));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

RMonitor::RMonitor(PrimaryMDLController *controller,
  BindingMap *bindings,
  Timestamp deadline,
  Timestamp sim_thz_timestamp,
  Fact *goal,
  Fact *f_imdl) : GMonitor(controller,
    bindings,
    deadline,
    sim_thz_timestamp,
    goal,
    f_imdl,
    NULL) { // goal is f0->g->f1->object.

  MonitoringJob<RMonitor> *j = new MonitoringJob<RMonitor>(this, deadline);
  _Mem::Get()->pushTimeJob(j);
}

bool RMonitor::signal(bool simulation) {

  if (target_->is_invalidated())
    return true;

  if (simulating_ && simulation) { // report the simulated outcome: this will inject a simulated prediction of the outcome, to allow any g-monitor deciding on this ground.

    if (((PrimaryMDLController *)controller_)->check_simulated_imdl(target_, bindings_, target_->get_goal()->get_sim()->root_))
      ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, true, target_); // report a simulated success.
    else
      ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, false, NULL); // report a simulated failure.
    return false;
  } else if (((PrimaryMDLController *)controller_)->check_imdl(target_, bindings_))
    return true;
  return false;
}

bool RMonitor::reduce(_Fact *input) { // catch simulated predictions only; requirements are observable in signal().

  if (target_->is_invalidated())
    return true;

  Pred *prediction = input->get_pred();
  if (prediction) { // input is f0->pred->f1->object.

    _Fact *_input = prediction->get_target(); // _input is f1->obj.
    if (simulating_) { // injected_goal==true.
      // Debug: It was passing target. Is it right to pass controller_?
      Sim *sim = prediction->get_simulation(controller_);
      if (sim) {

        Code *outcome = _input->get_reference(0);
        if (outcome->code(0).asOpcode() == Opcodes::Success) { // _input is f1->success or |f1->success.

          _Fact *f_success = (_Fact *)outcome->get_reference(outcome->code(SUCCESS_OBJ).asIndex());
          Goal *affected_goal = f_success->get_goal();
          if (affected_goal) {

            store_simulated_outcome(affected_goal, sim, _input->is_fact());
            return false;
          }
        }
      }
    }
  }

  return false;
}

void RMonitor::update(Timestamp &next_target) {

  if (target_->is_invalidated()) {

    ((PMDLController *)controller_)->remove_r_monitor(this);
    next_target = Timestamp(seconds(0));
  } else if (simulating_) {

    simulating_ = 0;
    commit();
    next_target = deadline_;
  } else {

    ((PMDLController *)controller_)->register_goal_outcome(target_, false, NULL);
    ((PMDLController *)controller_)->remove_r_monitor(this);
    next_target = Timestamp(seconds(0));
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SGMonitor::SGMonitor(PrimaryMDLController *controller,
  BindingMap *bindings,
  Timestamp sim_thz_timestamp,
  Fact *goal,
  Fact *f_imdl) : _GMonitor(controller,
    bindings,
    Timestamp(seconds(0)),
    sim_thz_timestamp,
    goal,
    f_imdl) { // goal is f0->g->f1->object.

  MonitoringJob<SGMonitor> *j = new MonitoringJob<SGMonitor>(this, sim_thz_timestamp);
  _Mem::Get()->pushTimeJob(j);
}

void SGMonitor::commit() { // the purpose is to invalidate damaging simulations and let the rest flow upward.

  Goal *monitored_goal = target_->get_goal();

  auto now = Now();

  SolutionList::const_iterator solution;

  for (solution = sim_failures_.mandatory_solutions.begin(); solution != sim_failures_.mandatory_solutions.end(); ++solution) { // check if any mandatory solution could result in the failure of more important a goal.

    if ((*solution).second->is_invalidated())
      continue;
    if ((*solution).first->get_strength(now) > monitored_goal->get_strength(now)) { // cave in.

      (*solution).second->invalidate();
      return;
    }
  }

  for (solution = sim_failures_.optional_solutions.begin(); solution != sim_failures_.optional_solutions.end(); ++solution) { // check if any optional solutions could result in the failure of more important a goal; invalidate the culprits.

    if ((*solution).second->is_invalidated())
      continue;
    if ((*solution).first->get_strength(now) > monitored_goal->get_strength(now))
      (*solution).second->invalidate();
  }
}

bool SGMonitor::reduce(_Fact *input) {

  if (target_->is_invalidated())
    return true;

  _Fact *_input;
  Pred *prediction = input->get_pred();
  if (prediction) { // input is f0->pred->f1->object.

    _input = prediction->get_target(); // _input is f1->obj.
    // Debug: It was passing target. Is it right to pass controller_?
    Sim *sim = prediction->get_simulation(controller_);
    if (sim) {

      Code *outcome = _input->get_reference(0);
      if (outcome->code(0).asOpcode() == Opcodes::Success) { // _input is f1->success or |f1->success.

        _Fact *f_success = (_Fact *)outcome->get_reference(outcome->code(SUCCESS_OBJ).asIndex());
        Goal *affected_goal = f_success->get_goal();
        if (affected_goal) {

          store_simulated_outcome(affected_goal, sim, _input->is_fact());
          return false;
        }
      }
    }
  } else
    _input = input;

  // Non-simulated input (can be actual or predicted): report the simulated outcome: this will inject a simulated prediction of the outcome, to allow any g-monitor deciding on this ground.
  switch (_input->is_evidence(goal_target_)) {
  case MATCH_SUCCESS_POSITIVE:
    ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, true, _input); // report a simulated success.
    return false;
  case MATCH_SUCCESS_NEGATIVE:
    ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, false, _input); // report a simulated failure.
    return false;
  case MATCH_FAILURE:
    return false;
  }
}

void SGMonitor::update(Timestamp &next_target) { // executed by a time core.

  if (!target_->is_invalidated())
    commit();
  ((PMDLController *)controller_)->remove_g_monitor(this);
  next_target = Timestamp(seconds(0));
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SRMonitor::SRMonitor(PrimaryMDLController *controller,
  BindingMap *bindings,
  Timestamp sim_thz_timestamp,
  Fact *goal,
  Fact *f_imdl) : SGMonitor(controller,
    bindings,
    sim_thz_timestamp,
    goal,
    f_imdl) { // goal is f0->g->f1->object.

  MonitoringJob<SRMonitor> *j = new MonitoringJob<SRMonitor>(this, sim_thz_timestamp);
  _Mem::Get()->pushTimeJob(j);
}

bool SRMonitor::signal(bool simulation) {

  if (target_->is_invalidated())
    return true;

  if (simulation) {

    if (((PrimaryMDLController *)controller_)->check_simulated_imdl(target_, bindings_, target_->get_goal()->get_sim()->root_))
      ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, true, target_); // report a simulated success.
  } else {

    if (((PrimaryMDLController *)controller_)->check_simulated_imdl(target_, bindings_, NULL))
      ((PMDLController *)controller_)->register_simulated_goal_outcome(target_, false, NULL); // report a simulated failure.
  }
  return false;
}

bool SRMonitor::reduce(_Fact *input) {

  if (target_->is_invalidated())
    return true;

  Pred *prediction = input->get_pred();
  if (prediction) { // input is f0->pred->f1->object.

    _Fact *_input = prediction->get_target(); // _input is f1->obj.
    // Debug: It was passing target. Is it right to pass controller_?
    Sim *sim = prediction->get_simulation(controller_);
    if (sim) {

      Code *outcome = _input->get_reference(0);
      if (outcome->code(0).asOpcode() == Opcodes::Success) { // _input is f1->success or |f1->success.

        _Fact *f_success = (_Fact *)outcome->get_reference(outcome->code(SUCCESS_OBJ).asIndex());
        Goal *affected_goal = f_success->get_goal();
        if (affected_goal) {

          store_simulated_outcome(affected_goal, sim, _input->is_fact());
          return false;
        }
      }
    }
  }

  return false;
}

void SRMonitor::update(Timestamp &next_target) {

  if (!target_->is_invalidated())
    commit();
  ((PMDLController *)controller_)->remove_r_monitor(this);
  next_target = Timestamp(seconds(0));
}
}
