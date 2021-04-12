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

#include "mdl_controller.h"
#include "mem.h"
#include "model_base.h"

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

MDLOverlay::MDLOverlay(Controller *c, const HLPBindingMap *bindings) : HLPOverlay(c, bindings, true) {

  load_patterns();
}

MDLOverlay::~MDLOverlay() {
}

void MDLOverlay::load_patterns() {

  patterns_.push_back(((MDLController *)controller_)->get_lhs());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PrimaryMDLOverlay::PrimaryMDLOverlay(Controller *c, const HLPBindingMap *bindings) : MDLOverlay(c, bindings) {
}

PrimaryMDLOverlay::~PrimaryMDLOverlay() {
}

bool PrimaryMDLOverlay::reduce(_Fact *input, Fact *f_p_f_imdl, MDLController *req_controller) {

  OUTPUT_LINE(MDL_IN, Utils::RelativeTime(Now()) << " mdl: " << controller_->getObject()->get_oid() << " <- input: " << input->get_oid());
  _Fact *input_object;
  Pred *prediction = input->get_pred();
  bool is_simulation;
  if (prediction) {

    input_object = prediction->get_target();
    is_simulation = prediction->is_simulation();
  } else {

    input_object = input;
    is_simulation = false;
  }

  P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
  bm->reset_fwd_timings(input_object);
  switch (bm->match_fwd_lenient(input_object, ((MDLController *)controller_)->get_lhs())) {
  case MATCH_SUCCESS_POSITIVE: {

    load_code();
    P<HLPBindingMap> original_bindings = bindings_;
    bindings_ = bm;
    bool is_req = ((MDLController *)controller_)->is_requirement();
    bool match = false;
    Fact *f_imdl = ((MDLController *)controller_)->get_f_ihlp(bm, false);
    RequirementsPair r_p;
    Fact *ground = f_p_f_imdl; // JTNote: retrieve_imdl_fwd assigns ground, so this assignment is not used.
    bool wr_enabled = false;
    bool stop = (req_controller != NULL);
    ChainingStatus c_s = ((MDLController *)controller_)->retrieve_imdl_fwd(bm, f_imdl, r_p, ground, req_controller, wr_enabled);
    f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(wr_enabled);
    bool chaining_allowed = (c_s >= WEAK_REQUIREMENT_ENABLED);
    bool did_check_simulated_chaining = false;
    bool check_simulated_chaining_result;
    switch (c_s) {
    case WEAK_REQUIREMENT_DISABLED:
    case STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT: // silent monitoring of a prediction that will not be injected.
      if (is_simulation) { // if there is simulated imdl for the root of one sim in prediction, allow forward chaining.

        check_simulated_chaining_result = check_simulated_chaining(bm, f_imdl, prediction);
        did_check_simulated_chaining = true;
        if (check_simulated_chaining_result)
          chaining_allowed = true;
        else
          break;
      }
    case NO_REQUIREMENT:
      if (!chaining_allowed && ((MDLController *)controller_)->has_tpl_args()) // there are tpl args, abort.
        break;
      else
        f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(false);
    case STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT: // silent monitoring of a prediction that will not be injected.
      if (is_simulation) { // if there is simulated imdl for the root of one sim in prediction, allow forward chaining.

        if (!did_check_simulated_chaining) {
          check_simulated_chaining_result = check_simulated_chaining(bm, f_imdl, prediction);
          did_check_simulated_chaining = true;
        }
        if (check_simulated_chaining_result)
          chaining_allowed = true;
        else
          break;
      }
    case WEAK_REQUIREMENT_ENABLED:
      if (evaluate_fwd_guards()) { // may update bindings.
        // JTNote: The prediction is made from the bindings that made f_imdl, but it is not used directly.
        f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
        ((PrimaryMDLController *)controller_)->predict(bindings_, input, f_imdl, chaining_allowed, r_p, ground);
        match = true;
      }
      break;
    }
    // reset.
    delete[] code_;
    code_ = NULL;
    bindings_ = original_bindings;
    if (f_p_f_imdl == NULL) // i.e. if reduction not triggered a requirement.
      store_evidence(input, prediction, is_simulation);
    return match;
  }case MATCH_SUCCESS_NEGATIVE: // counter-evidence WRT the lhs.
    if (f_p_f_imdl == NULL) // i.e. if reduction not triggered a requirement.
      store_evidence(input, prediction, is_simulation);
  case MATCH_FAILURE:
    //std::cout<<" no match\n";
    return false;
  }
}

bool PrimaryMDLOverlay::check_simulated_chaining(HLPBindingMap *bm, Fact *f_imdl, Pred *prediction) {

  for (uint32 i = 0; i < prediction->get_simulations_size(); ++i) {

    switch (((MDLController *)controller_)->retrieve_simulated_imdl_fwd(bm, f_imdl, prediction->get_simulation(i)->root_)) {
    case NO_REQUIREMENT:
    case WEAK_REQUIREMENT_ENABLED:
      return true;
    default:
      break;
    }
  }

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SecondaryMDLOverlay::SecondaryMDLOverlay(Controller *c, const HLPBindingMap *bindings) : MDLOverlay(c, bindings) {
}

SecondaryMDLOverlay::~SecondaryMDLOverlay() {
}

bool SecondaryMDLOverlay::reduce(_Fact *input, Fact *f_p_f_imdl, MDLController *req_controller) { // no caching since no bwd.
//std::cout<<std::hex<<this<<std::dec<<" "<<input->object->get_oid();
  P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
  bm->reset_fwd_timings(input);
  switch (bm->match_fwd_lenient(input, ((MDLController *)controller_)->get_lhs())) {
  case MATCH_SUCCESS_POSITIVE: {

    load_code();
    P<HLPBindingMap> original_bindings = bindings_;
    bindings_ = bm;
    bool match = false;
    Fact *f_imdl = ((MDLController *)controller_)->get_f_ihlp(bm, false);
    RequirementsPair r_p;
    Fact *ground = f_p_f_imdl;
    bool wr_enabled = false;
    ChainingStatus c_s = ((MDLController *)controller_)->retrieve_imdl_fwd(bm, f_imdl, r_p, ground, req_controller, wr_enabled);
    f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(wr_enabled);
    bool chaining_allowed = (c_s >= NO_REQUIREMENT);
    switch (c_s) {
    case WEAK_REQUIREMENT_DISABLED:
    case STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT: // silent monitoring of a prediction that will not be injected.
    case NO_REQUIREMENT:
      if (((MDLController *)controller_)->has_tpl_args()) // there are tpl args, abort.
        break;
      else
        f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(false);
    case STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT:
    case WEAK_REQUIREMENT_ENABLED:
      if (evaluate_fwd_guards()) { // may update bindings.
//std::cout<<" match\n";
        f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
        ((SecondaryMDLController *)controller_)->predict(bindings_, input, NULL, true, r_p, ground);
        match = true;
      }
      break;
    }
    // reset.
    delete[] code_;
    code_ = NULL;
    bindings_ = original_bindings;
    return match;
  }case MATCH_SUCCESS_NEGATIVE:
  case MATCH_FAILURE:
    //std::cout<<" no match\n";
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MDLController *MDLController::New(View *view, bool &inject_in_secondary_group) {

  Code *unpacked_mdl = view->object_->get_reference(view->object_->references_size() - MDL_HIDDEN_REFS);
  uint16 obj_set_index = unpacked_mdl->code(MDL_OBJS).asIndex();
  Code *rhs = unpacked_mdl->get_reference(unpacked_mdl->code(obj_set_index + 2).asIndex());

  if (rhs->get_reference(0)->code(0).asOpcode() == Opcodes::Ent) { // rhs is a drive.

    inject_in_secondary_group = false;
    return new TopLevelMDLController(view);
  }

  inject_in_secondary_group = true;
  return new PrimaryMDLController(view);
}

MDLController::MDLController(r_code::View *view) : HLPController(view) {

  Code *object = get_unpacked_object();
  uint16 obj_set_index = object->code(MDL_OBJS).asIndex();
  lhs_ = object->get_reference(object->code(obj_set_index + 1).asIndex());
  rhs_ = object->get_reference(object->code(obj_set_index + 2).asIndex());

  Group *host = get_host();
  controllers_.resize(2);

  Code *rhs_ihlp = rhs_->get_reference(0);
  is_requirement_ = NOT_A_REQUIREMENT;
  controllers_[RHSController] = NULL;
  uint16 opcode = rhs_ihlp->code(0).asOpcode();
  if (opcode == Opcodes::ICst ||
    opcode == Opcodes::IMdl) {

    Code *rhs_hlp = rhs_ihlp->get_reference(0);
    r_exec::View *rhs_hlp_v = (r_exec::View*)rhs_hlp->get_view(host, true);
    if (rhs_hlp_v) {

      if (opcode == Opcodes::IMdl)
        is_requirement_ = (rhs_->code(0).asOpcode() == Opcodes::AntiFact ? STRONG_REQUIREMENT : WEAK_REQUIREMENT);
      controllers_[RHSController] = (HLPController *)rhs_hlp_v->controller_;
    }
  }

  Code *lhs_ihlp = lhs_->get_reference(0);
  is_reuse_ = false;
  controllers_[LHSController] = NULL;
  opcode = lhs_ihlp->code(0).asOpcode();
  if (opcode == Opcodes::ICst ||
    opcode == Opcodes::IMdl) {

    Code *lhs_hlp = lhs_ihlp->get_reference(0);
    r_exec::View *lhs_hlp_v = (r_exec::View*)lhs_hlp->get_view(host, true);
    if (lhs_hlp_v) {

      if (opcode == Opcodes::IMdl)
        is_reuse_ = true;
      controllers_[LHSController] = (HLPController *)lhs_hlp_v->controller_;
    }
  }

  is_cmd_ = (lhs_ihlp->code(0).asOpcode() == Opcodes::Cmd);
}

float32 MDLController::get_cfd() const {

  return get_core_object()->code(MDL_SR).asFloat();
}

bool MDLController::monitor_predictions(_Fact *input) { // predictions are admissible inputs (for checking predicted counter-evidences).

  Pred *pred = input->get_pred();
  if (pred && pred->is_simulation()) // discard simulations.
    return false;

  bool r = false;
  r_code::list<P<PMonitor> >::const_iterator m;
  p_monitorsCS_.enter();
  for (m = p_monitors_.begin(); m != p_monitors_.end();) {

    if ((*m)->reduce(input)) {

      m = p_monitors_.erase(m);
      r = true;
    } else
      ++m;
  }
  p_monitorsCS_.leave();

  return r;
}

void MDLController::add_monitor(PMonitor *m) {

  p_monitorsCS_.enter();
  p_monitors_.push_front(m);
  p_monitorsCS_.leave();
}

void MDLController::remove_monitor(PMonitor *m) {

  p_monitorsCS_.enter();
  p_monitors_.remove(m);
  p_monitorsCS_.leave();
}

void MDLController::add_requirement_to_rhs() {

  if (is_requirement_ != NOT_A_REQUIREMENT) {

    HLPController *c = controllers_[RHSController];
    if (c)
      c->add_requirement(is_requirement_ == STRONG_REQUIREMENT);
  }
}

void MDLController::remove_requirement_from_rhs() {

  if (is_requirement_ != NOT_A_REQUIREMENT) {

    HLPController *c = controllers_[RHSController];
    if (c)
      c->remove_requirement(is_requirement_ == STRONG_REQUIREMENT);
  }
}

void MDLController::_store_requirement(r_code::list<RequirementEntry> *cache, RequirementEntry &e) {

  requirements_.CS.enter();
  auto now = Now();
  r_code::list<RequirementEntry>::const_iterator _e;
  for (_e = cache->begin(); _e != cache->end();) {

    if ((*_e).is_too_old(now)) // garbage collection.
      _e = cache->erase(_e);
    else
      ++_e;
  }
  cache->push_front(e);
  requirements_.CS.leave();
}

ChainingStatus MDLController::retrieve_simulated_imdl_fwd(HLPBindingMap *bm, Fact *f_imdl, Controller *root) {

  uint32 wr_count;
  uint32 sr_count;
  uint32 r_count = get_requirement_count(wr_count, sr_count);
  if (!r_count)
    return NO_REQUIREMENT;
  ChainingStatus r;
  HLPBindingMap original(bm);
  if (!sr_count) { // no strong req., some weak req.: true if there is one f->imdl complying with timings and bindings.

    r = WEAK_REQUIREMENT_DISABLED;
    requirements_.CS.enter();
    auto now = Now();
    r_code::list<RequirementEntry>::const_iterator e;
    for (e = simulated_requirements_.positive_evidences.begin(); e != simulated_requirements_.positive_evidences.end();) {

      if ((*e).is_too_old(now)) // garbage collection.
        e = simulated_requirements_.positive_evidences.erase(e);
      else {

        if ((*e).evidence_->get_pred()->get_simulation(root)) {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          //_f_imdl->get_reference(0)->trace();
          //f_imdl->get_reference(0)->trace();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          if (_original.match_fwd_strict(_f_imdl, f_imdl)) { // tpl args will be valuated in bm, but not in f_imdl yet.

            r = WEAK_REQUIREMENT_ENABLED;
            bm->load(&_original);
            break;
          }
        }
        ++e;
      }
    }

    requirements_.CS.leave();
    return r;
  } else {

    if (!wr_count) { // some strong req., no weak req.: true if there is no |f->imdl complying with timings and bindings.

      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = simulated_requirements_.negative_evidences.begin(); e != simulated_requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = simulated_requirements_.negative_evidences.erase(e);
        else {

          if ((*e).evidence_->get_pred()->get_simulation(root)) {

            _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
            HLPBindingMap _original = original; // matching updates the bm; always start afresh.
            if (_original.match_bwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) { // tpl args will be valuated in bm.

              bm->load(&_original);
              requirements_.CS.leave();
              return STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
            }
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return WEAK_REQUIREMENT_ENABLED;
    } else { // some strong req. and some weak req.: true if among the entries complying with timings and bindings, the youngest |f->imdl is weaker than the youngest f->imdl.

      r = WEAK_REQUIREMENT_DISABLED;
      float32 negative_cfd = 0;
      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = simulated_requirements_.negative_evidences.begin(); e != simulated_requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = simulated_requirements_.negative_evidences.erase(e);
        else {

          if ((*e).evidence_->get_pred()->get_simulation(root)) {

            _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
            HLPBindingMap _original = original; // matching updates the bm; always start afresh.
            if (_original.match_bwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) {

              negative_cfd = (*e).confidence_;
              r = STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
              bm->load(&_original);
              break;
            }
          }
          ++e;
        }
      }

      for (e = simulated_requirements_.positive_evidences.begin(); e != simulated_requirements_.positive_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = simulated_requirements_.positive_evidences.erase(e);
        else {
          //(*e).f->get_reference(0)->trace();
          //f->get_reference(0)->trace();
          if ((*e).evidence_->get_pred()->get_simulation(root)) {

            _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
            HLPBindingMap _original = original; // matching updates the bm; always start afresh.
            if (_original.match_bwd_strict(_f_imdl, f_imdl)) {

              if ((*e).confidence_ >= negative_cfd) {

                r = WEAK_REQUIREMENT_ENABLED;
                bm->load(&_original);
                break;
              } else
                r = STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT;
            }
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return r;
    }
  }
}

ChainingStatus MDLController::retrieve_simulated_imdl_bwd(HLPBindingMap *bm, Fact *f_imdl, Controller *root) {

  uint32 wr_count;
  uint32 sr_count;
  uint32 r_count = get_requirement_count(wr_count, sr_count);
  if (!r_count)
    return NO_REQUIREMENT;
  ChainingStatus r;
  HLPBindingMap original(bm);
  if (!sr_count) { // no strong req., some weak req.: true if there is one f->imdl complying with timings and bindings.

    r = WEAK_REQUIREMENT_DISABLED;
    requirements_.CS.enter();
    auto now = Now();
    r_code::list<RequirementEntry>::const_iterator e;
    for (e = simulated_requirements_.positive_evidences.begin(); e != simulated_requirements_.positive_evidences.end();) {

      if ((*e).is_too_old(now)) // garbage collection.
        e = simulated_requirements_.positive_evidences.erase(e);
      else {

        if ((*e).evidence_->get_pred()->get_simulation(root)) {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          //_f_imdl->get_reference(0)->trace();
          //f_imdl->get_reference(0)->trace();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
          if (_original.match_fwd_strict(_f_imdl, f_imdl)) { // tpl args will be valuated in bm, but not in f_imdl yet.

            bm->load(&_original);
            r = WEAK_REQUIREMENT_ENABLED;
            break;
          }
        }
        ++e;
      }
    }

    requirements_.CS.leave();
    return r;
  } else {

    if (!wr_count) { // some strong req., no weak req.: true if there is no |f->imdl complying with timings and bindings.

      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = simulated_requirements_.negative_evidences.begin(); e != simulated_requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = simulated_requirements_.negative_evidences.erase(e);
        else {

          if ((*e).evidence_->get_pred()->get_simulation(root)) {

            _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
            HLPBindingMap _original = original; // matching updates the bm; always start afresh.
            // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
            if (_original.match_fwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) { // tpl args will be valuated in bm.

              bm->load(&_original);
              requirements_.CS.leave();
              return STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
            }
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return WEAK_REQUIREMENT_ENABLED;
    } else { // some strong req. and some weak req.: true if among the entries complying with timings and bindings, the youngest |f->imdl is weaker than the youngest f->imdl.

      r = WEAK_REQUIREMENT_DISABLED;
      float32 negative_cfd = 0;
      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = simulated_requirements_.negative_evidences.begin(); e != simulated_requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = simulated_requirements_.negative_evidences.erase(e);
        else {

          if ((*e).evidence_->get_pred()->get_simulation(root)) {

            _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
            HLPBindingMap _original = original; // matching updates the bm; always start afresh.
            // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
            if (_original.match_fwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) {

              negative_cfd = (*e).confidence_;
              r = STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
              bm->load(&_original);
              break;
            }
          }
          ++e;
        }
      }

      for (e = simulated_requirements_.positive_evidences.begin(); e != simulated_requirements_.positive_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = simulated_requirements_.positive_evidences.erase(e);
        else {
          //(*e).f->get_reference(0)->trace();
          //f->get_reference(0)->trace();
          if ((*e).evidence_->get_pred()->get_simulation(root)) {

            _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
            HLPBindingMap _original = original; // matching updates the bm; always start afresh.
            // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
            if (_original.match_fwd_strict(_f_imdl, f_imdl)) {

              if ((*e).confidence_ >= negative_cfd) {

                r = WEAK_REQUIREMENT_ENABLED;
                bm->load(&_original);
                break;
              } else
                r = STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT;
            }
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return r;
    }
  }
}

ChainingStatus MDLController::retrieve_imdl_fwd(HLPBindingMap *bm, Fact *f_imdl, RequirementsPair &r_p, Fact *&ground, MDLController *req_controller, bool &wr_enabled) { // wr_enabled: true if there is at least one wr stronger than at least one sr.

  uint32 wr_count;
  uint32 sr_count;
  uint32 r_count = get_requirement_count(wr_count, sr_count);
  ground = NULL;
  if (!r_count)
    return NO_REQUIREMENT;
  ChainingStatus r;
  HLPBindingMap original(bm);
  if (!sr_count) { // no strong req., some weak req.: true if there is one f->imdl complying with timings and bindings.

    wr_enabled = false;
    // JTNote: We set ground = NULL above, so this is never true.
    if (ground != NULL) { // an imdl triggered the reduction of the cache.

      r_p.first.controllers.push_back(req_controller);
      r_p.first.f_imdl = ground;
      r_p.first.chaining_was_allowed = true;
      return WEAK_REQUIREMENT_ENABLED;
    }

    r = WEAK_REQUIREMENT_DISABLED;
    requirements_.CS.enter();
    auto now = Now();
    r_code::list<RequirementEntry>::const_iterator e;
    for (e = requirements_.positive_evidences.begin(); e != requirements_.positive_evidences.end();) {

      Code *imdl = (*e).evidence_->get_pred()->get_target()->get_reference(0);
      uint16 tpl_index = imdl->code(I_HLP_TPL_ARGS).asIndex();
      //std::cout<<"IMDL: "<<imdl->code(tpl_index+1).asFloat()<<" ["<<Time::ToString_seconds((*e).after-Utils::GetTimeReference())<<" "<<Time::ToString_seconds((*e).before-Utils::GetTimeReference())<<"]"<<std::endl;

      if ((*e).is_too_old(now)) // garbage collection.
        e = requirements_.positive_evidences.erase(e);
      else if ((*e).is_out_of_range(now))
        ++e;
      else {

        _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
        //_f_imdl->get_reference(0)->trace();
        //f_imdl->get_reference(0)->trace();
        HLPBindingMap _original = original; // matching updates the bm; always start afresh.
        if (_original.match_fwd_strict(_f_imdl, f_imdl)) { // tpl args will be valuated in bm, but not in f_imdl yet.
#ifdef WITH_DEBUG_OID
          OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " fact (" << f_imdl->get_debug_oid() << ") imdl mdl " <<
            f_imdl->get_reference(0)->get_reference(0)->get_oid() << " matches evidence fact (" <<
            _f_imdl->get_debug_oid() << ") imdl mdl " << _f_imdl->get_reference(0)->get_reference(0)->get_oid());
#endif
          if (r == WEAK_REQUIREMENT_DISABLED && (*e).chaining_was_allowed_) { // first match.

            r = WEAK_REQUIREMENT_ENABLED;
            bm->load(&_original);
            ground = (*e).evidence_;
            //std::cout<<"Chosen IMDL: "<<imdl->code(tpl_index+1).asFloat()<<" ["<<Time::ToString_seconds((*e).after-Utils::GetTimeReference())<<" "<<Time::ToString_seconds((*e).before-Utils::GetTimeReference())<<"]"<<std::endl;
          }

          r_p.first.controllers.push_back((*e).controller_);
          r_p.first.f_imdl = _f_imdl;
          r_p.first.chaining_was_allowed = (*e).chaining_was_allowed_;
        }
        ++e;
      }
    }

    requirements_.CS.leave();
    return r;
  } else {

    if (!wr_count) { // some strong req., no weak req.: true if there is no |f->imdl complying with timings and bindings.

      wr_enabled = false;
      r = WEAK_REQUIREMENT_ENABLED;
      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = requirements_.negative_evidences.begin(); e != requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = requirements_.positive_evidences.erase(e);
        else if ((*e).is_out_of_range(now))
          ++e;
        else {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          if (_original.match_fwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) { // tpl args will be valuated in bm.

            if (r == WEAK_REQUIREMENT_ENABLED && (*e).chaining_was_allowed_) // first match.
              r = STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;

            r_p.second.controllers.push_back((*e).controller_);
            r_p.second.f_imdl = _f_imdl;
            r_p.second.chaining_was_allowed = (*e).chaining_was_allowed_;
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return r;
    } else { // some strong req. and some weak req.: true if among the entries complying with timings and bindings, the youngest |f->imdl is weaker than the youngest f->imdl.

      r = NO_REQUIREMENT;
      requirements_.CS.enter();
      float32 negative_cfd = 0;
      auto now = Now();

      r_code::list<RequirementEntry>::const_iterator e;
      for (e = requirements_.negative_evidences.begin(); e != requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = requirements_.negative_evidences.erase(e);
        else if ((*e).is_out_of_range(now))
          ++e;
        else {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          if (_original.match_fwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) {

            if (r == NO_REQUIREMENT && (*e).chaining_was_allowed_) { // first match.

              negative_cfd = (*e).confidence_;
              r = STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
            }

            r_p.second.controllers.push_back((*e).controller_);
            r_p.second.f_imdl = _f_imdl;
            r_p.second.chaining_was_allowed = (*e).chaining_was_allowed_;
          }
          ++e;
        }
      }

      if (ground != NULL) { // an imdl triggered the reduction of the cache.

        requirements_.CS.leave();
        float32 confidence = ground->get_pred()->get_target()->get_cfd();
        if (confidence >= negative_cfd) {

          r = WEAK_REQUIREMENT_ENABLED;
          r_p.first.controllers.push_back(req_controller);
          r_p.first.f_imdl = ground;
          r_p.first.chaining_was_allowed = true;
          wr_enabled = true;
        }
        return r;
      } else for (e = requirements_.positive_evidences.begin(); e != requirements_.positive_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = requirements_.positive_evidences.erase(e);
        else if ((*e).is_out_of_range(now))
          ++e;
        else {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          if (_original.match_fwd_strict(_f_imdl, f_imdl)) {

            if (r != WEAK_REQUIREMENT_ENABLED && (*e).chaining_was_allowed_) { // first siginificant match.

              if ((*e).confidence_ >= negative_cfd) {

                r = WEAK_REQUIREMENT_ENABLED;
                ground = (*e).evidence_;
                wr_enabled = true;
              } else {

                r = STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT;
                wr_enabled = false;
              }
              bm->load(&_original);
            }

            r_p.first.controllers.push_back((*e).controller_);
            r_p.first.f_imdl = _f_imdl;
            r_p.first.chaining_was_allowed = (*e).chaining_was_allowed_;
          }
          ++e;
        }
      }
      requirements_.CS.leave();
      return r;
    }
  }
}

ChainingStatus MDLController::retrieve_imdl_bwd(HLPBindingMap *bm, Fact *f_imdl, Fact *&ground) {

  uint32 wr_count;
  uint32 sr_count;
  uint32 r_count = get_requirement_count(wr_count, sr_count);
  ground = NULL;
  if (!r_count)
    return NO_REQUIREMENT;
  ChainingStatus r;
  HLPBindingMap original(bm);
  if (!sr_count) { // no strong req., some weak req.: true if there is one f->imdl complying with timings and bindings.

    r = WEAK_REQUIREMENT_DISABLED;
    requirements_.CS.enter();
    auto now = Now();
    r_code::list<RequirementEntry>::const_iterator e;
    for (e = requirements_.positive_evidences.begin(); e != requirements_.positive_evidences.end();) {

      if ((*e).is_too_old(now)) // garbage collection.
        e = requirements_.positive_evidences.erase(e);
      else {

        _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
        //_f_imdl->get_reference(0)->trace();
        //f_imdl->get_reference(0)->trace();
        HLPBindingMap _original = original; // matching updates the bm; always start afresh.
        // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
        if (_original.match_fwd_strict(_f_imdl, f_imdl)) { // tpl args will be valuated in bm, but not in f_imdl yet.

          r = WEAK_REQUIREMENT_ENABLED;
          bm->load(&_original);
          ground = (*e).evidence_;
          break;
        }
        ++e;
      }
    }

    requirements_.CS.leave();
    return r;
  } else {

    if (!wr_count) { // some strong req., no weak req.: true if there is no |f->imdl complying with timings and bindings.

      ground = NULL;

      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = requirements_.negative_evidences.begin(); e != requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = requirements_.negative_evidences.erase(e);
        else {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
          if (_original.match_fwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) { // tpl args will be valuated in bm.

            requirements_.CS.leave();
            return STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return WEAK_REQUIREMENT_ENABLED;
    } else { // some strong req. and some weak req.: true if among the entries complying with timings and bindings, the youngest |f->imdl is weaker than the youngest f->imdl.

      r = WEAK_REQUIREMENT_DISABLED;
      float32 negative_cfd = 0;
      requirements_.CS.enter();
      auto now = Now();
      r_code::list<RequirementEntry>::const_iterator e;
      for (e = requirements_.negative_evidences.begin(); e != requirements_.negative_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = requirements_.negative_evidences.erase(e);
        else {

          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
          if (_original.match_fwd_lenient(_f_imdl, f_imdl) == MATCH_SUCCESS_NEGATIVE) {

            negative_cfd = (*e).confidence_;
            r = STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT;
            break;
          }
          ++e;
        }
      }

      for (e = requirements_.positive_evidences.begin(); e != requirements_.positive_evidences.end();) {

        if ((*e).is_too_old(now)) // garbage collection.
          e = requirements_.positive_evidences.erase(e);
        else {
          //(*e).f->get_reference(0)->trace();
          //f->get_reference(0)->trace();
          _Fact *_f_imdl = (*e).evidence_->get_pred()->get_target();
          HLPBindingMap _original = original; // matching updates the bm; always start afresh.
          // Use match_fwd because the f_imdl time interval matches the binding map's fwd_after and fwd_before from the model LHS.
          if (_original.match_fwd_strict(_f_imdl, f_imdl)) {

            if ((*e).confidence_ >= negative_cfd) {

              r = WEAK_REQUIREMENT_ENABLED;
              ground = (*e).evidence_;
              bm->load(&_original);
              break;
            } else
              r = STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT;
          }
          ++e;
        }
      }

      requirements_.CS.leave();
      return r;
    }
  }
}


// jm Replaced template definition with make_pair
void MDLController::register_requirement(_Fact *f_pred, RequirementsPair &r_p) {

  if (r_p.first.controllers.size() > 0 || r_p.second.controllers.size() > 0)
    active_requirements_.insert(std::make_pair(f_pred, r_p));
}

bool MDLController::get_imdl_template_timings(
    r_code::Code* imdl, Timestamp& after, Timestamp& before, uint16* after_ts_index, uint16* before_ts_index) {
  auto template_set_index = imdl->code(I_HLP_TPL_ARGS).asIndex();
  auto template_set_count = imdl->code(template_set_index).getAtomCount();
  auto template_after_index = template_set_index + (template_set_count - 1);
  auto template_before_index = template_set_index + template_set_count;
  if (!(template_set_count >= 2 &&
        imdl->code(template_after_index).getDescriptor() == Atom::I_PTR &&
        imdl->code(template_before_index).getDescriptor() == Atom::I_PTR))
    return false;

  after = Utils::GetTimestamp(imdl, template_after_index);
  before = Utils::GetTimestamp(imdl, template_before_index);
  if (after_ts_index)
    *after_ts_index = imdl->code(template_after_index).asIndex();
  if (before_ts_index)
    *before_ts_index = imdl->code(template_before_index).asIndex();

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MDLController::RequirementEntry::RequirementEntry() : PredictedEvidenceEntry(), controller_(NULL), chaining_was_allowed_(false) {
}

MDLController::RequirementEntry::RequirementEntry(_Fact *f_p_f_imdl, MDLController *c, bool chaining_was_allowed) : PredictedEvidenceEntry(f_p_f_imdl), controller_(c), chaining_was_allowed_(chaining_was_allowed) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PMDLController::PMDLController(r_code::View *view) : MDLController(view) {
}

void PMDLController::add_g_monitor(_GMonitor *m) {

  g_monitorsCS_.enter();
  g_monitors_.push_front(m);
  g_monitorsCS_.leave();
}

void PMDLController::remove_g_monitor(_GMonitor *m) {

  g_monitorsCS_.enter();
  g_monitors_.remove(m);
  g_monitorsCS_.leave();
}

void PMDLController::add_r_monitor(_GMonitor *m) {

  g_monitorsCS_.enter();
  r_monitors_.push_front(m);
  g_monitorsCS_.leave();
}

void PMDLController::remove_r_monitor(_GMonitor *m) {

  g_monitorsCS_.enter();
  r_monitors_.remove(m);
  g_monitorsCS_.leave();
}

void PMDLController::inject_goal(HLPBindingMap *bm, Fact *goal, Fact *f_imdl) const {

  Group *primary_grp = get_host();
  auto before = goal->get_before();
  auto now = Now();
  int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_grp, now, before - now);

  View *view = new View(View::SYNC_ONCE, now, 1, resilience, primary_grp, primary_grp, goal); // SYNC_ONCE,res=resilience.
  _Mem::Get()->inject(view);

  MkRdx *mk_rdx = new MkRdx(f_imdl, goal->get_goal()->get_super_goal(), goal, 1, bm);

  uint16 out_group_count = get_rdx_out_group_count();
  for (uint16 i = 0; i < out_group_count; ++i) {

    Group *out_group = (Group *)get_out_group(i);
    View *view = new NotificationView(primary_grp, out_group, mk_rdx);
    _Mem::Get()->inject_notification(view, true);
  }

  OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl "
    << f_imdl->get_reference(0)->get_reference(0)->get_oid() << " abduce -> mk.rdx " << mk_rdx->get_oid());
}

void PMDLController::inject_simulation(Fact *goal_pred) const { // f->pred->f->obj or f->goal->f->obj.

  Group *primary_grp = get_host();
  auto before = ((_Fact *)goal_pred->get_reference(0)->get_reference(0))->get_before();
  auto now = Now();
  int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_grp, now, before - now);

  View *view = new View(View::SYNC_ONCE, now, 1, resilience, primary_grp, primary_grp, goal_pred); // SYNC_ONCE,res=resilience.
  _Mem::Get()->inject(view);
}

bool PMDLController::monitor_goals(_Fact *input) {

  bool r = false;
  r_code::list<P<_GMonitor> >::const_iterator m;
  g_monitorsCS_.enter();
  for (m = g_monitors_.begin(); m != g_monitors_.end();) {

    if ((*m)->reduce(input)) {

      m = g_monitors_.erase(m);
      r = true;
    } else
      ++m;
  }
  g_monitorsCS_.leave();
  return r;
}

void PMDLController::register_predicted_goal_outcome(Fact *goal, HLPBindingMap *bm, Fact *f_imdl, bool success, bool injected_goal) { // called only for SIM_COMMITTED mode.

  if (success)
    goal->invalidate(); // monitor still running to detect failures (actual or predicted).
  else {

    if (!injected_goal) // the goal has not been injected; monitor still running.
      inject_goal(bm, goal, f_imdl);
    else {

      if (goal->is_invalidated()) { // the only case when the goal can be invalidated here is when a predicted failure follows a predicted success.

        Fact *new_goal = new Fact(goal);
        Goal *g = new_goal->get_goal();
        auto now = Now();
        auto deadline = g->get_target()->get_before();
        auto sim_thz = get_sim_thz(now, deadline);

        Sim *new_sim = new Sim(SIM_ROOT, sim_thz, g->get_sim()->get_f_super_goal(), false, this, 1);

        g->set_sim(new_sim);

        add_g_monitor(new GMonitor(this, bm, deadline, now + sim_thz, new_goal, f_imdl, NULL));

        inject_goal(bm, new_goal, f_imdl);
      }
    }
  }
}

inline microseconds PMDLController::get_sim_thz(Timestamp now, Timestamp deadline) const {

  auto min_sim_thz = _Mem::Get()->get_min_sim_time_horizon(); // time allowance for the simulated predictions to flow upward.
  auto sim_thz = _Mem::Get()->get_sim_time_horizon(deadline - now);
  if (sim_thz > min_sim_thz) {

    sim_thz -= min_sim_thz;
    auto max_sim_thz = _Mem::Get()->get_max_sim_time_horizon();
    if (sim_thz > max_sim_thz)
      sim_thz = max_sim_thz;
    return sim_thz;
  } else // no time to simulate.
    return microseconds(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TopLevelMDLController::TopLevelMDLController(r_code::View *view) : PMDLController(view) {
}

void TopLevelMDLController::store_requirement(_Fact *f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation) {
}

void TopLevelMDLController::take_input(r_exec::View *input) {

  if (input->object_->code(0).asOpcode() == Opcodes::Fact ||
    input->object_->code(0).asOpcode() == Opcodes::AntiFact) // discard everything but facts and |facts.
    Controller::__take_input<TopLevelMDLController>(input);
}

void TopLevelMDLController::reduce(r_exec::View *input) { // no lock.

  if (input->object_->is_invalidated())
    return;

  Goal *goal = ((_Fact *)input->object_)->get_goal();
  if (goal && goal->is_drive()) {

    _Fact *goal_target = goal->get_target(); // goal_target is f->object.
    float32 confidence = get_cfd()*goal_target->get_cfd(); // reading STRONG_REQUIREMENT is atomic.
    if (confidence <= get_host()->code(GRP_SLN_THR).asFloat()) // cfd is too low for any sub-goal to be injected.
      return;

    P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
    bm->reset_bwd_timings(goal_target);
    if (bm->match_bwd_strict(goal_target, rhs_)) // the rhs of a top-level model is never a |fact, hence strict matching instead of lenient.
      abduce(bm, (Fact *)input->object_, confidence);
    else if (!goal->is_requirement()) { // goal_target may be f->imdl and not a requirement: case of a reuse of the model, i.e. the goal target is for the model to make a prediction: this translates into making a sub-goal from the lhs.

      Code *imdl = goal_target->get_reference(0);
      if (imdl->code(0).asOpcode() == Opcodes::IMdl && imdl->get_reference(0) == getObject()) { // in that case, get the bm from the imdl, ignore the bwd guards, bind the rhs and inject.

        bm = new HLPBindingMap(bindings_);
        bm->reset_bwd_timings(goal_target);
        bm->init_from_f_ihlp(goal_target);
        abduce(bm, (Fact *)input->object_, confidence);
      }
    }
  } else {

    PrimaryMDLOverlay o(this, bindings_);
    o.reduce(input->object_, NULL, NULL); // matching is used to fill up the cache (no predictions).

    monitor_goals(input->object_);
  }
}

void TopLevelMDLController::abduce(HLPBindingMap *bm, Fact *super_goal, float32 confidence) { // super_goal is a drive.

  if (evaluate_bwd_guards(bm)) { // bm may be updated.

    P<_Fact> bound_lhs = (_Fact *)bm->bind_pattern(get_lhs());
    _Fact *evidence;
    Fact *f_imdl;

    switch (check_evidences(bound_lhs, evidence)) {
    case MATCH_SUCCESS_POSITIVE: // goal target is already known: report drive success.
      register_drive_outcome(super_goal, true);
      break;
    case MATCH_SUCCESS_NEGATIVE: // a counter evidence is already known: report drive failure.
      register_drive_outcome(super_goal, false);
      break;
    case MATCH_FAILURE:
      f_imdl = get_f_ihlp(bm, false);
      f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
      bound_lhs->set_cfd(confidence);
      switch (check_predicted_evidences(bound_lhs, evidence)) {
      case MATCH_SUCCESS_POSITIVE:
        break;
      case MATCH_SUCCESS_NEGATIVE:
      case MATCH_FAILURE:
        evidence = NULL;
        break;
      }
      abduce_lhs(bm, super_goal, bound_lhs, f_imdl, evidence);
      break;
    }
  }
}

void TopLevelMDLController::abduce_lhs(HLPBindingMap *bm,
  Fact *super_goal, // f->g->f->obj; actual goal.
  _Fact *sub_goal_target, // f->obj, i.e. bound lhs.
  Fact *f_imdl,
  _Fact *evidence) {

  auto now = Now();
  auto deadline = sub_goal_target->get_before();
  auto sim_thz = get_sim_thz(now, deadline);
  Sim *sub_sim = new Sim(SIM_ROOT, sim_thz, super_goal, false, this, 1);
  // Register the goal to make sure it doesn't become a subgoal.
  sub_sim->registerGoalTarget(sub_goal_target);

  Goal *sub_goal = new Goal(sub_goal_target, super_goal->get_goal()->get_actor(), sub_sim, 1);
  Fact *f_sub_goal = new Fact(sub_goal, now, now, 1, 1);

  if (!evidence)
    inject_goal(bm, f_sub_goal, f_imdl);
  add_g_monitor(new GMonitor(this, bm, deadline, now + sim_thz, f_sub_goal, f_imdl, evidence));
  OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " -> fact " << f_sub_goal->get_oid() << 
    " goal [" << Utils::RelativeTime(sub_goal_target->get_after()) << "," << Utils::RelativeTime(sub_goal_target->get_before()) << "]");
}

void TopLevelMDLController::predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground) { // no prediction here.
}

void TopLevelMDLController::register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures) {
}

void TopLevelMDLController::register_goal_outcome(Fact *goal, bool success, _Fact *evidence) const {

  goal->invalidate();

  auto now = Now();
  Code *goal_success;
  Code *f_goal_success;
  _Fact *absentee;
  if (success) {

    goal_success = new Success(goal, evidence, 1);
    f_goal_success = new Fact(goal_success, now, now, 1, 1);
    absentee = NULL;
  } else {

    if (!evidence) { // assert absence of the goal target.

      absentee = goal->get_goal()->get_target()->get_absentee();
      goal_success = new Success(goal, absentee, 1);
    } else {

      absentee = NULL;
      goal_success = new Success(goal, evidence, 1);
    }
    f_goal_success = new AntiFact(goal_success, now, now, 1, 1);
  }

  Group *primary_host = get_host();
  uint16 out_group_count = get_out_group_count() - 1;
  Group *drives_host = (Group *)get_out_group(out_group_count); // the drives group is the last of the output groups.
  for (uint16 i = 0; i < out_group_count; ++i) { // inject notification in out groups (drives host excepted).

    Group *out_group = (Group *)get_out_group(i);
    int32 resilience = _Mem::Get()->get_goal_pred_success_res(out_group, now, seconds(0));
    View *view = new View(View::SYNC_ONCE, now, 1, resilience, out_group, primary_host, f_goal_success);
    _Mem::Get()->inject(view);

    if (absentee) {

      view = new View(View::SYNC_ONCE, now, 1, 1, out_group, primary_host, absentee);
      _Mem::Get()->inject(view);
    }
  }

  register_drive_outcome(goal->get_goal()->get_sim()->get_f_super_goal(), success);
  if (success) OUTPUT_LINE(GOAL_MON, Utils::RelativeTime(Now()) << " fact " << f_goal_success->get_oid() << ": " << goal->get_oid() << " goal success (TopLevel)");
  else OUTPUT_LINE(GOAL_MON, Utils::RelativeTime(Now()) << " " << goal->get_oid() << " goal failure (TopLevel)");
}

void TopLevelMDLController::register_drive_outcome(Fact *drive, bool success) const {

  drive->invalidate();

  auto now = Now();
  Code *drive_success = new Success(drive, NULL, 1);
  Code *f_drive_success;
  if (success)
    f_drive_success = new Fact(drive_success, now, now, 1, 1);
  else
    f_drive_success = new AntiFact(drive_success, now, now, 1, 1);

  Group *primary_host = get_host();
  uint16 out_group_count = get_out_group_count() - 1;
  Group *drives_host = (Group *)get_out_group(out_group_count); // the drives group is the last of the output groups.
  View *view = new View(View::SYNC_ONCE, now, 1, 1, drives_host, primary_host, f_drive_success); // sln=1,res=1.
  _Mem::Get()->inject(view); // inject in the drives group (will be caught by the drive injectors).
  //if(success)std::cout<<Utils::RelativeTime(Now())<<" drive success\n";
  //else std::cout<<Utils::RelativeTime(Now())<<" drive failure\n";
}

void TopLevelMDLController::register_simulated_goal_outcome(Fact *goal, bool success, _Fact *evidence) const { // evidence is a simulated prediction.

  Code *success_object = new Success(goal, evidence, 1);
  Pred *evidence_pred = evidence->get_pred();
  float32 confidence = evidence_pred->get_target()->get_cfd();
  auto now = Now();
  _Fact *f_success_object;
  if (success)
    f_success_object = new Fact(success_object, now, now, confidence, 1);
  else
    f_success_object = new AntiFact(success_object, now, now, confidence, 1);

  Pred *pred = new Pred(f_success_object, evidence_pred, 1);
  Fact *f_pred = new Fact(pred, now, now, 1, 1);

  Group *primary_host = get_host();
  int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_host, now, seconds(0));
  View *view = new View(View::SYNC_ONCE, now, 1, resilience, primary_host, primary_host, f_pred);
  _Mem::Get()->inject(view); // inject in the primary group.
}

void TopLevelMDLController::register_req_outcome(Fact *f_pred, bool success, bool rate_failures) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PrimaryMDLController::PrimaryMDLController(r_code::View *view) : PMDLController(view) {
}

void PrimaryMDLController::set_secondary(SecondaryMDLController *secondary) {

  secondary_ = secondary;
  add_requirement_to_rhs();
  secondary->add_requirement_to_rhs();
}

void PrimaryMDLController::store_requirement(_Fact *f_p_f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation) {

  _Fact *f_imdl = f_p_f_imdl->get_pred()->get_target();
  Code *mdl = f_imdl->get_reference(0);
  RequirementEntry e(f_p_f_imdl, controller, chaining_was_allowed);
  if (f_imdl->is_fact()) { // in case of a positive requirement tell monitors they can check for chaining again.
    // Store the requirement before signaling the monitor so that its target will match the requirement.
    if (is_simulation)
      _store_requirement(&simulated_requirements_.positive_evidences, e);
    else
      _store_requirement(&requirements_.positive_evidences, e);

    r_code::list<P<_GMonitor> >::const_iterator m;
    g_monitorsCS_.enter();
    for (m = r_monitors_.begin(); m != r_monitors_.end();) { // signal r-monitors.

      if (!(*m)->is_alive())
        m = r_monitors_.erase(m);
      else {

        // If the requirement is simulated, then pass its Sim as the forwardSimulation for check_simulated_imdl.
        if (f_p_f_imdl->get_pred()->get_simulations_size() > 1)
          std::cerr << "WARNING: store_requirement: Prediction has multiple simulations. Passing NULL forwardSimulation to r-monitor" << std::endl;
        if ((*m)->signal(is_simulation, f_p_f_imdl->get_pred()->get_simulations_size() == 1 ? f_p_f_imdl->get_pred()->get_simulation((uint16)0) : NULL))
          m = r_monitors_.erase(m);
        else
          ++m;
      }
    }
    g_monitorsCS_.leave();

    reduce_cache<PrimaryMDLController>((Fact *)f_p_f_imdl, controller);
  } else if (!is_simulation)
    _store_requirement(&requirements_.negative_evidences, e);

  if (!is_simulation)
    secondary_->store_requirement(f_p_f_imdl, controller, chaining_was_allowed, false);
}

void PrimaryMDLController::take_input(r_exec::View *input) {

  if (become_invalidated())
    return;

  if (input->object_->code(0).asOpcode() == Opcodes::Fact ||
    input->object_->code(0).asOpcode() == Opcodes::AntiFact) // discard everything but facts and |facts.
    Controller::__take_input<PrimaryMDLController>(input);
}

void PrimaryMDLController::predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground) {

  _Fact *bound_rhs = (_Fact *)bm->bind_pattern(rhs_); // fact or |fact.
  Code* bound_rhs_value = bound_rhs->get_reference(0);
  if (bound_rhs_value->code(0).asOpcode() == Opcodes::IMdl ||
      bound_rhs_value->code(0).asOpcode() == Opcodes::ICst) {
    // Change all VL_PTR in the set of exposed values to wildcard so that a future 
    // prediction can include it as "ground" in the mk.rdx, and it can be decompiled.
    uint16 exposed_values_index = bound_rhs_value->code(I_HLP_EXPOSED_ARGS).asIndex();
    uint16 exposed_values_count = bound_rhs_value->code(exposed_values_index).getAtomCount();
    for (uint16 i = 1; i <= exposed_values_count; ++i) {
      Atom atom = bound_rhs_value->code(exposed_values_index + i);
      if (atom.getDescriptor() == Atom::VL_PTR)
        bound_rhs_value->code(exposed_values_index + i) = Atom::Wildcard();
    }
  }

  bool is_simulation;
  float32 confidence;
  Pred *prediction = input->get_pred();
  if (prediction) { // the input was a prediction.

    is_simulation = prediction->is_simulation();
    if (chaining_was_allowed)
      confidence = prediction->get_target()->get_cfd()*get_cfd();
    else
      return;
  } else {

    is_simulation = false;
    if (chaining_was_allowed)
      confidence = input->get_cfd()*get_cfd();
    else
      confidence = 0;
  }

  bound_rhs->set_cfd(confidence);

  Pred *pred;
  if (is_simulation)
    pred = new Pred(bound_rhs, prediction, 1);
  else
    pred = new Pred(bound_rhs, 1);
  auto now = Now();
  Fact *production = new Fact(pred, now, now, 1, 1);

  if (prediction && !is_simulation) { // store the antecedents.

    pred->grounds_.push_back(input);
    if (ground)
      pred->grounds_.push_back(ground);
  }

  if (is_requirement()) {

    if (is_invalidated())
      // Another thread has invalidated this controller which clears the controllers.
      return;
    // In the Pred constructor, we already copied the simulations from prediction.
    PrimaryMDLController *c = (PrimaryMDLController *)controllers_[RHSController]; // rhs controller: in the same view.
    c->store_requirement(production, this, chaining_was_allowed, is_simulation); // if not simulation, stores also in the secondary controller.
#ifdef WITH_DEBUG_OID
    OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " fact (" << f_imdl->get_debug_oid() << ") imdl mdl " << getObject()->get_oid() <<
      ": " << input->get_oid() << " -> fact (" << production->get_debug_oid() << ") pred fact (" <<
      bound_rhs->get_debug_oid() << ") imdl mdl " << bound_rhs->get_reference(0)->get_reference(0)->get_oid());
#else
    OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << ": " << input->get_oid() << 
      " -> fact pred fact imdl mdl " << bound_rhs->get_reference(0)->get_reference(0)->get_oid());
#endif
    return;
  }

  if (!is_simulation) { // rdx and monitor only for predictions built from actual inputs.

    register_requirement(production, r_p);

    if (!chaining_was_allowed) { // reaching this point in the code means that the input was not a prediction.

      PMonitor *m = new PMonitor(this, bm, production, false); // the model will not be rated in case of a failure; the requirements will be rated in both cases (if their own chaining was allowed, else only in case of success and recurse).
      MDLController::add_monitor(m);
    } else { // try to inject the prediction: if cfd too low, the prediction is not injected.

      auto before = bound_rhs->get_before();
      if (before <= now) // can happen if the input comes from the past and the predicted time is still in the past.
        return;
      if (prediction) { // no rdx nor monitoring if the input was a prediction; case of a reuse: f_imdl becomes f->p->f_imdl.

        Pred *pred_f_imdl;
        if (is_simulation)
          pred_f_imdl = new Pred(f_imdl, prediction, 1);
        else
          pred_f_imdl = new Pred(f_imdl, 1);
        Fact *f_pred_f_imdl = new Fact(pred_f_imdl, now, now, 1, 1);
        inject_prediction(production, f_pred_f_imdl, confidence, before - now, NULL);
        string f_imdl_info;
#ifdef WITH_DEBUG_OID
        f_imdl_info = " fact (" + to_string(f_imdl->get_debug_oid()) + ") imdl";
#endif
        OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << f_imdl_info << " mdl " << 
          getObject()->get_oid() << ": " << input->get_oid() << " -> fact " << production->get_oid() << 
          " pred fact mk.val VALUE " << bound_rhs->get_reference(0)->traceString(MK_VAL_VALUE));
      } else {

        Code *mk_rdx;
        if (ground)
          mk_rdx = new MkRdx(f_imdl, (Code *)input, ground, production, 1, bindings_);
        else
          mk_rdx = new MkRdx(f_imdl, (Code *)input, production, 1, bindings_);
        bool rate_failures = inject_prediction(production, f_imdl, confidence, before - now, mk_rdx);
        PMonitor *m = new PMonitor(this, bm, production, rate_failures); // not-injected predictions are monitored for rating the model that produced them (successes only).
        MDLController::add_monitor(m);
        Group *secondary_host = secondary_->getView()->get_host(); // inject f_imdl in secondary group.
        View *view = new View(View::SYNC_ONCE, now, confidence, 1, getView()->get_host(), secondary_host, f_imdl); // SYNC_ONCE,res=resilience.
        _Mem::Get()->inject(view);
        OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " predict -> mk.rdx " << mk_rdx->get_oid());
        string f_imdl_info;
#ifdef WITH_DEBUG_OID
        f_imdl_info = "(" + to_string(f_imdl->get_debug_oid()) + ")";
#endif
        OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " fact " << f_imdl->get_oid() << f_imdl_info << " imdl mdl " << 
          getObject()->get_oid() << ": " << input->get_oid() << " -> fact " << production->get_oid() << 
          " pred fact mk.val VALUE " << bound_rhs->get_reference(0)->traceString(MK_VAL_VALUE));
      }
    }
  } else { // no monitoring for simulated predictions.

    // In the Pred constructor, we already copied the simulations from prediction.
    HLPController::inject_prediction(production, confidence); // inject a simulated prediction in the primary group.
    OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << ": fact " <<
      input->get_oid() << " pred -> fact " << production->get_oid() << " simulated pred");
  }
}

bool PrimaryMDLController::inject_prediction(Fact *prediction, Fact *f_imdl, float32 confidence, Timestamp::duration time_to_live, Code *mk_rdx) const { // prediction: f->pred->f->target.

  auto now = Now();
  Group *primary_host = get_host();
  float32 sln_thr = primary_host->code(GRP_SLN_THR).asFloat();
  if (confidence > sln_thr) { // do not inject if cfd is too low.

    int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_host, now, time_to_live);
    View *view = new View(View::SYNC_ONCE, now, confidence, resilience, primary_host, primary_host, prediction); // SYNC_ONCE,res=resilience.
    _Mem::Get()->inject(view);

    view = new View(View::SYNC_ONCE, now, 1, 1, primary_host, primary_host, f_imdl); // SYNC_ONCE,res=resilience.
    _Mem::Get()->inject(view);

    if (mk_rdx) {

      uint16 out_group_count = get_out_group_count();
      for (uint16 i = 0; i < out_group_count; ++i) {

        Group *out_group = (Group *)get_out_group(i);
        View *view = new NotificationView(primary_host, out_group, mk_rdx);
        _Mem::Get()->inject_notification(view, true);
      }
    }
    return true;
  } else
    return false;
}

void PrimaryMDLController::reduce(r_exec::View *input) { // no lock.

  if (is_orphan())
    return;

  if (input->object_->is_invalidated())
    return;

  r_code::list<P<Code> >::const_iterator a;
  assumptionsCS_.enter();
  for (a = assumptions_.begin(); a != assumptions_.end();) { // ignore home-made assumptions and perform some garbage collection.

    if ((*a)->is_invalidated()) // garbage collection.
      a = assumptions_.erase(a);
    else if (((Code *)*a) == input->object_) {

      a = assumptions_.erase(a);
      assumptionsCS_.leave();
      break;
    } else
      ++a;
  }
  assumptionsCS_.leave();

  Goal *goal = ((_Fact *)input->object_)->get_goal();
  if (goal && goal->is_self_goal() && !goal->is_drive()) {

    _Fact *goal_target = goal->get_target(); // goal_target is f->object.
    float32 confidence = get_cfd()*goal_target->get_cfd(); // reading STRONG_REQUIREMENT is atomic.
    Code *host = get_host();
    if (confidence <= host->code(GRP_SLN_THR).asFloat()) // cfd is too low for any sub-goal to be injected.
      return;

    P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
    bm->reset_bwd_timings(goal_target);
    bool opposite = false;
    MatchResult match_result = bm->match_bwd_lenient(goal_target, rhs_);
    switch (match_result) {
    case MATCH_SUCCESS_NEGATIVE:
      opposite = true;
    case MATCH_SUCCESS_POSITIVE:
      abduce(bm, (Fact *)input->object_, opposite, confidence);
      break;
    default: // no match; however, goal_target may be f->imdl, i.e. case of a reuse of the model, i.e. the goal is for the model to make a prediction: this translates into making a sub-goal from the lhs.
      if (!goal->is_requirement()) { // models like imdl -> |rhs or |imdl -> rhs are not allowed.

        Code *imdl = goal_target->get_reference(0);
        if (imdl->code(0).asOpcode() == Opcodes::IMdl && imdl->get_reference(0) == getObject()) { // in that case, get the bm from the imdl, ignore the bwd guards, bind the rhs and inject.

          bm = new HLPBindingMap(bindings_);
          bm->reset_bwd_timings(goal_target);
          bm->init_from_f_ihlp(goal_target);
          // JTNote: opposite is always false.
          abduce(bm, (Fact *)input->object_, opposite, confidence);
        }
      }
      break;
    }
  } else {

    PrimaryMDLOverlay o(this, bindings_);
    bool match = o.reduce((_Fact *)input->object_, NULL, NULL);
    if (!match && !monitor_predictions((_Fact *)input->object_) && !monitor_goals((_Fact *)input->object_))
      assume((_Fact *)input->object_);
    check_last_match_time(match);
  }
}

void PrimaryMDLController::debug(View *input) {
}

void PrimaryMDLController::reduce_batch(Fact *f_p_f_imdl, MDLController *controller) {

  if (is_orphan())
    return;

  reduce_cache<EvidenceEntry>(&evidences_, f_p_f_imdl, controller);
  reduce_cache<PredictedEvidenceEntry>(&predicted_evidences_, f_p_f_imdl, controller);
}

void PrimaryMDLController::abduce(HLPBindingMap *bm, Fact *super_goal, bool opposite, float32 confidence, bool allow_simulation) { // goal is f->g->f->object or f->g->|f->object; called concurrently by redcue() and _GMonitor::update().

  if (!abduction_allowed(bm))
    return;

  P<Fact> f_imdl = get_f_ihlp(bm, false);
  Sim *sim = super_goal->get_goal()->get_sim();
  auto sim_thz = sim->get_thz(); // 0 if super-goal had no time for simulation.
  auto min_sim_thz = _Mem::Get()->get_min_sim_time_horizon() / 2; // time allowance for the simulated predictions to flow upward.
  if (!allow_simulation)
    // Force the time horizon to zero.
    sim_thz = seconds(0);

  Sim *sub_sim;
  if (sim_thz > min_sim_thz) {

    sim_thz -= min_sim_thz;

    f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.

    auto now = Now();
    switch (sim->get_mode()) {
    case SIM_ROOT:
      sub_sim = new Sim(opposite ? SIM_MANDATORY : SIM_OPTIONAL, sim_thz, super_goal, opposite, sim->root_, 1, this, confidence, Timestamp(seconds(0)));
      break;
    case SIM_OPTIONAL:
    case SIM_MANDATORY:
      sub_sim = new Sim(sim->get_mode(), sim_thz, sim->get_f_super_goal(), opposite, sim->root_, 1, sim->solution_controller_, sim->get_solution_cfd(), sim->get_solution_before());
      break;
    }

    Fact *ground;
    switch (retrieve_imdl_bwd(bm, f_imdl, ground)) {
    case WEAK_REQUIREMENT_ENABLED:
      f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(true);
    case NO_REQUIREMENT:
      if (sub_sim->get_mode() == SIM_ROOT)
        abduce_lhs(bm, super_goal, f_imdl, opposite, confidence, sub_sim, ground, true);
      else
        abduce_simulated_lhs(bm, super_goal, f_imdl, opposite, confidence, sub_sim, NULL);
      break;
    default: // WEAK_REQUIREMENT_DISABLED, STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT or STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT.
      switch (retrieve_simulated_imdl_bwd(bm, f_imdl, sim->root_)) {
      case WEAK_REQUIREMENT_ENABLED:
        f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(true);
      case NO_REQUIREMENT:
        if (sub_sim->get_mode() == SIM_ROOT)
          abduce_lhs(bm, super_goal, f_imdl, opposite, confidence, sub_sim, NULL, true);
        else
          abduce_simulated_lhs(bm, super_goal, f_imdl, opposite, confidence, sub_sim, NULL);
        break;
      default: // WEAK_REQUIREMENT_DISABLED, STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT or STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT.
        sub_sim->is_requirement_ = true;
        if (sub_sim->get_mode() == SIM_ROOT)
          abduce_imdl(bm, super_goal, f_imdl, opposite, confidence, sub_sim);
        else
          abduce_simulated_imdl(bm, super_goal, f_imdl, opposite, confidence, sub_sim);
        break;
      }
      break;
    }
  } else { // no time to simulate or allow_simulation==false.

    Fact *ground;
    SimMode mode = sim->get_mode();
    if (!allow_simulation)
      // Force the mode to ROOT so that we don't call predict_simulated_lhs.
      mode = SIM_ROOT;
    switch (mode) {
    case SIM_ROOT:
      f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
      switch (retrieve_imdl_bwd(bm, f_imdl, ground)) {
      case WEAK_REQUIREMENT_ENABLED:
        f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(true);
      case NO_REQUIREMENT:
        sub_sim = new Sim(SIM_ROOT, seconds(0), super_goal, opposite, this, 1);
        abduce_lhs(bm, super_goal, f_imdl, opposite, confidence, sub_sim, ground, false);
        break;
      default: // WEAK_REQUIREMENT_DISABLED, STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT or STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT.
        sub_sim = new Sim(SIM_ROOT, seconds(0), super_goal, opposite, this, 1);
        sub_sim->is_requirement_ = true;
        abduce_imdl(bm, super_goal, f_imdl, opposite, confidence, sub_sim);
        break;
      }
      break;
    case SIM_OPTIONAL:
    case SIM_MANDATORY: // stop the simulation branch.
      predict_simulated_lhs(bm, opposite, confidence, sim);
      break;
    }
  }
}

void PrimaryMDLController::abduce_no_simulation(Fact *f_super_goal, bool opposite, float32 confidence)
{
  // Imitate PrimaryMDLController::reduce to set up the binding map.
  _Fact *goal_target = f_super_goal->get_goal()->get_target();
  P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
  bm->reset_bwd_timings(goal_target);
  MatchResult match_result = bm->match_bwd_lenient(goal_target, get_rhs());
  if (!(match_result == MATCH_SUCCESS_NEGATIVE || match_result == MATCH_SUCCESS_POSITIVE))
    return;

  // Set allow_simulation false.
  abduce(bm, f_super_goal, opposite, confidence, false);
}

void PrimaryMDLController::abduce_lhs(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim, Fact *ground, bool set_before) { // goal is f->g->f->object or f->g->|f->object; called concurrently by reduce() and _GMonitor::update().

  if (evaluate_bwd_guards(bm)) { // bm may be updated.

    P<_Fact> bound_lhs = (_Fact *)bm->bind_pattern(get_lhs());
    if (opposite)
      bound_lhs->set_opposite();
    bound_lhs->set_cfd(confidence);

    _Fact *evidence;
    switch (check_evidences(bound_lhs, evidence)) {
    case MATCH_SUCCESS_POSITIVE: // goal target is already known: abort.
      break;
    case MATCH_SUCCESS_NEGATIVE: // a counter evidence is already known: abort.
      break;
    case MATCH_FAILURE: {

      f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.

      switch (check_predicted_evidences(bound_lhs, evidence)) {
      case MATCH_SUCCESS_POSITIVE:
        break;
      case MATCH_SUCCESS_NEGATIVE:
      case MATCH_FAILURE:
        evidence = NULL;
        break;
      }

      Goal *sub_goal = new Goal(bound_lhs, super_goal->get_goal()->get_actor(), sim, 1);
      sub_goal->ground_ = ground;
      if (set_before)
        sim->get_solution_before() = bound_lhs->get_before();

      auto now = Now();
      Fact *f_sub_goal = new Fact(sub_goal, now, now, 1, 1);

      add_g_monitor(new GMonitor(this, bm, bound_lhs->get_before(), Timestamp(seconds(0)), f_sub_goal, f_imdl, evidence));

      if (!evidence) {
        inject_goal(bm, f_sub_goal, f_imdl);
        OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " -> fact " << f_sub_goal->get_oid() << 
          " goal [" << Utils::RelativeTime(sub_goal->get_target()->get_after()) << "," << Utils::RelativeTime(sub_goal->get_target()->get_before()) << "]");
      }
      break;
    }
    }
  }
}

void PrimaryMDLController::abduce_imdl(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim) { // goal is f->g->f->object or f->g->|f->object; called concurrently by redcue() and _GMonitor::update().

  f_imdl->set_cfd(confidence);

  Goal *sub_goal = new Goal(f_imdl, super_goal->get_goal()->get_actor(), sim, 1);

  auto now = Now();
  Fact *f_sub_goal = new Fact(sub_goal, now, now, 1, 1);
  add_r_monitor(new RMonitor(this, bm, super_goal->get_goal()->get_target()->get_before(), now + sim->get_thz(), f_sub_goal, f_imdl)); // the monitor will wait until the deadline of the super-goal.
  inject_goal(bm, f_sub_goal, f_imdl);
  string fImdlDebugInfo;
#ifdef WITH_DEBUG_OID
  fImdlDebugInfo = "(" + to_string(f_imdl->get_debug_oid()) + ")";
#endif
  OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " " << getObject()->get_oid() << " -> fact " << f_sub_goal->get_oid() << " goal fact " << 
    f_imdl->get_oid() << fImdlDebugInfo << " imdl[" << f_imdl->get_reference(0)->get_reference(0)->get_oid() << "][" << 
    Utils::RelativeTime(sub_goal->get_target()->get_after()) << "," << Utils::RelativeTime(sub_goal->get_target()->get_before()) << "]");
}

void PrimaryMDLController::abduce_simulated_lhs(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim, Sim* forwardSimulation) { // goal is f->g->f->object or f->g->|f->object; called concurrently by redcue() and _GMonitor::update().

  if (evaluate_bwd_guards(bm)) { // bm may be updated.

    P<_Fact> bound_lhs = (_Fact *)bm->bind_pattern(get_lhs());
    if (opposite)
      bound_lhs->set_opposite();
    bound_lhs->set_cfd(confidence);

    _Fact *evidence;
    switch (check_evidences(bound_lhs, evidence)) {
    case MATCH_SUCCESS_POSITIVE: // an evidence is already known: stop the simulation.
      register_simulated_goal_outcome(super_goal, true, evidence);
      break;
    case MATCH_SUCCESS_NEGATIVE: // a counter evidence is already known: stop the simulation.
      register_simulated_goal_outcome(super_goal, false, evidence);
      break;
    case MATCH_FAILURE:
      switch (check_predicted_evidences(bound_lhs, evidence)) {
      case MATCH_SUCCESS_POSITIVE: // a predicted evidence is already known: stop the simulation.
        register_simulated_goal_outcome(super_goal, true, evidence);
        break;
      case MATCH_SUCCESS_NEGATIVE:
        register_simulated_goal_outcome(super_goal, false, evidence);
        break;
      case MATCH_FAILURE: {

        f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
        if (sim && !sim->registerGoalTarget(f_imdl))
          // We are already simulating from this goal, so abort to avoid loops.
          break;

        Goal *sub_goal = new Goal(bound_lhs, super_goal->get_goal()->get_actor(), sim, 1);

        auto now = Now();
        Fact *f_sub_goal = new Fact(sub_goal, now, now, 1, 1);

        add_g_monitor(new SGMonitor(this, bm, now + sim->get_thz(), f_sub_goal, f_imdl));
        inject_simulation(f_sub_goal);
        OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << ": fact " <<
          super_goal->get_oid() << " super_goal -> fact " << f_sub_goal->get_oid() << " simulated goal");
        break;
      }
      }
      break;
    }
  }
}

void PrimaryMDLController::abduce_simulated_imdl(HLPBindingMap *bm, Fact *super_goal, Fact *f_imdl, bool opposite, float32 confidence, Sim *sim) { // goal is f->g->f->object or f->g->|f->object; called concurrently by redcue() and _GMonitor::update().

  if (sim && !sim->registerGoalTarget(f_imdl))
    // We are already simulating from this goal, so abort to avoid loops.
    return;

  f_imdl->set_cfd(confidence);

  Goal *sub_goal = new Goal(f_imdl, super_goal->get_goal()->get_actor(), sim, 1);

  auto now = Now();
  Fact *f_sub_goal = new Fact(sub_goal, now, now, 1, 1);
  add_r_monitor(new SRMonitor(this, bm, now + sim->get_thz(), f_sub_goal, f_imdl));
  inject_simulation(f_sub_goal);
  OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << ": fact " <<
    super_goal->get_oid() << " super_goal -> fact " << f_sub_goal->get_oid() << " simulated goal");
}

bool PrimaryMDLController::check_imdl(Fact *goal, HLPBindingMap *bm) { // goal is f->g->f->imdl; called by r-monitors.

  Goal *g = goal->get_goal();
  Fact *f_imdl = (Fact *)g->get_target();

  Sim *sim = g->get_sim();
  Fact *ground;
  switch (retrieve_imdl_bwd(bm, f_imdl, ground)) {
  case WEAK_REQUIREMENT_ENABLED:
    f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(true);
  case NO_REQUIREMENT:
    if (evaluate_bwd_guards(bm)) { // bm may be updated.

      f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
      abduce_lhs(bm, sim->get_f_super_goal(), f_imdl, sim->get_opposite(), f_imdl->get_cfd(), new Sim(SIM_ROOT, seconds(0), sim->get_f_super_goal(), sim->get_opposite(), this, 1), ground, false);
      return true;
    }
    return false;
  default: // WEAK_REQUIREMENT_DISABLED, STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT or STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT.
    return false;
  }
}

bool PrimaryMDLController::check_simulated_imdl(Fact *goal, HLPBindingMap *bm, Controller *root, Sim* forwardSimulation) { // goal is f->g->f->imdl; called by sr-monitors.

  Goal *g = goal->get_goal();
  Fact *f_imdl = (Fact *)g->get_target();
  ChainingStatus c_s;
  if (root)
    c_s = retrieve_simulated_imdl_bwd(bm, f_imdl, root);
  else {

    Fact *ground;
    c_s = retrieve_imdl_bwd(bm, f_imdl, ground);
  }

  Sim *sim = g->get_sim();
  switch (c_s) {
  case WEAK_REQUIREMENT_ENABLED:
    f_imdl->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(true);
  case NO_REQUIREMENT:
    if (evaluate_bwd_guards(bm)) { // bm may be updated.

      f_imdl->set_reference(0, bm->bind_pattern(f_imdl->get_reference(0))); // valuate f_imdl from updated bm.
      abduce_simulated_lhs(bm, sim->get_f_super_goal(), f_imdl, sim->get_opposite(), f_imdl->get_cfd(), new Sim(sim), forwardSimulation);
      return true;
    }
    return false;
  default: // WEAK_REQUIREMENT_DISABLED, STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT or STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT.
    return false;
  }
}

inline void PrimaryMDLController::predict_simulated_lhs(HLPBindingMap *bm, bool opposite, float32 confidence, Sim *sim) {

  _Fact *bound_lhs = (_Fact *)bm->bind_pattern(get_lhs());
  if (opposite)
    bound_lhs->set_opposite();
  bound_lhs->set_cfd(confidence);

  predict_simulated_evidence(bound_lhs, sim);
}

inline Fact* PrimaryMDLController::predict_simulated_evidence(_Fact *evidence, Sim *sim) {

  Pred *pred = new Pred(evidence, sim, 1);

  auto now = Now();
  Fact* fact_pred = new Fact(pred, now, now, 1, 1);
  inject_simulation(fact_pred);
  return fact_pred;
}

void PrimaryMDLController::register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures) {

  f_pred->invalidate();

  if (confidence == 1) // else, evidence is an assumption: no rating.
    register_req_outcome(f_pred, success, rate_failures);

  if (is_requirement_)
    return;

  _Fact *f_evidence = evidence;
  if (!f_evidence) // failure: assert absence of the pred target.
    f_evidence = f_pred->get_pred()->get_target()->get_absentee();

  Success *success_object = new Success(f_pred, f_evidence, 1);
  Code *f_success_object;
  auto now = Now();
  // We print the result to the output below, after injecting f_success_object to get its OID.
  if (success)
    f_success_object = new Fact(success_object, now, now, confidence, 1);
  else
    f_success_object = new AntiFact(success_object, now, now, confidence, 1);

  Group *primary_host = get_host();
  uint16 out_group_count = get_out_group_count();
  for (uint16 i = 0; i < out_group_count; ++i) { // inject notification in out groups.

    Group *out_group = (Group *)get_out_group(i);
    int32 resilience = _Mem::Get()->get_goal_pred_success_res(out_group, now, seconds(0));
    View *view = new View(View::SYNC_ONCE, now, 1, resilience, out_group, primary_host, f_success_object);
    _Mem::Get()->inject(view);

    if (!evidence) {

      view = new View(View::SYNC_ONCE, now, 1, 1, out_group, primary_host, f_evidence);
      _Mem::Get()->inject(view);
    }
  }

  if (success)
    OUTPUT_LINE(PRED_MON, Utils::RelativeTime(now) << " fact " << evidence->get_oid() << " -> fact " <<
      f_success_object->get_oid() << " success fact " << f_pred->get_oid() << " pred");
  else
    OUTPUT_LINE(PRED_MON, Utils::RelativeTime(now) << " |fact " << f_success_object->get_oid() <<
    " fact " << f_pred->get_oid() << " pred failure");
}

void PrimaryMDLController::register_req_outcome(Fact *f_pred, bool success, bool rate_failures) {

  if (success)
    rate_model(true);
  else if (rate_failures)
    rate_model(false);

  active_requirementsCS_.enter();
  UNORDERED_MAP<P<_Fact>, RequirementsPair, PHash<_Fact> >::const_iterator r = active_requirements_.find(f_pred);
  if (r != active_requirements_.end()) { // some requirements were controlling the prediction: give feedback.

    for (uint32 i = 0; i < r->second.first.controllers.size(); ++i) {

      MDLController *c = r->second.first.controllers[i];
      if (!c->is_invalidated())
        c->register_req_outcome(r->second.first.f_imdl, success, r->second.first.chaining_was_allowed);
    }
    for (uint32 i = 0; i < r->second.second.controllers.size(); ++i) {

      MDLController *c = r->second.second.controllers[i];
      if (!c->is_invalidated())
        c->register_req_outcome(r->second.second.f_imdl, !success, r->second.second.chaining_was_allowed);
    }
    active_requirements_.erase(r);
  }
  active_requirementsCS_.leave();
}

void PrimaryMDLController::register_goal_outcome(Fact *goal, bool success, _Fact *evidence) const {

  goal->invalidate();

  auto now = Now();
  _Fact *f_success_object;
  _Fact *absentee;
  if (success) {

    Code *success_object = new Success(goal, evidence, 1);
    f_success_object = new Fact(success_object, now, now, 1, 1);
    absentee = NULL;
  } else {

    Code *success_object;
    if (!evidence) { // assert absence of the goal target.

      absentee = goal->get_goal()->get_target()->get_absentee();
      success_object = new Success(goal, absentee, 1);
      OUTPUT_LINE(PRED_MON, Utils::RelativeTime(now) << " " << goal->get_oid() << " goal success (Primary)");
    } else {

      absentee = NULL;
      success_object = new Success(goal, evidence, 1);
      OUTPUT_LINE(PRED_MON, Utils::RelativeTime(now) << " " << goal->get_oid() << " goal failure (Primary)");
    }
    f_success_object = new AntiFact(success_object, now, now, 1, 1);
  }

  Group *primary_host = get_host();
  //int32 resilience=_Mem::Get()->get_goal_pred_success_res(primary_host,0);
  //View *view=new View(true,now,1,resilience,primary_host,primary_host,f_success_object);

  uint16 out_group_count = get_out_group_count();
  for (uint16 i = 0; i < out_group_count; ++i) { // inject notification in out groups.

    Group *out_group = (Group *)get_out_group(i);
    int32 resilience = _Mem::Get()->get_goal_pred_success_res(out_group, now, seconds(0));
    View *view = new View(View::SYNC_ONCE, now, 1, resilience, out_group, primary_host, f_success_object);
    _Mem::Get()->inject(view);

    if (absentee) {

      view = new View(View::SYNC_ONCE, now, 1, 1, out_group, primary_host, absentee);
      _Mem::Get()->inject(view);
    }
  }
}

void PrimaryMDLController::register_simulated_goal_outcome(Fact *goal, bool success, _Fact *evidence) const {

  Code *success_object = new Success(goal, evidence, 1);
  _Fact *f_success;

  auto now = Now();
  if (success)
    f_success = new Fact(success_object, now, now, 1, 1);
  else
    f_success = new AntiFact(success_object, now, now, 1, 1);

  Pred *pred = new Pred(f_success, 1);
  Fact *f_pred = new Fact(pred, now, now, 1, 1);

  Group *primary_host = get_host();
  int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_host, now, seconds(0));
  View *view = new View(View::SYNC_ONCE, now, 1, resilience, primary_host, primary_host, f_pred);
}

void PrimaryMDLController::rate_model(bool success) {

  Code *model = get_core_object();

  codeCS_.enter(); // protects the model's data.
  if (is_invalidated()) {

    codeCS_.leave();
    return;
  }

  float32 strength = model->code(MDL_STRENGTH).asFloat();
  float32 evidence_count = model->code(MDL_CNT).asFloat();
  float32 success_count = model->code(MDL_SR).asFloat()*evidence_count;

  ++evidence_count;
  model->code(MDL_DSR) = model->code(MDL_SR);

  float32 success_rate;
  if (success) { // leave the model active in the primary group.

    ++success_count;
    success_rate = success_count / evidence_count;
    uint32 evidence_count_base = _Mem::Get()->get_mdl_inertia_cnt_thr();
    if (success_rate >= _Mem::Get()->get_mdl_inertia_sr_thr() && evidence_count >= evidence_count_base) { // make the model strong if not already; trim the evidence count to reduce the rating's inertia.

      evidence_count = (uint32)(1 / success_rate);
      success_rate = 1;
      model->code(MDL_STRENGTH) = Atom::Float(1);
    }

    model->code(MDL_CNT) = Atom::Float(evidence_count);
    model->code(MDL_SR) = Atom::Float(success_rate);
    getView()->set_act(success_rate);
    codeCS_.leave();
  } else {

    success_rate = success_count / evidence_count;
    if (success_rate > get_host()->get_act_thr()) { // model still good enough to remain in the primary group.

      model->code(MDL_CNT) = Atom::Float(evidence_count);
      model->code(MDL_SR) = Atom::Float(success_rate);
      getView()->set_act(success_rate);
      codeCS_.leave();
    } else if (strength == 1) { // activate out-of-context strong models in the secondary group, deactivate from the primary.

      getView()->set_act(0);
      secondary_->getView()->set_act(success_rate); // may trigger secondary->gain_activation().
      codeCS_.leave();
      OUTPUT_LINE(MDL_REV, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " phased out ");
    } else { // no weak models live in the secondary group.

      codeCS_.leave();
      ModelBase::Get()->register_mdl_failure(model);
      kill_views();
      OUTPUT_LINE(MDL_REV, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " deleted ");
    }
  }
  OUTPUT_LINE(MDL_REV, Utils::RelativeTime(Now()) << " mdl " << model->get_oid() << " cnt:" << evidence_count << " sr:" << success_rate);
}

void PrimaryMDLController::assume(_Fact *input) {

  if (!_Mem::Get()->get_enable_assumptions())
    // Assumptions are disabled in the global settings.
    return;
  if (is_requirement() || is_reuse() || is_cmd())
    return;

  Code *model = get_core_object();
  if (model->code(MDL_STRENGTH).asFloat() == 0) // only strong models compute assumptions.
    return;

  if (input->get_pred()) // discard predictions.
    return;

  float32 confidence = get_cfd()*input->get_cfd(); // reading STRONG_REQUIREMENT is atomic.
  Code *host = get_host();
  if (confidence <= host->code(GRP_SLN_THR).asFloat()) // cfd is too low for any assumption to be injected.
    return;

  P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
  bm->reset_bwd_timings(input);
  bool opposite = false;
  MatchResult match_result = bm->match_bwd_lenient(input, rhs_);
  switch (match_result) {
  case MATCH_SUCCESS_NEGATIVE:
    opposite = true;
  case MATCH_SUCCESS_POSITIVE:
    assume_lhs(bm, opposite, input, confidence);
    break;
  default:
    break;
  }
}

void PrimaryMDLController::assume_lhs(HLPBindingMap *bm, bool opposite, _Fact *input, float32 confidence) { // produce an assumption and inject in primary; no rdx.

  P<Fact> f_imdl = get_f_ihlp(bm, false);
  Fact *ground;
  switch (retrieve_imdl_bwd(bm, f_imdl, ground)) {
  case WEAK_REQUIREMENT_ENABLED:
  case NO_REQUIREMENT:
    if (evaluate_bwd_guards(bm)) // bm may be updated.
      break;
    return;
  default: // WEAK_REQUIREMENT_DISABLED, STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT or STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT.
    return;
  }

  _Fact *bound_lhs = (_Fact *)bm->bind_pattern(lhs_); // fact or |fact.
  bound_lhs->set_cfd(confidence);
  if (opposite)
    bound_lhs->set_opposite();

  assumptionsCS_.enter();
  assumptions_.push_back(bound_lhs);
  assumptionsCS_.leave();

  auto now = Now();
  Timestamp before = bound_lhs->get_before();
  Group *primary_host = get_host();
  Timestamp::duration time_to_live;
  if (before > now)
    time_to_live = before - now;
  else
    time_to_live = seconds(0);
  int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_host, now, time_to_live);
  View *view = new View(View::SYNC_ONCE, now, confidence, resilience, primary_host, primary_host, bound_lhs); // SYNC_ONCE,res=resilience.
  _Mem::Get()->inject(view);
  OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " -> " << bound_lhs->get_oid() << " asmp");
}

void PrimaryMDLController::kill_views() {

  reductionCS_.enter();
  if (is_invalidated()) {

    reductionCS_.leave();
    return;
  }

  remove_requirement_from_rhs();
  secondary_->remove_requirement_from_rhs();
  invalidate();
  getView()->force_res(0);
  secondary_->getView()->force_res(0);
  reductionCS_.leave();
}

void PrimaryMDLController::check_last_match_time(bool match) {

  Timestamp now;
  if (match) {

    last_match_timeCS_.enter();
    now = Now();
    last_match_time_ = now;
    last_match_timeCS_.leave();
  } else {

    now = Now();
    if (now - last_match_time_ > _Mem::Get()->get_primary_thz())
      getView()->set_act(0); // will trigger lose_activation(), which will activate the model in the secondary group.
  }
}

bool PrimaryMDLController::abduction_allowed(HLPBindingMap *bm) { // true if fwd timings valuated and all values used by the bwd guards can be evaluated (excepted the values in the tpl args).

  if (!bm)
    return false;
  if (!HLPOverlay::CheckFWDTimings(this, bm))
    return false;
  if (!HLPOverlay::ScanBWDGuards(this, bm))
    return false;
  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SecondaryMDLController::SecondaryMDLController(r_code::View *view) : MDLController(view) {
}

void SecondaryMDLController::set_primary(PrimaryMDLController *primary) {

  primary_ = primary;
}

void SecondaryMDLController::take_input(r_exec::View *input) {

  if (become_invalidated())
    return;

  if (input->object_->code(0).asOpcode() == Opcodes::Fact ||
    input->object_->code(0).asOpcode() == Opcodes::AntiFact) // discard everything but facts and |facts.
    Controller::__take_input<SecondaryMDLController>(input);
}

void SecondaryMDLController::reduce(r_exec::View *input) { // no lock.

  if (is_orphan())
    return;

  if (input->object_->is_invalidated())
    return;

  SecondaryMDLOverlay o(this, bindings_);
  bool match = o.reduce((_Fact *)input->object_, NULL, NULL); // forward chaining.

  if (!match)
    monitor_predictions((_Fact *)input->object_);

  check_last_match_time(match);
}

void SecondaryMDLController::reduce_batch(Fact *f_p_f_imdl, MDLController *controller) {

  if (is_orphan())
    return;

  reduce_cache<EvidenceEntry>(&evidences_, f_p_f_imdl, controller);
  reduce_cache<PredictedEvidenceEntry>(&predicted_evidences_, f_p_f_imdl, controller);
}

void SecondaryMDLController::predict(HLPBindingMap *bm, _Fact *input, Fact *f_imdl, bool chaining_was_allowed, RequirementsPair &r_p, Fact *ground) { // predicitons are not injected: they are silently produced for rating purposes.

    //rhs->trace();rhs->get_reference(0)->trace();bindings_->trace();
  _Fact *bound_rhs = (_Fact *)bm->bind_pattern(rhs_); // fact or |fact.
  Pred *_prediction = new Pred(bound_rhs, 1);
  auto now = Now();
  Fact *production = new Fact(_prediction, now, now, 1, 1);

  register_requirement(production, r_p);

  if (is_requirement()) { // store in the rhs controller, even if primary (to allow rating in any case).

    if (is_invalidated())
      // Another thread has invalidated this controller which clears the controllers.
      return;
    ((MDLController *)controllers_[RHSController])->store_requirement(production, this, chaining_was_allowed, false);
    return;
  }

  PMonitor *m = new PMonitor(this, bm, production, false); // predictions are monitored for rating (successes only); no injection.
  add_monitor(m);
}

void SecondaryMDLController::store_requirement(_Fact *f_imdl, MDLController *controller, bool chaining_was_allowed, bool is_simulation) {

  Code *mdl = f_imdl->get_reference(0);
  RequirementEntry e(f_imdl, controller, chaining_was_allowed);
  if (f_imdl->is_fact()) {

    _store_requirement(&requirements_.positive_evidences, e);
    reduce_cache<SecondaryMDLController>((Fact *)f_imdl, controller);
  } else
    _store_requirement(&requirements_.negative_evidences, e);
}

void SecondaryMDLController::rate_model() { // acknowledge successes only; the purpose is to wake strong models up upon a context switch.

  Code *model = get_core_object();

  codeCS_.enter(); // protects the model's data.
  if (is_invalidated()) {

    codeCS_.leave();
    return;
  }

  uint32 evidence_count = model->code(MDL_CNT).asFloat();
  uint32 success_count = model->code(MDL_SR).asFloat()*evidence_count;

  ++evidence_count;
  model->code(MDL_DSR) = model->code(MDL_SR);
  model->code(MDL_CNT) = Atom::Float(evidence_count);

  ++success_count;
  float32 success_rate = success_count / evidence_count; // no trimming.
  model->code(MDL_SR) = Atom::Float(success_rate);

  if (success_rate > primary_->getView()->get_host()->get_act_thr()) {

    getView()->set_act(0);
    primary_->getView()->set_act(success_rate); // activate the primary controller in its own group g: will be performmed at the nex g->upr.
    codeCS_.leave();
    OUTPUT_LINE(MDL_REV, Utils::RelativeTime(Now()) << " mdl " << getObject()->get_oid() << " phased in ");
  } else { // will trigger primary->gain_activation() at the next g->upr.

    if (success_rate > getView()->get_host()->get_act_thr()) // else: leave the model in the secondary group.
      getView()->set_act(success_rate);
    codeCS_.leave();
  }
}

void SecondaryMDLController::register_pred_outcome(Fact *f_pred, bool success, _Fact *evidence, float32 confidence, bool rate_failures) { // success==false means executed in the thread of a time core; otherwise, executed in the same thread as for Controller::reduce().

  register_req_outcome(f_pred, success, rate_failures);
}

void SecondaryMDLController::register_req_outcome(Fact *f_imdl, bool success, bool rate_failures) {

  if (success) {

    rate_model();

    active_requirementsCS_.enter();
    UNORDERED_MAP<P<_Fact>, RequirementsPair, PHash<_Fact> >::const_iterator r = active_requirements_.find(f_imdl);
    if (r != active_requirements_.end()) { // some requirements were controlling the prediction: give feedback.

      for (uint32 i = 0; i < r->second.first.controllers.size(); ++i) {

        if (!r->second.first.controllers[i]->is_invalidated())
          r->second.first.controllers[i]->register_req_outcome(r->second.first.f_imdl, success, r->second.first.chaining_was_allowed);
      }
      active_requirements_.erase(r);
    }
    active_requirementsCS_.leave();
  }
}

void SecondaryMDLController::kill_views() {

  remove_requirement_from_rhs();
  primary_->remove_requirement_from_rhs();
  invalidate();
  getView()->force_res(0);
  primary_->getView()->force_res(0);
}

void SecondaryMDLController::check_last_match_time(bool match) {

  Timestamp now;
  if (match) {

    last_match_timeCS_.enter();
    now = Now();
    last_match_time_ = now;
    last_match_timeCS_.leave();
  } else {

    now = Now();
    if (now - last_match_time_ > _Mem::Get()->get_secondary_thz()) {

      ModelBase::Get()->register_mdl_timeout(get_core_object());
      kill_views();
    }
  }
}
}
