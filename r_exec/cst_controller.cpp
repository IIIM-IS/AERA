//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2022 Jeff Thompson
//_/_/ Copyright (c) 2018-2022 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2022 Icelandic Institute for Intelligent Machines
//_/_/ http://www.iiim.is
//_/_/ 
//_/_/ Copyright (c) 2010-2012 Eric Nivel
//_/_/ Center for Analysis and Design of Intelligent Agents
//_/_/ Reykjavik University, Menntavegur 1, 102 Reykjavik, Iceland
//_/_/ http://cadia.ru.is
//_/_/ 
//_/_/ Part of this software was developed by Eric Nivel
//_/_/ in the HUMANOBS EU research project, which included
//_/_/ the following parties:
//_/_/
//_/_/ Autonomous Systems Laboratory
//_/_/ Technical University of Madrid, Spain
//_/_/ http://www.aslab.org/
//_/_/
//_/_/ Communicative Machines
//_/_/ Edinburgh, United Kingdom
//_/_/ http://www.cmlabs.com/
//_/_/
//_/_/ Istituto Dalle Molle di Studi sull'Intelligenza Artificiale
//_/_/ University of Lugano and SUPSI, Switzerland
//_/_/ http://www.idsia.ch/
//_/_/
//_/_/ Institute of Cognitive Sciences and Technologies
//_/_/ Consiglio Nazionale delle Ricerche, Italy
//_/_/ http://www.istc.cnr.it/
//_/_/
//_/_/ Dipartimento di Ingegneria Informatica
//_/_/ University of Palermo, Italy
//_/_/ http://diid.unipa.it/roboticslab/
//_/_/
//_/_/
//_/_/ --- HUMANOBS Open-Source BSD License, with CADIA Clause v 1.0 ---
//_/_/
//_/_/ Redistribution and use in source and binary forms, with or without
//_/_/ modification, is permitted provided that the following conditions
//_/_/ are met:
//_/_/ - Redistributions of source code must retain the above copyright
//_/_/   and collaboration notice, this list of conditions and the
//_/_/   following disclaimer.
//_/_/ - Redistributions in binary form must reproduce the above copyright
//_/_/   notice, this list of conditions and the following disclaimer 
//_/_/   in the documentation and/or other materials provided with 
//_/_/   the distribution.
//_/_/
//_/_/ - Neither the name of its copyright holders nor the names of its
//_/_/   contributors may be used to endorse or promote products
//_/_/   derived from this software without specific prior 
//_/_/   written permission.
//_/_/   
//_/_/ - CADIA Clause: The license granted in and to the software 
//_/_/   under this agreement is a limited-use license. 
//_/_/   The software may not be used in furtherance of:
//_/_/    (i)   intentionally causing bodily injury or severe emotional 
//_/_/          distress to any person;
//_/_/    (ii)  invading the personal privacy or violating the human 
//_/_/          rights of any person; or
//_/_/    (iii) committing or preparing for any act of war.
//_/_/
//_/_/ THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND 
//_/_/ CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, 
//_/_/ INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF 
//_/_/ MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE 
//_/_/ DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR 
//_/_/ CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
//_/_/ SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, 
//_/_/ BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR 
//_/_/ SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS 
//_/_/ INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, 
//_/_/ WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING 
//_/_/ NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
//_/_/ OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY 
//_/_/ OF SUCH DAMAGE.
//_/_/ 
//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/

#include "cst_controller.h"
#include "mem.h"
#include "hlp_context.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;

namespace r_exec {

CSTOverlay::CSTOverlay(Controller *c, HLPBindingMap *bindings) : HLPOverlay(c, bindings), match_deadline_(Timestamp(seconds(0))), lowest_cfd_(1) {
}

CSTOverlay::CSTOverlay(const CSTOverlay *original) : HLPOverlay(original->controller_, original->bindings_) {

  axiom_patterns_ = original->axiom_patterns_;
  non_axiom_patterns_ = original->non_axiom_patterns_;
  predictions_ = original->predictions_;
  simulations_ = original->simulations_;
  match_deadline_ = original->match_deadline_;
  lowest_cfd_ = original->lowest_cfd_;
  axiom_inputs_ = original->axiom_inputs_;
  non_axiom_inputs_ = original->non_axiom_inputs_;
  defeasible_validities_ = original->defeasible_validities_;
}

CSTOverlay::~CSTOverlay() {
}

void CSTOverlay::load_patterns() {

  Code *object = ((HLPController *)controller_)->get_unpacked_object();
  uint16 obj_set_index = object->code(CST_OBJS).asIndex();
  uint16 obj_count = object->code(obj_set_index).getAtomCount();
  for (uint16 i = 1; i <= obj_count; ++i) {

    _Fact *pattern = (_Fact *)object->get_reference(object->code(obj_set_index + i).asIndex());
    if (pattern->references_size() >= 1 && _Mem::Get()->matches_axiom(pattern->get_reference(0)))
      axiom_patterns_.push_back(pattern);
    else
      non_axiom_patterns_.push_back(pattern);
  }
}

bool CSTOverlay::can_match(Timestamp now) const { // to reach inputs until a given thz in the past, return now<deadline+thz.

  if (match_deadline_.time_since_epoch().count() == 0)
    return true;
  return now <= match_deadline_;
}

_Fact* CSTOverlay::inject_production(View* input) {

  Fact *f_icst = ((CSTController *)controller_)->get_f_icst(bindings_, &axiom_inputs_, &non_axiom_inputs_);
  auto now = Now();
  string inputs_info;
  for (uint32 i = 0; i < axiom_inputs_.size(); ++i)
    inputs_info += " " + to_string(axiom_inputs_[i]->get_oid());
  for (uint32 i = 0; i < non_axiom_inputs_.size(); ++i)
    inputs_info += " " + to_string(non_axiom_inputs_[i]->get_oid());

  if (!is_simulated()) {

    auto before = bindings_->get_fwd_before();
    Timestamp::duration time_to_live;
    if (now >= before)
      time_to_live = Timestamp::duration(seconds(0));
    else
      time_to_live = before - now;
    if (predictions_.size()) {

      Pred *prediction = new Pred(f_icst, 1);
      Fact *f_p_f_icst = new Fact(prediction, now, now, 1, 1);
      unordered_set<P<_Fact>, PHash<_Fact> >::const_iterator pred;
      for (pred = predictions_.begin(); pred != predictions_.end(); ++pred) // add antecedents to the prediction.
        prediction->grounds_.push_back(*pred);
      if (!((CSTController *)controller_)->inject_prediction(f_p_f_icst, lowest_cfd_, time_to_live)) // inject a f->pred->icst in the primary group, no rdx.
        return NULL;

      string f_icst_info;
#ifdef WITH_DETAIL_OID
      f_icst_info = "(" + to_string(f_icst->get_detail_oid()) + ") ";
#endif
      OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " fact " << f_p_f_icst->get_oid() <<
        " pred fact " << f_icst_info << "icst[" << controller_->get_object()->get_oid() << "][" << inputs_info << "]");
      return f_p_f_icst;
    } else {
      ((CSTController *)controller_)->inject_icst(f_icst, lowest_cfd_, time_to_live); // inject f->icst in the primary and secondary groups, and in the output groups.

      OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " fact " << f_icst->get_oid() << " icst[" << controller_->get_object()->get_oid() << "][" <<
        inputs_info << "]");
      return f_icst;
    }
  } else { // there are simulations; the production is therefore a prediction; add the simulations to the latter.

    // Make a temporary copy of simulations_ for calling the Pred constructor.
    vector<P<Sim>> simulations_copy;
    for (auto sim = simulations_.begin(); sim != simulations_.end(); ++sim)
      simulations_copy.push_back(*sim);
    // Add the simulations to the prediction.
    Pred *prediction = new Pred(f_icst, simulations_copy, 1);
    if (((_Fact*)input->object_)->get_pred())
      // Propagate the accumulated of DefeasibleValidity from all the inputs to the new prediction.
      prediction->defeasible_validities_ = defeasible_validities_;
    Fact *f_p_f_icst = new Fact(prediction, now, now, 1, 1);
    if (!((HLPController *)controller_)->inject_prediction(f_p_f_icst, lowest_cfd_)) // inject a simulated prediction in the main group.
      return NULL;
    OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " cst " << get_object()->get_oid() << ": fact " <<
      input->object_->get_oid() << " -> fact " << f_p_f_icst->get_oid() << " simulated pred fact icst [" <<
      inputs_info << "]");
    return f_p_f_icst;
  }
}

CSTOverlay *CSTOverlay::get_offspring(HLPBindingMap *map, _Fact *input, bool is_axiom, _Fact *bound_pattern) {

  CSTOverlay *offspring = new CSTOverlay(this);
  if (bound_pattern) {
    if (is_axiom)
      axiom_patterns_.remove(bound_pattern);
    else
      non_axiom_patterns_.remove(bound_pattern);
  }
  if (match_deadline_.time_since_epoch().count() == 0)
    match_deadline_ = map->get_fwd_before();
  update(map, input, is_axiom);
  return offspring;
}

void CSTOverlay::update(HLPBindingMap *map, _Fact *input, bool is_axiom) {

  bindings_ = map;
  if (is_axiom)
    axiom_inputs_.push_back(input);
  else
    non_axiom_inputs_.push_back(input);
  float32 last_cfd;
  Pred *prediction = input->get_pred();
  if (prediction) {

    last_cfd = prediction->get_target()->get_cfd();
    if (prediction->is_simulation()) {

      for (uint16 i = 0; i < prediction->get_simulations_size(); ++i) {
        auto predictionSimulation = prediction->get_simulation(i);
        if (!get_simulation(predictionSimulation->get_root_sim()->root_))
          // The simulations_ does not have a Sim with the same root_, so add.
          simulations_.insert(predictionSimulation);
      }
    } else
      predictions_.insert(input);

    // Accumulate the DefeasibleValidity objects from each input.
    defeasible_validities_.insert(prediction->defeasible_validities_.begin(), prediction->defeasible_validities_.end());
  } else
    last_cfd = input->get_cfd();

  if (last_cfd < lowest_cfd_)
    lowest_cfd_ = last_cfd;
}

bool CSTOverlay::reduce(View *input, CSTOverlay *&offspring) {

  offspring = NULL;

  if (input->object_->is_invalidated())
    return false;

  for (uint16 i = 0; i < axiom_inputs_.size(); ++i) { // discard inputs that already matched.

    if (axiom_inputs_[i]->is_invalidated()) {
      // If we make an icst from this overlay, CSTController::get_f_icst will copy axiom_inputs_
      // to the icst components_, and ICST::is_invalidated will see the invalidated
      // component and mark the icst as invalidated before it is even injected. So just
      // invalidate this overlay now.
      invalidate();
      return false;
    }
    if (((_Fact *)input->object_) == axiom_inputs_[i])
      return false;
  }
  for (uint16 i = 0; i < non_axiom_inputs_.size(); ++i) { // discard inputs that already matched.

    if (non_axiom_inputs_[i]->is_invalidated()) {
      // If we make an icst from this overlay, CSTController::get_f_icst will copy non_axiom_inputs_
      // to the icst components_, and ICST::is_invalidated will see the invalidated
      // component and mark the icst as invalidated before it is even injected. So just
      // invalidate this overlay now.
      invalidate();
      return false;
    }
    if (((_Fact *)input->object_) == non_axiom_inputs_[i])
      return false;
  }
  _Fact *input_object;
  Pred *prediction = ((_Fact *)input->object_)->get_pred();
  bool is_simulation;
  Sim* predictionSimulation = NULL;
  if (prediction) {

    input_object = prediction->get_target(); // input_object is f1 as in f0->pred->f1->object.
    is_simulation = prediction->is_simulation();
    // TODO: Handle a prediction with multiple simulations for different roots.
    if (prediction->get_simulations_size() == 1)
      predictionSimulation = prediction->get_simulation((uint16)0);
  } else {

    input_object = (_Fact *)input->object_;
    is_simulation = false;
  }

  // If the prediction is already promoted, don't examine it again.
  if (predictionSimulation && !prediction->is_promoted_ &&
      (simulations_.size() == 0 || simulations_.size() == 1 && *simulations_.begin() == predictionSimulation)) {
    // The input is for the same simulation as this overlay (or this overlay is non-simulated).
    auto now = Now();
    predictionSimulation->defeasible_promoted_facts_.CS_.enter();
    // Check if the input defeats a promoted fact.
    for (auto d = predictionSimulation->defeasible_promoted_facts_.list_.begin();
      d != predictionSimulation->defeasible_promoted_facts_.list_.end(); ) {
      if (input_object->is_evidence(d->promoted_fact_->get_pred()->get_target()) != MATCH_FAILURE) {
        // The actual input fact matches (positive or negative) a defeasible promoted fact, so invalidated it.
        predictionSimulation->defeating_facts_.push_back(input_object);
        d->defeasible_validity_->invalidate();
        OUTPUT_LINE(CST_OUT, Utils::RelativeTime(now) << " promoted simulated fact " << d->promoted_fact_->get_oid() <<
          " defeated by fact " << input->object_->get_oid());
        // We don't need this entry any more.
        d = predictionSimulation->defeasible_promoted_facts_.list_.erase(d);
      }
      else
        ++d;
    }

    // Only consider time intervals that are not zero duration.
    bool input_is_later = (input_object->get_before() > input_object->get_after() && bindings_->has_fwd_before() &&
      input_object->get_after() >= bindings_->get_fwd_before());
    if (input_is_later &&
      find(promoted_in_sim_.begin(), promoted_in_sim_.end(), predictionSimulation) == promoted_in_sim_.end()) {
      // We have not already promoted inputs from this overlay to a later time in this Sim.
      promoted_in_sim_.push_back(predictionSimulation);

      // Loop through this overlay's non-axiom saved inputs, checking if it needs to be promoted in this Sim.
      // (Axiom facts are promoted by bind_pattern when binding a non-axiom fact.)
      for (uint32 i = 0; i < non_axiom_inputs_.size(); ++i) {
        _Fact* saved_input = non_axiom_inputs_[i];
        Pred* saved_input_pred = saved_input->get_pred();
        if (saved_input_pred)
          saved_input = saved_input_pred->get_target();
        if (saved_input->get_reference(0)->code(0).asOpcode() == Opcodes::ICst)
          // Don't promote an icst. (Expect that its non-icst members will be promoted and re-make the composite icst.)
          continue;

        // Note: has_original_fact uses pointer equality, so it is a quick check.
        if (Sim::DefeasiblePromotedFact::has_original_fact(predictionSimulation->defeasible_promoted_facts_.list_, saved_input))
          // We already checked for promoting this fact in this Sim.
          continue;

        if (saved_input->is_timeless_evidence(input_object) != MATCH_FAILURE)
          // The input is an actual fact which already matches (positive or negative) the overlay's saved input. We don't need to promote.
          continue;

        // Make a new fact with the same Sim and timings as the input object.
        Fact* promoted_fact = new Fact(saved_input->get_reference(0), input_object->get_after(), input_object->get_before(),
          saved_input->get_cfd(), saved_input->get_psln_thr());
        if (!saved_input->is_fact())
          // The overlay's saved input is an anti-fact.
          promoted_fact->set_opposite();

        bool matches_defeating_fact = false;
        for (auto f = predictionSimulation->defeating_facts_.begin(); f != predictionSimulation->defeating_facts_.end(); ++f) {
          if (promoted_fact->is_evidence(*f) != MATCH_FAILURE) {
            matches_defeating_fact = true;
            break;
          }
        }
        if (matches_defeating_fact)
          // The candidate promoted_fact matches (positive or negative) an actual fact already made for this Sim.
          continue;

        Pred* p_promoted_fact = new Pred(promoted_fact, prediction, 1);
        Fact* f_p_promoted_fact = new Fact(p_promoted_fact, now, now, 1, 1);

        // The promoted fact may be defeated by a predicted fact for the same time interval, so make it defeasible.
        P<DefeasibleValidity> defeasible_validity = new DefeasibleValidity();
        p_promoted_fact->defeasible_validities_.insert(defeasible_validity);
        p_promoted_fact->is_promoted_ = true;

        if (((HLPController *)controller_)->inject_prediction(f_p_promoted_fact, ((_Fact *)input->object_)->get_cfd())) {
          // Mark that this fact was promoted for this Sim.
          predictionSimulation->defeasible_promoted_facts_.list_.push_front(
            Sim::DefeasiblePromotedFact(saved_input, f_p_promoted_fact, defeasible_validity));
          OUTPUT_LINE(CST_OUT, Utils::RelativeTime(now) << " fact " << non_axiom_inputs_[i]->get_oid() <<
            " -> promoted simulated pred fact " << f_p_promoted_fact->get_oid() << " w/ fact " <<
            input->object_->get_oid() << " timings");
        }
      }
    }
    predictionSimulation->defeasible_promoted_facts_.CS_.leave();
  }

  P<HLPBindingMap> bm = new HLPBindingMap();
  bool bound_pattern_is_axiom;
  _Fact *bound_pattern = bind_pattern(input_object, bm, predictionSimulation, bound_pattern_is_axiom);
  if (bound_pattern) {
    if (axiom_patterns_.size() + non_axiom_patterns_.size() == 1) { // last match.

      if (!code_) {

        load_code();
        bindings_ = bm;
        if (evaluate_fwd_guards()) { // may update bindings; full match.
          offspring = get_offspring(bm, (_Fact *)input->object_, bound_pattern_is_axiom);
          inject_production(input);
          invalidate();
          store_evidence(input->object_, prediction, is_simulation);
          return true;
        } else {
          delete[] code_;
          code_ = NULL;
          // JTNote: This returns after bindings_ is modified. Should they be restored?
          return false;
        }
      } else { // guards already evaluated, full match.
        offspring = get_offspring(bm, (_Fact *)input->object_, bound_pattern_is_axiom);
        inject_production(input);
        invalidate();
        store_evidence(input->object_, prediction, is_simulation);
        return true;
      }
    } else {
      offspring = get_offspring(bm, (_Fact *)input->object_, bound_pattern_is_axiom, bound_pattern);
      store_evidence(input->object_, prediction, is_simulation);
      return true;
    }
  }
  else
    return false;
}

_Fact* CSTOverlay::bind_pattern(_Fact *input, HLPBindingMap* map, Sim* predictionSimulation, bool& is_axiom)
{
  is_axiom = false;

  bool use_input_timings = (axiom_inputs_.size() + non_axiom_inputs_.size() == 0);
  if (predictionSimulation) {
    if (simulations_.size() > 1)
      // TODO: Handle the case where simulations_ has multiple simulation roots.
      return NULL;
    if (simulations_.size() == 1) {
      if (*simulations_.begin() != predictionSimulation)
        // This overlay is for a simulation, but for a different simulation than the input predictionSimulation.
        return NULL;
    }

    if (axiom_inputs_.size() > 0 && non_axiom_inputs_.size() == 0) {
      // This overlay's binding map forward timings have been set, and there are only saved axiom facts.
      if (input->get_after() < bindings_->get_fwd_before() &&
          input->get_before() > bindings_->get_fwd_after()) {
        // The timings overlap, so let match_fwd_strict update the map from the input timings.
      }
      else if (input->get_before() <= bindings_->get_fwd_after())
        // The input is earlier than the facts in this overlay, so don't match.
        return NULL;
      else
        // The input is later than the facts in this overlay.
        // Below, we update this overlay's binding map to use this input's timings.
        use_input_timings = true;
    }
  }

  r_code::list<P<_Fact> >::const_iterator p;
  for (p = axiom_patterns_.begin(); p != axiom_patterns_.end(); ++p) {

    map->load(bindings_);
    if (use_input_timings)
      map->reset_fwd_timings(input);
    if (map->match_fwd_strict(input, *p)) {
      is_axiom = true;
      return *p;
    }
  }
  for (p = non_axiom_patterns_.begin(); p != non_axiom_patterns_.end(); ++p) {

    map->load(bindings_);
    if (use_input_timings)
      map->reset_fwd_timings(input);
    if (map->match_fwd_strict(input, *p))
      return *p;
  }

  return NULL;
}

Sim* CSTOverlay::get_simulation(Controller *root) const {
  // TODO: Do we need a critical section to access simulations_?
  for (auto sim = simulations_.begin(); sim != simulations_.end(); ++sim) {
    if ((*sim)->root_ == root)
      return *sim;
  }

  return NULL;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CSTController::CSTController(_View *view) : HLPController(view) {

  CSTOverlay *o = new CSTOverlay(this, bindings_); // master overlay.
  o->load_patterns();
  overlays_.push_back(o);

  Group *host = get_host();
  Code *object = get_unpacked_object();
  uint16 obj_set_index = object->code(CST_OBJS).asIndex();
  uint16 obj_count = object->code(obj_set_index).getAtomCount();
  for (uint16 i = 0; i < obj_count; ++i) {

    Code *pattern = object->get_reference(object->code(obj_set_index + 1).asIndex());
    Code *pattern_ihlp = pattern->get_reference(0);
    uint16 opcode = pattern_ihlp->code(0).asOpcode();
    if (opcode == Opcodes::ICst ||
      opcode == Opcodes::IMdl) {

      Code *pattern_hlp = pattern_ihlp->get_reference(0);
      r_exec::View *pattern_hlp_v = (r_exec::View*)pattern_hlp->get_view(host, true);
      if (pattern_hlp_v)
        controllers_.push_back((HLPController *)pattern_hlp_v->controller_);
    }
  }
}

CSTController::~CSTController() {
}

void CSTController::take_input(r_exec::View *input) {

  if (become_invalidated())
    return;

  if (input->object_->code(0).asOpcode() == Opcodes::Fact ||
    input->object_->code(0).asOpcode() == Opcodes::AntiFact) { // discard everything but facts and |facts.

    OUTPUT_LINE(CST_IN, Utils::RelativeTime(Now()) << " cst " << get_object()->get_oid() << " <- " << input->object_->get_oid());
    Controller::__take_input<CSTController>(input);
  }
}

void CSTController::reduce(r_exec::View *input) {

  if (is_orphan())
    return;

  if (input->object_->is_invalidated())
    return;

  Goal *goal = ((_Fact *)input->object_)->get_goal();
  if (goal && goal->is_self_goal() && !goal->is_drive()) { // goal is g->f->target.

    _Fact *goal_target = goal->get_target(); // handle only icst.
    Code *target_ihlp = goal_target->get_reference(0);
    if (target_ihlp->code(0).asOpcode() == Opcodes::ICst && target_ihlp->get_reference(0) == get_object()) { // f is f->icst; produce as many sub-goals as there are patterns in the cst.

      if (!get_requirement_count()) { // models will attempt to produce the icst

        P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
        bm->init_from_f_ihlp(goal_target);
        if (evaluate_bwd_guards(bm)) { // leaves the controller constant: no need to protect; bm may be updated.
          if (goal->is_simulation())
            abduce_simulated(bm, input->object_);
          else
            abduce(bm, input->object_);
        }
      }
    }
  } else {
    bool match = false;
    CSTOverlay *offspring;
    r_code::list<P<Overlay> >::const_iterator o;
    reductionCS_.enter();
    auto now = Now();
    for (o = overlays_.begin(); o != overlays_.end();) {

      if (!((CSTOverlay *)*o)->can_match(now)) {
        (*o)->invalidate();
        o = overlays_.erase(o);
      }
      else if ((*o)->is_invalidated())
        o = overlays_.erase(o);
      else {

        match = (((CSTOverlay *)*o)->reduce(input, offspring) || match);
        if (offspring)
          overlays_.push_front(offspring);

        ++o;
      }
    }
    reductionCS_.leave();

    check_last_match_time(match);
  }
}

void CSTController::abduce_simulated(HLPBindingMap *bm, Fact *f_super_goal) {
  Goal *g = f_super_goal->get_goal();
  _Fact *super_goal_target = g->get_target();
  Fact* remade_f_icst = get_f_ihlp(bm, false);

  for (auto o = overlays_.begin(); o != overlays_.end(); ++o) {
    if (((CSTOverlay*)(*o))->is_simulated())
      // Skip overlays that were produced during simulated forward chaining.
      continue;

    HLPBindingMap bm_copy(bm);
    // TODO: this is inefficient. We want to merge (*o)->binding_ into bm_copy, but use an icst.
    Fact* overlay_f_icst = get_f_ihlp(((HLPOverlay*)(*o))->bindings_, false);
    if (bm_copy.match_object(overlay_f_icst->get_reference(0), remade_f_icst->get_reference(0)))
      abduce(&bm_copy, f_super_goal);
  }
}

void CSTController::abduce(HLPBindingMap *bm, Fact *f_super_goal) {

  Goal *g = f_super_goal->get_goal();
  _Fact *super_goal_target = g->get_target();
  bool opposite = (super_goal_target->is_anti_fact());

  float32 confidence = super_goal_target->get_cfd();

  Sim *sim = g->get_sim();
  Sim *sub_sim;
  auto now = Now();
  if (sim->get_mode() == SIM_ROOT)
    sub_sim = new Sim(opposite ? SIM_MANDATORY : SIM_OPTIONAL, sim->get_thz(), f_super_goal, opposite, sim->root_, 1, sim->solution_controller_, sim->get_solution_cfd(), now + sim->get_thz());
  else
    sub_sim = new Sim(sim->get_mode(), sim->get_thz(), f_super_goal, opposite, sim->root_, 1, sim->solution_controller_, sim->get_solution_cfd(), sim->get_solution_before());

  Code *cst = get_unpacked_object();
  uint16 obj_set_index = cst->code(CST_OBJS).asIndex();
  uint16 obj_count = cst->code(obj_set_index).getAtomCount();
  Group *host = get_host();
  for (uint16 i = 1; i <= obj_count; ++i) {

    _Fact *pattern = (_Fact *)cst->get_reference(cst->code(obj_set_index + i).asIndex());
    _Fact *bound_pattern = (_Fact *)bm->bind_pattern(pattern);
    if (_Mem::Get()->matches_axiom(bound_pattern->get_reference(0)))
      // Don't make a goal of a member which is an axiom.
      continue;

    _Fact *evidence;
    if (opposite)
      bound_pattern->set_opposite();
    switch (check_evidences(bound_pattern, evidence)) {
    case MATCH_SUCCESS_POSITIVE: // positive evidence, no need to produce a sub-goal: skip.
      break;
    case MATCH_SUCCESS_NEGATIVE: // negative evidence, no need to produce a sub-goal, the super-goal will probably fail within the target time frame: skip.
      break;
    case MATCH_FAILURE:
      switch (check_predicted_evidences(bound_pattern, evidence)) {
      case MATCH_SUCCESS_POSITIVE:
        break;
      case MATCH_SUCCESS_NEGATIVE:
      case MATCH_FAILURE: // inject a sub-goal for the missing predicted positive evidence.
        inject_goal(bm, f_super_goal, bound_pattern, sub_sim, now, confidence, host); // all sub-goals share the same sim.
        break;
      }
    }
  }
}

void CSTController::inject_goal(HLPBindingMap *bm,
  Fact *f_super_goal,
  _Fact *sub_goal_target, // f1.
  Sim *sim,
  Timestamp now,
  float32 confidence,
  Code *group) const {

  if (sim && !sim->register_goal_target(sub_goal_target))
    // We are already simulating from this goal, so abort to avoid loops.
    return;

  sub_goal_target->set_cfd(confidence);

  Goal *sub_goal = new Goal(sub_goal_target, f_super_goal->get_goal()->get_actor(), sim, 1);

  _Fact *f_icst = f_super_goal->get_goal()->get_target();
  _Fact *sub_goal_f = new Fact(sub_goal, now, now, 1, 1);

  View *view = new View(View::SYNC_ONCE, now, confidence, 1, group, group, sub_goal_f); // SYNC_ONCE,res=1.
  _Mem::Get()->inject(view);
  OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " cst " << get_object()->get_oid() << ": fact " <<
    f_super_goal->get_oid() << " super_goal -> fact " << sub_goal_f->get_oid() << " simulated goal");

  if (sim->get_mode() == SIM_ROOT) { // no rdx for SIM_OPTIONAL or SIM_MANDATORY.

    MkRdx *mk_rdx = new MkRdx(f_icst, f_super_goal, sub_goal, 1, bm);
    uint16 out_group_count = get_out_group_count();
    for (uint16 i = 0; i < out_group_count; ++i) {

      Group *out_group = (Group *)get_out_group(i);
      View *view = new NotificationView(group, out_group, mk_rdx);
      _Mem::Get()->inject_notification(view, true);
    }
  }
}

Fact *CSTController::get_f_ihlp(HLPBindingMap *bindings, bool wr_enabled) const {

  return bindings->build_f_ihlp(get_object(), Opcodes::ICst, false);
}

Fact *CSTController::get_f_icst(HLPBindingMap *bindings, vector<P<_Fact> > *axiom_inputs, vector<P<_Fact> > *non_axiom_inputs) const {

  Fact *f_icst = get_f_ihlp(bindings, false);
  ICST* icst = (ICST *)f_icst->get_reference(0);
  icst->bindings_ = bindings;
  icst->components_ = *axiom_inputs;
  icst->components_.insert(icst->components_.end(), non_axiom_inputs->begin(), non_axiom_inputs->end());
  return f_icst;
}

bool CSTController::inject_prediction(Fact *prediction, float32 confidence, microseconds time_to_live) const { // prediction: f->pred->f->target.

  auto now = Now();
  Group *primary_host = get_host();
  float32 sln_thr = primary_host->code(GRP_SLN_THR).asFloat();
  if (confidence > sln_thr) { // do not inject if cfd is too low.

    int32 resilience = _Mem::Get()->get_goal_pred_success_res(primary_host, now, time_to_live);
    View *view = new View(View::SYNC_ONCE, now, confidence, resilience, primary_host, primary_host, prediction); // SYNC_ONCE,res=resilience.
    _Mem::Get()->inject(view);
    return true;
  } else
    return false;
}

void CSTController::inject_icst(Fact *production, float32 confidence, microseconds time_to_live) const { // production: f->icst.

  auto now = Now();
  Group *primary_host = get_host();
  float32 sln_thr = primary_host->code(GRP_SLN_THR).asFloat();
  if (confidence > sln_thr) {

    View *view = new View(View::SYNC_ONCE, now, 1, Utils::GetResilience(now, time_to_live, primary_host->get_upr() * Utils::GetBasePeriod().count()), primary_host, primary_host, production);
    _Mem::Get()->inject(view); // inject f->icst in the primary group: needed for hlps like M[icst -> X] and S[icst X Y].
    uint16 out_group_count = get_out_group_count();
    for (uint16 i = 0; i < out_group_count; ++i) {

      Group *out_group = (Group *)get_out_group(i);
      View *view = new View(View::SYNC_ONCE, now, 1, 1, out_group, primary_host, production);
      _Mem::Get()->inject(view);
    }
  }

  sln_thr = secondary_host_->code(GRP_SLN_THR).asFloat();
  if (confidence > sln_thr) {

    View *view = new View(View::SYNC_ONCE, now, 1, Utils::GetResilience(now, time_to_live, secondary_host_->get_upr() * Utils::GetBasePeriod().count()), secondary_host_, primary_host, production);
    _Mem::Get()->inject(view); // inject f->icst in the secondary group: same reason as above.
  }
}

void CSTController::set_secondary_host(Group *host) {

  secondary_host_ = host;
}

Group *CSTController::get_secondary_host() const {

  return secondary_host_;
}

void CSTController::kill_views() {

  invalidate();
  get_view()->force_res(0);
}

void CSTController::check_last_match_time(bool match) {

  auto now = Now();
  if (match)
    last_match_time_ = now;
  else if (now - last_match_time_ > _Mem::Get()->get_primary_thz())
    kill_views();
}
}
