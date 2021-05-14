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

#include "cst_controller.h"
#include "mem.h"
#include "hlp_context.h"

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

CSTOverlay::CSTOverlay(Controller *c, HLPBindingMap *bindings) : HLPOverlay(c, bindings), match_deadline_(Timestamp(seconds(0))), lowest_cfd_(1) {
  original_patterns_size_ = 0;
}

CSTOverlay::CSTOverlay(const CSTOverlay *original) : HLPOverlay(original->controller_, original->bindings_) {

  patterns_ = original->patterns_;
  predictions_ = original->predictions_;
  simulations_ = original->simulations_;
  match_deadline_ = original->match_deadline_;
  lowest_cfd_ = original->lowest_cfd_;
  original_patterns_size_ = original->original_patterns_size_;
  inputs_ = original->inputs_;
}

CSTOverlay::~CSTOverlay() {
}

void CSTOverlay::load_patterns() {

  Code *object = ((HLPController *)controller_)->get_unpacked_object();
  uint16 obj_set_index = object->code(CST_OBJS).asIndex();
  uint16 obj_count = object->code(obj_set_index).getAtomCount();
  for (uint16 i = 1; i <= obj_count; ++i) {

    _Fact *pattern = (_Fact *)object->get_reference(object->code(obj_set_index + i).asIndex());
    patterns_.push_back(pattern);
  }
  original_patterns_size_ = patterns_.size();
}

bool CSTOverlay::can_match(Timestamp now) const { // to reach inputs until a given thz in the past, return now<deadline+thz.

  if (match_deadline_.time_since_epoch().count() == 0)
    return true;
  return now <= match_deadline_;
}

void CSTOverlay::inject_production(View* input) {

  Fact *f_icst = ((CSTController *)controller_)->get_f_icst(bindings_, &inputs_);
  auto now = Now();//f_icst->get_reference(0)->trace();
  string inputsInfo;
  for (uint32 i = 0; i < inputs_.size(); ++i)
    inputsInfo += " " + to_string(inputs_[i]->get_oid());

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
      UNORDERED_SET<P<_Fact>, PHash<_Fact> >::const_iterator pred;
      for (pred = predictions_.begin(); pred != predictions_.end(); ++pred) // add antecedents to the prediction.
        prediction->grounds_.push_back(*pred);
      ((CSTController *)controller_)->inject_prediction(f_p_f_icst, lowest_cfd_, time_to_live); // inject a f->pred->icst in the primary group, no rdx.

      string f_p_f_icst_info;
#ifdef WITH_DEBUG_OID
      f_p_f_icst_info = "(" + to_string(f_p_f_icst->get_debug_oid()) + ")";
#endif
      string f_icst_info;
#ifdef WITH_DEBUG_OID
      f_icst_info = "(" + to_string(f_icst->get_debug_oid()) + ") ";
#endif
      OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " fact " << f_p_f_icst->get_oid() << f_p_f_icst_info <<
        " pred fact " << f_icst_info << "icst[" << controller_->getObject()->get_oid() << "][" << inputsInfo << "]");
    } else {
      ((CSTController *)controller_)->inject_icst(f_icst, lowest_cfd_, time_to_live); // inject f->icst in the primary and secondary groups, and in the output groups.

      OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " fact " << f_icst->get_oid() << " icst[" << controller_->getObject()->get_oid() << "][" <<
        inputsInfo << "]");
    }
  } else { // there are simulations; the production is therefore a prediction; add the simulations to the latter.

    // Make a temporary copy of simulations_ for calling the Pred constructor.
    std::vector<P<Sim>> simulations_copy;
    for (auto sim = simulations_.begin(); sim != simulations_.end(); ++sim)
      simulations_copy.push_back(*sim);
    // Add the simulations to the prediction.
    Pred *prediction = new Pred(f_icst, simulations_copy, 1);
    Fact *f_p_f_icst = new Fact(prediction, now, now, 1, 1);
    ((HLPController *)controller_)->inject_prediction(f_p_f_icst, lowest_cfd_); // inject a simulated prediction in the main group.
    OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " cst " << getObject()->get_oid() << ": fact " <<
      input->object_->get_oid() << " -> fact " << f_p_f_icst->get_oid() << " simulated pred fact icst [" <<
      inputsInfo << "]");
  }
}

CSTOverlay *CSTOverlay::get_offspring(HLPBindingMap *map, _Fact *input, _Fact *bound_pattern) {

  CSTOverlay *offspring = new CSTOverlay(this);
  if (bound_pattern)
    patterns_.remove(bound_pattern);
  if (match_deadline_.time_since_epoch().count() == 0)
    match_deadline_ = map->get_fwd_before();
  update(map, input);
  //std::cout<<std::hex<<this<<std::dec<<" produced: "<<std::hex<<offspring<<std::dec<<std::endl;
  return offspring;
}

void CSTOverlay::update(HLPBindingMap *map, _Fact *input) {

  bindings_ = map;
  inputs_.push_back(input);
  float32 last_cfd;
  Pred *prediction = input->get_pred();
  if (prediction) {

    last_cfd = prediction->get_target()->get_cfd();
    if (prediction->is_simulation()) {

      for (uint16 i = 0; i < prediction->get_simulations_size(); ++i) {
        auto predictionSimulation = prediction->get_simulation(i);
        if (!get_simulation(predictionSimulation->getRootSim()->root_))
          // The simulations_ does not have a Sim with the same root_, so add.
          simulations_.insert(predictionSimulation);
      }
    } else
      predictions_.insert(input);
  } else
    last_cfd = input->get_cfd();

  if (lowest_cfd_ > last_cfd)
    lowest_cfd_ = last_cfd;
}

bool CSTOverlay::reduce(View *input, CSTOverlay *&offspring) {

  offspring = NULL;

  if (input->object_->is_invalidated())
    return false;

  for (uint16 i = 0; i < inputs_.size(); ++i) { // discard inputs that already matched.

    if (((_Fact *)input->object_) == inputs_[i])
      return false;
  }
  auto now = Now();
  // if(match_deadline.time_since_epoch().count() == 0)
  // std::cout<<Time::ToString_seconds(Now()-st)<<" "<<std::hex<<this<<std::dec<<" (0) "<<input->object->get_oid()<<std::endl;
  // else
  // std::cout<<Time::ToString_seconds(Now()-st)<<" "<<std::hex<<this<<std::dec<<" ("<<Time::ToString_seconds(match_deadline-st)<<") "<<input->object->get_oid()<<std::endl;
  _Fact *input_object;
  Pred *prediction = ((_Fact *)input->object_)->get_pred();
  bool is_simulation;
  if (prediction) {

    input_object = prediction->get_target(); // input_object is f1 as in f0->pred->f1->object.
    is_simulation = prediction->is_simulation();
  } else {

    input_object = (_Fact *)input->object_;
    is_simulation = false;
  }

  P<HLPBindingMap> bm = new HLPBindingMap();
  _Fact *bound_pattern = bindPattern(input_object, bm);
  if (bound_pattern) {
    //if(match_deadline.time_since_epoch().count() == 0){
    // std::cout<<Time::ToString_seconds(now-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" (0) ";
    //} else{
    // std::cout<<Time::ToString_seconds(now-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" ("<<Time::ToString_seconds(match_deadline-Utils::GetTimeReference())<<") ";
    //}
    if (patterns_.size() == 1) { // last match.

      if (!code_) {

        load_code();
        bindings_ = bm;
        if (evaluate_fwd_guards()) { // may update bindings; full match.
//std::cout<<Time::ToString_seconds(now-Utils::GetTimeReference())<<" full match\n";
          // JTNote: The offspring is made with the modified bindings_. That doesn't seem right.
          offspring = get_offspring(bm, (_Fact *)input->object_);
          inject_production(input);
          invalidate();
          store_evidence(input->object_, prediction, is_simulation);
          return true;
        } else {
          //std::cout<<" guards failed\n";
          delete[] code_;
          code_ = NULL;
          // JTNote: This returns after bindings_ is modified. Should they be restored?
          return false;
        }
      } else { // guards already evaluated, full match.
//std::cout<<Time::ToString_seconds(now-Utils::GetTimeReference())<<" full match\n";
        offspring = get_offspring(bm, (_Fact *)input->object_);
        if (inputs_.size() == original_patterns_size_) inject_production(input);
        invalidate();
        store_evidence(input->object_, prediction, is_simulation);
        return true;
      }
    } else {
      //std::cout<<" match\n";
      offspring = get_offspring(bm, (_Fact *)input->object_, bound_pattern);
      store_evidence(input->object_, prediction, is_simulation);
      return true;
    }
  }
  else
    return false;
}

_Fact* CSTOverlay::bindPattern(_Fact *input, HLPBindingMap* map)
{
  r_code::list<P<_Fact> >::const_iterator p;
  for (p = patterns_.begin(); p != patterns_.end(); ++p) {

    map->load(bindings_);
    if (inputs_.size() == 0)
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

CSTController::CSTController(r_code::View *view) : HLPController(view) {

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

    OUTPUT_LINE(CST_IN, Utils::RelativeTime(Now()) << " cst " << getObject()->get_oid() << " <- " << input->object_->get_oid());
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
    if (target_ihlp->code(0).asOpcode() == Opcodes::ICst && target_ihlp->get_reference(0) == getObject()) { // f is f->icst; produce as many sub-goals as there are patterns in the cst.

      if (!get_requirement_count()) { // models will attempt to produce the icst

        P<HLPBindingMap> bm = new HLPBindingMap(bindings_);
        bm->init_from_f_ihlp(goal_target);
        if (evaluate_bwd_guards(bm)) // leaves the controller constant: no need to protect; bm may be updated.
          abduce(bm, input->object_);
      }
    }
    else {
      // Make an icst by matching a single member, then make it a goal to instantiate it. This will
      // propagate bindings to other members of the composite state and make subgoals from them..
      // Imitate CSTOverlay::reduce.
      CSTOverlay overlay(this, bindings_);
      overlay.load_patterns();
      P<HLPBindingMap> bm = new HLPBindingMap();
      _Fact *bound_pattern = overlay.bindPattern(goal_target, bm);
      if (bound_pattern) {
        // The match has filled in the binding map. Make an icst from it and inject it as a goal.
        // TODO: Call load_code (if needed) and evaluate backward guards.
        std::vector<P<_Fact> > empty_inputs;
        Fact *f_icst = get_f_icst(bm, &empty_inputs);
        Sim* sim = goal->get_sim();
        Sim* sub_sim;
        bool opposite = goal->get_target()->is_anti_fact();
        if (sim->get_mode() == SIM_ROOT)
          sub_sim = new Sim(opposite ? SIM_MANDATORY : SIM_OPTIONAL, sim->get_thz(), input->object_, opposite, sim->root_, 1,
            sim->solution_controller_, sim->get_solution_cfd(), Timestamp(seconds(0)));
        else
          sub_sim = new Sim(sim->get_mode(), sim->get_thz(), input->object_, opposite, sim->root_, 1,
            sim->solution_controller_, sim->get_solution_cfd(), sim->get_solution_before());

        inject_goal(bm, input->object_, f_icst, sub_sim, Now(), goal->get_target()->get_cfd(), get_host());
      }
    }
  } else {
    // std::cout<<"CTRL: "<<get_host()->get_oid()<<" > "<<input->object->get_oid()<<std::endl;
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

        match = ((CSTOverlay *)*o)->reduce(input, offspring);
        if (offspring) {
          overlays_.push_front(offspring);
          ++o;
        }
        else if (match) {
          // full match: no offspring.
          (*o)->invalidate();
          o = overlays_.erase(o);
        }
        else
          ++o;
      }
    }
    reductionCS_.leave();

    check_last_match_time(match);
  }
}

void CSTController::abduce(HLPBindingMap *bm, Fact *f_super_goal) {

  Goal *g = f_super_goal->get_goal();
  _Fact *super_goal_target = g->get_target();
  bool opposite = (super_goal_target->is_anti_fact());

  float32 confidence = super_goal_target->get_cfd();

  Sim *sim = g->get_sim();

  Code *cst = get_unpacked_object();
  uint16 obj_set_index = cst->code(CST_OBJS).asIndex();
  uint16 obj_count = cst->code(obj_set_index).getAtomCount();
  Group *host = get_host();
  auto now = Now();
  for (uint16 i = 1; i <= obj_count; ++i) {

    _Fact *pattern = (_Fact *)cst->get_reference(cst->code(obj_set_index + i).asIndex());
    _Fact *bound_pattern = (_Fact *)bm->bind_pattern(pattern);
    if (_Mem::Get()->matchesAxiom(bound_pattern->get_reference(0)))
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
        inject_goal(bm, f_super_goal, bound_pattern, sim, now, confidence, host); // all sub-goals share the same sim.
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

  if (sim && !sim->registerGoalTarget(sub_goal_target))
    // We are already simulating from this goal, so abort to avoid loops.
    return;

  sub_goal_target->set_cfd(confidence);

  Goal *sub_goal = new Goal(sub_goal_target, f_super_goal->get_goal()->get_actor(), sim, 1);

  _Fact *f_icst = f_super_goal->get_goal()->get_target();
  _Fact *sub_goal_f = new Fact(sub_goal, now, now, 1, 1);

  View *view = new View(View::SYNC_ONCE, now, confidence, 1, group, group, sub_goal_f); // SYNC_ONCE,res=1.
  _Mem::Get()->inject(view);
  OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " cst " << getObject()->get_oid() << ": fact " <<
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

  return bindings->build_f_ihlp(getObject(), Opcodes::ICst, false);
}

Fact *CSTController::get_f_icst(HLPBindingMap *bindings, std::vector<P<_Fact> > *inputs) const {

  Fact *f_icst = get_f_ihlp(bindings, false);
  ((ICST *)f_icst->get_reference(0))->bindings_ = bindings;
  ((ICST *)f_icst->get_reference(0))->components_ = *inputs;
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
  getView()->force_res(0);
}

void CSTController::check_last_match_time(bool match) {

  auto now = Now();
  if (match)
    last_match_time_ = now;
  else if (now - last_match_time_ > _Mem::Get()->get_primary_thz())
    kill_views();
}
}
