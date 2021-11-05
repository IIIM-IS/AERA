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

#include "factory.h"
#include "mdl_controller.h"
#include "mem.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;

namespace r_exec {

MkNew::MkNew(r_code::Mem *m, Code *object) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkNew, 2);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkLowRes::MkLowRes(r_code::Mem *m, Code *object) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkLowRes, 2);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkLowSln::MkLowSln(r_code::Mem *m, Code *object) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkLowSln, 2);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkHighSln::MkHighSln(r_code::Mem *m, Code *object) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkHighSln, 2);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkLowAct::MkLowAct(r_code::Mem *m, Code *object) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkLowAct, 2);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkHighAct::MkHighAct(r_code::Mem *m, Code *object) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkHighAct, 2);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkSlnChg::MkSlnChg(r_code::Mem *m, Code *object, float32 value) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkSlnChg, 3);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(value); // change.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkActChg::MkActChg(r_code::Mem *m, Code *object, float32 value) : LObject(m) {

  uint16 write_index = 0;
  code(write_index++) = Atom::Marker(Opcodes::MkActChg, 3);
  code(write_index++) = Atom::RPointer(0); // object.
  code(write_index++) = Atom::Float(value); // change.
  code(write_index++) = Atom::Float(0); // psln_thr.
  set_reference(0, object);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

_Fact::_Fact() : LObject() {
}

_Fact::_Fact(SysObject *source) : LObject(source) {
}

_Fact::_Fact(_Fact *f) {

  for (uint16 i = 0; i < f->code_size(); ++i)
    code(i) = f->code(i);
  for (uint16 i = 0; i < f->references_size(); ++i)
    add_reference(f->get_reference(i));
}

_Fact::_Fact(uint16 opcode, Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr) : LObject() {

  code(0) = Atom::Object(opcode, FACT_ARITY);
  code(FACT_OBJ) = Atom::RPointer(0);
  code(FACT_AFTER) = Atom::IPointer(FACT_ARITY + 1);
  code(FACT_BEFORE) = Atom::IPointer(FACT_ARITY + 4);
  code(FACT_CFD) = Atom::Float(confidence);
  code(FACT_ARITY) = Atom::Float(psln_thr);
  Utils::SetTimestamp<Code>(this, FACT_AFTER, after);
  Utils::SetTimestamp<Code>(this, FACT_BEFORE, before);
  add_reference(object);
}

_Fact *_Fact::get_absentee() const {

  _Fact *absentee;
  if (is_fact())
    absentee = new AntiFact(get_reference(0), get_after(), get_before(), 1, 1);
  else
    absentee = new Fact(get_reference(0), get_after(), get_before(), 1, 1);

  return absentee;
}

bool _Fact::is_invalidated() {

  if (LObject::is_invalidated())
    return true;
  if (get_reference(0)->is_invalidated()) {

    invalidate();
    return true;
  }
  return false;
}

bool _Fact::match_timings_sync(const _Fact *evidence) const { // intervals of the form [after,before].

  auto after = get_after();
  auto e_after = evidence->get_after();
  auto e_before = evidence->get_before();

  return !(e_after > after + Utils::GetTimeTolerance() || e_before <= after);
}

bool _Fact::match_timings_overlap(const _Fact *evidence) const { // intervals of the form [after,before].

  auto after = get_after();
  auto before = get_before();
  auto e_after = evidence->get_after();
  auto e_before = evidence->get_before();

  return !(e_after >= before || e_before <= after);
}

bool _Fact::match_timings_inclusive(const _Fact *evidence) const { // intervals of the form [after,before].

  auto after = get_after();
  auto before = get_before();
  auto e_after = evidence->get_after();
  auto e_before = evidence->get_before();

  return (e_after <= after && e_before >= before);
}

bool _Fact::Match(const Code *lhs, uint16 lhs_base_index, uint16 lhs_index, const Code *rhs, uint16 rhs_index, uint16 lhs_arity, bool same_binding_state) {

  uint16 lhs_full_index = lhs_base_index + lhs_index;
  Atom lhs_atom = lhs->code(lhs_full_index);
  Atom rhs_atom = rhs->code(rhs_index);
  uint8 lhs_descriptor = lhs_atom.getDescriptor();
  uint8 rhs_descriptor = rhs_atom.getDescriptor();

  if (same_binding_state) {
    if (lhs_descriptor == Atom::T_WILDCARD || lhs_descriptor == Atom::WILDCARD ||
        rhs_descriptor == Atom::T_WILDCARD || rhs_descriptor == Atom::WILDCARD) {
      // A wildcard will match anything including a variable when it becomes bound.
    }
    else {
      bool lhs_is_VL_PTR = (lhs_descriptor == Atom::VL_PTR);
      bool rhs_is_VL_PTR = (rhs_descriptor == Atom::VL_PTR);
      if (lhs_is_VL_PTR != rhs_is_VL_PTR)
        // Not the same binding state.
        return false;
    }
  }

  switch (lhs_descriptor) {
  case Atom::T_WILDCARD:
  case Atom::WILDCARD:
  case Atom::VL_PTR:
    break;
  case Atom::I_PTR:
    switch (rhs_descriptor) {
    case Atom::I_PTR:
      if (!MatchStructure(lhs, lhs_atom.asIndex(), 0, rhs, rhs_atom.asIndex(), same_binding_state))
        return false;
      break;
    case Atom::T_WILDCARD:
    case Atom::WILDCARD:
    case Atom::VL_PTR:
      break;
    default:
      return false;
    }
    break;
  case Atom::R_PTR:
    switch (rhs_descriptor) {
    case Atom::R_PTR:
      if (!MatchObject(lhs->get_reference(lhs_atom.asIndex()), rhs->get_reference(rhs_atom.asIndex()), same_binding_state))
        return false;
      break;
    case Atom::T_WILDCARD:
    case Atom::WILDCARD:
    case Atom::VL_PTR:
      break;
    default:
      return false;
    }
    break;
  default:
    switch (rhs_descriptor) {
    case Atom::T_WILDCARD:
    case Atom::WILDCARD:
    case Atom::VL_PTR:
      break;
    default:
      if (!MatchAtom(lhs_atom, rhs_atom))
        return false;
      break;
    }
    break;
  }

  if (lhs_index == lhs_arity)
    return true;
  return Match(lhs, lhs_base_index, lhs_index + 1, rhs, rhs_index + 1, lhs_arity, same_binding_state);
}

bool _Fact::MatchAtom(Atom lhs, Atom rhs) {

  if (lhs == rhs)
    return true;

  if (lhs.isFloat() && rhs.isFloat())
    return Utils::Equal(lhs.asFloat(), rhs.asFloat());

  return false;
}

bool _Fact::MatchStructure(const Code *lhs, uint16 lhs_base_index, uint16 lhs_index, const Code *rhs, uint16 rhs_index, bool same_binding_state) {

  uint16 lhs_full_index = lhs_base_index + lhs_index;
  Atom lhs_atom = lhs->code(lhs_full_index);
  Atom rhs_atom = rhs->code(rhs_index);
  if (lhs_atom != rhs_atom)
    return false;
  uint16 arity = lhs_atom.getAtomCount();
  if (arity == 0) // empty sets.
    return true;
  if (lhs_atom.getDescriptor() == Atom::TIMESTAMP)
    return Utils::Synchronous(Utils::GetTimestamp(&lhs->code(lhs_full_index)), Utils::GetTimestamp(&rhs->code(rhs_index)));
  return Match(lhs, lhs_base_index, lhs_index + 1, rhs, rhs_index + 1, arity, same_binding_state);
}

bool _Fact::MatchObject(const Code *lhs, const Code *rhs, bool same_binding_state) {

  if (lhs->code(0) != rhs->code(0))
    return false;
  uint16 lhs_opcode = lhs->code(0).asOpcode();
  if (lhs_opcode == Opcodes::Ent ||
    lhs_opcode == Opcodes::Ont ||
    lhs_opcode == Opcodes::Mdl ||
    lhs_opcode == Opcodes::Cst)
    return lhs == rhs;
  return Match(lhs, 0, 1, rhs, 1, lhs->code(0).getAtomCount(), same_binding_state);
}

bool _Fact::CounterEvidence(const Code *lhs, const Code *rhs) {

  uint16 opcode = lhs->code(0).asOpcode();
  if (opcode == Opcodes::Ent ||
    opcode == Opcodes::Ont ||
    opcode == Opcodes::IMdl)
    return false;
  if (lhs->code(0) != rhs->code(0))
    return false;
  if (opcode == Opcodes::MkVal) {

    if (lhs->get_reference(lhs->code(MK_VAL_OBJ).asIndex()) == rhs->get_reference(rhs->code(MK_VAL_OBJ).asIndex()) &&
      lhs->get_reference(lhs->code(MK_VAL_ATTR).asIndex()) == rhs->get_reference(rhs->code(MK_VAL_ATTR).asIndex())) { // same attribute for the same object; value_: r_ptr, atomic value or structure.

      Atom lhs_atom = lhs->code(MK_VAL_VALUE);
      Atom rhs_atom = rhs->code(MK_VAL_VALUE);

      if (lhs_atom.isFloat()) {

        if (rhs_atom.isFloat())
          return !MatchAtom(lhs_atom, rhs_atom);
        else
          return false;
      } else if (rhs_atom.isFloat())
        return false;

      uint16 lhs_desc = lhs_atom.getDescriptor();
      uint16 rhs_desc = rhs_atom.getDescriptor();

      if (lhs_desc == Atom::NIL && rhs_desc == Atom::R_PTR ||
          lhs_desc == Atom::R_PTR && rhs_desc == Atom::NIL)
        // nil is counter-evidence of a referenced object.
        return true;

      if (lhs_desc != rhs_desc) // values of different types.
        return false;
      switch (lhs_desc) {
      case Atom::T_WILDCARD:
      case Atom::WILDCARD:
        return false;
      case Atom::R_PTR:
        return !MatchObject(lhs->get_reference(lhs->code(MK_VAL_VALUE).asIndex()), rhs->get_reference(rhs->code(MK_VAL_VALUE).asIndex()));
      case Atom::I_PTR:
        return !MatchStructure(lhs, MK_VAL_VALUE, lhs_atom.asIndex(), rhs, rhs_atom.asIndex(), false);
      default:
        return !MatchAtom(lhs_atom, rhs_atom);
      }
    }
  } else if (opcode == Opcodes::ICst) {

    if (lhs->get_reference(0) != rhs->get_reference(0)) // check if the icsts instantiate the same cst.
      return false;

    for (uint32 i = 0; i < ((ICST *)lhs)->components_.size(); ++i) { // compare all components 2 by 2.

      // JTNote: This assumes that the components_ in the lhs and rhs are in the same order.
      if (CounterEvidence(((ICST *)lhs)->components_[i], ((ICST *)rhs)->components_[i]))
        return true;
    }
  }

  return false;
}

MatchResult _Fact::is_evidence(const _Fact *target) const {

  if (MatchObject(get_reference(0), target->get_reference(0))) {

    MatchResult r;
    if (target->code(0) == code(0))
      r = MATCH_SUCCESS_POSITIVE;
    else
      r = MATCH_SUCCESS_NEGATIVE;

    if (target->match_timings_overlap(this))
      return r;
  } else if (target->code(0) == code(0)) { // check for a counter-evidence only if both the lhs and rhs are of the same kind of fact.

    if (target->match_timings_inclusive(this)) { // check timings first as this is less expensive than the counter-evidence check.

      if (CounterEvidence(get_reference(0), target->get_reference(0)))
        return MATCH_SUCCESS_NEGATIVE;
    }
  }
  return MATCH_FAILURE;
}

MatchResult _Fact::is_timeless_evidence(const _Fact *target) const {

  if (MatchObject(get_reference(0), target->get_reference(0))) {

    MatchResult r;
    if (target->code(0) == code(0))
      r = MATCH_SUCCESS_POSITIVE;
    else
      r = MATCH_SUCCESS_NEGATIVE;

    return r;
  } else if (target->code(0) == code(0)) { // check for a counter-evidence only if both the lhs and rhs are of the same kind of fact.

    if (CounterEvidence(get_reference(0), target->get_reference(0)))
      return MATCH_SUCCESS_NEGATIVE;
  }
  return MATCH_FAILURE;
}

Timestamp _Fact::get_after() const {

  return Utils::GetTimestamp<Code>(this, FACT_AFTER);
}

Timestamp _Fact::get_before() const {

  return Utils::GetTimestamp<Code>(this, FACT_BEFORE);
}

void _Fact::trace() const {

  std::cout << "<" << get_oid() << " " << Utils::RelativeTime(get_after()) << " " << Utils::RelativeTime(get_before()) << ">" << std::endl;
}

////////////////////////////////////////////////////////////////

void *Fact::operator new(size_t s) {

  return _Mem::Get()->_build_object(Atom::Object(Opcodes::Fact, FACT_ARITY));
}

Fact::Fact() : _Fact() {

  code(0) = Atom::Object(Opcodes::Fact, FACT_ARITY);
}

Fact::Fact(SysObject *source) : _Fact(source) {
}

Fact::Fact(Fact *f) : _Fact(f) {
}

Fact::Fact(Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr) : _Fact(Opcodes::Fact, object, after, before, confidence, psln_thr) {
}

////////////////////////////////////////////////////////////////

void *AntiFact::operator new(size_t s) {

  return _Mem::Get()->_build_object(Atom::Object(Opcodes::AntiFact, FACT_ARITY));
}

AntiFact::AntiFact() : _Fact() {

  code(0) = Atom::Object(Opcodes::AntiFact, FACT_ARITY);
}

AntiFact::AntiFact(SysObject *source) : _Fact(source) {
}

AntiFact::AntiFact(AntiFact *f) : _Fact(f) {
}

AntiFact::AntiFact(Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr) : _Fact(Opcodes::AntiFact, object, after, before, confidence, psln_thr) {
}

////////////////////////////////////////////////////////////////

Pred::Pred() : LObject(), is_promoted_(false) {
}

Pred::Pred(SysObject *source) : LObject(source), is_promoted_(false) {
}

Pred::Pred(_Fact *target, const Pred* simulations_source, float32 psln_thr) : LObject() {
  vector<P<Sim>> simulations_copy;
  for (uint16 i = 0; i < simulations_source->get_simulations_size(); ++i)
    simulations_copy.push_back(simulations_source->get_simulation(i));
  construct(target, simulations_copy, psln_thr);
  defeasible_validities_ = simulations_source->defeasible_validities_;
}

void Pred::construct(_Fact *target, const vector<P<Sim> >& simulations, float32 psln_thr) {

  uint16 write_index = PRED_ARITY;
  code(0) = Atom::Object(Opcodes::Pred, PRED_ARITY);
  code(PRED_TARGET) = Atom::RPointer(0);
  add_reference(target);

  code(PRED_SIMS) = Atom::IPointer(++write_index);
  code(write_index) = Atom::Set(simulations.size());
  for (auto i = 0; i < simulations.size(); ++i) {
    code(++write_index) = Atom::RPointer(references_size());
    add_reference(simulations[i]);
  }

  code(PRED_ARITY) = Atom::Float(psln_thr);
  is_promoted_ = false;
}

bool Pred::is_invalidated() {

  if (LObject::is_invalidated())
    return true;
  for (uint16 i = 0; i < get_simulations_size(); ++i) {

    if (get_simulation(i)->is_invalidated()) {

      invalidate();
      return true;
    }
  }
  for (uint32 i = 0; i < grounds_.size(); ++i) {

    if (grounds_[i]->is_invalidated()) {

      invalidate();
      return true;
    }
  }

  for (auto d = defeasible_validities_.begin(); d != defeasible_validities_.end(); ++d) {
    if ((*d)->is_invalidated()) {
      invalidate();
      return true;
    }
  }

  if (get_reference(0)->is_invalidated()) {

    invalidate();
    return true;
  }
  return false;
}

bool Pred::grounds_invalidated(_Fact *evidence) {

  for (uint32 i = 0; i < grounds_.size(); ++i) {

    switch (evidence->is_evidence(grounds_[i])) {
    case MATCH_SUCCESS_NEGATIVE:
      return true;
    default:
      break;
    }
  }
  return false;
}

Sim *Pred::get_simulation(Controller *root) const {

  for (uint16 i = 0; i < get_simulations_size(); ++i) {

    auto sim = get_simulation(i);
    if (sim->root_ == root)
      return sim;
  }

  return NULL;
}

bool Pred::has_simulation(Sim* sim) const
{
  for (uint16 i = 0; i < get_simulations_size(); ++i) {
    if (get_simulation(i) == sim)
      return true;
  }

  return false;
}

////////////////////////////////////////////////////////////////

Goal::Goal() : LObject(), ground_(NULL) {
}

Goal::Goal(SysObject *source) : LObject(source), ground_(NULL) {
}

Goal::Goal(_Fact *target, Code *actor, Sim* sim, float32 psln_thr) : LObject(), ground_(NULL) {

  code(0) = Atom::Object(Opcodes::Goal, GOAL_ARITY);
  code(GOAL_TARGET) = Atom::RPointer(0);
  code(GOAL_ACTR) = Atom::RPointer(1);
  code(GOAL_SIM) = (sim ? Atom::RPointer(2) : Atom::Nil());
  code(GOAL_ARITY) = Atom::Float(psln_thr);
  add_reference(target);
  add_reference(actor);
  if (sim)
    add_reference(sim);
}

bool Goal::invalidate() { // return false when was not invalidated, true otherwise.

  if (has_sim())
    get_sim()->invalidate();
  return LObject::invalidate();
}

bool Goal::is_invalidated() {

  if (LObject::is_invalidated())
    return true;
  if (has_sim() && get_sim()->get_f_super_goal()->is_invalidated()) {

    invalidate();
    return true;
  }
  return false;
}

bool Goal::ground_invalidated(_Fact *evidence) {

  if (ground_ != NULL)
    return ground_->get_pred()->grounds_invalidated(evidence);
  return false;
}

bool Goal::is_requirement() const {

  if (has_sim() && get_sim()->is_requirement_)
    return true;
  return false;
}

bool Goal::is_self_goal() const {

  return (get_actor() == _Mem::Get()->get_self());
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Sim::Sim(Sim *s) : root_(s->root_), solution_controller_(s->solution_controller_), is_requirement_(false) {
  for (uint16 i = 0; i < s->code_size(); ++i)
    code(i) = s->code(i);
  for (uint16 i = 0; i < s->references_size(); ++i)
    add_reference(s->get_reference(i));
}

Sim::Sim(SimMode mode, microseconds thz, Fact *super_goal, bool opposite, Controller *root, float32 psln_thr, Controller *solution_controller,
  float32 solution_cfd, Timestamp solution_before) : root_(root), solution_controller_(solution_controller), is_requirement_(false) {
  code(0) = Atom::Object(Opcodes::Sim, SIM_ARITY);
  code(SIM_MODE) = Atom::Float(mode);
  code(SIM_THZ) = Atom::IPointer(SIM_ARITY + 1);
  code(SIM_F_SUPER_GOAL) = Atom::RPointer(0);
  add_reference(super_goal);
  code(SIM_OPPOSITE) = Atom::Boolean(opposite);

  // Special case: Put the controller's model in the code so that it appears in the decompiled output,
  // otherwise the C++ code doesn't look at these objects.
  // This only works because the seed code never creates a sim object or changes its controllers.
  if (root) {
    code(SIM_ROOT_MODEL) = Atom::RPointer(references_size());
    add_reference(root->get_core_object());
  }
  else
    code(SIM_ROOT_MODEL) = Atom::Nil();
  if (solution_controller) {
    code(SIM_SOLUTION_MODEL) = Atom::RPointer(references_size());
    add_reference(solution_controller->get_core_object());
  }
  else
    code(SIM_SOLUTION_MODEL) = Atom::Nil();

  code(SIM_SOLUTION_CFD) = Atom::Float(solution_cfd);
  code(SIM_SOLUTION_BEFORE) = Atom::IPointer(SIM_ARITY + 4);
  code(SIM_ARITY) = Atom::Float(psln_thr);

  // The time horizon is stored as a timestamp, but it is actually a duration.
  Utils::SetTimestamp<Code>(this, SIM_THZ, Timestamp(thz));
  Utils::SetTimestamp<Code>(this, SIM_SOLUTION_BEFORE, solution_before);
}

bool Sim::invalidate() {

  if (LObject::is_invalidated())
    return true;
  if (get_f_super_goal()->is_invalidated())
    return true;

  return LObject::invalidate();
}

bool Sim::is_invalidated() {

  if (LObject::is_invalidated())
    return true;
  if (get_f_super_goal()->is_invalidated()) {

    invalidate();
    return true;
  }
  return false;
}

Sim* Sim::get_root_sim()
{
  Sim* result = this;
  while (result) {
    if (result->get_mode() == SIM_ROOT)
      return result;

    result = result->get_f_super_goal()->get_goal()->get_sim();
  }

  // A goal had a NULL Sim object. This shouldn't happen.
  return NULL;
}

bool Sim::register_goal_target(_Fact* f_obj) {
  Sim* rootSim = get_root_sim();
  if (!rootSim)
    // We don't expect this. Return true so that f_obj is injected as a goal anyway.
    return true;

  uint16 opcode = f_obj->code(0).asOpcode();
  Code* obj = f_obj->get_reference(0);
  // Debug: Do we need a critical section for this?
  for (int i = 0; i < rootSim->goalTargets_.size(); ++i) {
    _Fact* goalTarget = rootSim->goalTargets_[i];
    // TODO: Actually match timings. For now, ignore timings and just check the fact opcode and match the object.
    if (opcode == goalTarget->code(0).asOpcode() && _Fact::MatchObject(obj, goalTarget->get_reference(0), true))
      // Already registered.
      return false;
  }

  rootSim->goalTargets_.push_back(f_obj);
  return true;
}

bool Sim::DefeasiblePromotedFact::has_original_fact(const r_code::list<DefeasiblePromotedFact>& list, const _Fact* original_fact) {
  for (auto d = list.begin(); d != list.end(); ++d) {
    if (original_fact == d->original_fact_)
      return true;
  }

  return false;
}


////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MkRdx::MkRdx() : LObject(), bindings_(NULL) {
}

MkRdx::MkRdx(SysObject *source) : LObject(source), bindings_(NULL) {
}

MkRdx::MkRdx(Code *imdl_fact, Code *input, Code *output, float32 psln_thr, BindingMap *binding_map) : LObject(), bindings_(binding_map) {

  uint16 extent_index = MK_RDX_ARITY + 1;
  code(0) = Atom::Marker(Opcodes::MkRdx, MK_RDX_ARITY);
  code(MK_RDX_CODE) = Atom::RPointer(0); // code.
  add_reference(imdl_fact);
  code(MK_RDX_INPUTS) = Atom::IPointer(extent_index); // inputs.
  code(MK_RDX_ARITY) = Atom::Float(psln_thr);
  code(extent_index++) = Atom::Set(1); // set of one input.
  code(extent_index++) = Atom::RPointer(1);
  add_reference(input);
  code(MK_RDX_PRODS) = Atom::IPointer(extent_index); // set of one production.
  code(extent_index++) = Atom::Set(1);
  code(extent_index++) = Atom::RPointer(2);
  add_reference(output);
}

MkRdx::MkRdx(Code *imdl_fact, Code *inpu1, Code *input2, Code *output, float32 psln_thr, BindingMap *binding_map) : LObject(), bindings_(binding_map) {

  uint16 extent_index = MK_RDX_ARITY + 1;
  code(0) = Atom::Marker(Opcodes::MkRdx, MK_RDX_ARITY);
  code(MK_RDX_CODE) = Atom::RPointer(0); // code.
  add_reference(imdl_fact);
  code(MK_RDX_INPUTS) = Atom::IPointer(extent_index); // inputs.
  code(MK_RDX_ARITY) = Atom::Float(psln_thr);
  code(extent_index++) = Atom::Set(2); // set of two inputs.
  code(extent_index++) = Atom::RPointer(1);
  add_reference(inpu1);
  code(extent_index++) = Atom::RPointer(2);
  add_reference(input2);
  code(MK_RDX_PRODS) = Atom::IPointer(extent_index); // set of one production.
  code(extent_index++) = Atom::Set(1);
  code(extent_index++) = Atom::RPointer(3);
  add_reference(output);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Success::Success() : LObject() {
}

Success::Success(_Fact *object, _Fact *evidence, float32 psln_thr) : LObject() {

  code(0) = Atom::Object(Opcodes::Success, SUCCESS_ARITY);
  code(SUCCESS_OBJ) = Atom::RPointer(0);
  if (evidence)
    code(SUCCESS_EVD) = Atom::RPointer(1);
  else
    code(SUCCESS_EVD) = Atom::Nil();
  code(SUCCESS_ARITY) = Atom::Float(psln_thr);
  add_reference(object);
  if (evidence)
    add_reference(evidence);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

Perf::Perf() : LObject() {
}

Perf::Perf(microseconds reduction_job_avg_latency, microseconds d_reduction_job_avg_latency, microseconds time_job_avg_latency, microseconds d_time_job_avg_latency) : LObject() {

  code(0) = Atom::Object(Opcodes::Perf, PERF_ARITY);
  code(PERF_RDX_LTCY) = Atom::Float(reduction_job_avg_latency.count());
  code(PERF_D_RDX_LTCY) = Atom::Float(d_reduction_job_avg_latency.count());
  code(PERF_TIME_LTCY) = Atom::Float(time_job_avg_latency.count());
  code(PERF_D_TIME_LTCY) = Atom::Float(d_time_job_avg_latency.count());
  code(PERF_ARITY) = Atom::Float(1);
}

////////////////////////////////////////////////////////////////

ICST::ICST() : LObject() {
}

ICST::ICST(SysObject *source) : LObject(source) {
}

bool ICST::is_invalidated() {

  if (LObject::is_invalidated()) {
    //std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" icst was invalidated"<<std::endl;
    return true; }
  for (uint32 i = 0; i < components_.size(); ++i) {

    if (components_[i]->is_invalidated()) {

      invalidate();
      //std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" "<<std::hex<<this<<std::dec<<" icst invalidated"<<std::endl;
      return true;
    }
  }
  return false;
}

bool ICST::contains(_Fact *component, uint16 &component_index) const {

  for (uint32 i = 0; i < components_.size(); ++i) {

    if (components_[i] == component) {

      component_index = i;
      return true;
    }
  }

  return false;
}
}
