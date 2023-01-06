//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
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

#include "hlp_overlay.h"
#include "hlp_controller.h"
#include "hlp_context.h"
#include "mem.h"

using namespace r_code;

namespace r_exec {

bool HLPOverlay::EvaluateBWDGuards(Controller *c, HLPBindingMap *bindings) {

  HLPOverlay o(c, bindings);
  return o.evaluate_bwd_guards();
}

bool HLPOverlay::EvaluateFWDTimings(Controller *c, HLPBindingMap *bindings) {

  HLPOverlay o(c, bindings);
  return o.evaluate_fwd_timings();
}

bool HLPOverlay::ScanBWDGuards(Controller *c, HLPBindingMap *bindings) {

  const HLPOverlay o(c, bindings);
  return o.scan_bwd_guards();
}

HLPOverlay::HLPOverlay(Controller *c, HLPBindingMap *bindings) : Overlay(c, true), bindings_(bindings) {
}

HLPOverlay::HLPOverlay(Controller *c, const HLPBindingMap *bindings, bool load_code) : Overlay(c, load_code) {

  bindings_ = new HLPBindingMap((HLPBindingMap *)bindings);
}

HLPOverlay::~HLPOverlay() {
}

Atom *HLPOverlay::get_value_code(uint16 id) const {

  return bindings_->get_value_code(id);
}

uint16 HLPOverlay::get_value_code_size(uint16 id) const {

  return bindings_->get_value_code_size(id);
}

inline bool HLPOverlay::evaluate_guards(uint16 guard_set_iptr_index) {

  uint16 guard_set_index = code_[guard_set_iptr_index].asIndex();
  uint16 guard_count = code_[guard_set_index].getAtomCount();
  for (uint16 i = 1; i <= guard_count; ++i) {

    // Get the HLPContext like in HLPOverlay::evaluate.
    HLPContext c(code_, guard_set_index + i, this);
    if (!c.evaluate())
      return false;
    if (c.dereference()[0].isBooleanFalse())
      // This is a boolean guard (not an assignment) and it is false.
      return false;
  }
  return true;
}

bool HLPOverlay::evaluate_fwd_guards() {

  return evaluate_guards(HLP_FWD_GUARDS);
}

bool HLPOverlay::evaluate_bwd_guards() {

  return evaluate_guards(HLP_BWD_GUARDS);
}

bool HLPOverlay::evaluate(uint16 index) {

  HLPContext c(code_, index, this);
  return c.evaluate();
}

bool HLPOverlay::evaluate_fwd_timings() {

  int16 fwd_after_guard_index = -1;
  int16 fwd_before_guard_index = -1;

  uint16 bm_fwd_after_index = bindings_->get_fwd_after_index();
  uint16 bm_fwd_before_index = bindings_->get_fwd_before_index();

  uint16 guard_set_index = code_[HLP_BWD_GUARDS].asIndex();
  uint16 guard_count = code_[guard_set_index].getAtomCount();
  for (uint16 i = 1; i <= guard_count; ++i) { // find the relevant guards.

    uint16 index = guard_set_index + i;
    Atom a = code_[index];
    if (a.getDescriptor() == Atom::ASSIGN_PTR) {

      uint16 _i = a.asAssignmentIndex();
      if (_i == bm_fwd_after_index)
        fwd_after_guard_index = i;
      if (_i == bm_fwd_before_index)
        fwd_before_guard_index = i;
    }
  }

  // These are assignment guards, so we don't need result_index to check a boolean guard.
  if (!bindings_->has_fwd_before()) {
    // We need to evaluate forward before.
    if (fwd_before_guard_index == -1)
      // None of the backward guards assigns the variable for forward before.
      return false;
    if (!evaluate(guard_set_index + fwd_before_guard_index))
#if 1 // Debug: temporary solution to handle dependecies among guards. The full solution would recurse through the guards.
    {
      // This may depend on forward after, so try evaluating it first.
      if (fwd_after_guard_index == -1)
        // None of the backward guards assigns the variable for forward after.
        return false;
      if (!evaluate(guard_set_index + fwd_after_guard_index))
        return false;
      // Now try again to evaluate forward before.
      if (!evaluate(guard_set_index + fwd_before_guard_index))
        return false;
    }
#else
      return false;
#endif
  }

  if (!bindings_->has_fwd_after()) {
    // We need to evaluate forward after.
    if (fwd_after_guard_index == -1)
      // None of the backward guards assigns the variable for forward after.
      return false;
    if (!evaluate(guard_set_index + fwd_after_guard_index))
      return false;
  }

  return true;
}

bool HLPOverlay::scan_bwd_guards() const {

  uint16 guard_set_index = code_[HLP_BWD_GUARDS].asIndex();
  uint16 guard_count = code_[guard_set_index].getAtomCount();
  for (uint16 i = 1; i <= guard_count; ++i) {

    uint16 index = guard_set_index + i;
    Atom a = code_[index];
    switch (a.getDescriptor()) {
    case Atom::I_PTR:
      if (!scan_location(a.asIndex()))
        return false;
      break;
    case Atom::ASSIGN_PTR:
      // If scan_location fails, then succeed if the assignment variable is already bound.
      if (!scan_location(a.asIndex()) && !bindings_->scan_variable(a.asAssignmentIndex()))
        return false;
      break;
    }
  }
  return true;
}

bool HLPOverlay::scan_location(uint16 index) const {

  Atom a = code_[index];
  switch (a.getDescriptor()) {
  case Atom::I_PTR:
    return scan_location(a.asIndex());
  case Atom::ASSIGN_PTR:
    return scan_location(a.asIndex());
  case Atom::VL_PTR:
    if (bindings_->scan_variable(a.asIndex()))
      return true;
    else
      return scan_variable(a.asIndex());
  case Atom::OPERATOR: {
    uint16 atom_count = a.getAtomCount();
    for (uint16 j = 1; j <= atom_count; ++j) {

      if (!scan_location(index + j))
        return false;
    }
    return true;
  }
  default:
    return true;
  }
}

bool HLPOverlay::scan_variable(uint16 index) const { // check if the variable can be bound.

  uint16 guard_set_index = code_[HLP_BWD_GUARDS].asIndex();
  uint16 guard_count = code_[guard_set_index].getAtomCount();
  for (uint16 i = 1; i <= guard_count; ++i) {

    uint16 guard_index = guard_set_index + i;
    Atom a = code_[guard_index];
    switch (a.getDescriptor()) {
    case Atom::ASSIGN_PTR:
      if (a.asAssignmentIndex() == index)
        return scan_location(a.asIndex());
      break;
    }
  }

  return false;
}

Code *HLPOverlay::get_unpacked_object() const {

  return ((HLPController *)controller_)->get_unpacked_object();
}

void HLPOverlay::store_evidence(_Fact *evidence, bool prediction, bool is_simulation) {

  if (prediction) {

    if (!is_simulation)
      ((HLPController *)controller_)->store_predicted_evidence(evidence);
  } else
    ((HLPController *)controller_)->store_evidence(evidence);
}
}
