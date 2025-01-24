//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2025 Jeff Thompson
//_/_/ Copyright (c) 2018-2025 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2025 Icelandic Institute for Intelligent Machines
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

#include "hlp_context.h"
#include "operator.h"
#include "opcodes.h"


namespace r_exec {

HLPContext::HLPContext() : _Context(NULL, 0, NULL, UNDEFINED) {
}

HLPContext::HLPContext(Atom *code, uint16 index, HLPOverlay *const overlay, Data data) : _Context(code, index, overlay, data) {
}

bool HLPContext::operator ==(const HLPContext &c) const {

  HLPContext lhs = dereference();
  HLPContext rhs = c.dereference();

  if (lhs[0] != rhs[0]) // both contexts point to an atom which is not a pointer.
    return false;

  if (lhs[0].isStructural()) { // both are structural.

    uint16 atom_count = lhs.get_children_count();
    for (uint16 i = 1; i <= atom_count; ++i)
      if (lhs.get_child_deref(i) != rhs.get_child_deref(i))
        return false;
    return true;
  }
  return true;
}

bool HLPContext::operator !=(const HLPContext &c) const {

  return !(*this == c);
}

HLPContext HLPContext::dereference() const {

  switch ((*this)[0].getDescriptor()) {
  case Atom::I_PTR:
    return HLPContext(code_, (*this)[0].asIndex(), (HLPOverlay *)overlay_, data_).dereference();
  case Atom::VL_PTR: {
    Atom *value_code = ((HLPOverlay *)overlay_)->get_value_code((*this)[0].asIndex());
    if (value_code)
      return HLPContext(value_code, 0, (HLPOverlay *)overlay_, BINDING_MAP).dereference();
    else // unbound variable.
      return HLPContext(); // data=undefined: evaluation will return false.
  }case Atom::VALUE_PTR:
    return HLPContext(&overlay_->values_[0], (*this)[0].asIndex(), (HLPOverlay *)overlay_, VALUE_ARRAY).dereference();
  default:
    return *this;
  }
}

bool HLPContext::evaluate_no_dereference() const {

  switch (data_) {
  case VALUE_ARRAY:
  case BINDING_MAP:
    return true;
  case UNDEFINED:
    return false;
  }

  switch (code_[index_].getDescriptor()) {
  case Atom::ASSIGN_PTR: {

    HLPContext c(code_, code_[index_].asIndex(), (HLPOverlay *)overlay_);
    if (c.evaluate_no_dereference()) {

      ((HLPOverlay *)overlay_)->bindings_->bind_variable(code_, code_[index_].asAssignmentIndex(), code_[index_].asIndex(), &overlay_->values_[0]);
      return true;
    } else if (((HLPOverlay*)overlay_)->bindings_->scan_variable(code_[index_].asAssignmentIndex())) {
      // The assignment expression could not be evaluated, but the assignment variable is already bound.
      return true;
    } else
      return false;
  }case Atom::OPERATOR: {

    uint16 atom_count = get_children_count();
    for (uint16 i = 1; i <= atom_count; ++i) {

      if (!get_child_deref(i).evaluate_no_dereference())
        return false;
    }

    Operator op = Operator::Get((*this)[0].asOpcode());
    HLPContext *c = new HLPContext(*this);
    Context _c(c);
    return op(_c);
  }case Atom::OBJECT:
  case Atom::MARKER:
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_CPP_PROGRAM:
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
  case Atom::INSTANTIATED_ANTI_PROGRAM:
  case Atom::COMPOSITE_STATE:
  case Atom::MODEL:
  case Atom::GROUP:
  case Atom::SET:
  case Atom::S_SET: {

    uint16 atom_count = get_children_count();
    for (uint16 i = 1; i <= atom_count; ++i) {

      if (!get_child_deref(i).evaluate_no_dereference())
        return false;
    }
    return true;
  }default:
    return true;
  }
}

uint16 HLPContext::get_object_code_size() const {

  switch (data_) {
  case STEM:
    return ((HLPOverlay *)overlay_)->get_unpacked_object()->code_size();
  case BINDING_MAP:
    return ((HLPOverlay *)overlay_)->get_value_code_size((*this)[0].asIndex());
  case VALUE_ARRAY:
    return overlay_->values_.size();
  default:
    return 0;
  }
}
}
