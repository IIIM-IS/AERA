//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2021 Jeff Thompson
//_/_/ Copyright (c) 2018-2021 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2021 Icelandic Institute for Intelligent Machines
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

#include "context.h"
#include "pgm_overlay.h"
#include "operator.h"
#include "opcodes.h"
#include "mem.h"


using namespace std;
using namespace r_code;

namespace r_exec {

bool IPGMContext::operator ==(const IPGMContext &c) const {
  IPGMContext lhs = dereference();
  IPGMContext rhs = c.dereference();

  if (lhs.data_ == REFERENCE && lhs.index_ == 0 &&
    rhs.data_ == REFERENCE && rhs.index_ == 0) // both point to an object's head, not one of its members.
    return lhs.object_ == rhs.object_;

  // both contexts point to an atom which is not a pointer.
  if (lhs[0] != rhs[0])
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

inline bool IPGMContext::operator !=(const IPGMContext &c) const {

  return !(*this == c);
}

bool IPGMContext::match(const IPGMContext &input) const {

  if (data_ == REFERENCE && index_ == 0 &&
    input.data_ == REFERENCE && input.index_ == 0) { // both point to an object's head, not one of its members.

    if (object_ == input.object_)
      return true;
    return false;
  }

  if (code_[index_].isStructural()) {

    uint16 atom_count = get_children_count();
    if (input.get_children_count() != atom_count)
      return false;

    if (((*this)[0].atom_ & 0xFFFFFFF8) != (input[0].atom_ & 0xFFFFFFF8)) // masking a possible tolerance embedded in timestamps; otherwise, the last 3 bits encode an arity.
      return false;

    for (uint16 i = 1; i <= atom_count; ++i) {

      IPGMContext pc = get_child_deref(i);
      IPGMContext ic = input.get_child_deref(i);
      if (!pc.match(ic))
        return false;
    }
    return true;
  }

  switch ((*this)[0].getDescriptor()) {
  case Atom::WILDCARD:
  case Atom::T_WILDCARD:
    return true;
  case Atom::IPGM_PTR:
    return dereference().match(input);
  default:
    return (*this)[0] == input[0];
  }
}

void IPGMContext::dereference_once() {

  switch ((*this)[0].getDescriptor()) {
  case Atom::I_PTR:
    index_ = (*this)[0].asIndex();
    break;
  default:
    break;
  }
}

IPGMContext IPGMContext::dereference() const {

  switch ((*this)[0].getDescriptor()) {
  case Atom::CODE_VL_PTR: { // evaluate the code if necessary.
      // TODO: OPTIMIZATION: if not in a cptr and if this eventually points to an r-ptr or in-obj-ptr,
      // patch the code at this->index, i.e. replace the vl ptr by the in-obj-ptr or rptr.
    Atom a = code_[(*this)[0].asIndex()];
    uint16 structure_index;
    if (a.getDescriptor() == Atom::I_PTR) // dereference once.
      a = code_[structure_index = a.asIndex()];
    if (a.isStructural()) { // the target location is not evaluated yet.

      IPGMContext s(object_, view_, code_, structure_index, (InputLessPGMOverlay *)overlay_, data_);
      if (s.evaluate_no_dereference())
        return s; // patched code: atom at CODE_VL_PTR's index changed to VALUE_PTR or 32 bits result.
      else // evaluation failed, return undefined context.
        return IPGMContext();
    } else
      return IPGMContext(object_, view_, code_, (*this)[0].asIndex(), (InputLessPGMOverlay *)overlay_, data_).dereference();
  }case Atom::I_PTR:
    return IPGMContext(object_, view_, code_, (*this)[0].asIndex(), (InputLessPGMOverlay *)overlay_, data_).dereference();
  case Atom::R_PTR: {
    Code *o = object_->get_reference((*this)[0].asIndex());
    return IPGMContext(o, NULL, &o->code(0), 0, NULL, REFERENCE);
  }case Atom::C_PTR: {

    IPGMContext c = get_child_deref(1);
    for (uint16 i = 2; i <= get_children_count(); ++i) {

      switch ((*this)[i].getDescriptor()) {
      case Atom::VIEW: // accessible only for this and input objects.
        if (c.view_)
          c = IPGMContext(c.get_object(), c.view_, &c.view_->code(0), 0, NULL, VIEW);
        else
          return IPGMContext();
        break;
      case Atom::MKS:
        return IPGMContext(c.get_object(), MKS);
        break;
      case Atom::VWS:
        return IPGMContext(c.get_object(), VWS);
        break;
      default:
        c = c.get_child_deref((*this)[i].asIndex());
      }
    }
    return c;
  }
  case Atom::THIS: // refers to the ipgm; the pgm view is not available.
    return IPGMContext(overlay_->get_object(), overlay_->get_view(), &overlay_->get_object()->code(0), 0, (InputLessPGMOverlay *)overlay_, REFERENCE);
  case Atom::VIEW: // never a reference, always in a cptr.
    if (overlay_ && object_ == overlay_->get_object())
      return IPGMContext(object_, view_, &view_->code(0), 0, NULL, VIEW);
    return *this;
  case Atom::MKS:
    return IPGMContext(object_, MKS);
  case Atom::VWS:
    return IPGMContext(object_, VWS);
  case Atom::VALUE_PTR:
    return IPGMContext(object_, view_, &overlay_->values_[0], (*this)[0].asIndex(), (InputLessPGMOverlay *)overlay_, VALUE_ARRAY);
  case Atom::IPGM_PTR:
    return IPGMContext(overlay_->get_object(), (*this)[0].asIndex()).dereference();
  case Atom::IN_OBJ_PTR: {
    Code *input_object = ((PGMOverlay *)overlay_)->getInputObject((*this)[0].asInputIndex());
    View *input_view = (r_exec::View*)((PGMOverlay *)overlay_)->getInputView((*this)[0].asInputIndex());
    return IPGMContext(input_object, input_view, &input_object->code(0), (*this)[0].asIndex(), NULL, REFERENCE).dereference();
  }case Atom::D_IN_OBJ_PTR: {
    IPGMContext parent = IPGMContext(object_, view_, code_, (*this)[0].asRelativeIndex(), (InputLessPGMOverlay *)overlay_, data_).dereference();
    Code *parent_object = parent.object_;
    return IPGMContext(parent_object, NULL, &parent_object->code(0), (*this)[0].asIndex(), NULL, REFERENCE).dereference();
  }case Atom::PROD_PTR:
    return IPGMContext(((InputLessPGMOverlay *)overlay_)->productions_[(*this)[0].asIndex()], 0);
  default:
    return *this;
  }
}

bool IPGMContext::evaluate_no_dereference() const {

  switch (data_) {
  case REFERENCE:
  case VALUE_ARRAY:
    return true;
  case UNDEFINED:
    return false;
  case VIEW:
  case MKS:
  case VWS:
    return true;
  }
  switch (code_[index_].getDescriptor()) {
  case Atom::OPERATOR: {

    Operator op = Operator::Get((*this)[0].asOpcode());
    if (op.is_syn())
      return true;
    if (!op.is_red()) { // red will prevent the evaluation of its productions before reducting its input.

      uint16 atom_count = get_children_count();
      for (uint16 i = 1; i <= atom_count; ++i) {

        if (!get_child_deref(i).evaluate_no_dereference())
          return false;
      }
    }
    IPGMContext *__c = new IPGMContext(*this);
    Context _c(__c);
    return op(_c);
  }case Atom::OBJECT: // incl. cmd.
    if ((*this)[0].asOpcode() == Opcodes::Ptn || (*this)[0].asOpcode() == Opcodes::AntiPtn) { // skip patterns.

      return true;
    }
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

inline bool IPGMContext::evaluate() const {

  if (data_ == REFERENCE || data_ == VALUE_ARRAY)
    return true;

  IPGMContext c = dereference();
  return c.evaluate_no_dereference();
}

void IPGMContext::copy_to_value_array(uint16 &position) {

  position = overlay_->values_.size();
  uint16 extent_index;
  if (code_[index_].isStructural())
    copy_structure_to_value_array(false, overlay_->values_.size(), extent_index, true);
  else
    copy_member_to_value_array(0, false, overlay_->values_.size(), extent_index, true);
}

void IPGMContext::copy_structure_to_value_array(bool prefix, uint16 write_index, uint16 &extent_index, bool dereference_cptr) { // prefix: by itpr or not.

  if (code_[index_].getDescriptor() == Atom::OPERATOR && Operator::Get(code_[index_].asOpcode()).is_syn())
    copy_member_to_value_array(1, prefix, write_index, extent_index, dereference_cptr);
  else {

    uint16 atom_count = get_children_count();
    overlay_->values_[write_index++] = code_[index_];
    extent_index = write_index + atom_count;
    switch (code_[index_].getDescriptor()) {
    case Atom::TIMESTAMP:
      for (uint16 i = 1; i <= atom_count; ++i)
        overlay_->values_[write_index++] = code_[index_ + i];
      break;
    case Atom::DURATION:
      for (uint16 i = 1; i <= atom_count; ++i)
        overlay_->values_[write_index++] = code_[index_ + i];
      break;
    case Atom::C_PTR:
      if (!dereference_cptr) {

        copy_member_to_value_array(1, prefix, write_index++, extent_index, false);
        for (uint16 i = 2; i <= atom_count; ++i)
          overlay_->values_[write_index++] = code_[index_ + i];
        break;
      } // else, dereference the c_ptr.
    default:
      if (is_cmd_with_cptr()) {

        for (uint16 i = 1; i <= atom_count; ++i)
          copy_member_to_value_array(i, prefix, write_index++, extent_index, i != 2);
      } else {

        for (uint16 i = 1; i <= atom_count; ++i)
          copy_member_to_value_array(i, prefix, write_index++, extent_index, !(!dereference_cptr && i == 1));
      }
    }
  }
}

void IPGMContext::copy_member_to_value_array(uint16 child_index, bool prefix, uint16 write_index, uint16 &extent_index, bool dereference_cptr) {

  uint16 _index = index_ + child_index;
  Atom head;
dereference:
  head = code_[_index];
  switch (head.getDescriptor()) { // dereference until either we reach some non-pointer or a reference or a value pointer.
  case Atom::I_PTR:
  case Atom::CODE_VL_PTR:
    _index = head.asIndex();
    goto dereference;
  case Atom::VALUE_PTR:
    overlay_->values_[write_index] = Atom::IPointer(head.asIndex());
    return;
  case Atom::PROD_PTR:
  case Atom::IN_OBJ_PTR:
  case Atom::D_IN_OBJ_PTR:
  case Atom::R_PTR:
    overlay_->values_[write_index] = head;
    return;
  case Atom::C_PTR:
    if (dereference_cptr) {

      uint16 saved_index = index_;
      index_ = _index;
      IPGMContext cptr = dereference();
      overlay_->values_[write_index] = cptr[0];
      index_ = saved_index;
      return;
    }
  }

  if (head.isStructural()) {

    uint16 saved_index = index_;
    index_ = _index;
    if (prefix) {

      overlay_->values_[write_index] = Atom::IPointer(extent_index);
      copy_structure_to_value_array(true, extent_index, extent_index, dereference_cptr);
    } else
      copy_structure_to_value_array(true, write_index, extent_index, dereference_cptr);
    index_ = saved_index;
  } else
    overlay_->values_[write_index] = head;
}

void IPGMContext::getMember(void *&object, uint32 &view_oid, ObjectType &object_type, int16 &member_index) const {

  if ((*this)[0].getDescriptor() != Atom::I_PTR) { // ill-formed mod/set expression.

    object = NULL;
    object_type = TYPE_UNDEFINED;
    member_index = 0;
    return;
  }

  IPGMContext cptr = IPGMContext(object_, view_, code_, (*this)[0].asIndex(), (InputLessPGMOverlay *)overlay_, data_); // dereference manually (1 offset) to retain the cptr as is.

  if (cptr[0].getDescriptor() != Atom::C_PTR) { // ill-formed mod/set expression.

    object = NULL;
    object_type = TYPE_UNDEFINED;
    member_index = 0;
    return;
  }

  IPGMContext c = cptr.get_child_deref(1); // this, vl_ptr, value_ptr or rptr.

  uint16 atom_count = cptr.get_children_count();
  for (uint16 i = 2; i < atom_count; ++i) { // stop before the last iptr.

    if (cptr[i].getDescriptor() == Atom::VIEW)
      c = IPGMContext(c.get_object(), c.view_, &c.view_->code(0), 0, NULL, VIEW);
    else
      c = c.get_child_deref(cptr[i].asIndex());
  }

  // at this point, c is an iptr dereferenced to an object or a view; the next iptr holds the member index.
  // c is pointing at the first atom of an object or a view.
  switch (c[0].getDescriptor()) {
  case Atom::OBJECT:
  case Atom::MARKER:
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_CPP_PROGRAM:
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
  case Atom::INSTANTIATED_ANTI_PROGRAM:
  case Atom::COMPOSITE_STATE:
  case Atom::MODEL:
    object_type = TYPE_OBJECT;
    object = c.get_object();
    break;
  case Atom::GROUP:
  case Atom::SET: // dynamically generated views can be sets.
  case Atom::S_SET: // views are always copied; set the object to the view's group on which to perform an operation for the view's oid.
    if (c.data_ == VALUE_ARRAY)
      object = (Group *)c[VIEW_CODE_MAX_SIZE].atom_; // first reference of grp in a view stored in athe value array.
    else
      object = c.view_->get_host();
    view_oid = c[VIEW_OID].atom_; // oid is hidden at the end of the view code; stored directly as a uint32.
    object_type = TYPE_VIEW;
    break;
  default: // atomic value or ill-formed mod/set expression.
    object = NULL;
    object_type = TYPE_UNDEFINED;
    return;
  }

  // get the index held by the last iptr.
  member_index = cptr[atom_count].asIndex();
}

bool IPGMContext::is_cmd_with_cptr() const {

  if ((*this)[0].getDescriptor() != Atom::OBJECT)
    return false;
  if ((*this)[0].asOpcode() != Opcodes::ICmd)
    return false;
  return (*this)[1].asOpcode() == Opcodes::Mod ||
    (*this)[1].asOpcode() == Opcodes::Set;
}

uint16 IPGMContext::addProduction(Code *object, bool check_for_existence) const { // called by operators (ins and red).

  ((InputLessPGMOverlay *)overlay_)->productions_.push_back(_Mem::Get()->check_existence(object));
  return ((InputLessPGMOverlay *)overlay_)->productions_.size() - 1;
}

// Utilities for executive-dependent operators ////////////////////////////////////////////////////////////////////////////////

bool match(const IPGMContext &input, const IPGMContext &pattern) { // in red, patterns like (ptn object: [guards]) are allowed.

    // patch the pattern with a ptr to the input.
  if (input.is_reference()) {

    uint16 ptr = pattern.addProduction(input.get_object(), false); // the object obviously is not new.
    pattern.patch_code(pattern.getIndex() + 1, Atom::ProductionPointer(ptr));
  } else
    pattern.patch_code(pattern.getIndex() + 1, Atom::IPointer(input.getIndex()));

  // evaluate the set of guards.
  IPGMContext guard_set = pattern.get_child_deref(2);
  uint16 guard_count = guard_set.get_children_count();
  for (uint16 i = 1; i <= guard_count; ++i) {

    if (!guard_set.get_child_deref(i).evaluate_no_dereference()) // WARNING: no check for duplicates.
      return false;
  }

  return true;
}

bool match(const IPGMContext &input, const IPGMContext &pattern, const IPGMContext &productions, vector<uint16> &production_indices) {

  IPGMContext &skeleton = IPGMContext();
  uint16 last_patch_index;
  if (pattern[0].asOpcode() == Opcodes::Ptn) {

    skeleton = pattern.get_child_deref(1);
    if (!skeleton.match(input))
      return false;
    last_patch_index = pattern.get_last_patch_index();
    if (!match(input, pattern))
      return false;

    goto build_productions;
  }

  if (pattern[0].asOpcode() == Opcodes::AntiPtn) {

    skeleton = pattern.get_child_deref(1);
    if (skeleton.match(input))
      return false;
    last_patch_index = pattern.get_last_patch_index();
    if (match(input, pattern))
      return false;

    goto build_productions;
  }

  return false;

build_productions:
  // compute all productions for this input.
  uint16 production_count = productions.get_children_count();
  uint16 production_index;
  for (uint16 i = 1; i <= production_count; ++i) {

    IPGMContext prod = productions.get_child(i);
    prod.evaluate();
    prod.copy_to_value_array(production_index);
    production_indices.push_back(production_index);
  }

  pattern.unpatch_code(last_patch_index);
  return true;
}

void reduce(const IPGMContext &context, const IPGMContext &input_set, const IPGMContext &section, vector<uint16> &input_indices, vector<uint16> &production_indices) {

  IPGMContext pattern = section.get_child_deref(1);
  if (pattern[0].asOpcode() != Opcodes::Ptn && pattern[0].asOpcode() != Opcodes::AntiPtn)
    return;

  IPGMContext productions = section.get_child_deref(2);
  if (productions[0].getDescriptor() != Atom::SET)
    return;

  uint16 production_count = productions.get_children_count();
  if (!production_count)
    return;

  vector<uint16>::iterator i;
  for (i = input_indices.begin(); i != input_indices.end();) { // to be successful, at least one input must match the pattern.

    IPGMContext c = input_set.get_child_deref(*i);
    if (c.is_undefined()) {

      i = input_indices.erase(i);
      continue;
    }

    bool failure = false;
    if (!match(c, pattern, productions, production_indices)) {

      failure = true;
      break;
    }

    if (failure)
      ++i;
    else // pattern matched: remove the input from the todo list.
      i = input_indices.erase(i);
  }
}

bool IPGMContext::Red(const IPGMContext &context) {
  IPGMContext input_set = context.get_child_deref(1);
  if (!input_set.evaluate_no_dereference())
    return false;

  // a section is a set of one pattern and a set of productions.
  IPGMContext positive_section = context.get_child_deref(2);
  if (!positive_section.get_child_deref(1).evaluate_no_dereference()) // evaluate the pattern only.
    return false;

  IPGMContext negative_section = context.get_child_deref(3);
  if (!negative_section.get_child_deref(1).evaluate_no_dereference()) // evaluate the pattern only.
    return false;

  vector<uint16> input_indices; // todo list of inputs to match.
  for (uint16 i = 1; i <= input_set.get_children_count(); ++i)
    input_indices.push_back(i);

  vector<uint16> production_indices; // list of productions built upon successful matches.

  if (input_set[0].getDescriptor() != Atom::SET &&
    input_set[0].getDescriptor() != Atom::S_SET &&
    positive_section[0].getDescriptor() != Atom::SET &&
    negative_section[0].getDescriptor() != Atom::SET)
    goto failure;

  uint16 input_count = input_set.get_children_count();
  if (!input_count)
    goto failure;

  reduce(context, input_set, positive_section, input_indices, production_indices); // input_indices now filled only with the inputs that did not match the positive pattern.
  reduce(context, input_set, negative_section, input_indices, production_indices); // input_indices now filled only with the inputs that did not match the positive nor the negative pattern.
  if (production_indices.size()) {

    // build the set of all productions in the value array.
    context.setCompoundResultHead(Atom::Set(production_indices.size()));
    for (uint16 i = 0; i < production_indices.size(); ++i) // fill the set with iptrs to productions: the latter are copied in the value array.
      context.addCompoundResultPart(Atom::IPointer(production_indices[i]));
    return true;
  }
failure:
  context.setCompoundResultHead(Atom::Set(0));
  return false;
}

bool IPGMContext::Ins(const IPGMContext &context) {

  IPGMContext object = context.get_child_deref(1);
  IPGMContext args = context.get_child_deref(2);
  IPGMContext run = context.get_child_deref(3);
  IPGMContext tsc = context.get_child_deref(4);
  IPGMContext res = context.get_child_deref(5);
  IPGMContext nfr = context.get_child_deref(6);

  Code *pgm = object.get_object();
  uint16 pgm_opcode = pgm->code(0).asOpcode();
  if (pgm_opcode != Opcodes::Pgm &&
    pgm_opcode != Opcodes::AntiPgm) {

    context.setAtomicResult(Atom::Nil());
    return false;
  }

  if (pgm && args[0].getDescriptor() == Atom::SET) {

    uint16 pattern_set_index = pgm->code(PGM_TPL_ARGS).asIndex();
    uint16 arg_count = args[0].getAtomCount();
    if (pgm->code(pattern_set_index).getAtomCount() != arg_count) {

      context.setAtomicResult(Atom::Nil());
      return false;
    }

    // match args with the tpl patterns in _object.
    IPGMContext pattern_set(pgm, pattern_set_index);
    for (uint16 i = 1; i <= arg_count; ++i) {

      IPGMContext arg = args.get_child_deref(i);
      IPGMContext skel = pattern_set.get_child_deref(i).get_child_deref(1);
      if (!skel.match(arg)) {

        context.setAtomicResult(Atom::Nil());
        return false;
      }
    }

    Code *ipgm; // created in the production array.
    if (pgm_opcode == Opcodes::AntiPgm)
      ipgm = context.build_object(Atom::InstantiatedAntiProgram(Opcodes::IPgm, IPGM_ARITY));
    else {

      uint16 input_index = pgm->code(PGM_INPUTS).asIndex();
      uint16 input_count = pgm->code(input_index).getAtomCount();
      if (input_count == 0)
        ipgm = context.build_object(Atom::InstantiatedInputLessProgram(Opcodes::IPgm, IPGM_ARITY));
      else
        ipgm = context.build_object(Atom::InstantiatedProgram(Opcodes::IPgm, IPGM_ARITY));
    }

    ipgm->code(IPGM_PGM) = Atom::RPointer(0); // points to the pgm object.
    ipgm->add_reference(pgm);

    uint16 extent_index = 0;
    ipgm->code(IPGM_ARGS) = Atom::IPointer(IPGM_ARITY + 1); // points to the arg set.
    args.copy(ipgm, IPGM_ARITY + 1, extent_index); // writes the args after psln_thr.

    ipgm->code(IPGM_RUN) = run[0];
    ipgm->code(IPGM_TSC) = Atom::IPointer(extent_index); // points to the tsc.

    ipgm->code(extent_index++) = tsc[0]; // writes the tsc after the args.
    ipgm->code(extent_index++) = tsc[1];
    ipgm->code(extent_index++) = tsc[2];

    ipgm->code(IPGM_RES) = res[0]; // res.
    ipgm->code(IPGM_NFR) = nfr[0]; // nfr.
    ipgm->code(IPGM_ARITY) = Atom::Float(1); // psln_thr.

    context.setAtomicResult(Atom::ProductionPointer(context.addProduction(ipgm, true))); // object may be new: we don't know at this point, therefore check=true.
    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

bool IPGMContext::Fvw(const IPGMContext &context) {

  IPGMContext object = context.get_child_deref(1);
  IPGMContext group = context.get_child_deref(2);

  Code *_object = object.get_object();
  Group *_group = (Group *)group.get_object();
  if (!_object || !_group) {

    context.setAtomicResult(Atom::Nil());
    return false;
  }

  View *v = (View *)_object->get_view(_group, true); // returns (a copy of: deprecated) of the view, if any.
  if (v) { // copy the view in the value array: code on VIEW_CODE_MAX_SIZE followed by 2 atoms holding raw pointers to grp and org.

    context.setCompoundResultHead(v->code(0));
    for (uint16 i = 1; i < VIEW_CODE_MAX_SIZE; ++i)
      context.addCompoundResultPart(v->code(i));
    context.addCompoundResultPart(Atom((uint32)v->references_[0]));
    context.addCompoundResultPart(Atom((uint32)v->references_[1]));
    delete v;
    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}
}
