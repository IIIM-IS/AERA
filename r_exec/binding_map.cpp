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

#include "mem.h"
#include "binding_map.h"
#include "factory.h"

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

Value::Value(BindingMap *map) : _Object(), map_(map) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

BoundValue::BoundValue(BindingMap *map) : Value(map) {
}

////////////////////////////////////////////////////////////////////////////////////////////////////

UnboundValue::UnboundValue(BindingMap *map, uint8 index) : Value(map), index_(index) {

  ++map->unbound_values_;
}

UnboundValue::~UnboundValue() {

  --map_->unbound_values_;
}

Value *UnboundValue::copy(BindingMap *map) const {

  return new UnboundValue(map, index_);
}

void UnboundValue::valuate(Code *destination, uint16 write_index, uint16& /* extent_index */) const {

  destination->code(write_index) = Atom::VLPointer(index_);
}

bool UnboundValue::match(const Code *object, uint16 index) {

  Atom o_atom = object->code(index);
  switch (o_atom.getDescriptor()) {
  case Atom::I_PTR:
    map_->bind_variable(new StructureValue(map_, object, o_atom.asIndex()), index_);
    break;
  case Atom::R_PTR:
    map_->bind_variable(new ObjectValue(map_, object->get_reference(o_atom.asIndex())), index_);
    break;
  case Atom::WILDCARD:
    break;
  default:
    map_->bind_variable(new AtomValue(map_, o_atom), index_);
    break;
  }

  return true;
}

Atom *UnboundValue::get_code() {

  return NULL;
}

Code *UnboundValue::get_object() {

  return NULL;
}

uint16 UnboundValue::get_code_size() {

  return 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

AtomValue::AtomValue(BindingMap *map, Atom atom) : BoundValue(map), atom_(atom) {
}

Value *AtomValue::copy(BindingMap *map) const {

  return new AtomValue(map, atom_);
}

void AtomValue::valuate(Code *destination, uint16 write_index, uint16& /* extent_index */) const {

  destination->code(write_index) = atom_;
}

bool AtomValue::match(const Code *object, uint16 index) {

  return map_->match_atom(object->code(index), atom_);
}

Atom *AtomValue::get_code() {

  return &atom_;
}

Code *AtomValue::get_object() {

  return NULL;
}

uint16 AtomValue::get_code_size() {

  return 1;
}

bool AtomValue::intersect(const Value *v) const {

  return v->_intersect(this);
}

bool AtomValue::_intersect(const AtomValue *v) const {

  return contains(v->atom_);
}

bool AtomValue::contains(const Atom a) const {

  if (atom_ == a)
    return true;

  if (atom_.isFloat() && a.isFloat())
    return Utils::Equal(atom_.asFloat(), a.asFloat());

  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

StructureValue::StructureValue(BindingMap *map, const Atom *source, uint16 structure_index) : BoundValue(map) {

  structure_ = new LocalObject();
  uint16 extent_index = 0;
  copy_structure(structure_, extent_index, source, structure_index);
}

StructureValue::StructureValue(BindingMap *map, Timestamp time) : BoundValue(map) {

  structure_ = new LocalObject();
  structure_->resize_code(3);
  Utils::SetTimestamp(&structure_->code(0), time);
}

StructureValue::StructureValue(BindingMap *map, microseconds duration) : BoundValue(map) {
  structure_ = new LocalObject();
  structure_->resize_code(3);
  Utils::SetDuration(&structure_->code(0), duration);
}

Value *StructureValue::copy(BindingMap *map) const {

  return new StructureValue(map, structure_);
}

void StructureValue::copy_structure(
  Code *destination, uint16 &extent_index, const Atom* source, uint16 source_index) {

  uint16 extent_start = extent_index;
  uint8 atom_count = source[source_index].getAtomCount();
  // Increment extent_index now in case we need to write more after the structure.
  extent_index += (1 + atom_count);
  for (uint16 i = 0; i <= atom_count; ++i) {
    Atom a = source[source_index + i];
    destination->code(extent_start + i) = a;
  }
}

bool StructureValue::match(const Code *object, uint16 index) {

  if (object->code(index).getDescriptor() != Atom::I_PTR)
    return false;
  return map_->match_structure(object, object->code(index).asIndex(), 0, structure_, 0);
}

Atom *StructureValue::get_code() {

  return &structure_->code(0);
}

Code *StructureValue::get_object() {

  return NULL;
}

uint16 StructureValue::get_code_size() {

  return structure_->code_size();
}

bool StructureValue::intersect(const Value *v) const {

  return v->_intersect(this);
}

bool StructureValue::_intersect(const StructureValue *v) const {

  return contains(&v->structure_->code(0));
}

bool StructureValue::contains(const Atom *s) const {

  if (structure_->code(0) != s[0])
    return false;
  if (structure_->code(0).getDescriptor() == Atom::TIMESTAMP)
    return Utils::Synchronous(Utils::GetTimestamp(&structure_->code(0)), Utils::GetTimestamp(s));
  for (uint16 i = 1; i < structure_->code_size(); ++i) {

    Atom a = structure_->code(i);
    Atom _a = s[i];
    if (a == _a)
      continue;

    if (a.isFloat() && _a.isFloat())
      return Utils::Equal(a.asFloat(), _a.asFloat());

    return false;
  }

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

ObjectValue::ObjectValue(BindingMap *map, Code *object) : BoundValue(map), object_(object) {
}

Value *ObjectValue::copy(BindingMap *map) const {

  return new ObjectValue(map, object_);
}

void ObjectValue::valuate(Code *destination, uint16 write_index, uint16& /* extent_index */) const {

  destination->code(write_index) = Atom::RPointer(destination->references_size());
  destination->add_reference(object_);
}

bool ObjectValue::match(const Code *object, uint16 index) {

  return map_->match_object(object->get_reference(object->code(index).asIndex()), object_);
}

Atom *ObjectValue::get_code() {

  return &object_->code(0);
}

Code *ObjectValue::get_object() {

  return object_;
}

uint16 ObjectValue::get_code_size() {

  return object_->code_size();
}

bool ObjectValue::intersect(const Value *v) const {

  return v->_intersect(this);
}

bool ObjectValue::_intersect(const ObjectValue *v) const {

  return contains(v->object_);
}

bool ObjectValue::contains(const Code *o) const {

  return object_ == o;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

_Fact *BindingMap::abstract_f_ihlp(_Fact *f_ihlp) const { // bindings are set already (coming from a mk.rdx caught by auto-focus).

  uint16 opcode;
  Code *ihlp = f_ihlp->get_reference(0);
  if (ihlp->code(0).asOpcode() == Opcodes::ICst)
    opcode = Opcodes::ICst;
  else
    opcode = Opcodes::IMdl;

  _Fact *_f_ihlp = new Fact();
  for (uint16 i = 0; i < f_ihlp->code_size(); ++i)
    _f_ihlp->code(i) = f_ihlp->code(i);

  Code *_ihlp = _Mem::Get()->build_object(Atom::Object(opcode, I_HLP_ARITY));
  _ihlp->code(I_HLP_OBJ) = Atom::RPointer(0);

  uint16 extent_index = I_HLP_ARITY + 1;

  uint32 map_index = 0;

  uint16 tpl_arg_set_index = ihlp->code(I_HLP_TPL_ARGS).asIndex();
  uint16 tpl_arg_count = ihlp->code(tpl_arg_set_index).getAtomCount();

  _ihlp->code(I_HLP_TPL_ARGS) = Atom::IPointer(extent_index);
  _ihlp->code(extent_index) = Atom::Set(tpl_arg_count);
  for (uint16 i = 1; i <= tpl_arg_count; ++i)
    _ihlp->code(extent_index + i) = Atom::VLPointer(map_index++);
  extent_index += tpl_arg_count;

  uint16 arg_set_index = ihlp->code(I_HLP_EXPOSED_ARGS).asIndex();
  uint16 arg_count = ihlp->code(arg_set_index).getAtomCount();
  _ihlp->code(I_HLP_EXPOSED_ARGS) = Atom::IPointer(extent_index);
  _ihlp->code(extent_index) = Atom::Set(arg_count);
  for (uint16 i = 1; i <= arg_count; ++i)
    _ihlp->code(extent_index + i) = Atom::VLPointer(map_index++);

  _ihlp->code(I_HLP_ARITY) = ihlp->code(I_HLP_ARITY);

  _ihlp->add_reference(f_ihlp->get_reference(f_ihlp->code(I_HLP_OBJ).asIndex()));

  _f_ihlp->add_reference(_ihlp);
  return _f_ihlp;
}

_Fact *BindingMap::abstract_fact(_Fact *fact, const _Fact *original, bool force_sync, int timing_vars_first_search_index) { // abstract values as they are encountered.

  if (fwd_after_index_ == -1)
    first_index_ = map_.size();

  uint16 extent_index = FACT_ARITY + 1;
  abstract_member(original, FACT_OBJ, fact, FACT_OBJ, extent_index, timing_vars_first_search_index);
  if (fwd_after_index_ != -1 && force_sync) {

    fact->code(FACT_AFTER) = Atom::VLPointer(fwd_after_index_);
    fact->code(FACT_BEFORE) = Atom::VLPointer(fwd_before_index_);
  } else {

    abstract_member(original, FACT_AFTER, fact, FACT_AFTER, extent_index, timing_vars_first_search_index);
    abstract_member(original, FACT_BEFORE, fact, FACT_BEFORE, extent_index, timing_vars_first_search_index);
  }
  fact->code(FACT_CFD) = Atom::Wildcard();
  fact->code(FACT_ARITY) = Atom::Wildcard();

  if (fwd_after_index_ == -1) {

    // Get the indexes that were put into the fact by abstract_member().
    fwd_after_index_ = fact->code(FACT_AFTER).asIndex();
    fwd_before_index_ = fact->code(FACT_BEFORE).asIndex();
  }

  return fact;
}

Code *BindingMap::abstract_object(Code *object, bool force_sync, int timing_vars_first_search_index) { // abstract values as they are encountered.

  Code *abstracted_object = NULL;

  uint16 opcode = object->code(0).asOpcode();
  if (opcode == Opcodes::Fact)
    return abstract_fact(new Fact(), (_Fact *)object, force_sync, timing_vars_first_search_index);
  else if (opcode == Opcodes::AntiFact)
    return abstract_fact(new AntiFact(), (_Fact *)object, force_sync, timing_vars_first_search_index);
  else if (opcode == Opcodes::Cmd) {

    uint16 extent_index = CMD_ARITY + 1;
    abstracted_object = _Mem::Get()->build_object(object->code(0));
    abstracted_object->code(CMD_FUNCTION) = object->code(CMD_FUNCTION);
    abstract_member(object, CMD_ARGS, abstracted_object, CMD_ARGS, extent_index);
    abstracted_object->code(CMD_ARITY) = Atom::Wildcard();
  } else if (opcode == Opcodes::MkVal) {

    uint16 extent_index = MK_VAL_ARITY + 1;
    abstracted_object = _Mem::Get()->build_object(object->code(0));
    abstract_member(object, MK_VAL_OBJ, abstracted_object, MK_VAL_OBJ, extent_index);
    abstract_member(object, MK_VAL_ATTR, abstracted_object, MK_VAL_ATTR, extent_index);
    abstract_member(object, MK_VAL_VALUE, abstracted_object, MK_VAL_VALUE, extent_index);
    abstracted_object->code(MK_VAL_ARITY) = Atom::Wildcard();
  } else if (opcode == Opcodes::IMdl || opcode == Opcodes::ICst) {

    uint16 extent_index = I_HLP_ARITY + 1;
    abstracted_object = _Mem::Get()->build_object(object->code(0));
    abstract_member(object, I_HLP_OBJ, abstracted_object, I_HLP_OBJ, extent_index);
    // Set first_search_index to allow search to match with the model template args.
    abstract_member(object, I_HLP_TPL_ARGS, abstracted_object, I_HLP_TPL_ARGS, extent_index, 0);
    // Set first_search_index to not search because exposed args are "output values" which can't be assume to be the same as other values.
    abstract_member(object, I_HLP_EXPOSED_ARGS, abstracted_object, I_HLP_EXPOSED_ARGS, extent_index, -1);
    abstracted_object->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Wildcard();
    abstracted_object->code(I_HLP_ARITY) = Atom::Wildcard();
  } else
    return object;
  return abstracted_object;
}

uint16 BindingMap::get_abstracted_ihlp_exposed_args_index(const Code* ihlp) {
  if (!(ihlp->code(0).asOpcode() != Opcodes::IMdl || ihlp->code(0).asOpcode() != Opcodes::ICst))
    // We don't expect this to happen.
    return 0;

  uint16 template_args_index = ihlp->code(I_HLP_TPL_ARGS).asIndex();
  // The abstracted exposed args will start right after the template args.
  return template_args_index + 1 + ihlp->code(template_args_index).getAtomCount();
}

void BindingMap::abstract_member(const Code *object, uint16 index, Code *abstracted_object, uint16 write_index, uint16 &extent_index, int first_search_index) {

  Atom a = object->code(index);
  uint16 ai = a.asIndex();
  switch (a.getDescriptor()) {
  case Atom::R_PTR: {
    Code *reference = object->get_reference(ai);
    if (reference->code(0).asOpcode() == Opcodes::Ont) { // ontologies resist abstraction.

      abstracted_object->code(write_index) = Atom::RPointer(abstracted_object->references_size());
      abstracted_object->add_reference(reference);
    } else if (reference->code(0).asOpcode() == Opcodes::Ent) // entities are always abstracted.
      abstracted_object->code(write_index) = get_object_variable(reference);
    else { // abstract the reference.

      abstracted_object->code(write_index) = Atom::RPointer(abstracted_object->references_size());
      abstracted_object->add_reference(abstract_object(reference, false, first_search_index));
    }
    break;
  }case Atom::I_PTR:
    // If there is a SET or an OBJECT, then we use its structure.
    if (object->code(ai).getDescriptor() == Atom::SET || object->code(ai).getDescriptor() == Atom::OBJECT) {

      abstracted_object->code(write_index) = Atom::IPointer(extent_index);

      uint16 element_count = object->code(ai).getAtomCount();
      abstracted_object->code(extent_index) = object->code(ai);
      uint16 _write_index = extent_index;
      extent_index += element_count + 1;
      for (uint16 i = 1; i <= element_count; ++i)
        abstract_member(object, ai + i, abstracted_object, _write_index + i, extent_index, first_search_index);
    } else
      abstracted_object->code(write_index) = get_structure_variable(object, ai, first_search_index);
    break;
  default:
    abstracted_object->code(write_index) = get_atom_variable(a);
    break;
  }
}

void BindingMap::init(const Code *object, uint16 index) {

  Atom a = object->code(index);
  switch (a.getDescriptor()) {
  case Atom::R_PTR:
    get_object_variable(object->get_reference(a.asIndex()));
    break;
  case Atom::I_PTR:
    if (object->code(a.asIndex()).getDescriptor() == Atom::SET &&
        object->code(a.asIndex()).getAtomCount() == 1)
      // Special case: Treat a singleton set as just its one member (without recursion). To generalize, we may
      // need a Value class which can hold other Value classes, and maybe restrict recursion.
      return init(object, a.asIndex() + 1);

    get_structure_variable(object, a.asIndex());
    break;
  default:
    get_atom_variable(a);
    break;
  }
}

Atom BindingMap::get_atom_variable(Atom a) {

  for (uint32 i = 0; i < map_.size(); ++i) {

    if (map_[i]->contains(a))
      return Atom::VLPointer(i);
  }

  uint32 size = map_.size();
  map_.push_back(new AtomValue(this, a));
  return Atom::VLPointer(size);
}

Atom BindingMap::get_structure_variable(const Code *object, uint16 index, int first_search_index) {

  if (first_search_index >= 0 && map_.size() > 0) {
    if (first_search_index > map_.size())
      first_search_index = map_.size() - 1;

    for (uint32 i = first_search_index; true;) {

      if (map_[i]->contains(&object->code(index)))
        return Atom::VLPointer(i);

      ++i;
      if (i >= map_.size())
        // Wrap around.
        i = 0;
      if (i == first_search_index)
        // We reached the starting point.
        break;
    }
  }

  uint32 size = map_.size();
  map_.push_back(new StructureValue(this, &object->code(0), index));
  return Atom::VLPointer(size);
}

Atom BindingMap::get_object_variable(Code *object) {

  for (uint32 i = 0; i < map_.size(); ++i) {

    if (map_[i]->contains(object))
      return Atom::VLPointer(i);
  }

  uint32 size = map_.size();
  map_.push_back(new ObjectValue(this, object));
  return Atom::VLPointer(size);
}

BindingMap::BindingMap() : _Object(), first_index_(-1), fwd_after_index_(-1), fwd_before_index_(-1), unbound_values_(0) {
}

BindingMap::BindingMap(const BindingMap *source) : _Object() {

  *this = *source;
}

BindingMap::BindingMap(const BindingMap &source) : _Object() {

  *this = source;
}

BindingMap::~BindingMap() {
}

void BindingMap::clear() {

  map_.clear();
  fwd_after_index_ = fwd_before_index_ = -1;
}

BindingMap &BindingMap::operator =(const BindingMap &source) {

  clear();
  for (uint8 i = 0; i < source.map_.size(); ++i)
    map_.push_back(source.map_[i]->copy(this));
  first_index_ = source.first_index_;
  fwd_after_index_ = source.fwd_after_index_;
  fwd_before_index_ = source.fwd_before_index_;
  unbound_values_ = source.unbound_values_;
  return *this;
}

void BindingMap::load(const BindingMap *source) {

  *this = *source;
}

void BindingMap::add_unbound_value(uint8 id) {

  if (id >= map_.size())
    map_.resize(id + 1);
  map_[id] = new UnboundValue(this, id);
}

bool BindingMap::match(const Code *object, uint16 o_base_index, uint16 o_index, const Code *pattern, uint16 p_index, uint16 o_arity) {

  uint16 o_full_index = o_base_index + o_index;
  Atom o_atom = object->code(o_full_index);
  Atom p_atom = pattern->code(p_index);
  switch (o_atom.getDescriptor()) {
  case Atom::T_WILDCARD:
  case Atom::WILDCARD:
  case Atom::VL_PTR:
    break;
  case Atom::I_PTR:
    switch (p_atom.getDescriptor()) {
    case Atom::VL_PTR:
      if (!map_[p_atom.asIndex()]->match(object, o_full_index))
        return false;
      break;
    case Atom::I_PTR:
      if (!match_structure(object, o_atom.asIndex(), 0, pattern, p_atom.asIndex()))
        return false;
      break;
    case Atom::T_WILDCARD:
    case Atom::WILDCARD:
      break;
    default:
      return false;
    }
    break;
  case Atom::R_PTR:
    switch (p_atom.getDescriptor()) {
    case Atom::VL_PTR:
      if (!map_[p_atom.asIndex()]->match(object, o_full_index))
        return false;
      break;
    case Atom::R_PTR:
      if (!match_object(object->get_reference(o_atom.asIndex()), pattern->get_reference(p_atom.asIndex())))
        return false;
      break;
    case Atom::T_WILDCARD:
    case Atom::WILDCARD:
      break;
    default:
      return false;
    }
    break;
  default:
    switch (p_atom.getDescriptor()) {
    case Atom::VL_PTR:
      if (!map_[p_atom.asIndex()]->match(object, o_full_index))
        return false;
      break;
    case Atom::T_WILDCARD:
    case Atom::WILDCARD:
      break;
    default:
      if (!match_atom(o_atom, p_atom))
        return false;
      break;
    }
    break;
  }

  if (o_index == o_arity)
    return true;
  return match(object, o_base_index, o_index + 1, pattern, p_index + 1, o_arity);
}

bool BindingMap::match_atom(Atom o_atom, Atom p_atom) {

  if (p_atom == o_atom)
    return true;

  if (p_atom.isFloat() && o_atom.isFloat())
    return Utils::Equal(o_atom.asFloat(), p_atom.asFloat());

  return false;
}

bool BindingMap::match_structure(const Code *object, uint16 o_base_index, uint16 o_index, const Code *pattern, uint16 p_index) {

  uint16 o_full_index = o_base_index + o_index;
  Atom o_atom = object->code(o_full_index);
  Atom p_atom = pattern->code(p_index);
  if (o_atom != p_atom)
    return false;
  uint16 arity = o_atom.getAtomCount();
  if (arity == 0) // empty sets.
    return true;
  if (o_atom.getDescriptor() == Atom::TIMESTAMP)
    return Utils::Synchronous(Utils::GetTimestamp(&object->code(o_full_index)), Utils::GetTimestamp(&pattern->code(p_index)));
  if (p_atom == Atom::Object(Opcodes::TI, 2)) {
    // This is a (ti : :). Check the args.
    Atom after_atom = pattern->code(p_index + 1);
    Atom before_atom = pattern->code(p_index + 2);
    if (after_atom.getDescriptor() == Atom::VL_PTR && before_atom.getDescriptor() == Atom::VL_PTR &&
        map_[after_atom.asIndex()]->get_code() != NULL && map_[before_atom.asIndex()]->get_code() != NULL) {
      // Both args are variables pointing to a value.
      if (Utils::HasTimestamp<Code>(object, o_full_index + 1) &&
          Utils::HasTimestamp<Code>(object, o_full_index + 2))
        // The object to match has timestamp values in its (ti After Before). Use the same "smart" match_timings
        // method that match_fwd_timings, etc. use to match the time intervals of two facts.
        return match_timings(
          Utils::GetTimestamp<Code>(object, o_full_index + 1),
          Utils::GetTimestamp<Code>(object, o_full_index + 2), after_atom.asIndex(), before_atom.asIndex());
    }
  }
  return match(object, o_base_index, o_index + 1, pattern, p_index + 1, arity);
}

void BindingMap::reset_fwd_timings(_Fact *reference_fact) { // valuate at after_index and after_index+1 from the timings of the reference object.

  if (fwd_after_index_ >= 0 && fwd_after_index_ < map_.size())
    map_[fwd_after_index_] = new StructureValue(this, reference_fact, reference_fact->code(FACT_AFTER).asIndex());
  if (fwd_before_index_ >= 0 && fwd_before_index_ < map_.size())
    map_[fwd_before_index_] = new StructureValue(this, reference_fact, reference_fact->code(FACT_BEFORE).asIndex());
}

bool BindingMap::match_timings(Timestamp after, Timestamp before, uint32 after_index, uint32 before_index) {

  Atom* after_code = map_[after_index]->get_code();
  Atom* before_code = map_[before_index]->get_code();
  if (after_code[0].getDescriptor() != Atom::TIMESTAMP ||
      before_code[0].getDescriptor() != Atom::TIMESTAMP)
    return false;
  Timestamp stored_after = Utils::GetTimestamp(after_code);
  Timestamp stored_before = Utils::GetTimestamp(before_code);

  if (stored_after <= after) {

    if (stored_before >= before) { // sa a b sb

      Utils::SetTimestamp(after_code, after);
      Utils::SetTimestamp(before_code, before);
      return true;
    } else {

      if (stored_before > after) { // sa a sb b

        Utils::SetTimestamp(after_code, after);
        return true;
      }
      return false;
    }
  } else {

    if (stored_before <= before) // a sa sb b
      return true;
    else if (stored_after < before) { // a sa b sb

      Utils::SetTimestamp(before_code, before);
      return true;
    }
    return false;
  }
}

bool BindingMap::match_fwd_timings(const _Fact *f_object) {

  return match_fwd_timings(f_object->get_after(), f_object->get_before());
}

bool BindingMap::match_fwd_strict(const _Fact *f_object, const _Fact *f_pattern) {

  if (match_object(f_object->get_reference(0), f_pattern->get_reference(0))) {

    if (f_object->code(0) != f_pattern->code(0))
      return false;

    return match_fwd_timings(f_object);
  } else
    return false;
}

MatchResult BindingMap::match_fwd_lenient(const _Fact *f_object, const _Fact *f_pattern) {

  if (match_object(f_object->get_reference(0), f_pattern->get_reference(0))) {

    MatchResult r;
    if (f_pattern->code(0) == f_object->code(0))
      r = MATCH_SUCCESS_POSITIVE;
    else
      r = MATCH_SUCCESS_NEGATIVE;

    if (match_fwd_timings(f_object))
      return r;
    return MATCH_FAILURE;
  } else
    return MATCH_FAILURE;
}

Timestamp BindingMap::get_fwd_after() const {

  return Utils::GetTimestamp(map_[fwd_after_index_]->get_code());
}

Timestamp BindingMap::get_fwd_before() const {

  return Utils::GetTimestamp(map_[fwd_before_index_]->get_code());
}

bool BindingMap::match_object(const Code *object, const Code *pattern) {

  if (object->code(0) != pattern->code(0))
    return false;
  uint16 pattern_opcode = pattern->code(0).asOpcode();
  if (pattern_opcode == Opcodes::Ent ||
    pattern_opcode == Opcodes::Ont ||
    pattern_opcode == Opcodes::Mdl ||
    pattern_opcode == Opcodes::Cst)
    return object == pattern;
  return match(object, 0, 1, pattern, 1, object->code(0).getAtomCount());
}

void BindingMap::bind_variable(BoundValue *value, uint8 id) {

  map_[id] = value;
}

void BindingMap::bind_variable(Atom *code, uint8 id, uint16 value_index, Atom *intermediate_results) { // assigment.

  Atom v_atom = code[value_index];
  if (v_atom.isFloat())
    bind_variable(new AtomValue(this, v_atom), id);
  else switch (v_atom.getDescriptor()) {
  case Atom::VALUE_PTR:
    bind_variable(new StructureValue(this, intermediate_results, v_atom.asIndex()), id);
    break;
  }
}

Atom *BindingMap::get_value_code(uint16 id) const {

  return map_[id]->get_code();
}

uint16 BindingMap::get_value_code_size(uint16 id) const {

  return map_[id]->get_code_size();
}

bool BindingMap::scan_variable(uint16 id) const {

  if (id < first_index_)
    return true;
  return (map_[id]->get_code() != NULL);
}

bool BindingMap::intersect(const BindingMap *bm) const {

  for (uint32 i = 0; i < map_.size();) {

    if (i == fwd_after_index_) { // ignore fact timings.

      i += 2;
      continue;
    }

    for (uint32 j = 0; j < bm->map_.size();) {

      if (j == bm->fwd_after_index_) { // ignore fact timings.

        j += 2;
        continue;
      }

      if (map_[i]->intersect(bm->map_[j]))
        return true;

      ++j;
    }

    ++i;
  }

  return false;
}

bool BindingMap::is_fully_specified() const {

  return unbound_values_ == 0;
}

////////////////////////////////////////////////////////////////////////////////////////////////////

HLPBindingMap::HLPBindingMap() : BindingMap(), bwd_after_index_(-1), bwd_before_index_(-1) {
}

HLPBindingMap::HLPBindingMap(const HLPBindingMap *source) : BindingMap() {

  *this = *source;
}

HLPBindingMap::HLPBindingMap(const HLPBindingMap &source) : BindingMap() {

  *this = source;
}

HLPBindingMap::~HLPBindingMap() {
}

void HLPBindingMap::clear() {

  BindingMap::clear();
  bwd_after_index_ = bwd_before_index_ = -1;
}

HLPBindingMap &HLPBindingMap::operator =(const HLPBindingMap &source) {

  clear();
  for (uint8 i = 0; i < source.map_.size(); ++i)
    map_.push_back(source.map_[i]->copy(this));
  first_index_ = source.first_index_;
  fwd_after_index_ = source.fwd_after_index_;
  fwd_before_index_ = source.fwd_before_index_;
  bwd_after_index_ = source.bwd_after_index_;
  bwd_before_index_ = source.bwd_before_index_;
  unbound_values_ = source.unbound_values_;
  return *this;
}

void HLPBindingMap::load(const HLPBindingMap *source) {

  *this = *source;
}

void HLPBindingMap::init_from_pattern(const Code *source, int16 position) { // source is abstracted.

  bool set_fwd_timing_index = (position == 0);
  bool set_bwd_timing_index = (position == 1);
  for (uint16 i = 1; i < source->code_size(); ++i) {

    Atom s = source->code(i);
    switch (s.getDescriptor()) {
    case Atom::VL_PTR: {
      uint8 value_index = source->code(i).asIndex();
      add_unbound_value(value_index);
      if (set_fwd_timing_index && i == FACT_AFTER)
        fwd_after_index_ = value_index;
      else if (set_fwd_timing_index && i == FACT_BEFORE) {

        fwd_before_index_ = value_index;
        set_fwd_timing_index = false;
      } else if (set_bwd_timing_index && i == FACT_AFTER)
        bwd_after_index_ = value_index;
      else if (set_bwd_timing_index && i == FACT_BEFORE) {

        bwd_before_index_ = value_index;
        set_bwd_timing_index = false;
      }
      break;
    }default:
      break;
    }
  }

  for (uint16 i = 0; i < source->references_size(); ++i)
    init_from_pattern(source->get_reference(i), -1);
}

void HLPBindingMap::add_unbound_values(const Code* hlp, uint16 structure_index) {
  uint16 arg_count = hlp->code(structure_index).getAtomCount();
  for (uint16 i = 1; i <= arg_count; ++i) {

    Atom a = hlp->code(structure_index + i);
    if (a.getDescriptor() == Atom::VL_PTR)
      add_unbound_value(a.asIndex());
    else if (a.getDescriptor() == Atom::I_PTR)
      // Recurse into the structure.
      add_unbound_values(hlp, hlp->code(structure_index + i).asIndex());
  }
}

void HLPBindingMap::init_from_hlp(const Code *hlp) { // hlp is cst or mdl.

  add_unbound_values(hlp, hlp->code(HLP_TPL_ARGS).asIndex());

  first_index_ = map_.size();

  uint16 obj_set_index = hlp->code(HLP_OBJS).asIndex();
  uint16 obj_count = hlp->code(obj_set_index).getAtomCount();
  for (uint16 i = 1; i <= obj_count; ++i) {

    _Fact *pattern = (_Fact *)hlp->get_reference(hlp->code(obj_set_index + i).asIndex());
    init_from_pattern(pattern, i - 1);
  }
}

void HLPBindingMap::init_from_ihlp_args(
  const Code* hlp, uint16 hlp_args_index, const Code* ihlp, uint16 ihlp_args_index) {
  if (hlp->code(hlp_args_index) != ihlp->code(ihlp_args_index))
    // The type of structure or the arity doesn't match.
    return;

  uint16 count = ihlp->code(ihlp_args_index).getAtomCount();
  // valuate args.
  for (uint16 i = 0; i < count; ++i) {

    Atom hlp_atom = hlp->code(hlp_args_index + 1 + i);
    Atom ihlp_atom = ihlp->code(ihlp_args_index + 1 + i);

    if (hlp_atom.getDescriptor() == Atom::I_PTR) {
      if (ihlp_atom.getDescriptor() != Atom::I_PTR)
        // Can't get values from the ihlp
        continue;
      // Recurse into the structure.
      init_from_ihlp_args(hlp, hlp_atom.asIndex(), ihlp, ihlp_atom.asIndex());
      continue;
    }
    if (hlp_atom.getDescriptor() != Atom::VL_PTR)
      // No variable to set.
      continue;

    switch (ihlp_atom.getDescriptor()) {
    case Atom::R_PTR:
      map_[hlp_atom.asIndex()] = new ObjectValue(this, ihlp->get_reference(ihlp_atom.asIndex()));
      break;
    case Atom::I_PTR:
      map_[hlp_atom.asIndex()] = new StructureValue(this, ihlp, ihlp_atom.asIndex());
      break;
    default:
      map_[hlp_atom.asIndex()] = new AtomValue(this, ihlp_atom);
      break;
    }
  }
}

void HLPBindingMap::init_from_f_ihlp(const _Fact *f_ihlp) { // source is f->icst or f->imdl; map already initialized with values from hlp.

  Code *ihlp = f_ihlp->get_reference(0);

  Code* hlp = ihlp->get_reference(0);
  init_from_ihlp_args(hlp, hlp->code(HLP_TPL_ARGS).asIndex(), ihlp, ihlp->code(I_HLP_TPL_ARGS).asIndex());

  uint16 val_set_index = ihlp->code(I_HLP_EXPOSED_ARGS).asIndex() + 1;
  uint32 i = 0;
  for (uint32 j = first_index_; j < map_.size(); ++j) { // valuate args.

    if (j == fwd_after_index_ || j == fwd_before_index_)
      // The forward timestamps don't appear in the args set, so skip.
      continue;

    Atom atom = ihlp->code(val_set_index + i);
    switch (atom.getDescriptor()) {
    case Atom::R_PTR:
      map_[j] = new ObjectValue(this, ihlp->get_reference(atom.asIndex()));
      break;
    case Atom::I_PTR:
      map_[j] = new StructureValue(this, ihlp, atom.asIndex());
      break;
    case Atom::WILDCARD:
    case Atom::T_WILDCARD:
    case Atom::VL_PTR:
      break;
    default:
      map_[j] = new AtomValue(this, atom);
      break;
    }

    // Increment to look at the next value in the args set in the ihlp.
    ++i;
  }

  // Valuate timings; fwd_after_index is already known. We know that time stamps are I_PTR to the time stamp structure.
  map_[fwd_after_index_] = new StructureValue(this, f_ihlp, f_ihlp->code(FACT_AFTER).asIndex());
  map_[fwd_before_index_] = new StructureValue(this, f_ihlp, f_ihlp->code(FACT_BEFORE).asIndex());
}

void HLPBindingMap::build_ihlp_structure(
  const Code* hlp, uint16 hlp_structure_index, Code* ihlp, uint16& extent_index) const
{
  uint16 count = hlp->code(hlp_structure_index).getAtomCount();
  ihlp->code(extent_index) = hlp->code(hlp_structure_index);
  uint16 write_index = extent_index + 1;
  extent_index = write_index + count;
  bool is_ti = (hlp->code(hlp_structure_index) == Atom::Object(Opcodes::TI, 2));

  // Copy from the HLP structure, valuating each VL_PTR.
  for (uint16 i = 0; i < count; ++i) {
    Atom a = hlp->code(hlp_structure_index + 1 + i);
    // Leave the args of (ti After: Before:) as variables so that Match can "narrow" them.
    if (a.getDescriptor() == Atom::VL_PTR && !is_ti)
      // Valuate the arg.
      map_[a.asIndex()]->valuate(ihlp, write_index, extent_index);
    else if (a.getDescriptor() == Atom::I_PTR) {
      ihlp->code(write_index) = Atom::IPointer(extent_index);
      // Recurse into the structure.
      build_ihlp_structure(hlp, a.asIndex(), ihlp, extent_index);
    }
    else
      ihlp->code(write_index) = a;

    ++write_index;
  }
}

Fact *HLPBindingMap::build_f_ihlp(Code *hlp, uint16 opcode, bool wr_enabled) const {

  Code *ihlp = _Mem::Get()->build_object(Atom::Object(opcode, I_HLP_ARITY));
  ihlp->code(I_HLP_OBJ) = Atom::RPointer(0);
  ihlp->add_reference(hlp);

  uint16 tpl_arg_index = I_HLP_ARITY + 1;
  ihlp->code(I_HLP_TPL_ARGS) = Atom::IPointer(tpl_arg_index);
  uint16 extent_index = tpl_arg_index;
  build_ihlp_structure(hlp, hlp->code(HLP_TPL_ARGS).asIndex(), ihlp, extent_index);

  ihlp->code(I_HLP_EXPOSED_ARGS) = Atom::IPointer(extent_index);
  uint16 exposed_arg_start = first_index_;
  uint16 exposed_arg_count = map_.size() - exposed_arg_start - 2; // -2: do not expose the first after/before timestamps.
  ihlp->code(extent_index) = Atom::Set(exposed_arg_count);

  uint16 write_index = extent_index + 1;
  extent_index = write_index + exposed_arg_count;
  for (uint16 i = exposed_arg_start; i < map_.size(); ++i) { // valuate args.

    if (i == fwd_after_index_)
      continue;
    if (i == fwd_before_index_)
      continue;
    map_[i]->valuate(ihlp, write_index, extent_index);
    ++write_index;
  }

  ihlp->code(I_HLP_WEAK_REQUIREMENT_ENABLED) = Atom::Boolean(wr_enabled);
  ihlp->code(I_HLP_ARITY) = Atom::Float(1); // psln_thr.

  Fact *f_ihlp = new Fact(ihlp, Timestamp(seconds(0)), Timestamp(seconds(0)), 1, 1);
  extent_index = FACT_ARITY + 1;
  map_[fwd_after_index_]->valuate(f_ihlp, FACT_AFTER, extent_index);
  map_[fwd_before_index_]->valuate(f_ihlp, FACT_BEFORE, extent_index);
  return f_ihlp;
}

Code *HLPBindingMap::bind_pattern(Code *pattern) const {

  if (!need_binding(pattern))
    return pattern;

  Code *bound_pattern = _Mem::Get()->build_object(pattern->code(0));

  for (uint16 i = 0; i < pattern->references_size(); ++i) { // bind references when needed; must be done before binding the code as this may add references.

    Code *reference = pattern->get_reference(i);
    if (need_binding(reference))
      bound_pattern->set_reference(i, bind_pattern(reference));
    else
      bound_pattern->set_reference(i, reference);
  }

  uint16 extent_index = pattern->code_size();
  for (uint16 i = 1; i < pattern->code_size(); ++i) { // transform code containing variables into actual values when they exist: objects -> r_ptr, structures -> i_ptr, atoms -> atoms.

    Atom p_atom = pattern->code(i);
    switch (p_atom.getDescriptor()) {
    case Atom::VL_PTR:
      map_[p_atom.asIndex()]->valuate(bound_pattern, i, extent_index);
      break;
    case Atom::TIMESTAMP:
    case Atom::DURATION:
    case Atom::STRING: { // avoid misinterpreting raw data that could be lead by descriptors.
      bound_pattern->code(i) = p_atom;
      uint16 atom_count = p_atom.getAtomCount();
      for (uint16 j = i + 1; j <= i + atom_count; ++j)
        bound_pattern->code(j) = pattern->code(j);
      i += atom_count;
      break;
    }default:
      bound_pattern->code(i) = p_atom;
      break;
    }
  }

  return bound_pattern;
}

bool HLPBindingMap::need_binding(Code *pattern) const {

  if (pattern->code(0).asOpcode() == Opcodes::Ont ||
    pattern->code(0).asOpcode() == Opcodes::Ent ||
    pattern->code(0).asOpcode() == Opcodes::Mdl ||
    pattern->code(0).asOpcode() == Opcodes::Cst)
    return false;

  for (uint16 i = 0; i < pattern->references_size(); ++i) {

    if (need_binding(pattern->get_reference(i)))
      return true;
  }

  for (uint16 i = 1; i < pattern->code_size(); ++i) {

    Atom p_atom = pattern->code(i);
    switch (p_atom.getDescriptor()) {
    case Atom::VL_PTR:
      return true;
    case Atom::TIMESTAMP:
    case Atom::DURATION:
    case Atom::STRING:
      i += p_atom.getAtomCount();
      break;
    default:
      break;
    }
  }

  return false;
}

void HLPBindingMap::reset_bwd_timings(_Fact *reference_fact) { // valuate at after_index and after_index+1 from the timings of the reference fact.

  if (bwd_after_index_ >= 0 && bwd_after_index_ < map_.size())
    map_[bwd_after_index_] = new StructureValue(this, reference_fact, reference_fact->code(FACT_AFTER).asIndex());
  if (bwd_before_index_ >= 0 && bwd_before_index_ < map_.size())
    map_[bwd_before_index_] = new StructureValue(this, reference_fact, reference_fact->code(FACT_BEFORE).asIndex());
}

bool HLPBindingMap::match_bwd_timings(const _Fact *f_object, const _Fact *f_pattern) {

  return match_timings(f_object->get_after(), f_object->get_before(), bwd_after_index_, bwd_before_index_);
}

bool HLPBindingMap::match_bwd_strict(const _Fact *f_object, const _Fact *f_pattern) {

  if (match_object(f_object->get_reference(0), f_pattern->get_reference(0))) {

    if (f_object->code(0) != f_pattern->code(0))
      return false;

    return match_bwd_timings(f_object, f_pattern);
  } else
    return false;
}

MatchResult HLPBindingMap::match_bwd_lenient(const _Fact *f_object, const _Fact *f_pattern) {

  if (match_object(f_object->get_reference(0), f_pattern->get_reference(0))) {

    MatchResult r;
    if (f_pattern->code(0) == f_object->code(0))
      r = MATCH_SUCCESS_POSITIVE;
    else
      r = MATCH_SUCCESS_NEGATIVE;

    if (match_bwd_timings(f_object, f_pattern))
      return r;
    return MATCH_FAILURE;
  } else
    return MATCH_FAILURE;
}

Timestamp HLPBindingMap::get_bwd_after() const {

  return Utils::GetTimestamp(map_[bwd_after_index_]->get_code());
}

Timestamp HLPBindingMap::get_bwd_before() const {

  return Utils::GetTimestamp(map_[bwd_before_index_]->get_code());
}
}
