//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_comp
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

#include "segments.h"

#include <iostream>

using namespace std::chrono;
using namespace r_code;

namespace r_comp {

Reference::Reference() {
}

Reference::Reference(const uint16 i, const Class &c, const Class &cc) : index_(i), class_(c), cast_class_(cc) {
}

////////////////////////////////////////////////////////////////

Metadata::Metadata() {
}

Class *Metadata::get_class(std::string &class_name) {

  unordered_map<std::string, Class>::iterator it = classes_.find(class_name);
  if (it != classes_.end())
    return &it->second;
  return NULL;
}

Class *Metadata::get_class(uint16 opcode) {

  return &classes_by_opcodes_[opcode];
}

void Metadata::write(word32 *data) {

  data[0] = classes_by_opcodes_.size();
  uint32 i;
  uint32 offset = 1;
  for (i = 0; i < classes_by_opcodes_.size(); ++i) {

    classes_by_opcodes_[i].write(data + offset);
    offset += classes_by_opcodes_[i].get_size();
  }

  data[offset++] = classes_.size();
  unordered_map<std::string, Class>::iterator it = classes_.begin();
  for (; it != classes_.end(); ++it) {

    r_code::Write(data + offset, it->first);
    offset += r_code::GetSize(it->first);
    data[offset] = it->second.atom_.asOpcode();
    offset++;
  }

  data[offset++] = sys_classes_.size();
  it = sys_classes_.begin();
  for (; it != sys_classes_.end(); ++it) {

    r_code::Write(data + offset, it->first);
    offset += r_code::GetSize(it->first);
    data[offset] = it->second.atom_.asOpcode();
    offset++;
  }

  data[offset++] = class_names_.size();
  for (i = 0; i < class_names_.size(); ++i) {

    r_code::Write(data + offset, class_names_[i]);
    offset += r_code::GetSize(class_names_[i]);
  }

  data[offset++] = operator_names_.size();
  for (i = 0; i < operator_names_.size(); ++i) {

    r_code::Write(data + offset, operator_names_[i]);
    offset += r_code::GetSize(operator_names_[i]);
  }

  data[offset++] = function_names_.size();
  for (i = 0; i < function_names_.size(); ++i) {

    r_code::Write(data + offset, function_names_[i]);
    offset += r_code::GetSize(function_names_[i]);
  }
}

void Metadata::read(word32 *data, uint32 size) {

  uint16 class_count = data[0];
  uint16 i;
  uint16 offset = 1;
  for (i = 0; i < class_count; ++i) {

    Class c;
    c.read(data + offset);
    classes_by_opcodes_.push_back(c);
    offset += c.get_size();
  }

  uint16 classes_count = data[offset++];
  for (i = 0; i < classes_count; ++i) {

    std::string s;
    r_code::Read(data + offset, s);
    offset += r_code::GetSize(s);
    classes_[s] = classes_by_opcodes_[data[offset++]];
  }

  uint16 sys_classes_count = data[offset++];
  for (i = 0; i < sys_classes_count; ++i) {

    std::string s;
    r_code::Read(data + offset, s);
    offset += r_code::GetSize(s);
    sys_classes_[s] = classes_by_opcodes_[data[offset++]];
  }

  uint16 class_names_count = data[offset++];
  for (i = 0; i < class_names_count; ++i) {

    std::string s;
    r_code::Read(data + offset, s);
    class_names_.push_back(s);
    offset += r_code::GetSize(s);
  }

  uint16 operator_names_count = data[offset++];
  for (i = 0; i < operator_names_count; ++i) {

    std::string s;
    r_code::Read(data + offset, s);
    operator_names_.push_back(s);
    offset += r_code::GetSize(s);
  }

  uint16 function_names_count = data[offset++];
  for (i = 0; i < function_names_count; ++i) {

    std::string s;
    r_code::Read(data + offset, s);
    function_names_.push_back(s);
    offset += r_code::GetSize(s);
  }
}

uint32 Metadata::get_size() {

  return get_class_array_size() +
    get_classes_size() +
    get_sys_classes_size() +
    get_class_names_size() +
    get_operator_names_size() +
    get_function_names_size();
}

// RAM layout:
// - class array
// - number of elements
// - list of classes:
// - atom
// - string (str_opcode)
// - return type
// - usage
// - things to read:
// - number of elements
// - list of structure members:
// - ID of a Compiler::*_Read function
// - return type
// - string (_class)
// - iteration
// - classes:
// - number of elements
// - list of pairs:
// - string
// - index of a class in the class array
// - sys_classes:
// - number of elements
// - list of pairs:
// - string
// - index of a class in the class array
// - class names:
// - number of elements
// - list of strings
// - operator names:
// - number of elements
// - list of strings
// - function names:
// - number of elements
// - list of strings
//
// String layout:
// - size in word32
// - list of words: contain the charaacters; the last one is \0; some of the least significant bytes of the last word my be empty

uint32 Metadata::get_class_array_size() {

  uint32 size = 1; // size of the array
  for (uint32 i = 0; i < classes_by_opcodes_.size(); ++i)
    size += classes_by_opcodes_[i].get_size();
  return size;
}

uint32 Metadata::get_classes_size() {

  uint32 size = 1; // size of the hash table
  unordered_map<std::string, Class>::iterator it = classes_.begin();
  for (; it != classes_.end(); ++it)
    size += r_code::GetSize(it->first) + 1; // +1: index to the class in the class array
  return size;
}

uint32 Metadata::get_sys_classes_size() {

  uint32 size = 1; // size of the hash table
  unordered_map<std::string, Class>::iterator it = sys_classes_.begin();
  for (; it != sys_classes_.end(); ++it)
    size += r_code::GetSize(it->first) + 1; // +1: index to the class in the class array
  return size;
}

uint32 Metadata::get_class_names_size() {

  uint32 size = 1; // size of the vector
  for (uint32 i = 0; i < class_names_.size(); ++i)
    size += r_code::GetSize(class_names_[i]);
  return size;
}

uint32 Metadata::get_operator_names_size() {

  uint32 size = 1; // size of the vector
  for (uint32 i = 0; i < operator_names_.size(); ++i)
    size += r_code::GetSize(operator_names_[i]);
  return size;
}

uint32 Metadata::get_function_names_size() {

  uint32 size = 1; // size of the vector
  for (uint32 i = 0; i < function_names_.size(); ++i)
    size += r_code::GetSize(function_names_[i]);
  return size;
}

////////////////////////////////////////////////////////////////

void ObjectMap::shift(uint16 offset) {

  for (uint32 i = 0; i < objects_.size(); ++i)
    objects_[i] += offset;
}

void ObjectMap::write(word32 *data) {

  for (uint32 i = 0; i < objects_.size(); ++i)
    data[i] = objects_[i];
}

void ObjectMap::read(word32 *data, uint32 size) {

  for (uint16 i = 0; i < size; ++i)
    objects_.push_back(data[i]);
}

uint32 ObjectMap::get_size() const {

  return objects_.size();
}

////////////////////////////////////////////////////////////////

CodeSegment::~CodeSegment() {

  for (uint32 i = 0; i < objects_.size(); ++i)
    delete objects_[i];
}

void CodeSegment::write(word32 *data) {

  uint32 offset = 0;
  for (uint32 i = 0; i < objects_.size(); ++i) {

    objects_[i]->write(data + offset);
    offset += objects_[i]->get_size();
  }
}

void CodeSegment::read(word32 *data, uint16 object_count) {

  uint16 offset = 0;
  for (uint16 i = 0; i < object_count; ++i) {

    SysObject *o = new SysObject();
    o->read(data + offset);
    objects_.push_back(o);
    offset += o->get_size();
  }
}

uint32 CodeSegment::get_size() {

  uint32 size = 0;
  for (uint32 i = 0; i < objects_.size(); ++i)
    size += objects_[i]->get_size();
  return size;
}

////////////////////////////////////////////////////////////////

// Format:
// number of entries
// list of entries (one per user-defined symbol)
// oid
// symbol length
// symbol characters

ObjectNames::~ObjectNames() {
}

void ObjectNames::write(word32 *data) {

  data[0] = symbols_.size();

  uint32 index = 1;

  unordered_map<uint32, std::string>::const_iterator n;
  for (n = symbols_.begin(); n != symbols_.end(); ++n) {

    data[index] = n->first;
    uint32 symbol_length = n->second.length() + 1; // add a trailing null character (for reading).
    uint32 _symbol_length = symbol_length / 4;
    uint32 __symbol_length = symbol_length % 4;
    if (__symbol_length)
      ++_symbol_length;
    data[index + 1] = _symbol_length;
    memcpy(data + index + 2, n->second.c_str(), symbol_length);
    index += _symbol_length + 2;
  }
}

void ObjectNames::read(word32 *data) {

  uint32 symbol_count = data[0];
  uint32 index = 1;
  for (uint32 i = 0; i < symbol_count; ++i) {

    uint32 oid = data[index];
    uint32 symbol_length = data[index + 1]; // number of words needed to store all the characters.
    std::string symbol((char *)(data + index + 2));
    symbols_[oid] = symbol;

    index += symbol_length + 2;
  }
}

uint32 ObjectNames::get_size() {

  uint32 size = 1; // size of symbols_.

  unordered_map<uint32, std::string>::const_iterator n;
  for (n = symbols_.begin(); n != symbols_.end(); ++n) {

    size += 2; // oid and symbol's length.
    uint32 symbol_length = n->second.length() + 1;
    uint32 _symbol_length = symbol_length / 4;
    uint32 __symbol_length = symbol_length % 4;
    if (__symbol_length)
      ++_symbol_length;
    size += _symbol_length; // characters packed in 32 bits words.
  }

  return size;
}

uint32 ObjectNames::findSymbol(const std::string& name) {
  for (auto entry = symbols_.begin(); entry != symbols_.end(); ++entry) {
    if (entry->second == name)
      return entry->first;
  }

  return UNDEFINED_OID;
}

////////////////////////////////////////////////////////////////

Image::Image() : map_offset_(0), timestamp_(seconds(0)) {
}

Image::~Image() {
}

void Image::add_sys_object(SysObject *object, std::string name) {

  add_sys_object(object);
  if (!name.empty())
    object_names_.symbols_[object->oid_] = name;
}

void Image::add_sys_object(SysObject *object) {

  code_segment_.objects_.push_back(object);
  object_map_.objects_.push_back(map_offset_);
  map_offset_ += object->get_size();
}

void Image::add_objects(r_code::list<P<r_code::Code> > &objects, bool include_invalidated) {

  r_code::list<P<r_code::Code> >::const_iterator o;
  for (o = objects.begin(); o != objects.end(); ++o) {

    if (include_invalidated || !(*o)->is_invalidated())
      add_object(*o, include_invalidated);
  }

  build_references();
}

void Image::add_objects(r_code::list<P<r_code::Code> > &objects, std::vector<SysObject *> &imported_objects) {

  r_code::list<P<r_code::Code> >::const_iterator o;
  for (o = objects.begin(); o != objects.end(); ++o)
    add_object(*o, imported_objects);

  build_references();
}

inline uint32 Image::get_reference_count(const Code *object) const {

  switch (object->code(0).getDescriptor()) { // ignore the last reference as it is the unpacked version of the object.
  case Atom::MODEL:
    return object->references_size() - MDL_HIDDEN_REFS;
  case Atom::COMPOSITE_STATE:
    return object->references_size() - CST_HIDDEN_REFS;
  default:
    return object->references_size();
  }
}

void Image::add_object(Code *object, bool include_invalidated) {

  unordered_map<Code *, uint16>::iterator it = ptrs_to_indices_.find(object);
  if (it != ptrs_to_indices_.end()) // object already there.
    return;

  // Add the references first so that they are defined when referenced.
  uint16 reference_count = get_reference_count(object);
  for (uint16 i = 0; i < reference_count; ++i) { // follow reference pointers and recurse.

    Code *reference = object->get_reference(i);
    if (include_invalidated || reference->get_oid() == UNDEFINED_OID ||
      reference->is_invalidated()) // the referenced object is not in the image and will not be added otherwise.
      add_object(reference, include_invalidated);
  }

  uint16 object_index;
  ptrs_to_indices_[object] = object_index = code_segment_.objects_.as_std()->size();
  SysObject *sys_object = new SysObject(object);
  add_sys_object(sys_object);

  // Temporarily store the memory address of object in sys_object->references_. This will be
  // recovered in build_references.
#if defined ARCH_32
  uint32 _object = (uint32)object;
  sys_object->references_[0] = (_object & 0x0000FFFF);
  sys_object->references_[1] = (_object >> 16);
#elif defined ARCH_64
  uint64 _object = (uint64)object;
  sys_object->references_[0] = _object & 0x0000FFFF;
  sys_object->references_[1] = (_object >> 16) & 0x0000FFFF;
  sys_object->references_[2] = (_object >> 32) & 0x0000FFFF;
  sys_object->references_[3] = (_object >> 48) & 0x0000FFFF;
#endif
}

SysObject *Image::add_object(Code *object, std::vector<SysObject *> &imported_objects) {

  unordered_map<Code *, uint16>::iterator it = ptrs_to_indices_.find(object);
  if (it != ptrs_to_indices_.end()) // object already there.
    return code_segment_.objects_[it->second];

  uint16 object_index;
  ptrs_to_indices_[object] = object_index = code_segment_.objects_.as_std()->size();
  SysObject *sys_object = new SysObject(object);
  add_sys_object(sys_object);

  uint16 reference_count = get_reference_count(object);
  for (uint16 i = 0; i < reference_count; ++i) { // follow the object's reference pointers and recurse.

    Code *reference = object->get_reference(i);
    if (reference->get_oid() == UNDEFINED_OID) // the referenced object is not in the image and will not be added otherwise.
      add_object(reference, imported_objects);
    else { // add the referenced object if not present in the list.

      unordered_map<Code *, uint16>::iterator it = ptrs_to_indices_.find(reference);
      if (it == ptrs_to_indices_.end()) {

        SysObject *sys_ref = add_object(reference, imported_objects);
        imported_objects.push_back(sys_ref);
      }
    }
  }

  object->acq_views();
  unordered_set<_View *, _View::Hash, _View::Equal>::const_iterator v;
  for (v = object->views_.begin(); v != object->views_.end(); ++v) { // follow the view's reference pointers and recurse.

    for (uint8 j = 0; j < 2; ++j) { // 2 refs maximum per view; may be NULL.

      Code *reference = (*v)->references_[j];
      if (reference) {

        unordered_map<r_code::Code *, uint16>::const_iterator index = ptrs_to_indices_.find(reference);
        if (index == ptrs_to_indices_.end()) {

          SysObject *sys_ref = add_object(reference, imported_objects);
          imported_objects.push_back(sys_ref);
        }
      }
    }
  }
  object->rel_views();

  // Temporarily store the memory address of object in sys_object->references_. This will be
  // recovered in build_references.
#if defined ARCH_32
  uint32 _object = (uint32)object;
  sys_object->references_[0] = (_object & 0x0000FFFF);
  sys_object->references_[1] = (_object >> 16);
#elif defined ARCH_64
  uint64 _object = (uint64)object;
  sys_object->references_[0] = _object & 0x0000FFFF;
  sys_object->references_[1] = (_object >> 16) & 0x0000FFFF;
  sys_object->references_[2] = (_object >> 32) & 0x0000FFFF;
  sys_object->references_[3] = (_object >> 48) & 0x0000FFFF;
#endif

  return sys_object;
}

void Image::build_references() {

  Code *object;
  SysObject *sys_object;
  for (uint32 i = 0; i < code_segment_.objects_.as_std()->size(); ++i) {

    sys_object = code_segment_.objects_[i];
    // The memory address of the original object was stored in sys_object->references_, so recover it.
#if defined ARCH_32
    uint32 _object = sys_object->references_[0];
    _object |= (sys_object->references_[1] << 16);
    object = (Code *)_object;
#elif defined ARCH_64
    uint64 _object = sys_object->references_[0];
    _object |= ((uint64)(sys_object->references_[1]) << 16);
    _object |= ((uint64)(sys_object->references_[2]) << 32);
    _object |= ((uint64)(sys_object->references_[3]) << 48);
    object = (Code *)_object;
#endif
    sys_object->references_.as_std()->clear();
    build_references(sys_object, object);
  }
}

void Image::build_references(SysObject *sys_object, Code *object) {

  // Translate pointers into indices: valuate the sys_object's references to object, incl. sys_object's view references.
  uint16 i;
  uint16 referenced_object_index;
  uint16 reference_count = get_reference_count(object);
  for (i = 0; i < reference_count; ++i) {

    unordered_map<r_code::Code *, uint16>::const_iterator index = ptrs_to_indices_.find(object->get_reference(i));
    if (index != ptrs_to_indices_.end()) {

      referenced_object_index = index->second;
      sys_object->references_.push_back(referenced_object_index);
    }
  }

  object->acq_views();
  unordered_set<_View *, _View::Hash, _View::Equal>::const_iterator v;
  for (i = 0, v = object->views_.begin(); v != object->views_.end(); ++i, ++v) {

    for (uint8 j = 0; j < 2; ++j) { // 2 refs maximum per view; may be NULL.

      Code *reference = (*v)->references_[j];
      if (reference) {

        unordered_map<r_code::Code *, uint16>::const_iterator index = ptrs_to_indices_.find(reference);
        if (index != ptrs_to_indices_.end()) {

          referenced_object_index = index->second;
          sys_object->views_[i]->references_[j] = referenced_object_index;
        }
      }
    }
  }
  object->rel_views();
}

void Image::get_objects(Mem *mem, r_code::vector<Code *> &ram_objects) {

  for (uint16 i = 0; i < code_segment_.objects_.size(); ++i)
    ram_objects[i] = mem->build_object(code_segment_.objects_[i]);
  unpack_objects(ram_objects);
}

void Image::unpack_objects(r_code::vector<Code *> &ram_objects) {

  // For each object, translate its reference indices into pointers; build its views; for each view translate its reference indices into pointers.
  for (uint16 i = 0; i < code_segment_.objects_.size(); ++i) {

    SysObject *sys_object = code_segment_.objects_[i];
    Code *ram_object = ram_objects[i];

    for (uint16 j = 0; j < sys_object->views_.as_std()->size(); ++j) {

      SysView *sys_v = sys_object->views_[j];
      _View *v = ram_object->build_view(sys_v);
      for (uint16 k = 0; k < sys_v->references_.as_std()->size(); ++k)
        v->references_[k] = ram_objects[sys_v->references_[k]];

      ram_object->views_.insert(v);
    }

    for (uint16 j = 0; j < sys_object->references_.as_std()->size(); ++j)
      ram_object->set_reference(j, ram_objects[sys_object->references_[j]]);
  }
}
}
