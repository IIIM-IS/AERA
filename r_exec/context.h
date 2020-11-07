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

#ifndef ipgm_context_h
#define ipgm_context_h

#include "../r_code/atom.h"
#include "../r_code/utils.h"
#include "object.h"
#include "_context.h"
#include "pgm_overlay.h"


namespace r_exec {

class InputLessPGMOverlay;

// Evaluation context.
class dll_export IPGMContext :
  public _Context {
private:
  r_code::Code *object_; // the object the code belongs to; unchanged when the context is dereferenced to the overlay's value array.
  View *view_; // the object's view, can be NULL when the context is dereferenced to a reference_set or a marker_set.

  bool is_cmd_with_cptr() const;

  void addReference(r_code::Code *destination, uint16 write_index, r_code::Code *referenced_object) const {

    for (uint16 i = 0; i < destination->references_size(); ++i)
      if (referenced_object == destination->get_reference(i)) {

        destination->code(write_index) = Atom::RPointer(i);
        return;
      }

    destination->add_reference(referenced_object);
    destination->code(write_index) = Atom::RPointer(destination->references_size() - 1);
  }

  void addReference(View *destination, uint16 write_index, r_code::Code *referenced_object) const { // view references are set in order (index 0 then 1).

    uint16 r_ptr_index;
    if (destination->references_[0]) // first ref already in place.
      r_ptr_index = 1;
    else
      r_ptr_index = 0;
    destination->references_[r_ptr_index] = referenced_object;
    destination->code(write_index) = Atom::RPointer(r_ptr_index);
  }

  template<class C> void copy_structure(C *destination,
    uint16 write_index,
    uint16 &extent_index,
    bool dereference_cptr,
    int32 pgm_index) const { // assumes the context is a structure; C: Object or View.

    if ((*this)[0].getDescriptor() == Atom::OPERATOR && Operator::Get((*this)[0].asOpcode()).is_syn()) { // (\ (expression ...)).

      IPGMContext c = getChild(1);
      c.copy_member(destination, write_index, extent_index, dereference_cptr, pgm_index);
    } else {

      destination->code(write_index++) = (*this)[0];

      uint16 atom_count = getChildrenCount();
      extent_index = write_index + atom_count;

      switch ((*this)[0].getDescriptor()) {
      case Atom::C_PTR: // copy members as is (no dereference).
        if ((*this)[1].getDescriptor() == Atom::CODE_VL_PTR) {

          Atom a = code_[(*this)[1].asIndex()];
          if (a.getDescriptor() == Atom::I_PTR)
            a = code_[a.asIndex()];
          switch (a.getDescriptor()) {
          case Atom::OUT_OBJ_PTR:
            destination->code(write_index++) = Atom::CodeVLPointer(code_[(*this)[1].asIndex()].asIndex());
            break;
          case Atom::IN_OBJ_PTR: // TMP: assumes destination is a mk.rdx.
            destination->code(write_index++) = Atom::RPointer(1 + a.asInputIndex());
            break;
          default:
            destination->code(write_index++) = (*this)[1];
            break;
          }
        } else
          destination->code(write_index++) = (*this)[1];
        for (uint16 i = 2; i <= atom_count; ++i)
          destination->code(write_index++) = (*this)[i];
        break;
      case Atom::TIMESTAMP: // copy members as is (no dereference).
        for (uint16 i = 1; i <= atom_count; ++i)
          destination->code(write_index++) = (*this)[i];
        break;
      default:
        if (is_cmd_with_cptr()) {

          for (uint16 i = 1; i <= atom_count; ++i) {

            IPGMContext c = getChild(i);
            c.copy_member(destination, write_index++, extent_index, i != 2, pgm_index);
          }
        } else { // if a pgm is being copied, indicate the starting index of the pgm so that we can turn on code patching and know if a cptr is referencing code inside the pgm (in that case it will not be dereferenced).

          int32 _pgm_index;
          if (pgm_index >= 0)
            _pgm_index = pgm_index;
          else if ((*this)[0].getDescriptor() == Atom::OBJECT && ((*this)[0].asOpcode() == Opcodes::Pgm || (*this)[0].asOpcode() == Opcodes::AntiPgm))
            _pgm_index = index_;
          else
            _pgm_index = -1;

          for (uint16 i = 1; i <= atom_count; ++i) {

            IPGMContext c = getChild(i);
            c.copy_member(destination, write_index++, extent_index, !(!dereference_cptr && i == 1), _pgm_index);
          }
        }
        break;
      }
    }
  }

  template<class C> void copy_member(C *destination,
    uint16 write_index,
    uint16 &extent_index,
    bool dereference_cptr,
    int32 pgm_index) const {

    switch ((*this)[0].getDescriptor()) {
    case Atom::I_PTR:
    case Atom::VALUE_PTR:
    case Atom::IPGM_PTR: {

      if (data_ == REFERENCE && index_ == 0) { // points to an object, not to one of its members.

        addReference(destination, write_index, object_);
        break;
      }

      if (code_[(*this)[0].asIndex()].getDescriptor() == Atom::C_PTR) {

        if (!dereference_cptr || (pgm_index > 0 && code_[(*this)[0].asIndex() + 1].asIndex() > pgm_index)) { // the latter case occurs when a cptr references code inside a pgm being copied.

          IPGMContext c = IPGMContext(object_, view_, code_, (*this)[0].asIndex(), (InputLessPGMOverlay *)overlay_, data_);
          destination->code(write_index) = Atom::IPointer(extent_index);
          c.copy_structure(destination, extent_index, extent_index, dereference_cptr, pgm_index);
          break;
        }
      }

      IPGMContext c = **this;
      if (c[0].isStructural()) {

        destination->code(write_index) = Atom::IPointer(extent_index);
        if (pgm_index > 0 && index_ > pgm_index && data_ == STEM)
          patch_code(index_, Atom::OutObjPointer(write_index));
        c.copy_structure(destination, extent_index, extent_index, dereference_cptr, pgm_index);
      } else
        destination->code(write_index) = c[0];
      break;
    }
    case Atom::CODE_VL_PTR:
      switch (code_[(*this)[0].asIndex()].getDescriptor()) {
      case Atom::PROD_PTR:
        addReference(destination, write_index, ((InputLessPGMOverlay *)overlay_)->productions_[code_[(*this)[0].asIndex()].asIndex()]);
        break;
      case Atom::I_PTR:
        if (code_[code_[(*this)[0].asIndex()].asIndex()].getDescriptor() == Atom::IN_OBJ_PTR)
          addReference(destination, write_index, ((PGMOverlay *)overlay_)->getInputObject(code_[code_[(*this)[0].asIndex()].asIndex()].asInputIndex()));
        else
          (**this).copy_member(destination, write_index, extent_index, dereference_cptr, pgm_index);
        break;
      case Atom::OUT_OBJ_PTR:
        destination->code(write_index) = Atom::CodeVLPointer(code_[(*this)[0].asIndex()].asIndex());
        break;
      case Atom::IN_OBJ_PTR:
      case Atom::D_IN_OBJ_PTR: {

        IPGMContext c = **this;
        if (c.index_ == 0)
          addReference(destination, write_index, c.object_);
        else if (c[0].isStructural()) {

          destination->code(write_index++) = Atom::IPointer(extent_index);
          c.copy_structure(destination, extent_index, extent_index, dereference_cptr, pgm_index);
        } else
          c.copy_member(destination, write_index, extent_index, dereference_cptr, pgm_index);
        break;
      }default:
        (**this).copy_member(destination, write_index, extent_index, dereference_cptr, pgm_index);
        break;
      }
      break;
    case Atom::R_PTR:
      addReference(destination, write_index, object_->get_reference((*this)[0].asIndex()));
      break;
    case Atom::PROD_PTR:
      addReference(destination, write_index, ((InputLessPGMOverlay *)overlay_)->productions_[(*this)[0].asIndex()]);
      break;
    case Atom::IN_OBJ_PTR:
      addReference(destination, write_index, ((PGMOverlay *)overlay_)->getInputObject((*this)[0].asIndex()));
      break;
    case Atom::D_IN_OBJ_PTR: {
      IPGMContext c = **this;
      addReference(destination, write_index, c.object_);
      break;
    }case Atom::OPERATOR:
    case Atom::OBJECT:
    case Atom::MARKER:
    case Atom::INSTANTIATED_PROGRAM:
    case Atom::INSTANTIATED_CPP_PROGRAM:
    case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
    case Atom::INSTANTIATED_ANTI_PROGRAM:
    case Atom::COMPOSITE_STATE:
    case Atom::MODEL:
    case Atom::GROUP:
    case Atom::SET:
    case Atom::S_SET:
    case Atom::TIMESTAMP:
    case Atom::STRING:
      destination->code(write_index) = Atom::IPointer(extent_index);
      if (pgm_index > 0 && index_ > pgm_index && data_ == STEM)
        patch_code(index_, Atom::OutObjPointer(write_index));
      copy_structure(destination, extent_index, extent_index, dereference_cptr, pgm_index);
      break;
    case Atom::T_WILDCARD:
      destination->code(write_index) = (*this)[0];
      break;
    default:
      destination->code(write_index) = (*this)[0];
      if (pgm_index > 0 && index_ > pgm_index && data_ == STEM)
        patch_code(index_, Atom::OutObjPointer(write_index));
      break;
    }
  }

  void IPGMContext::copy_structure_to_value_array(bool prefix, uint16 write_index, uint16 &extent_index, bool dereference_cptr);
  void IPGMContext::copy_member_to_value_array(uint16 child_index, bool prefix, uint16 write_index, uint16 &extent_index, bool dereference_cptr);
public:
  static IPGMContext GetContextFromInput(View *input, InputLessPGMOverlay *overlay) { return IPGMContext(input->object_, input, &input->object_->code(0), 0, overlay, REFERENCE); }

  IPGMContext() : _Context(NULL, 0, NULL, UNDEFINED), object_(NULL), view_(NULL) {} // undefined context (happens when accessing the view of an object when it has not been provided).
  IPGMContext(r_code::Code *object, View *view, Atom *code, uint16 index, InputLessPGMOverlay *const overlay, Data data = STEM) : _Context(code, index, overlay, data), object_(object), view_(view) {}
  IPGMContext(r_code::Code *object, uint16 index) : _Context(&object->code(0), index, NULL, REFERENCE), object_(object), view_(NULL) {}
  IPGMContext(r_code::Code *object, Data data) : _Context(&object->code(0), index_, NULL, data), object_(object), view_(NULL) {}

  // _Context implementation.
  _Context *assign(const _Context *c) {

    IPGMContext *_c = new IPGMContext(*(IPGMContext *)c);
    return _c;
  }

  bool equal(const _Context *c) const { return *this == *(IPGMContext *)c; }

  Atom &get_atom(uint16 i) const { return this->operator [](i); }

  uint16 get_object_code_size() const { return object_->code_size(); }

  uint16 getChildrenCount() const {

    uint16 c;
    switch (data_) {
    case MKS:
      object_->acq_markers();
      c = object_->markers_.size();
      object_->rel_markers();
      return c;
    case VWS:
      object_->acq_views();
      c = object_->views_.size();
      object_->rel_views();
      return c;
    default:
      return code_[index_].getAtomCount();
    }
  }
  _Context *_getChild(uint16 index) const {

    IPGMContext *_c = new IPGMContext(getChild(index));
    return _c;
  }

  _Context *dereference() const {

    IPGMContext *_c = new IPGMContext(**this);
    return _c;
  }

  // IPGM specifics.
  bool evaluate(uint16 &result_index) const; // index is set to the index of the result, undefined in case of failure.
  bool evaluate_no_dereference(uint16 &result_index) const;

  IPGMContext &operator =(const IPGMContext &c) {

    object_ = c.object_;
    view_ = c.view_;
    code_ = c.code_;
    index_ = c.index_;
    data_ = c.data_;
    return *this;
  }

  bool operator ==(const IPGMContext &c) const;
  bool operator !=(const IPGMContext &c) const;

  IPGMContext getChild(uint16 index) const {

    switch (data_) {
    case STEM:
      return IPGMContext(object_, view_, code_, index_ + index, (InputLessPGMOverlay *)overlay_, STEM);
    case REFERENCE:
      return IPGMContext(object_, view_, code_, index_ + index, (InputLessPGMOverlay *)overlay_, REFERENCE);
    case VIEW:
      return IPGMContext(object_, view_, code_, index_ + index, NULL, VIEW);
    case MKS: {

      uint16 i = 0;
      r_code::list<r_code::Code *>::const_iterator m;
      object_->acq_markers();
      for (m = object_->markers_.begin(); i < index - 1; ++i, ++m) {

        if (m == object_->markers_.end()) { // happens when the list has changed after the call to getChildrenCount.

          object_->rel_markers();
          return IPGMContext();
        }
      }
      object_->rel_markers();
      return IPGMContext(*m, 0);
    }case VWS: {

      uint16 i = 0;
      UNORDERED_SET<r_code::View *, r_code::View::Hash, r_code::View::Equal>::const_iterator v;
      object_->acq_views();
      for (v = object_->views_.begin(); i < index - 1; ++i, ++v) {

        if (v == object_->views_.end()) { // happens when the list has changed after the call to getChildrenCount.

          object_->rel_views();
          return IPGMContext();
        }
      }
      object_->rel_views();
      return IPGMContext(object_, (r_exec::View*)*v, &(*v)->code(0), index_ + index, NULL, VIEW);
    }case VALUE_ARRAY:
      return IPGMContext(object_, view_, code_, index_ + index, (InputLessPGMOverlay *)overlay_, VALUE_ARRAY);
    default: // undefined context.
      return IPGMContext();
    }
  }
  Atom &operator [](uint16 i) const { return code_[index_ + i]; }
  r_code::Code *getObject() const { return object_; }
  uint16 getIndex() const { return index_; }

  IPGMContext operator *() const;
  void dereference_once();

  bool is_reference() const { return data_ == REFERENCE; }
  bool is_undefined() const { return data_ == UNDEFINED; }

  void patch_input_code(uint16 pgm_code_index, uint16 input_index) const { ((InputLessPGMOverlay *)overlay_)->patch_input_code(pgm_code_index, input_index, 0); }

  uint16 addProduction(r_code::Code *object, bool check_for_existence) const; // if check_for_existence==false, the object is assumed not to be new.

  template<class C> void copy(C *destination, uint16 write_index) const {

    uint16 extent_index = 0;
    copy_structure(destination, write_index, extent_index, true, -1);
  }

  template<class C> void copy(C *destination, uint16 write_index, uint16 &extent_index) const {

    copy_structure(destination, write_index, extent_index, true, -1);
  }

  void copy_to_value_array(uint16 &position);

  typedef enum {
    TYPE_OBJECT = 0,
    TYPE_VIEW = 1,
    TYPE_GROUP = 2,
    TYPE_UNDEFINED = 3
  }ObjectType;

  // To retrieve objects, groups and views in mod/set expressions; views are copied.
  void getMember(void *&object, uint32 &view_oid, ObjectType &object_type, int16 &member_index) const;

  // 'this' is a context on a pattern skeleton.
  bool match(const IPGMContext &input) const;

  // called by operators.
  r_code::Code *build_object(Atom head) const { return overlay_->build_object(head); }

  // Implementation of executive-dependent operators.
  static bool Ins(const IPGMContext &context, uint16 &index);
  static bool Fvw(const IPGMContext &context, uint16 &index);
  static bool Red(const IPGMContext &context, uint16 &index);
};
}


#endif
