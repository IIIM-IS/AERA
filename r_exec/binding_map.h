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

#ifndef binding_map_h
#define binding_map_h

#include "object.h"
#include "dll.h"


namespace r_exec {

class BindingMap;
class AtomValue;
class StructureValue;
class ObjectValue;

class r_exec_dll Value :
  public _Object {
protected:
  BindingMap *map_;
  Value(BindingMap *map);
public:
  virtual Value *copy(BindingMap *map) const = 0;
  virtual void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const = 0;
  virtual bool match(const r_code::Code *object, uint16 index) = 0;
  virtual Atom *get_code() = 0;
  virtual r_code::Code *get_object() = 0;
  virtual uint16 get_code_size() = 0;

  virtual bool intersect(const Value *v) const { return false; }
  virtual bool _intersect(const AtomValue *v) const { return false; }
  virtual bool _intersect(const StructureValue *v) const { return false; }
  virtual bool _intersect(const ObjectValue *v) const { return false; }

  virtual bool contains(const Atom a) const { return false; }
  virtual bool contains(const Atom *s) const { return false; }
  virtual bool contains(const r_code::Code *o) const { return false; }

  /**
   * Return the trace of the value as a string by creating a temporary Code
   * object and calling valuate(). For debugging purposes only(can be inefficient).
   */
  std::string traceString() const {
    P<LObject> code = new LObject();
    uint16 extent_index = 1;
    valuate(code, 0, extent_index);
    return code->traceString();
  }
};

class r_exec_dll BoundValue :
  public Value {
protected:
  BoundValue(BindingMap *map);
public:
};

class r_exec_dll UnboundValue :
  public Value {
private:
  uint8 index_;
public:
  UnboundValue(BindingMap *map, uint8 index);
  ~UnboundValue();

  Value *copy(BindingMap *map) const;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const;
  bool match(const r_code::Code *object, uint16 index);
  Atom *get_code();
  r_code::Code *get_object();
  uint16 get_code_size();
};

class r_exec_dll AtomValue :
  public BoundValue {
private:
  Atom atom_;
public:
  AtomValue(BindingMap *map, Atom atom);

  Value *copy(BindingMap *map) const;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const;
  bool match(const r_code::Code *object, uint16 index);
  Atom *get_code();
  r_code::Code *get_object();
  uint16 get_code_size();

  bool intersect(const Value *v) const;
  bool _intersect(const AtomValue *v) const;

  bool contains(const Atom a) const;
};

class r_exec_dll StructureValue :
  public BoundValue {
private:
  P<r_code::Code> structure_;
  StructureValue(BindingMap *map, const r_code::Code *structure);
public:
  StructureValue(BindingMap *map, const r_code::Code *source, uint16 structure_index);
  StructureValue(BindingMap *map, Atom *source, uint16 structure_index);
  StructureValue(BindingMap *map, Timestamp time);

  Value *copy(BindingMap *map) const;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const;
  bool match(const r_code::Code *object, uint16 index);
  Atom *get_code();
  r_code::Code *get_object();
  uint16 get_code_size();

  bool intersect(const Value *v) const;
  bool _intersect(const StructureValue *v) const;

  bool contains(const Atom *s) const;
};

class r_exec_dll ObjectValue :
  public BoundValue {
private:
  const P<r_code::Code> object_;
public:
  ObjectValue(BindingMap *map, r_code::Code *object);

  Value *copy(BindingMap *map) const;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const;
  bool match(const r_code::Code *object, uint16 index);
  Atom *get_code();
  r_code::Code *get_object();
  uint16 get_code_size();

  bool intersect(const Value *v) const;
  bool _intersect(const ObjectValue *v) const;

  bool contains(const r_code::Code *o) const;
};

typedef enum {
  MATCH_SUCCESS_POSITIVE = 0,
  MATCH_SUCCESS_NEGATIVE = 1,
  MATCH_FAILURE = 2
}MatchResult;

class _Fact;
class Fact;

class r_exec_dll BindingMap :
  public _Object {
  friend class UnboundValue;
protected:
  std::vector<P<Value> > map_; // indexed by vl-ptrs.

  uint32 unbound_values_;

  void add_unbound_value(uint8 id);

  uint16 first_index_; // index of the first value found in the first fact.
  int16 fwd_after_index_; // tpl args (if any) are located before fwd_after_index.
  int16 fwd_before_index_;

  bool match_timings(Timestamp stored_after, Timestamp stored_before, Timestamp after, Timestamp before, uint32 destination_after_index, uint32 destination_before_index);
  bool match_fwd_timings(const _Fact *f_object, const _Fact *f_pattern);
  bool match(const r_code::Code *object, uint16 o_base_index, uint16 o_index, const r_code::Code *pattern, uint16 p_index, uint16 o_arity);

  void abstract_member(r_code::Code *object, uint16 index, r_code::Code *abstracted_object, uint16 write_index, uint16 &extent_index, bool allow_shared_variable = true);
  Atom get_atom_variable(Atom a);

  /**
   * Get a VLPointer for the structre at object->code(index), adding a new binding if needed.
   * \param object The code with the structure.
   * \param index The index in the object code of the structure.
   * \param allow_shared_variable (optional) If true then check if the bindings already contains the structure and
   * return a VLPointer to the binding index if found. If false, then always add the structure as a new binding.
   * If omitted, then use true.
   * \return A VLPointer to the index in the bindings of the structure (possible of a new binding).
   */
  Atom get_structure_variable(r_code::Code *object, uint16 index, bool allow_shared_variable = true);
  Atom get_object_variable(r_code::Code *object);
public:
  BindingMap();
  BindingMap(const BindingMap *source);
  BindingMap(const BindingMap &source);
  virtual ~BindingMap();

  BindingMap& operator =(const BindingMap &source);
  void load(const BindingMap *source);

  virtual void clear();

  void init(r_code::Code *object, uint16 index);

  _Fact *abstract_f_ihlp(_Fact *fact) const; // for icst and imdl.

  /**
   * Fill in the fresh fact object as an abstract copy of the original fact, creating new bindings as needed.
   * \param fact A fresh Fact or AntiFact object.
   * \param original The original object to copy.
   * \param allow_shared_timing_vars (optional) Use this for allow_shared_variable when calling
   * abstract_member() for the fact's timing values. If omitted, use true.
   */
  _Fact *abstract_fact(_Fact *fact, _Fact *original, bool force_sync, bool allow_shared_timing_vars = true);
  r_code::Code *abstract_object(r_code::Code *object, bool force_sync, bool allow_shared_timing_vars = true);

  void reset_fwd_timings(_Fact *reference_fact); // reset after and before from the timings of the reference object.

  MatchResult match_fwd_lenient(const _Fact *f_object, const _Fact *f_pattern); // use for facts when we are lenient about fact vs |fact.
  bool match_fwd_strict(const _Fact *f_object, const _Fact *f_pattern); // use for facts when we need sharp match.

  /**
   * Check if the map entry at i is bound to a timestamp.
   * @return True if it is a timestamp.
   */
  bool is_timestamp(uint16 i) const {
    return i >= 0 && i < map_.size() && map_[i]->get_code() != NULL &&
      map_[i]->get_code()[0].getDescriptor() == Atom::TIMESTAMP;
  }
  bool has_fwd_after() const { return is_timestamp(fwd_after_index_); }
  bool has_fwd_before() const { return is_timestamp(fwd_before_index_); }
  Timestamp get_fwd_after() const; // assumes the timings are valuated.
  Timestamp get_fwd_before() const; // idem.

  bool match_object(const r_code::Code *object, const r_code::Code *pattern);
  bool match_structure(const r_code::Code *object, uint16 o_base_index, uint16 o_index, const r_code::Code *pattern, uint16 p_index);
  bool match_atom(Atom o_atom, Atom p_atom);

  void bind_variable(BoundValue *value, uint8 id);
  void bind_variable(Atom *code, uint8 id, uint16 value_index, Atom *intermediate_results);

  Atom *get_value_code(uint16 id);
  uint16 get_value_code_size(uint16 id);

  bool intersect(BindingMap *bm);
  bool is_fully_specified() const;

  Atom *get_code(uint16 i) const { return map_[i]->get_code(); }
  r_code::Code *get_object(uint16 i) const { return map_[i]->get_object(); }
  int16 get_fwd_after_index() const { return fwd_after_index_; }
  int16 get_fwd_before_index() const { return fwd_before_index_; }
  bool scan_variable(uint16 id) const; // return true if id<first_index or map[id] is not an UnboundValue.
};

class r_exec_dll HLPBindingMap :
  public BindingMap {
private:
  int16 bwd_after_index_;
  int16 bwd_before_index_;

  bool match_bwd_timings(const _Fact *f_object, const _Fact *f_pattern);

  bool need_binding(r_code::Code *pattern) const;
  void init_from_pattern(const r_code::Code *source, int16 position); // first source is f->obj.
public:
  HLPBindingMap();
  HLPBindingMap(const HLPBindingMap *source);
  HLPBindingMap(const HLPBindingMap &source);
  ~HLPBindingMap();

  HLPBindingMap& operator =(const HLPBindingMap &source);
  void load(const HLPBindingMap *source);
  void clear();

  void init_from_hlp(const r_code::Code *hlp);
  void init_from_f_ihlp(const _Fact *f_ihlp);
  Fact *build_f_ihlp(r_code::Code *hlp, uint16 opcode, bool wr_enabled) const; // return f->ihlp.
  r_code::Code *bind_pattern(r_code::Code *pattern) const;

  void reset_bwd_timings(_Fact *reference_fact); // idem for the last 2 unbound variables (i.e. timings of the second pattern in a mdl).

  MatchResult match_bwd_lenient(const _Fact *f_object, const _Fact *f_pattern); // use for facts when we are lenient about fact vs |fact.
  bool match_bwd_strict(const _Fact *f_object, const _Fact *f_pattern); // use for facts when we need sharp match.

  bool has_bwd_after() const { return is_timestamp(bwd_after_index_); }
  bool has_bwd_before() const { return is_timestamp(bwd_before_index_); }
  Timestamp get_bwd_after() const; // assumes the timings are valuated.
  Timestamp get_bwd_before() const; // idem.
};
}


#endif
