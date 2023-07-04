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

  virtual bool intersect(const Value* /* v */) const { return false; }
  virtual bool _intersect(const AtomValue* /* v */) const { return false; }
  virtual bool _intersect(const StructureValue* /* v */) const { return false; }
  virtual bool _intersect(const ObjectValue* /* v */) const { return false; }

  virtual bool contains(const Atom /* a */) const { return false; }
  virtual bool contains(const Atom* /* s */) const { return false; }
  virtual bool contains(const r_code::Code* /* o */) const { return false; }

  /**
   * Return the trace of the value as a string by creating a temporary Code
   * object and calling valuate(). For debugging purposes only (can be inefficient).
   */
  std::string trace_string() const {
    P<LObject> code = new LObject();
    uint16 extent_index = 1;
    valuate(code, 0, extent_index);
    return code->trace_string();
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

  Value *copy(BindingMap *map) const override;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const override;
  bool match(const r_code::Code *object, uint16 index) override;
  Atom *get_code() override;
  r_code::Code *get_object() override;
  uint16 get_code_size() override;
};

class r_exec_dll AtomValue :
  public BoundValue {
private:
  Atom atom_;
public:
  AtomValue(BindingMap *map, Atom atom);

  Value *copy(BindingMap *map) const override;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const override;
  bool match(const r_code::Code *object, uint16 index) override;
  Atom *get_code() override;
  r_code::Code *get_object() override;
  uint16 get_code_size() override;

  bool intersect(const Value *v) const override;
  bool _intersect(const AtomValue *v) const override;

  bool contains(const Atom a) const override;
};

class r_exec_dll StructureValue :
  public BoundValue {
private:
  P<r_code::Code> structure_;
  StructureValue(BindingMap* map, const r_code::Code* structure)
    : StructureValue(map, &structure->code(0), 0) {}
public:
  StructureValue(BindingMap *map, const r_code::Code *source, uint16 structure_index)
    : StructureValue(map, &source->code(0), structure_index) {}
  StructureValue(BindingMap *map, const Atom *source, uint16 structure_index);
  StructureValue(BindingMap *map, Timestamp time);
  StructureValue(BindingMap *map, std::chrono::microseconds duration);

  Value *copy(BindingMap *map) const override;
  void valuate(r_code::Code* destination, uint16 write_index, uint16& extent_index) const override {
    destination->code(write_index) = Atom::IPointer(extent_index);
    copy_structure(destination, extent_index, &structure_->code(0), 0);
  }

  /**
   * This is a helper to copy the structure starting at source(source_index) to the destination at extent_index.
   * If needed, you should already have done destination->code(write_index) = Atom::IPointer(extent_index);
   * This copies the structure to destination[extent_index].
   * \param destination The destination code.
   * \param extent_index Copy the structure to destination starting at this index and update the index.
   * \param source_index Copy the structure from code(source_index).
   */
  static void copy_structure(
    r_code::Code* destination, uint16& extent_index, const Atom* source, uint16 source_index);

  bool match(const r_code::Code *object, uint16 index) override;
  Atom *get_code() override;
  r_code::Code *get_object() override;
  uint16 get_code_size() override;

  bool intersect(const Value *v) const override;
  bool _intersect(const StructureValue *v) const override;

  bool contains(const Atom *s) const override;
};

class r_exec_dll ObjectValue :
  public BoundValue {
private:
  const P<r_code::Code> object_;
public:
  ObjectValue(BindingMap *map, r_code::Code *object);

  Value *copy(BindingMap *map) const override;
  void valuate(r_code::Code *destination, uint16 write_index, uint16 &extent_index) const override;
  bool match(const r_code::Code *object, uint16 index) override;
  Atom *get_code() override;
  r_code::Code *get_object() override;
  uint16 get_code_size() override;

  bool intersect(const Value *v) const override;
  bool _intersect(const ObjectValue *v) const override;

  bool contains(const r_code::Code *o) const override;
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

  bool match_fwd_timings(const _Fact *f_object);
  bool match(const r_code::Code *object, uint16 o_base_index, uint16 o_index, const r_code::Code *pattern, uint16 p_index, uint16 o_arity);

  void abstract_member(const r_code::Code *object, uint16 index, r_code::Code *abstracted_object, uint16 write_index, uint16 &extent_index, int first_search_index = 0);
  Atom get_atom_variable(Atom a);

  /**
   * Get a VLPointer for the structre at object->code(index), adding a new binding if needed.
   * \param object The code with the structure.
   * \param index The index in the object code of the structure.
   * \param first_search_index (optional) If >= 0, use this as the start index in the binding map to
   * check if it already contains the structure and return a VLPointer to the binding index if found.
   * (This starts at the index and wraps around to search the entire binding map.) If -1, then always
   * add the structure as a new binding. If omitted, then use 0 to search the whole map from the beginning.
   * \return A VLPointer to the index in the bindings of the structure (possible of a new binding).
   */
  Atom get_structure_variable(const r_code::Code *object, uint16 index, int first_search_index = 0);
  Atom get_object_variable(r_code::Code *object);
public:
  BindingMap();
  BindingMap(const BindingMap *source);
  BindingMap(const BindingMap &source);
  virtual ~BindingMap();

  BindingMap& operator =(const BindingMap &source);
  void load(const BindingMap *source);

  virtual void clear();

  void init(const r_code::Code *object, uint16 index);

  _Fact *abstract_f_ihlp(_Fact *fact) const; // for icst and imdl.

  /**
   * Fill in the fresh fact object as an abstract copy of the original fact, creating new bindings as needed.
   * \param fact A fresh Fact or AntiFact object.
   * \param original The original object to copy.
   * \param timing_vars_first_search_index (optional) Use this for first_search_index when calling
   * abstract_member() for the fact's timing values. If omitted, use 0.
   */
  _Fact *abstract_fact(_Fact *fact, const _Fact *original, bool force_sync, int timing_vars_first_search_index = 0);
  r_code::Code *abstract_object(r_code::Code *object, bool force_sync, int timing_vars_first_search_index = 0);

  void reset_fwd_timings(_Fact *reference_fact); // reset after and before from the timings of the reference object.

  /**
   * Match the given time interval to the time interval at this binding map's fwd_after_index_ and fwd_before_index_,
   * updating this binding map's values to "narrow" them to the given time interval if needed.
   * This assumes you have already called has_fwd_after() and has_fwd_before() to make sure that there are
   * valid Timestamp bindings at fwd_after_index_ and fwd_before_index_.
   * \param after The beginning of the time interval to compare with this binding map's fwd time interval.
   * \param before The end of the time interval to compare with this binding map's fwd time interval.
   * \return True if the time intervals match, in which case this binding maps' values may have been updated
   * to "narrow" the time interval.
   */
  bool match_fwd_timings(Timestamp after, Timestamp before) {
    return match_timings(after, before, fwd_after_index_, fwd_before_index_);
  }

  MatchResult match_fwd_lenient(const _Fact *f_object, const _Fact *f_pattern); // use for facts when we are lenient about fact vs |fact.
  bool match_fwd_strict(const _Fact *f_object, const _Fact *f_pattern); // use for facts when we need sharp match.

  /**
   * Check if the map entry at i is bound to a timestamp.
   * \return True if it is a timestamp.
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

  Atom *get_value_code(uint16 id) const;
  uint16 get_value_code_size(uint16 id) const;
  uint16 get_first_index() const { return first_index_;  }

  /**
   * Check if any value in bm matches any value in this BindingMap
   * \param bm The other BindingMap with values to check.
   * \return True if any two values match.
   */
  bool intersect(const BindingMap *bm) const;

  bool is_fully_specified() const;

  Atom *get_code(uint16 i) const { return map_[i]->get_code(); }
  r_code::Code *get_object(uint16 i) const { return map_[i]->get_object(); }
  int16 get_fwd_after_index() const { return fwd_after_index_; }
  int16 get_fwd_before_index() const { return fwd_before_index_; }
  bool scan_variable(uint16 id) const; // return true if id<first_index or map[id] is not an UnboundValue.

  /**
   * Match the given time interval to the time interval at this binding map's after_index and before_index,
   * updating the binding map's values to "narrow" them to the given time interval if needed.
   * This assumes you have already checked to make sure that there are valid Timestamp bindings at
   * after_index and before_index.
   * \param after The beginning of the time interval to compare with this binding map's time interval.
   * \param before The end of the time interval to compare with this binding map's time interval.
   * \param after_index The index in the binding map of the time interval after Timestamp.
   * \param before_index The index in the binding map of the time interval before Timestamp.
   * \return True if the time intervals match, in which case this binding map's values may have been updated
   * to "narrow" the time interval.
   */
  bool match_timings(Timestamp after, Timestamp before, uint32 after_index, uint32 before_index);

  /**
   * Get the code index where the exposed args will start after calling abstract_object on the ihlp. This is
   * Necessary because the ihlp may have structured values before the exposed args which will be removed by
   * abstract_object (which only has variables and no structured values).
   * \param ihlp The imdl or icst.
   * \return The exposed args index.
   */
  static uint16 get_abstracted_ihlp_exposed_args_index(const r_code::Code* ihlp);

  /**
   * Return the trace of the value at index i as a string by creating a temporary Code
   * object and calling valuate(). For debugging purposes only (can be inefficient).
   */
  std::string trace_string(uint16 i) const { return map_[i]->trace_string(); }
};

class r_exec_dll HLPBindingMap :
  public BindingMap {
private:
  int16 bwd_after_index_;
  int16 bwd_before_index_;

  bool match_bwd_timings(const _Fact *f_object, const _Fact *f_pattern);

  bool need_binding(r_code::Code *pattern) const;

  /**
   * If source->code(FACT_AFTER) and source->code(FACT_BEFORE) are VL_PTR, then set the respective
   * after_index and before_index.
   */
  void init_timing_indexes(const r_code::Code* source, int16& after_index, int16& before_index) {
    if (source->code_size() <= FACT_BEFORE)
      // We don't expect this.
      return;
    if (source->code(FACT_AFTER).getDescriptor() == Atom::VL_PTR)
      after_index = source->code(FACT_AFTER).asIndex();
    if (source->code(FACT_BEFORE).getDescriptor() == Atom::VL_PTR)
      before_index = source->code(FACT_BEFORE).asIndex();
  }

  /**
   * Scan the structure in hlp at structure_index and call add_unbound_value for each VL_PTR.
   * \param hlp The HLP with the code.
   * \param structure_index The index in hlp of the structure to scan.
   */
  void add_unbound_values(const r_code::Code* hlp, uint16 structure_index);

  /**
   * Copy the structure in hlp at hlp_structure_index to ihlp at extent_index, and call valuate for
   * each VL_PTR.
   * \param hlp The HLP with the code.
   * \param hlp_structure_index The index in hlp of the structure to copy.
   * \param ihlp The ihlp to write to.
   * \param extent_index The index in ihlp to write to. This updates extent_index to the end of written code.
   */
  void build_ihlp_structure(
    const r_code::Code* hlp, uint16 hlp_structure_index, r_code::Code* ihlp, uint16& extent_index) const;

  /**
   * Scan the structure in hlp at hlp_args_index and for each VL_PTR set the binding map to a subclass of
   * BoundValue based on the value in ihlp at ihlp_args_index.
   * \param hlp The HLP with the code to scan.
   * \param hlp_args_index The index in hlp of the structure to scan.
   * \param ihlp The ihlp to init the binding map from.
   * \param ihlp_args_index The index in ihlp of the equivalent structure as in hlp.
   */
  void init_from_ihlp_args(
    const r_code::Code* hlp, uint16 hlp_args_index, const r_code::Code* ihlp, uint16 ihlp_args_index);
public:
  HLPBindingMap();
  HLPBindingMap(const HLPBindingMap *source);
  HLPBindingMap(const HLPBindingMap &source);
  ~HLPBindingMap();

  HLPBindingMap& operator =(const HLPBindingMap &source);
  void load(const HLPBindingMap *source);
  void clear() override;

  void init_from_hlp(const r_code::Code *hlp, const r_code::Code* packed_hlp);
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
