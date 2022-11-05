//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2022 Jeff Thompson
//_/_/ Copyright (c) 2018-2022 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2022 Icelandic Institute for Intelligent Machines
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

#ifndef r_code_object_h
#define r_code_object_h

#include <ostream>
#include <sstream>
#include <unordered_set>
#include "atom.h"
#include "resized_vector.h"
#include "list.h"
#include "replicode_defs.h"

#include "../submodules/CoreLibrary/CoreLibrary/base.h"
#include "utils.h"


using namespace core;

namespace r_code {

// I/O from/to r_code::Image ////////////////////////////////////////////////////////////////////////

class dll_export ImageObject {
public:
  r_code::resized_vector<Atom> code_;
  r_code::resized_vector<uint16> references_;

  virtual void write(word32 *data) = 0;
  virtual void read(word32 *data) = 0;
  virtual void trace(std::ostream& out) = 0;
};

class _View;

class dll_export SysView :
  public ImageObject {
public:
  SysView();
  SysView(_View *source);

  void write(word32 *data);
  void read(word32 *data);
  uint32 get_size() const;
  void trace(std::ostream& out);
#ifdef WITH_DETAIL_OID
  int detail_oid_;
#endif
};

class Code;

class dll_export SysObject :
  public ImageObject {
private:
  static uint32 lastOID_;
public:
  r_code::resized_vector<uint32> markers_; // indexes in the relocation segment
  r_code::resized_vector<SysView *> views_;

  uint32 oid_;
#ifdef WITH_DETAIL_OID
  uint64 detail_oid_;
#endif

  SysObject();
  SysObject(Code *source);
  ~SysObject();

  void write(word32 *data);
  void read(word32 *data);
  uint32 get_size();
  void trace(std::ostream& out);
  void trace();
};

// Interfaces for r_exec classes ////////////////////////////////////////////////////////////////////////

class Object;

class dll_export _View :
  public _Object {
protected:
  Atom code_[VIEW_CODE_MAX_SIZE]; // dimensioned to hold the largest view (group view): head atom, iptr to ijt, sln, res, rptr to grp, rptr to org, vis, cov, 3 atoms for ijt's timestamp; oid is the last word32 (not an atom).
public:
  Code *references_[2]; // does not include the viewed object; no smart pointer here (a view is held by a group and holds a ref to said group in references[0]).
  P<Code> object_; // viewed object.

  _View() : object_(NULL) {

    references_[0] = references_[1] = NULL;
  }

  _View(SysView *source, Code *object) {

    for (uint16 i = 0; i < source->code_.size(); ++i)
      code_[i] = source->code_[i];
    references_[0] = references_[1] = NULL;
    object_ = object;
  }

  virtual ~_View() {}

  Atom &code(uint16 i) { return code_[i]; }
  Atom code(uint16 i) const { return code_[i]; }

  typedef enum {
    SYNC_ONCE = 0,
    SYNC_PERIODIC = 1,
    SYNC_HOLD = 2,
    SYNC_AXIOM = 3,
    SYNC_ONCE_AXIOM = 4
  }SyncMode;

  SyncMode get_sync() const { return (SyncMode)(uint32)code_[VIEW_SYNC].asFloat(); }
  Timestamp get_ijt() const { return Utils::GetTimestamp(code_ + code_[VIEW_IJT].asIndex()); }
  void set_ijt(Timestamp ijt) { Utils::SetTimestamp(code_ + code_[VIEW_IJT].asIndex(), ijt); }

  class Hash {
  public:
    size_t operator ()(_View *v) const {
      return (size_t)(Code *)v->references_[0]; // i.e. the group the view belongs to.
    }
  };

  class Equal {
  public:
    bool operator ()(const _View *lhs, const _View *rhs) const {
      return lhs->references_[0] == rhs->references_[0];
    }
  };

  class Less {
  public:
    bool operator ()(const _View *lhs, const _View *rhs) const {
      return lhs->get_ijt() < rhs->get_ijt();
    }
  };
};

class dll_export Code :
  public _Object {
public:
  static const int32 null_storage_index = -1;
  static const uint32 CodeMarkersInitialSize = 8;
protected:
  int32 storage_index_; // -1: not stored; >= 0: index of the object in a vector-based container.

  void load(SysObject *source) {

    for (uint16 i = 0; i < source->code_.size(); ++i)
      code(i) = source->code_[i];
    set_oid(source->oid_);
  }
  template<class V> _View *build_view(SysView *source) {

    return new V(source, this);
  }
public:
  void set_strorage_index(int32 i) { storage_index_ = i; }
  bool is_registered() const { return storage_index_ > null_storage_index; }
  int32 get_storage_index() const { return storage_index_; }

  virtual uint32 get_oid() const = 0;
  virtual void set_oid(uint32 oid) = 0;

  virtual Atom &code(uint16 i) = 0;
  virtual Atom &code(uint16 i) const = 0;
  virtual uint16 code_size() const = 0;
  virtual void resize_code(uint16 new_size) = 0;
  virtual void set_reference(uint16 i, Code *object) = 0;
  virtual Code *get_reference(uint16 i) const = 0;
  virtual uint16 references_size() const = 0;
  virtual void clear_references() = 0;
  virtual void set_references(std::vector<P<Code> > &new_references) = 0;

  virtual bool is_compact() const { return false; }
  virtual bool is_invalidated() { return false; }
  virtual bool invalidate() { return false; }

  r_code::list<Code *> markers_;
  std::unordered_set<_View *, _View::Hash, _View::Equal> views_; // indexed by groups.

  virtual _View *build_view(SysView *source) = 0;

  virtual void acq_views() {}
  virtual void rel_views() {}
  virtual void acq_markers() {}
  virtual void rel_markers() {}

  virtual float32 get_psln_thr() { return 1; }

  Code() : storage_index_(null_storage_index) { markers_.reserve(CodeMarkersInitialSize); }
  virtual ~Code() {}

  virtual void mod(uint16 /* member_index */, float32 /* value */) {};
  virtual void set(uint16 /* member_index */, float32 /* value */) {};
  virtual _View *get_view(Code* /* group */, bool /* lock */) { return NULL; }
  virtual void add_reference(Code* /* object */) const {} // called only on local objects.
  void remove_marker(Code *m) {

    acq_markers();
    markers_.remove(m);
    rel_markers();
  }

  /**
   * Print the trace of code(i) to the out stream, using the given TraceContext.
   */
  void trace(uint16 i, std::ostream& out, Atom::TraceContext& context) const {
    Atom& atom = code(i);
    atom.trace(context, out);
    if (atom.getDescriptor() == Atom::R_PTR) {
      if (atom.asIndex() < references_size()) {
        out << " -> " << get_reference(atom.asIndex())->get_oid();
#ifdef WITH_DETAIL_OID
        out << "(" << get_reference(atom.asIndex())->get_detail_oid() << ")";
#endif
      }
      else
        out << " (unassigned) ";
    }
  }

  /**
   * Print the trace of this Code to the out stream.
   */
  void trace(std::ostream& out) const {

    out << "--------\n";
    Atom::TraceContext context;
    for (uint16 i = 0; i < code_size(); ++i) {

      out << i << "\t";
      trace(i, out, context);
      out << std::endl;
    }
    out << "OID: " << get_oid();
#ifdef WITH_DETAIL_OID
    out << "(" << get_detail_oid() << ")";
#endif
    out << std::endl;
  }

  void trace() const { trace(std::cout); }

  /**
   * Return the trace as a string. For debugging purposes only(can be inefficient).
   */
  std::string trace_string() const {
    std::ostringstream out;
    trace(out);
    return out.str();
  }

  /**
   * Return the trace of code(i) as a string, using a default TraceContext (no indentation).
   */
  std::string trace_string(uint16 i) const {
    Atom::TraceContext context;
    std::ostringstream out;
    trace(i, out, context);
    return out.str();
  }
};

// Implementation for local objects (non distributed).
class dll_export LocalObject :
  public Code {
private:
  uint32 oid_;
  r_code::resized_vector<Atom> code_;
  r_code::resized_vector<P<Code> > references_;
public:
  LocalObject() : Code() {}
  LocalObject(SysObject *source) : Code() {

    load(source);
  }
  virtual ~LocalObject() {}

  _View *build_view(SysView *source) {

    return Code::build_view<_View>(source);
  }

  uint32 get_oid() const { return oid_; }
  void set_oid(uint32 oid) { oid_ = oid; }

  Atom &code(uint16 i) { return code_[i]; }
  Atom &code(uint16 i) const { return (*code_.as_std())[i]; }
  uint16 code_size() const {
    // There can't be more than 65536 code bytes. Explicitly cast to the return type.
    return (uint16)code_.size();
  }
  void resize_code(uint16 new_size) { code_.as_std()->resize(new_size); }
  void set_reference(uint16 i, Code *object) { references_[i] = object; }
  Code *get_reference(uint16 i) const { return (*references_.as_std())[i]; }
  uint16 references_size() const {
    // There can't be more than 65536 references. Explicitly cast to the return type.
    return (uint16)references_.size();
  }
  void clear_references() { references_.as_std()->clear(); }
  void set_references(std::vector<P<Code> > &new_references) { (*references_.as_std()) = new_references; }
  void add_reference(Code *object) const { references_.as_std()->push_back(object); }
};

class dll_export Mem {
protected:
  static Mem *singleton_;
  Mem();
public:
  static Mem *Get();

  virtual Code *build_object(SysObject *source) const = 0;
  virtual void delete_object(Code *object) = 0;
};
}


#endif
