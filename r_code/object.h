//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_Code
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

#ifndef r_code_object_h
#define r_code_object_h

#include <ostream>
#include <sstream>
#include "atom.h"
#include "vector.h"
#include "list.h"
#include "replicode_defs.h"

#include "../submodules/CoreLibrary/CoreLibrary/base.h"
#include "utils.h"


using namespace core;

namespace r_code {

// I/O from/to r_code::Image ////////////////////////////////////////////////////////////////////////

class dll_export ImageObject {
public:
  r_code::vector<Atom> code_;
  r_code::vector<uint16> references_;

  virtual void write(word32 *data) = 0;
  virtual void read(word32 *data) = 0;
  virtual void trace(std::ostream& out) = 0;
};

class View;

class dll_export SysView :
  public ImageObject {
public:
  SysView();
  SysView(View *source);

  void write(word32 *data);
  void read(word32 *data);
  uint32 get_size() const;
  void trace(std::ostream& out);
#ifdef WITH_DEBUG_OID
  int debug_oid_;
#endif
};

class Code;

class dll_export SysObject :
  public ImageObject {
private:
  static uint32 lastOID_;
public:
  r_code::vector<uint32> markers_; // indexes in the relocation segment
  r_code::vector<SysView *> views_;

  uint32 oid_;
#ifdef WITH_DEBUG_OID
  uint64 debug_oid_;
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

class dll_export View :
  public _Object {
private:
  uint16 index_; // for unpacking: index is the index of the view in the SysObject.
protected:
  Atom code_[VIEW_CODE_MAX_SIZE]; // dimensioned to hold the largest view (group view): head atom, iptr to ijt, sln, res, rptr to grp, rptr to org, vis, cov, 3 atoms for ijt's timestamp; oid is the last word32 (not an atom).
public:
  Code *references_[2]; // does not include the viewed object; no smart pointer here (a view is held by a group and holds a ref to said group in references[0]).
  P<Code> object_; // viewed object.

  View() : object_(NULL) {

    references_[0] = references_[1] = NULL;
  }

  View(SysView *source, Code *object) {

    for (uint16 i = 0; i < source->code_.size(); ++i)
      code_[i] = source->code_[i];
    references_[0] = references_[1] = NULL;
    object_ = object;
  }

  virtual ~View() {}

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
    size_t operator ()(View *v) const {
      return (size_t)(Code *)v->references_[0]; // i.e. the group the view belongs to.
    }
  };

  class Equal {
  public:
    bool operator ()(const View *lhs, const View *rhs) const {
      return lhs->references_[0] == rhs->references_[0];
    }
  };

  class Less {
  public:
    bool operator ()(const View *lhs, const View *rhs) const {
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
  int32 storage_index_; // -1: not sored; >0 index of the object in a vector-based container.

  void load(SysObject *source) {

    for (uint16 i = 0; i < source->code_.size(); ++i)
      code(i) = source->code_[i];
    set_oid(source->oid_);
  }
  template<class V> View *build_view(SysView *source) {

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
  UNORDERED_SET<View *, View::Hash, View::Equal> views_; // indexed by groups.

  virtual View *build_view(SysView *source) = 0;

  virtual void acq_views() {}
  virtual void rel_views() {}
  virtual void acq_markers() {}
  virtual void rel_markers() {}

  virtual float32 get_psln_thr() { return 1; }

  Code() : storage_index_(null_storage_index) { markers_.reserve(CodeMarkersInitialSize); }
  virtual ~Code() {}

  virtual void mod(uint16 member_index, float32 value) {};
  virtual void set(uint16 member_index, float32 value) {};
  virtual View *get_view(Code *group, bool lock) { return NULL; }
  virtual void add_reference(Code *object) const {} // called only on local objects.
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
#ifdef WITH_DEBUG_OID
        out << "(" << get_reference(atom.asIndex())->get_debug_oid() << ")";
#endif
      }
      else
        out << " (unassigned) ";
    }
  }

  /**
   * Print the trace of code(i) to the out stream, using a default TraceContext (no indentation).
   */
  void trace(uint16 i, std::ostream& out) const {
    Atom::TraceContext context;
    trace(i, out, context);
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
#ifdef WITH_DEBUG_OID
    out << "(" << get_debug_oid() << ")";
#endif
    out << std::endl;
  }

  void trace() const { trace(std::cout); }

  /**
   * Return the trace as a string. For debugging purposes only(can be inefficient).
   */
  std::string traceString() const {
    std::ostringstream out;
    trace(out);
    return out.str();
  }

  /**
   * Return the trace of code(i) as a string, using a default TraceContext (no indentation).
   */
  std::string traceString(uint16 i) const {
    std::ostringstream out;
    trace(i, out);
    return out.str();
  }
};

// Implementation for local objects (non distributed).
class dll_export LObject :
  public Code {
protected:
  uint32 oid_;
  r_code::vector<Atom> code_;
  r_code::vector<P<Code> > references_;
public:
  LObject() : Code() {}
  LObject(SysObject *source) : Code() {

    load(source);
  }
  virtual ~LObject() {}

  View *build_view(SysView *source) {

    return Code::build_view<View>(source);
  }

  uint32 get_oid() const { return oid_; }
  void set_oid(uint32 oid) { oid_ = oid; }

  Atom &code(uint16 i) { return code_[i]; }
  Atom &code(uint16 i) const { return (*code_.as_std())[i]; }
  uint16 code_size() const { return code_.size(); }
  void resize_code(uint16 new_size) { code_.as_std()->resize(new_size); }
  void set_reference(uint16 i, Code *object) { references_[i] = object; }
  Code *get_reference(uint16 i) const { return (*references_.as_std())[i]; }
  uint16 references_size() const { return references_.size(); }
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
