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

#ifndef overlay_h
#define overlay_h

#include "../submodules/CoreLibrary/CoreLibrary/base.h"
#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
#include "../r_code/object.h"
#include "dll.h"

using r_code::Atom;

namespace r_exec {

/**
 * The TraceLevel enum defines bit positions for the "trace_levels" parameter
 * for "Debug" in settings.xml.
 * The number of bits should match RuntimeOutputStreamCount.
 */
typedef enum {
  CST_IN = 0,
  CST_OUT = 1,
  MDL_IN = 2,
  MDL_OUT = 3,
  PRED_MON = 4,
  GOAL_MON = 5,
  MDL_REV = 6,
  HLP_INJ = 7,
  IO_DEVICE_INJ_EJT = 8,
  AUTO_FOCUS = 9
} TraceLevel;

/**
 * See _Mem::Output. This namespace-level function is declared here so that it can be 
 * used without including mem.h, which causes dependency loops if an inline header 
 * method needs to use it.
 */
std::ostream __declspec(dllexport) &_Mem_Output(TraceLevel l);

 /**
  * This similar to "_Mem_Output(level) << vals << endl", where vals can be "x << y << z". Except
  * that we first use an ostringstream to buffer the values plus the line terminator, and
  * then send the entire string to the output stream and flush. Assuming that the output
  * stream will output the entire string as a single operation, then when all threads use
  * OUTPUT_LINE it will avoid scrambling output of individual values.
  */
#define OUTPUT_LINE(level, vals) (_Mem_Output(level) << (ostringstream() << vals << '\n').str()).flush()

class View;

// Upon invocation of take_input() the overlays older than tsc are killed, assuming stc>0; otherwise, overlays live unitl the ipgm dies.
// Controllers are built at loading time and at the view's injection time.
// Derived classes must expose a function: void reduce(r_code::View*input); (called by reduction jobs).
class r_exec_dll Controller :
  public _Object {
protected:
  volatile uint32 invalidated_; // 32 bit alignment.
  volatile uint32 activated_; // 32 bit alignment.

  std::chrono::microseconds time_scope_;

  r_code::View *view_;

  CriticalSection reductionCS_;

  virtual void take_input(r_exec::View *input) {}
  template<class C> void __take_input(r_exec::View *input) { // utility: to be called by sub-classes.

    View *_view = new View(input);
    ReductionJob<C> *j = new ReductionJob<C>(input/*_view*/, (C *)this);
    _Mem::Get()->pushReductionJob(j);
  }

  Controller(r_code::View *view);
public:
  virtual ~Controller();

  std::chrono::microseconds get_tsc() { return time_scope_; }

  virtual void invalidate() { invalidated_ = 1; }
  bool is_invalidated() { return invalidated_ == 1; };
  void activate(bool a) { activated_ = (a ? 1 : 0); }
  bool is_activated() const { return activated_ == 1; }
  bool is_alive() const { return invalidated_ == 0 && activated_ == 1; }

  virtual r_code::Code *get_core_object() const = 0;

  r_code::Code *getObject() const { return view_->object_; } // return the reduction object (e.g. ipgm, icpp_pgm, cst, mdl).
  r_exec::View *getView() const { return (r_exec::View *)view_; } // return the reduction object's view.

  void _take_input(r_exec::View *input); // called by the rMem at update time and at injection time.

  virtual void gain_activation() { activate(true); }
  virtual void lose_activation() { activate(false); }

  void set_view(View *view);

  void debug(View *input) {}
};

class _Context;
class IPGMContext;
class HLPContext;

class r_exec_dll Overlay :
  public _Object {
  friend class _Context;
  friend class IPGMContext;
  friend class HLPContext;
protected:
  volatile uint32 invalidated_;

  Controller *controller_;

  r_code::vector<r_code::Atom> values_; // value array: stores the results of computations.
  // Copy of the pgm/hlp code. Will be patched during matching and evaluation:
  // any area indexed by a vl_ptr will be overwritten with:
  //   the evaluation result if it fits in a single atom,
  //   a ptr to the value array if the result is larger than a single atom,
  //   a ptr to an input if the result is a pattern input.
  r_code::Atom *code_;
  uint16 code_size_;
  std::vector<uint16> patch_indices_; // indices where patches are applied; used for rollbacks.
  uint16 value_commit_index_; // index of the last computed value_+1; used for rollbacks.

  void load_code();
  void patch_code(uint16 index, r_code::Atom value);
  uint16 get_last_patch_index();
  void unpatch_code(uint16 patch_index);

  void rollback(); // reset the overlay to the last commited state: unpatch code and values.
  void commit(); // empty the patch_indices_ and set value_commit_index_ to values.size().

  r_code::Code *get_core_object() const; // pgm, mdl, cst.

  Overlay();
  Overlay(Controller *c, bool load_code = true);
public:
  virtual ~Overlay();

  virtual void reset(); // reset to original state.
  virtual Overlay *reduce(r_exec::View *input); // returns an offspring in case of a match.

  void invalidate() { invalidated_ = 1; }
  virtual bool is_invalidated() { return invalidated_ == 1; }

  r_code::Code *getObject() const { return ((Controller *)controller_)->getObject(); }
  r_exec::View *getView() const { return ((Controller *)controller_)->getView(); }

  r_code::Code *build_object(r_code::Atom head) const;
};

class r_exec_dll OController :
  public Controller {
protected:
  r_code::list<P<Overlay> > overlays_;

  OController(r_code::View *view);
public:
  virtual ~OController();
};
}


#endif
