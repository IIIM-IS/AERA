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

#ifndef overlay_h
#define overlay_h

#include "../submodules/CoreLibrary/CoreLibrary/base.h"
#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
#include "../r_code/object.h"
#include "controller.h"
#include "reduction_job.h"
#include "dll.h"

namespace r_exec {

class _Context;
class IPGMContext;
class HLPContext;
class Controller;
class View;

class r_exec_dll Overlay :
  public _Object {
  friend class _Context;
  friend class IPGMContext;
  friend class HLPContext;
protected:
  volatile uint32 invalidated_;

  Controller *controller_;

  r_code::resized_vector<r_code::Atom> values_; // value array: stores the results of computations.
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
  Overlay(size_t values_size);
  virtual ~Overlay();

  virtual void reset(); // reset to original state.
  virtual Overlay *reduce(r_exec::View *input); // returns an offspring in case of a match.

  void invalidate() { invalidated_ = 1; }
  virtual bool is_invalidated() { return invalidated_ == 1; }

  r_code::Code* get_object() const;
  r_exec::View* get_view() const;

  r_code::Code *build_object(r_code::Atom head) const;
  const r_code::Atom* values() const { return &values_[0]; }
};

/**
 * A DefeasibleValidity is an object this is attached to a defeasible prediction and copied
 * to each later prediction in forward chaining. If a new fact defeats the grounds of
 * the original prediction, then call invalidate() to invalidate the defeasible prediction
 * and all predictions which followed from it. (The is_invalidate() method of Pred checks its
 * set of DefeasibleValidity and invalidates the Pred if a DefeasibleValidity is invalidated.)
 */
class r_exec_dll DefeasibleValidity :
  public _Object {
public:
  /**
   * Create a DefeasibleValidity that is not invalidated.
   */
  DefeasibleValidity() : invalidated_(0) {}

  /**
   * Check if this is invalidated
   * \return True if this is invalidated.
   */
  bool is_invalidated() { return invalidated_ != 0; }

  /**
   * Set this to invalidated so that is_invalidated() returns true.
   */
  void invalidate() { invalidated_ = 1; }

private:
  volatile uint32 invalidated_; // 32 bit alignment.
};

template<class T> class CriticalSectionList {
public:
  CriticalSection CS_;
  r_code::list<T> list_;
};

}


#endif
