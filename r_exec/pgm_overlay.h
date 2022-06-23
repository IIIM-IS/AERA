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

#ifndef pgm_overlay_h
#define pgm_overlay_h

#include "overlay.h"


namespace r_exec {

class PGMController;
class InputLessPGMController;
class IPGMContext;

// Overlays for input-less programs.
// Base class for other programs (with inputs, anti-programs).
class r_exec_dll InputLessPGMOverlay :
  public Overlay {
  friend class PGMController;
  friend class InputLessPGMController;
  friend class IPGMContext;
protected:
  std::vector<P<r_code::Code> > productions_; // receives the results of ins, inj and eje; views are retrieved (fvw) or built (reduction) in the value array.

  /**
   * Create an IPGMContext and evaluate code_ at index.
   * \param index The index in code_ to evaluate.
   * \return True if successfully evaluated, false for problem evaluated including
   * unbound variables. Note that if the code at index is a boolean expression,
   * then this can return true for successful evaluation even though the expression value
   * is boolean false.
   */
  bool evaluate(uint16 index);

  virtual r_code::Code *get_mk_rdx(uint16 &extent_index) const;

  void patch_tpl_args(); // no views in tpl args; patches the ptn skeleton's first atom with IPGM_PTR with an index in the ipgm arg set; patches wildcards with similar IPGM_PTRs.
  void patch_tpl_code(uint16 pgm_code_index, uint16 ipgm_code_index); // to recurse.
  virtual void patch_input_code(uint16 pgm_code_index, uint16 input_index, uint16 input_code_index, int16 parent_index = -1); // defined in PGMOverlay.

  InputLessPGMOverlay();
  InputLessPGMOverlay(Controller *c);
public:
  virtual ~InputLessPGMOverlay();

  virtual void reset(); // reset to original state (pristine copy of the pgm code and empty value set).

  bool inject_productions(); // return true upon successful evaluation; no existence check in simulation mode.
};

// Overlay with inputs.
// Several ReductionCores can attempt to reduce the same overlay simultaneously (each with a different input).
class r_exec_dll PGMOverlay :
  public InputLessPGMOverlay {
  friend class PGMController;
  friend class IPGMContext;
private:
  bool is_volatile_;
  Timestamp birth_time_; // used for ipgms: overlays older than ipgm->tsc are killed; birth_time set to the time of the first match, 0 if no match occurred.
protected:
  r_code::list<uint16> input_pattern_indices_; // stores the input patterns still waiting for a match: will be plucked upon each successful match.
  std::vector<P<r_code::_View> > input_views_; // copies of the inputs; vector updated at each successful match.

  typedef enum {
    SUCCESS = 0,
    FAILURE = 1,
    IMPOSSIBLE = 3 // when the input's class does not even match the object class in the pattern's skeleton.
  }MatchResult;

  MatchResult match(r_exec::View *input, uint16 &input_index); // delegates to _match; input_index is set to the index of the pattern that matched the input.
  bool check_guards(); // return true upon successful evaluation.

  MatchResult _match(r_exec::View *input, uint16 pattern_index); // delegates to __match.

  /**
   * \return SUCCESS upon a successful match, IMPOSSIBLE if the input is not of the right class, 
   * FAILURE otherwise.
   */
  MatchResult __match(r_exec::View *input, uint16 pattern_index);

  r_code::Code *dereference_in_ptr(Atom a);
  void patch_input_code(uint16 pgm_code_index, uint16 input_index, uint16 input_code_index, int16 parent_index = -1);

  virtual r_code::Code *get_mk_rdx(uint16 &extent_index) const;

  void init();

  PGMOverlay(Controller *c);
  PGMOverlay(PGMOverlay *original, uint16 last_input_index, uint16 value_commit_index); // copy from the original and rollback.
public:
  virtual ~PGMOverlay();

  void reset() {
    InputLessPGMOverlay::reset();
    patch_indices_.clear();
    input_views_.clear();
    input_pattern_indices_.clear();
    init();
  }

  virtual Overlay *reduce(r_exec::View *input); // called upon the processing of a reduction job.

  r_code::Code *getInputObject(uint16 i) const;
  r_code::_View *getInputView(uint16 i) const;

  Timestamp get_birth_time() const { return birth_time_; }

  bool is_invalidated();
};

// Several ReductionCores can attempt to reduce the same overlay simultaneously (each with a different input).
// In addition, ReductionCores and signalling jobs can attempt to inject productions concurrently.
// Usues the same mk.rdx as for InputLessPGMOverlays.
class r_exec_dll AntiPGMOverlay :
  public PGMOverlay {
  friend class AntiPGMController;
private:
  AntiPGMOverlay(Controller *c) : PGMOverlay(c) {}
  AntiPGMOverlay(AntiPGMOverlay *original, uint16 last_input_index, uint16 value_limit)
  : PGMOverlay(original, last_input_index, value_limit) {}
public:
  ~AntiPGMOverlay();

  Overlay *reduce(r_exec::View *input); // called upon the processing of a reduction job.
};
}


#include "pgm_overlay.inline.cpp"


#endif
