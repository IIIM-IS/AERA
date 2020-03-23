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
  std::vector<P<Code> > productions; // receives the results of ins, inj and eje; views are retrieved (fvw) or built (reduction) in the value array.

  bool evaluate(uint16 index); // evaluates the pgm_code at the specified index.

  virtual Code *get_mk_rdx(uint16 &extent_index) const;

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
  bool is_volatile;
  uint64 birth_time; // used for ipgms: overlays older than ipgm->tsc are killed; birth_time set to the time of the first match, 0 if no match occurred.
protected:
  r_code::list<uint16> input_pattern_indices; // stores the input patterns still waiting for a match: will be plucked upon each successful match.
  std::vector<P<r_code::View> > input_views; // copies of the inputs; vector updated at each successful match.

  typedef enum {
    SUCCESS = 0,
    FAILURE = 1,
    IMPOSSIBLE = 3 // when the input's class does not even match the object class in the pattern's skeleton.
  }MatchResult;

  MatchResult match(r_exec::View *input, uint16 &input_index); // delegates to _match; input_index is set to the index of the pattern that matched the input.
  bool check_guards(); // return true upon successful evaluation.

  MatchResult _match(r_exec::View *input, uint16 pattern_index); // delegates to __match.
  MatchResult __match(r_exec::View *input, uint16 pattern_index); // return SUCCESS upon a successful match, IMPOSSIBLE if the input is not of the right class, FAILURE otherwise.

  Code *dereference_in_ptr(Atom a);
  void patch_input_code(uint16 pgm_code_index, uint16 input_index, uint16 input_code_index, int16 parent_index = -1);

  virtual Code *get_mk_rdx(uint16 &extent_index) const;

  void init();

  PGMOverlay(Controller *c);
  PGMOverlay(PGMOverlay *original, uint16 last_input_index, uint16 value_commit_index); // copy from the original and rollback.
public:
  virtual ~PGMOverlay();

  void reset();

  virtual Overlay *reduce(r_exec::View *input); // called upon the processing of a reduction job.

  r_code::Code *getInputObject(uint16 i) const;
  r_code::View *getInputView(uint16 i) const;

  uint64 get_birth_time() const { return birth_time; }

  bool is_invalidated();
};

// Several ReductionCores can attempt to reduce the same overlay simultaneously (each with a different input).
// In addition, ReductionCores and signalling jobs can attempt to inject productions concurrently.
// Usues the same mk.rdx as for InputLessPGMOverlays.
class r_exec_dll AntiPGMOverlay :
  public PGMOverlay {
  friend class AntiPGMController;
private:
  AntiPGMOverlay(Controller *c);
  AntiPGMOverlay(AntiPGMOverlay *original, uint16 last_input_index, uint16 value_limit);
public:
  ~AntiPGMOverlay();

  Overlay *reduce(r_exec::View *input); // called upon the processing of a reduction job.
};
}


#include "pgm_overlay.inline.cpp"


#endif
