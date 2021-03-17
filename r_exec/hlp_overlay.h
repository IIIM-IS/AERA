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

#ifndef hlp_overlay_h
#define hlp_overlay_h

#include "overlay.h"
#include "binding_map.h"


namespace r_exec {

class HLPContext;

// HLP: high-level patterns.
class HLPOverlay :
  public Overlay {
  friend class HLPContext;
public:
  P<HLPBindingMap> bindings_;
protected:

  r_code::list<P<_Fact> > patterns_;

  bool evaluate_guards(uint16 guard_set_iptr_index);
  bool evaluate_fwd_guards();
  /**
   * Create an HLPContext and evaluate code_ at index.
   * \param index The index in code_ to evaluate.
   * \param result_index The result is at code_[result_index]. If this returns false,
   * then this is undefined.
   * \return True if successfully evaluated, false for problem evaluated including
   * unbound variables. Note that if the code at index is a boolean expression,
   * then this can return true for successful evaluation even though code_[result_index]
   * is boolean false.
   */
  bool evaluate(uint16 index, uint16 &result_index);

  bool check_fwd_timings();

  bool scan_bwd_guards();
  bool scan_location(uint16 index);
  bool scan_variable(uint16 index);

  void store_evidence(_Fact *evidence, bool prediction, bool is_simulation); // stores both actual and non-simulated predicted evidences.

  HLPOverlay(Controller *c, HLPBindingMap *bindings);
public:
  static bool EvaluateBWDGuards(Controller *c, HLPBindingMap *bindings); // updates the bindings.
  static bool CheckFWDTimings(Controller *c, HLPBindingMap *bindings); // updates the bindings.
  static bool ScanBWDGuards(Controller *c, HLPBindingMap *bindings); // does not update the bindings.

  HLPOverlay(Controller *c, const HLPBindingMap *bindings, bool load_code);
  virtual ~HLPOverlay();

  HLPBindingMap *get_bindings() const { return bindings_; }

  Atom *get_value_code(uint16 id) const;
  uint16 get_value_code_size(uint16 id) const;

  r_code::Code *get_unpacked_object() const;

  bool evaluate_bwd_guards();
};
}


#endif
