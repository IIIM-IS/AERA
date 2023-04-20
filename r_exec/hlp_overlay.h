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
protected:
  friend class CSTController; // For abduce_simulated.
  P<HLPBindingMap> bindings_;

  bool evaluate_guards(uint16 guard_set_iptr_index);
  bool evaluate_fwd_guards();
  /**
   * Create an HLPContext and evaluate code_ at index.
   * \param index The index in code_ to evaluate.
   * \return True if successfully evaluated, false for problem evaluated including
   * unbound variables. Note that if the code at index is a boolean expression,
   * then this can return true for successful evaluation even though the expression value
   * is boolean false.
   */
  bool evaluate(uint16 index);

  bool evaluate_fwd_timings();

  bool scan_bwd_guards() const;
  bool scan_location(uint16 index, uint16 parent_guard_index) const;
  bool scan_variable(uint16 index, uint16 parent_guard_index) const;

  void store_evidence(_Fact *evidence, bool prediction, bool is_simulation); // stores both actual and non-simulated predicted evidences.

  HLPOverlay(Controller *c, HLPBindingMap *bindings);
public:
  static bool EvaluateBWDGuards(Controller *c, HLPBindingMap *bindings); // updates the bindings.

  /**
   * Find the backward guards which assign the forward timings and evaluate them.
   * \param c The model controller with the code for the backward guards.
   * \param bindings Update the binding map forward timings.
   * \return True if evaluted, false if no backward guards assign the forward timings or if can't evaluate.
   */
  static bool EvaluateFWDTimings(Controller *c, HLPBindingMap *bindings);

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
