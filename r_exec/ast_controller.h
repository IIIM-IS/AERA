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

#ifndef ast_controller_h
#define ast_controller_h

#include "overlay.h"
#include "factory.h"
#include "pattern_extractor.h"


namespace r_exec {

// Atomic state controller. Attached to a null-pgm and monitoring to a (repeated) input fact (SYNC_PERIODIC or SYNC_HOLD).
// Has a resilience of 2 times the upr of the group its target comes from.
// Upon catching a counter-evidence, signal the TPX and kill the object (i.e. invalidate and kill views); this will kill the controller and TPX.
// Catching a predicted evidence means that there is a model that predicts the next value of the object: kill the CTPX.
// AST live in primary groups and take their inputs therefrom: these are filtered by the A/F WRT goals/predictions.
// There is no control over AST: instead, computation is minimal (just pattern-matching) and CTPX are killed asap whenever a model predicts a value change.
// There cannot be any control based on the semantics of the inputs as these are atomic and henceforth no icst is available at injection time.
template<class U> class ASTController :
  public OController {
protected:
  P<CTPX> tpx_;
  P<_Fact> target_; // the repeated fact to be monitored.
  Timestamp thz_timestamp_; // time horizon: if an input is caught with ijt<thz (meaning it's too old), discard it.

  void kill();

  ASTController(AutoFocusController *auto_focus, View *target);
public:
  virtual ~ASTController();

  Code *get_core_object() const { return getObject(); }

  void take_input(r_exec::View *input);
  void reduce(View *input);
};

// For SYNC_PERIODIC targets.
class PASTController :
  public ASTController<PASTController> {
public:
  PASTController(AutoFocusController *auto_focus, View *target);
  ~PASTController();

  void reduce(View *input) { this->ASTController<PASTController>::reduce(input); }
  void reduce(View *v, _Fact *input);
};

// For SYNC_HOLD targets.
class HASTController :
  public ASTController<HASTController> {
private:
  P<_Fact> source_; // to be invalidated if a counter-evidence is found.
public:
  HASTController(AutoFocusController *auto_focus, View *target, _Fact *source);
  ~HASTController();

  void reduce(View *input) { this->ASTController<HASTController>::reduce(input); }
  void reduce(View *v, _Fact *input);
};
}


#include "ast_controller.tpl.cpp"


#endif
