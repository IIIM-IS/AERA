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

#include "ast_controller.h"
#include "mem.h"
#include "factory.h"
#include "auto_focus.h"


namespace r_exec {

template<class U> ASTController<U>::ASTController(AutoFocusController *auto_focus, View *target) : OController(NULL) {

  this->target_ = (_Fact *)target->object_;
  tpx_ = new CTPX(auto_focus, target); // target is the premise, i.e. the tpx' target to be defeated.
  thz_ = Now() - Utils::GetTimeTolerance();
}

template<class U> ASTController<U>::~ASTController() {
}

template<class U> void ASTController<U>::take_input(r_exec::View *input) {

  if (is_invalidated())
    return;

  if (input->object_->code(0).asOpcode() == Opcodes::Fact ||
    input->object_->code(0).asOpcode() == Opcodes::AntiFact) { // discard everything but facts and |facts.

    Goal *goal = ((_Fact *)input->object_)->get_goal();
    if (goal && goal->get_actor() == _Mem::Get()->get_self()) // ignore self's goals.
      return;
    if (input->get_ijt() >= thz_) // input is too old.
      Controller::__take_input<U>(input);
  }
}

template<class U> void ASTController<U>::reduce(View *input) {

  if (is_invalidated())
    return;

  _Fact *input_object = input->object_;
  if (input_object->is_invalidated()) {//std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" TPX"<<target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" got inv data "<<input->object->get_oid()<<std::endl;
    return; }
  //uint64 t0=Now();
  reductionCS_.enter();

  if (input_object == target_) {

    tpx_->store_input(input);
    reductionCS_.leave();
    return;
  }

  Pred *prediction = input_object->get_pred();
  if (prediction) {
    //std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" TPX"<<target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" got pred "<<prediction->get_target()->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<std::endl;
    switch (prediction->get_target()->is_timeless_evidence(target_)) {
    case MATCH_SUCCESS_POSITIVE:
    case MATCH_SUCCESS_NEGATIVE: // a model predicted the next value of the target.
        //std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" TPX"<<target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" killed\n";
      kill();
      break;
    case MATCH_FAILURE:
      break;
    }

    reductionCS_.leave();
    return;
  }
  //std::cout<<Time::ToString_seconds(Now()-Utils::GetTimeReference())<<" TPX"<<target->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<" got "<<input->object->get_reference(0)->code(MK_VAL_VALUE).asFloat()<<std::endl;
  ((U *)this)->reduce(input, input_object);

  reductionCS_.leave();
  //uint64 t1=Now();
  //std::cout<<"ASTC: "<<t1-t0<<std::endl;
}

template<class U> void ASTController<U>::kill() {

  invalidate();
  getView()->force_res(0);
}
}
