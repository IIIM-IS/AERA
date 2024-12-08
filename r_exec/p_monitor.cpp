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

#include "p_monitor.h"
#include "mem.h"
#include "mdl_controller.h"

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

PMonitor::PMonitor(MDLController *controller,
  BindingMap *bindings,
  Fact *prediction,
  Code* mk_rdx,
  bool rate_failures) : Monitor(controller, bindings, prediction, mk_rdx), rate_failures_(rate_failures) { // prediction is f0->pred->f1->obj; not simulated.

  prediction_target_ = prediction->get_pred()->get_target(); // f1.
  auto now = Now();

  bindings->reset_fwd_timings(prediction_target_);

  MonitoringJob<PMonitor> *j = new MonitoringJob<PMonitor>(this, prediction_target_->get_before() + Utils::GetTimeTolerance());
  _Mem::Get()->push_time_job(j);
}

PMonitor::~PMonitor() {
}

bool PMonitor::reduce(_Fact *input) { // input is always an actual fact.

  if (target_->is_invalidated()) {
    return true; }

  if (target_->get_pred()->grounds_invalidated(input)) { // input is a counter-evidence for one of the antecedents: abort.

    target_->invalidate();
    return true;
  }

  Pred *prediction = input->get_pred();
  if (prediction) {

    switch (prediction->get_target()->is_evidence(prediction_target_)) {
    case MATCH_SUCCESS_POSITIVE: // predicted confirmation, skip.
      return false;
    case MATCH_SUCCESS_NEGATIVE:
      if (prediction->get_target()->get_cfd() > prediction_target_->get_cfd()) {

        target_->invalidate(); // a predicted counter evidence is stronger than the target, invalidate and abort: don't rate the model.
        return true;
      } else
        return false;
    case MATCH_FAILURE:
      return false;
    }
  } else {
    //uint32 oid=input->get_oid();
    switch (((Fact *)input)->is_evidence(prediction_target_)) {
    case MATCH_SUCCESS_POSITIVE:
      controller_->register_pred_outcome(target_, mk_rdx_, true, input, input->get_cfd(), rate_failures_);
      return true;
    case MATCH_SUCCESS_NEGATIVE:
      if (rate_failures_)
        controller_->register_pred_outcome(target_, mk_rdx_, false, input, input->get_cfd(), rate_failures_);
      return true;
    case MATCH_FAILURE:
      return false;
    }
  }

  return false;
}

void PMonitor::update(Timestamp &next_target) { // executed by a time core, upon reaching the expected time of occurrence of the target of the prediction.

  if (!target_->is_invalidated()) {

    // Received nothing matching the target's object so far (neither positively nor negatively).
    // It is only a failure if the target is a fact. If the target is an anti-fact then reaching this
    // point means that the object was not observed *as expected* (not a failure).
    // TODO: If the model correctly predicts an anti-fact that the object won't be observed, then should we register
    // a success for the model? If yes then what should "evidence" point? Maybe nil?
    if (rate_failures_ && target_->get_pred()->get_target()->is_fact())
      controller_->register_pred_outcome(target_, mk_rdx_, false, NULL, 1, rate_failures_);
  }
  controller_->remove_monitor(this);
  next_target = Timestamp(seconds(0));
}
}
