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

#include "hlp_controller.h"
#include "hlp_overlay.h"
#include "mem.h"

using namespace r_code;

namespace r_exec {

HLPController::HLPController(r_code::View *view) : OController(view), strong_requirement_count_(0), weak_requirement_count_(0), requirement_count_(0) {

  bindings_ = new HLPBindingMap();

  Code *object = get_unpacked_object();
  bindings_->init_from_hlp(object); // init a binding map from the patterns.

  has_tpl_args_ = object->code(object->code(HLP_TPL_ARGS).asIndex()).getAtomCount() > 0;
  ref_count_ = 0;
  last_match_time_ = Now();
}

HLPController::~HLPController() {
}

void HLPController::invalidate() {

  invalidated_ = 1;
  controllers_.clear();
}

void HLPController::add_requirement(bool strong) {

  reductionCS_.enter();
  if (strong)
    ++strong_requirement_count_;
  else
    ++weak_requirement_count_;
  ++requirement_count_;
  reductionCS_.leave();
}

void HLPController::remove_requirement(bool strong) {

  reductionCS_.enter();
  if (strong)
    --strong_requirement_count_;
  else
    --weak_requirement_count_;
  --requirement_count_;
  reductionCS_.leave();
}

uint32 HLPController::get_requirement_count(uint32 &weak_requirement_count, uint32 &strong_requirement_count) {

  uint32 r_c;
  reductionCS_.enter();
  r_c = requirement_count_;
  weak_requirement_count = weak_requirement_count_;
  strong_requirement_count = strong_requirement_count_;
  reductionCS_.leave();
  return r_c;
}

uint32 HLPController::get_requirement_count() {

  uint32 r_c;
  reductionCS_.enter();
  r_c = requirement_count_;
  reductionCS_.leave();
  return r_c;
}

uint16 HLPController::get_out_group_count() const {

  return getObject()->code(getObject()->code(HLP_OUT_GRPS).asIndex()).getAtomCount();
}

Code *HLPController::get_out_group(uint16 i) const {

  Code *hlp = getObject();
  uint16 out_groups_index = hlp->code(HLP_OUT_GRPS).asIndex() + 1; // first output group index.
  return hlp->get_reference(hlp->code(out_groups_index + i).asIndex());
}

bool HLPController::evaluate_bwd_guards(HLPBindingMap *bm, bool narrow_fwd_timings) {

  bool saved_fwd_timings = false;
  Timestamp save_fwd_after, save_fwd_before;
  if (narrow_fwd_timings && bm->has_fwd_after() && bm->has_fwd_before()) {
    // The forward timings have previously-bound values from matching the requirement.
    save_fwd_after = bm->get_fwd_after();
    save_fwd_before = bm->get_fwd_before();
    saved_fwd_timings = true;
  }

  bool result = HLPOverlay::EvaluateBWDGuards(this, bm);

  if (saved_fwd_timings) {
    // Call match_fwd_timings to narrow the forward timings set by the backward guards to the previously-bound values.
    if (!bm->match_fwd_timings(save_fwd_after, save_fwd_before))
      // The backward guards updated the forward timings outside the previously-bound values. (We don't expect this.)
      return false;
  }

  return result;
}

bool HLPController::inject_prediction(Fact *prediction, float32 confidence) const { // prediction is simulated: f->pred->f->target.

  Code *primary_host = get_host();
  float32 sln_thr = primary_host->code(GRP_SLN_THR).asFloat();
  if (confidence > sln_thr) { // do not inject if cfd is too low.

    View *view = new View(View::SYNC_ONCE, Now(), confidence, 1, primary_host, primary_host, prediction); // SYNC_ONCE,res=1.
    _Mem::Get()->inject(view);
    return true;
  }

  return false;
}

MatchResult HLPController::check_evidences(_Fact *target, _Fact *&evidence) {

  MatchResult r = MATCH_FAILURE;
  evidences_.CS_.enter();
  auto now = Now();
  r_code::list<EvidenceEntry>::const_iterator e;
  for (e = evidences_.list_.begin(); e != evidences_.list_.end();) {

    if ((*e).is_too_old(now)) // garbage collection.
      e = evidences_.list_.erase(e);
    else {

      if ((r = (*e).evidence_->is_evidence(target)) != MATCH_FAILURE) {

        evidence = (*e).evidence_;
        evidences_.CS_.leave();
        return r;
      }
      ++e;
    }
  }
  evidence = NULL;
  evidences_.CS_.leave();
  return r;
}

MatchResult HLPController::check_predicted_evidences(_Fact *target, _Fact *&evidence) {

  MatchResult r = MATCH_FAILURE;
  predicted_evidences_.CS_.enter();
  auto now = Now();
  r_code::list<PredictedEvidenceEntry>::const_iterator e;
  for (e = predicted_evidences_.list_.begin(); e != predicted_evidences_.list_.end();) {

    if ((*e).is_too_old(now)) // garbage collection.
      e = predicted_evidences_.list_.erase(e);
    else {

      if ((r = (*e).evidence_->is_evidence(target)) != MATCH_FAILURE) {

        if (target->get_cfd() < (*e).evidence_->get_cfd()) {

          evidence = (*e).evidence_;
          predicted_evidences_.CS_.leave();
          return r;
        } else
          r = MATCH_FAILURE;
      }
      ++e;
    }
  }
  evidence = NULL;
  predicted_evidences_.CS_.leave();
  return r;
}

bool HLPController::become_invalidated() {

  if (is_invalidated())
    return true;

  for (uint16 i = 0; i < controllers_.size(); ++i) {

    if (controllers_[i] != NULL && controllers_[i]->is_invalidated()) {

      kill_views();
      return true;
    }
  }

  if (has_tpl_args()) {

    if (refCount_ == 1 && ref_count_ > 1) { // the ctrler was referenced by others, is no longer and has tpl args: it will not be able to predict: kill.

      kill_views();
      return true;
    } else
      ref_count_ = refCount_;
  }

  return false;
}

bool HLPController::is_orphan() {

  if (has_tpl_args_) {

    if (get_requirement_count() == 0) {

      kill_views();
      return true;
    }
  }
  return false;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HLPController::EvidenceEntry::EvidenceEntry() : evidence_(NULL) {
}

HLPController::EvidenceEntry::EvidenceEntry(_Fact *evidence) : evidence_(evidence) {

  load_data(evidence);
}

HLPController::EvidenceEntry::EvidenceEntry(_Fact *evidence, _Fact *payload) : evidence_(evidence) {

  load_data(payload);
}

void HLPController::EvidenceEntry::load_data(_Fact *evidence) {

  after_ = evidence->get_after();
  before_ = evidence->get_before();
  confidence_ = evidence->get_cfd();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

HLPController::PredictedEvidenceEntry::PredictedEvidenceEntry() : EvidenceEntry() {
}

HLPController::PredictedEvidenceEntry::PredictedEvidenceEntry(_Fact *evidence) : EvidenceEntry(evidence, evidence->get_pred()->get_target()) {
}
}
