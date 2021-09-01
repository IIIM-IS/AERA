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

#include "group.h"
#include "factory.h"
#include "mem.h"
#include "pgm_controller.h"
#include "cst_controller.h"
#include "mdl_controller.h"
#include <math.h>

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

inline bool Group::is_active_pgm(View *view) {

  return view->get_act() > get_act_thr() &&
    get_c_sln() > get_c_sln_thr() &&
    get_c_act() > get_c_act_thr(); // active ipgm/icpp_pgm/cst/mdl in a c-salient and c-active group.
}

inline bool Group::is_eligible_input(View *view) {

  return view->get_sln() > get_sln_thr() &&
    get_c_sln() > get_c_sln_thr() &&
    get_c_act() > get_c_act_thr(); // active ipgm/icpp_pgm/cst/mdl in a c-salient and c-active group.
}

View *Group::get_view(uint32 OID) {

  UNORDERED_MAP<uint32, P<View> >::const_iterator it = other_views_.find(OID);
  if (it != other_views_.end())
    return it->second;
  it = group_views_.find(OID);
  if (it != group_views_.end())
    return it->second;
  it = ipgm_views_.find(OID);
  if (it != ipgm_views_.end())
    return it->second;
  it = anti_ipgm_views_.find(OID);
  if (it != anti_ipgm_views_.end())
    return it->second;
  it = input_less_ipgm_views_.find(OID);
  if (it != input_less_ipgm_views_.end())
    return it->second;
  it = notification_views_.find(OID);
  if (it != notification_views_.end())
    return it->second;
  return NULL;
}

void Group::reset_ctrl_values() {

  sln_thr_changes_ = 0;
  acc_sln_thr_ = 0;
  act_thr_changes_ = 0;
  acc_act_thr_ = 0;
  vis_thr_changes_ = 0;
  acc_vis_thr_ = 0;
  c_sln_changes_ = 0;
  acc_c_sln_ = 0;
  c_act_changes_ = 0;
  acc_c_act_ = 0;
  c_sln_thr_changes_ = 0;
  acc_c_sln_thr_ = 0;
  c_act_thr_changes_ = 0;
  acc_c_act_thr_ = 0;
}

void Group::reset_stats() {

  avg_sln_ = 0;
  high_sln_ = 0;
  low_sln_ = 1;
  avg_act_ = 0;
  high_act_ = 0;
  low_act_ = 1;

  sln_updates_ = 0;
  act_updates_ = 0;

  if (sln_change_monitoring_periods_to_go_ <= 0) { // 0:reactivate, -1: activate.

    if (get_sln_chg_thr() < 1) // activate monitoring.
      sln_change_monitoring_periods_to_go_ = get_sln_chg_prd();
  } else if (get_sln_chg_thr() == 1) // deactivate monitoring.
    sln_change_monitoring_periods_to_go_ = -1;
  else
    --sln_change_monitoring_periods_to_go_; // notification will occur when =0.

  if (act_change_monitoring_periods_to_go_ <= 0) { // 0:reactivate, -1: activate.

    if (get_act_chg_thr() < 1) // activate monitoring.
      act_change_monitoring_periods_to_go_ = get_act_chg_prd();
  } else if (get_act_chg_thr() == 1) // deactivate monitoring.
    act_change_monitoring_periods_to_go_ = -1;
  else
    --act_change_monitoring_periods_to_go_; // notification will occur when =0.
}

void Group::update_stats() {

  if (decay_periods_to_go_ > 0)
    --decay_periods_to_go_;

  // auto restart: iff dcy_auto==1.
  if (code(GRP_DCY_AUTO).asFloat()) {

    float32 period = code(GRP_DCY_PRD).asFloat();
    if (decay_periods_to_go_ <= 0 && period > 0)
      decay_periods_to_go_ = period;
  }

  if (sln_updates_)
    avg_sln_ = avg_sln_ / (float32)sln_updates_;
  if (act_updates_)
    avg_act_ = avg_act_ / (float32)act_updates_;

  code(GRP_AVG_SLN) = Atom::Float(avg_sln_);
  code(GRP_AVG_ACT) = Atom::Float(avg_act_);
  code(GRP_HIGH_SLN) = Atom::Float(high_sln_);
  code(GRP_LOW_SLN) = Atom::Float(low_sln_);
  code(GRP_HIGH_ACT) = Atom::Float(high_act_);
  code(GRP_LOW_ACT) = Atom::Float(low_act_);

  if (sln_change_monitoring_periods_to_go_ == 0) {

    FOR_ALL_NON_NTF_VIEWS_BEGIN(this, v)

      float32 change = v->second->update_sln_delta();
    if (fabs(change) > get_sln_chg_thr()) {

      uint16 ntf_grp_count = get_ntf_grp_count();
      for (uint16 i = 1; i <= ntf_grp_count; ++i)
        _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkSlnChg(_Mem::Get(), v->second->object_, change)), false);
    }

    FOR_ALL_NON_NTF_VIEWS_END
  }

  if (act_change_monitoring_periods_to_go_ == 0) {

    FOR_ALL_NON_NTF_VIEWS_BEGIN(this, v)

      float32 change = v->second->update_act_delta();
    if (fabs(change) > get_act_chg_thr()) {

      uint16 ntf_grp_count = get_ntf_grp_count();
      for (uint16 i = 1; i <= ntf_grp_count; ++i)
        _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkActChg(_Mem::Get(), v->second->object_, change)), false);
    }

    FOR_ALL_NON_NTF_VIEWS_END
  }
}

void Group::reset_decay_values() {

  sln_thr_decay_ = 0;
  sln_decay_ = 0;
  decay_periods_to_go_ = -1;
  decay_percentage_per_period_ = 0;
  decay_target_ = -1;
}

float32 Group::update_sln_thr() {

  float32 percentage = code(GRP_DCY_PER).asFloat();
  float32 period = code(GRP_DCY_PRD).asFloat();
  if (percentage == 0 || period == 0)
    reset_decay_values();
  else {

    float32 percentage_per_period = percentage / period;
    if (percentage_per_period != decay_percentage_per_period_ || code(GRP_DCY_TGT).asFloat() != decay_target_) { // recompute decay.

      decay_periods_to_go_ = period;
      decay_percentage_per_period_ = percentage_per_period;
      decay_target_ = code(GRP_DCY_TGT).asFloat();

      if (code(GRP_DCY_TGT).asFloat() == 0) {

        sln_thr_decay_ = 0;
        sln_decay_ = percentage_per_period;
      } else {

        sln_decay_ = 0;
        sln_thr_decay_ = percentage_per_period;
      }
    }
  }

  if (decay_periods_to_go_ > 0) {

    if (sln_thr_decay_ != 0)
      mod_sln_thr(get_sln_thr()*sln_thr_decay_);
  }

  if (sln_thr_changes_) {

    float32 new_sln_thr = get_sln_thr() + acc_sln_thr_ / sln_thr_changes_;
    if (new_sln_thr < 0)
      new_sln_thr = 0;
    else if (new_sln_thr > 1)
      new_sln_thr = 1;
    code(GRP_SLN_THR) = r_code::Atom::Float(new_sln_thr);
  }
  acc_sln_thr_ = 0;
  sln_thr_changes_ = 0;
  return get_sln_thr();
}

float32 Group::update_act_thr() {

  if (act_thr_changes_) {

    float32 new_act_thr = get_act_thr() + acc_act_thr_ / act_thr_changes_;
    if (new_act_thr < 0)
      new_act_thr = 0;
    else if (new_act_thr > 1)
      new_act_thr = 1;
    code(GRP_ACT_THR) = r_code::Atom::Float(new_act_thr);
  }
  acc_act_thr_ = 0;
  act_thr_changes_ = 0;
  return get_act_thr();
}

float32 Group::update_vis_thr() {

  if (vis_thr_changes_) {

    float32 new_vis_thr = get_vis_thr() + acc_vis_thr_ / vis_thr_changes_;
    if (new_vis_thr < 0)
      new_vis_thr = 0;
    else if (new_vis_thr > 1)
      new_vis_thr = 1;
    code(GRP_VIS_THR) = r_code::Atom::Float(new_vis_thr);
  }
  acc_vis_thr_ = 0;
  vis_thr_changes_ = 0;
  return get_vis_thr();
}

float32 Group::update_c_sln() {

  if (c_sln_changes_) {

    float32 new_c_sln = get_c_sln() + acc_c_sln_ / c_sln_changes_;
    if (new_c_sln < 0)
      new_c_sln = 0;
    else if (new_c_sln > 1)
      new_c_sln = 1;
    code(GRP_C_SLN) = r_code::Atom::Float(new_c_sln);
  }
  acc_c_sln_ = 0;
  c_sln_changes_ = 0;
  return get_c_sln();
}

float32 Group::update_c_act() {

  if (c_act_changes_) {

    float32 new_c_act = get_c_act() + acc_c_act_ / c_act_changes_;
    if (new_c_act < 0)
      new_c_act = 0;
    else if (new_c_act > 1)
      new_c_act = 1;
    code(GRP_C_ACT) = r_code::Atom::Float(new_c_act);
  }
  acc_c_act_ = 0;
  c_act_changes_ = 0;
  return get_c_act();
}

float32 Group::update_c_sln_thr() {

  if (c_sln_thr_changes_) {

    float32 new_c_sln_thr = get_c_sln_thr() + acc_c_sln_thr_ / c_sln_thr_changes_;
    if (new_c_sln_thr < 0)
      new_c_sln_thr = 0;
    else if (new_c_sln_thr > 1)
      new_c_sln_thr = 1;
    code(GRP_C_SLN_THR) = r_code::Atom::Float(new_c_sln_thr);
  }
  acc_c_sln_thr_ = 0;
  c_sln_thr_changes_ = 0;
  return get_c_sln_thr();
}

float32 Group::update_c_act_thr() {

  if (c_act_thr_changes_) {

    float32 new_c_act_thr = get_c_act_thr() + acc_c_act_thr_ / c_act_thr_changes_;
    if (new_c_act_thr < 0)
      new_c_act_thr = 0;
    else if (new_c_act_thr > 1)
      new_c_act_thr = 1;
    code(GRP_C_ACT_THR) = r_code::Atom::Float(new_c_act_thr);
  }
  acc_c_act_thr_ = 0;
  c_act_thr_changes_ = 0;
  return get_c_act_thr();
}

float32 Group::update_res(View *v) {

  if (v->object_->is_invalidated())
    return 0;
  float res = v->update_res();
  if (!v->is_notification() && res > 0 && res < get_low_res_thr()) {

    uint16 ntf_grp_count = get_ntf_grp_count();
    for (uint16 i = 1; i <= ntf_grp_count; ++i)
      _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkLowRes(_Mem::Get(), v->object_)), false);
  }
  return res;
}

float32 Group::update_sln(View *v) {

  if (decay_periods_to_go_ > 0 && sln_decay_ != 0)
    v->mod_sln(v->get_sln()*sln_decay_);

  float32 sln = v->update_sln(get_low_sln_thr(), get_high_sln_thr());
  avg_sln_ += sln;
  if (sln > high_sln_)
    high_sln_ = sln;
  else if (sln < low_sln_)
    low_sln_ = sln;
  ++sln_updates_;
  if (!v->is_notification()) {

    if (v->periods_at_high_sln_ == get_sln_ntf_prd()) {

      v->periods_at_high_sln_ = 0;
      uint16 ntf_grp_count = get_ntf_grp_count();
      for (uint16 i = 1; i <= ntf_grp_count; ++i)
        _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkHighSln(_Mem::Get(), v->object_)), false);
    } else if (v->periods_at_low_sln_ == get_sln_ntf_prd()) {

      v->periods_at_low_sln_ = 0;
      uint16 ntf_grp_count = get_ntf_grp_count();
      for (uint16 i = 1; i <= ntf_grp_count; ++i)
        _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkLowSln(_Mem::Get(), v->object_)), false);
    }
  }
  return sln;
}

float32 Group::update_act(View *v) {

  float32 act = v->update_act(get_low_act_thr(), get_high_act_thr());
  avg_act_ += act;
  if (act > high_act_)
    high_act_ = act;
  else if (act < low_act_)
    low_act_ = act;
  ++act_updates_;
  if (!v->is_notification()) {

    if (v->periods_at_high_act_ == get_act_ntf_prd()) {

      v->periods_at_high_act_ = 0;
      uint16 ntf_grp_count = get_ntf_grp_count();
      for (uint16 i = 1; i <= ntf_grp_count; ++i)
        _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkHighAct(_Mem::Get(), v->object_)), false);
    } else if (v->periods_at_low_act_ == get_act_ntf_prd()) {

      v->periods_at_low_act_ = 0;
      uint16 ntf_grp_count = get_ntf_grp_count();
      for (uint16 i = 1; i <= ntf_grp_count; ++i)
        _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkLowAct(_Mem::Get(), v->object_)), false);
    }
  }
  return act;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

bool Group::load(View *view, Code *object) {

  switch (object->code(0).getDescriptor()) {
  case Atom::GROUP: {
    group_views_[view->get_oid()] = view;

    // init viewing_group.
    bool viewing_c_active = get_c_act() > get_c_act_thr();
    bool viewing_c_salient = get_c_sln() > get_c_sln_thr();
    bool viewed_visible = view->get_vis() > get_vis_thr();
    if (viewing_c_active && viewing_c_salient && viewed_visible) // visible group in a c-salient, c-active group.
      ((Group *)object)->viewing_groups_[this] = view->get_cov(); // init the group's viewing groups.
    break;
  }case Atom::INSTANTIATED_PROGRAM: {
    ipgm_views_[view->get_oid()] = view;
    PGMController *c = new PGMController(view); // now will be added to the deadline at start time.
    view->controller_ = c;
    if (is_active_pgm(view))
      c->gain_activation();
    break;
  }case Atom::INSTANTIATED_INPUT_LESS_PROGRAM: {
    input_less_ipgm_views_[view->get_oid()] = view;
    InputLessPGMController *c = new InputLessPGMController(view); // now will be added to the deadline at start time.
    view->controller_ = c;
    if (is_active_pgm(view))
      c->gain_activation();
    break;
  }case Atom::INSTANTIATED_ANTI_PROGRAM: {
    anti_ipgm_views_[view->get_oid()] = view;
    AntiPGMController *c = new AntiPGMController(view); // now will be added to the deadline at start time.
    view->controller_ = c;
    if (is_active_pgm(view))
      c->gain_activation();
    break;
  }case Atom::INSTANTIATED_CPP_PROGRAM: {
    ipgm_views_[view->get_oid()] = view;
    Controller *c = CPPPrograms::New(Utils::GetString<Code>(view->object_, ICPP_PGM_NAME), view); // now will be added to the deadline at start time.
    if (!c)
      return false;
    view->controller_ = c;
    if (is_active_pgm(view))
      c->gain_activation();
    break;
  }case Atom::COMPOSITE_STATE: {
    ipgm_views_[view->get_oid()] = view;
    CSTController *c = new CSTController(view);
    view->controller_ = c;
    c->set_secondary_host(get_secondary_group());
    if (is_active_pgm(view))
      c->gain_activation();
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE(MDL_OUT, "load cst " << view->object_->get_oid() <<
      ", CSTController(" + to_string(c->get_detail_oid()) + ")");
#endif
    break;
  }case Atom::MODEL: {
    ipgm_views_[view->get_oid()] = view;
    bool inject_in_secondary_group;
    MDLController *c = MDLController::New(view, inject_in_secondary_group);
    view->controller_ = c;
    if (inject_in_secondary_group) {
      Group* secondary_group = get_secondary_group();
      // We don't expect the secondary_group to be NULL, but check anyway.
      if (secondary_group)
        secondary_group->load_secondary_mdl_controller(view);
    }
    if (is_active_pgm(view))
      c->gain_activation();
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE(MDL_OUT, "load mdl " << view->object_->get_oid() << 
      ", MDLController(" + to_string(c->get_detail_oid()) + ") strength:" <<
      view->object_->code(MDL_STRENGTH).asFloat() << " cnt:" << view->object_->code(MDL_CNT).asFloat() <<
      " sr:" << view->object_->code(MDL_SR).asFloat());
#endif
    break;
  }case Atom::MARKER: // populate the marker set of the referenced objects.
    for (uint32 i = 0; i < object->references_size(); ++i)
      object->get_reference(i)->markers_.push_back(object);
    other_views_[view->get_oid()] = view;
    break;
  case Atom::OBJECT:
    other_views_[view->get_oid()] = view;
    break;
  }

  return true;
}

void Group::update(Timestamp planned_time) {

  enter();

  if (this != _Mem::Get()->get_root() && views_.size() == 0) {

    invalidate();
    leave();
    return;
  }

  auto now = Now();
  //if(get_secondary_group()!=NULL)
  // std::cout<<Utils::RelativeTime(Now())<<" UPR\n";
  //if(this==_Mem::Get()->get_stdin())
  // std::cout<<Utils::RelativeTime(Now())<<" ----------------------------------------------------------------\n";
  newly_salient_views_.clear();

  // execute pending operations.
  for (uint32 i = 0; i < pending_operations_.size(); ++i) {

    pending_operations_[i]->execute(this);
    delete pending_operations_[i];
  }
  pending_operations_.clear();

  // update group's ctrl values.
  update_sln_thr(); // applies decay on sln thr.
  update_act_thr();
  update_vis_thr();

  GroupState state(get_sln_thr(), get_c_act() > get_c_act_thr(), update_c_act() > get_c_act_thr(), get_c_sln() > get_c_sln_thr(), update_c_sln() > get_c_sln_thr());

  reset_stats();

  FOR_ALL_VIEWS_BEGIN_NO_INC(this, v)

    if (v->second->object_->is_invalidated()) // no need to update the view set.
      delete_view(v);
    else {

      auto ijt = v->second->get_ijt();
      if (ijt >= planned_time) { // in case the update happens later than planned, don't touch views that were injected after the planned update time: update next time.

        ++v;
        continue;
      }

      float32 res = update_res(v->second); // update resilience: decrement res by 1 in addition to the accumulated changes.
      if (res > 0) {

        _update_saliency(&state, v->second); // apply decay.

        switch (v->second->object_->code(0).getDescriptor()) {
        case Atom::GROUP:
          _update_visibility(&state, v->second);
          break;
        case Atom::NULL_PROGRAM:
        case Atom::INSTANTIATED_PROGRAM:
        case Atom::INSTANTIATED_ANTI_PROGRAM:
        case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
        case Atom::INSTANTIATED_CPP_PROGRAM:
        case Atom::COMPOSITE_STATE:
        case Atom::MODEL:
          _update_activation(&state, v->second);
          break;
        }
        ++v;
      } else { // view has no resilience: delete it from the group.

        v->second->delete_from_object();
        delete_view(v);
      }
    }
  FOR_ALL_VIEWS_END

    if (state.is_c_salient)
      cov();

  // build reduction jobs.
  std::multiset<P<View>, r_code::View::Less>::const_iterator v;
  for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
    inject_reduction_jobs(*v);

  if (state.is_c_active && state.is_c_salient) { // build signaling jobs for new ipgms.

    for (uint32 i = 0; i < new_controllers_.size(); ++i) {

      switch (new_controllers_[i]->getObject()->code(0).getDescriptor()) {
      case Atom::INSTANTIATED_ANTI_PROGRAM: { // inject signaling jobs for |ipgm (tsc).
        // The time scope is stored as a timestamp, but it is actually a duration.
        P<TimeJob> j = new AntiPGMSignalingJob((r_exec::View *)new_controllers_[i]->getView(), now + Utils::GetTimestamp<Code>(new_controllers_[i]->getObject(), IPGM_TSC).time_since_epoch());
        _Mem::Get()->push_time_job(j);
        break;
      }case Atom::INSTANTIATED_INPUT_LESS_PROGRAM: { // inject a signaling job for an input-less pgm.

        P<TimeJob> j = new InputLessPGMSignalingJob((r_exec::View *)new_controllers_[i]->getView(), now + Utils::GetTimestamp<Code>(new_controllers_[i]->getObject(), IPGM_TSC).time_since_epoch());
        _Mem::Get()->push_time_job(j);
        break;
      }
      }
    }

    new_controllers_.clear();
  }

  update_stats(); // triggers notifications.

  if (get_upr() > 0) { // inject the next update job for the group.

    P<TimeJob> j = new UpdateJob(this, planned_time + get_upr()*Utils::GetBasePeriod());
    _Mem::Get()->push_time_job(j);
  }

  leave();

  //if(get_secondary_group()!=NULL)
  //if(this==_Mem::Get()->get_stdin())
  // std::cout<<Utils::RelativeTime(Now())<<" ++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++\n";
}

void Group::_update_saliency(GroupState *state, View *view) {

  float32 view_old_sln = view->get_sln();
  bool wiew_was_salient = view_old_sln > state->former_sln_thr;
  float32 view_new_sln = update_sln(view);
  bool wiew_is_salient = view_new_sln > get_sln_thr();

  if (state->is_c_salient) {

    if (wiew_is_salient) {

      switch (view->get_sync()) {
      case View::SYNC_ONCE:
      case View::SYNC_ONCE_AXIOM:
      case View::SYNC_PERIODIC:
        if (!wiew_was_salient) // sync on front: crosses the threshold upward: record as a newly salient view.
          newly_salient_views_.insert(view);
        break;
      case View::SYNC_HOLD:
      case View::SYNC_AXIOM: // sync on state: treat as if it was a new injection.
        view->set_ijt(Now());
        newly_salient_views_.insert(view);
        break;
      }
    }

    // inject sln propagation jobs.
    // the idea is to propagate sln changes when a view "occurs to the mind", i.e. becomes more salient in a group and is eligible for reduction in that group.
    // - when a view is now salient because the group becomes c-salient, no propagation;
    // - when a view is now salient because the group's sln_thr gets lower, no propagation;
    // - propagation can occur only if the group is c_active. For efficiency reasons, no propagation occurs even if some of the group's viewing groups are c-active and c-salient.
    if (state->is_c_active)
      _initiate_sln_propagation(view->object_, view_new_sln - view_old_sln, get_sln_thr());
  }
}

void Group::_update_visibility(GroupState *state, View *view) {

  bool view_was_visible = view->get_vis() > get_vis_thr();
  bool view_is_visible = view->update_vis() > get_vis_thr();
  bool cov = view->get_cov();

  // update viewing groups.
  if (state->was_c_active && state->was_c_salient) {

    if (!state->is_c_active || !state->is_c_salient) // group is not c-active and c-salient anymore: unregister as a viewing group.
      ((Group *)view->object_)->viewing_groups_.erase(this);
    else { // group remains c-active and c-salient.

      if (!view_was_visible) {

        if (view_is_visible) // newly visible view.
          ((Group *)view->object_)->viewing_groups_[this] = cov;
      } else {

        if (!view_is_visible) // the view is no longer visible.
          ((Group *)view->object_)->viewing_groups_.erase(this);
        else // the view is still visible, cov might have changed.
          ((Group *)view->object_)->viewing_groups_[this] = cov;
      }
    }
  } else if (state->is_c_active && state->is_c_salient) { // group becomes c-active and c-salient.

    if (view_is_visible) // update viewing groups for any visible group.
      ((Group *)view->object_)->viewing_groups_[this] = cov;
  }
}

void Group::_update_activation(GroupState *state, View *view) {

  bool view_was_active = view->get_act() > get_act_thr();
  bool view_is_active = update_act(view) > get_act_thr();

  // kill newly inactive controllers, register newly active ones.
  if (state->was_c_active && state->was_c_salient) {

    if (!state->is_c_active || !state->is_c_salient) // group is not c-active and c-salient anymore: kill the view's controller.
      view->controller_->lose_activation();
    else { // group remains c-active and c-salient.

      if (!view_was_active) {

        if (view_is_active) { // register the controller for the newly active ipgm view.

          view->controller_->gain_activation();
          new_controllers_.push_back(view->controller_);
        }
      } else {

        if (!view_is_active) // kill the newly inactive ipgm view's overlays.
          view->controller_->lose_activation();
      }
    }
  } else if (state->is_c_active && state->is_c_salient) { // group becomes c-active and c-salient.

    if (view_is_active) { // register the controller for any active ipgm view.

      view->controller_->gain_activation();
      new_controllers_.push_back(view->controller_);
    }
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Group::_initiate_sln_propagation(Code *object, float32 change, float32 source_sln_thr) const {

  if (fabs(change) > object->get_psln_thr()) {

    std::vector<Code *> path;
    path.push_back(object);

    if (object->code(0).getDescriptor() == Atom::MARKER) { // if marker, propagate to references.

      for (uint16 i = 0; i < object->references_size(); ++i)
        _propagate_sln(object->get_reference(i), change, source_sln_thr, path);
    }

    // propagate to markers
    object->acq_markers();
    r_code::list<Code *>::const_iterator m;
    for (m = object->markers_.begin(); m != object->markers_.end(); ++m)
      _propagate_sln(*m, change, source_sln_thr, path);
    object->rel_markers();
  }
}

void Group::_initiate_sln_propagation(Code *object, float32 change, float32 source_sln_thr, std::vector<Code *> &path) const {

  if (fabs(change) > object->get_psln_thr()) {

    // prevent loops.
    for (uint32 i = 0; i < path.size(); ++i)
      if (path[i] == object)
        return;
    path.push_back(object);

    if (object->code(0).getDescriptor() == Atom::MARKER) // if marker, propagate to references.
      for (uint16 i = 0; i < object->references_size(); ++i)
        _propagate_sln(object->get_reference(i), change, source_sln_thr, path);

    // propagate to markers
    object->acq_markers();
    r_code::list<Code *>::const_iterator m;
    for (m = object->markers_.begin(); m != object->markers_.end(); ++m)
      _propagate_sln(*m, change, source_sln_thr, path);
    object->rel_markers();
  }
}

void Group::_propagate_sln(Code *object, float32 change, float32 source_sln_thr, std::vector<Code *> &path) const {

  if (object == _Mem::Get()->get_root())
    return;

  // prevent loops.
  for (uint32 i = 0; i < path.size(); ++i)
    if (path[i] == object)
      return;
  path.push_back(object);

  P<TimeJob> j = new SaliencyPropagationJob(object, change, source_sln_thr, Timestamp(seconds(0)));
  _Mem::Get()->push_time_job(j);

  _initiate_sln_propagation(object, change, source_sln_thr, path);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

void Group::inject_hlps(std::vector<View *> &views) {

  enter();

  std::vector<View *>::const_iterator view;
  for (view = views.begin(); view != views.end(); ++view) {

    Atom a = (*view)->object_->code(0);
    switch (a.getDescriptor()) {
    case Atom::COMPOSITE_STATE: {
      ipgm_views_[(*view)->get_oid()] = *view;
      CSTController *c = new CSTController(*view);
      (*view)->controller_ = c;
      c->set_secondary_host(get_secondary_group());
      string controllerInfo;
#ifdef WITH_DETAIL_OID
      controllerInfo = ", CSTController(" + to_string(c->get_detail_oid()) + ")";
#endif
      OUTPUT_LINE(CST_OUT, Utils::RelativeTime(Now()) << " -> cst " << (*view)->object_->get_oid() << controllerInfo);
      break;
    }
    case Atom::MODEL: {
      ipgm_views_[(*view)->get_oid()] = *view;
      bool inject_in_secondary_group;
      MDLController *c = MDLController::New(*view, inject_in_secondary_group);
      (*view)->controller_ = c;
      if (inject_in_secondary_group)
        get_secondary_group()->inject_secondary_mdl_controller(*view);
      string controllerInfo;
#ifdef WITH_DETAIL_OID
      controllerInfo = ", MDLController(" + to_string(c->get_detail_oid()) + ")";
#endif
      OUTPUT_LINE(MDL_OUT, Utils::RelativeTime(Now()) << " -> mdl " << (*view)->object_->get_oid() << controllerInfo);
      break;
    }
    }
    (*view)->set_ijt(Now());
  }

  for (view = views.begin(); view != views.end(); ++view) {

    if (is_active_pgm(*view)) {

      (*view)->controller_->gain_activation();
      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
        (*view)->controller_->_take_input(*v); // view will be copied.
    }
  }

  leave();
}

void Group::inject(View *view) { // the view can hold anything but groups and notifications.

  enter();

  Atom a = view->object_->code(0);
  auto now = Now();
  view->set_ijt(now);
  switch (a.getDescriptor()) {
  case Atom::NULL_PROGRAM: // the view comes with a controller.
    ipgm_views_[view->get_oid()] = view;
    if (is_active_pgm(view)) {

      view->controller_->gain_activation();
      if (a.takesPastInputs()) {

        std::multiset<P<View>, r_code::View::Less>::const_iterator v;
        for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
          view->controller_->_take_input(*v); // view will be copied.
      }
    }
    break;
  case Atom::INSTANTIATED_PROGRAM: {
    ipgm_views_[view->get_oid()] = view;
    PGMController *c = new PGMController(view);
    view->controller_ = c;
    if (is_active_pgm(view)) {

      c->gain_activation();
      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
        c->_take_input(*v); // view will be copied.
    }
    break;
  }case Atom::INSTANTIATED_CPP_PROGRAM: {
    ipgm_views_[view->get_oid()] = view;
    Controller *c = CPPPrograms::New(Utils::GetString<Code>(view->object_, ICPP_PGM_NAME), view);
    if (!c)
      return;
    view->controller_ = c;
    if (is_active_pgm(view)) {

      c->gain_activation();
      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
        c->_take_input(*v); // view will be copied.
    }
    break;
  }case Atom::INSTANTIATED_ANTI_PROGRAM: {
    anti_ipgm_views_[view->get_oid()] = view;
    AntiPGMController *c = new AntiPGMController(view);
    view->controller_ = c;
    if (is_active_pgm(view)) {

      c->gain_activation();
      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
        c->_take_input(*v); // view will be copied.
      // The time scope is stored as a timestamp, but it is actually a duration.
      _Mem::Get()->push_time_job(new AntiPGMSignalingJob(view, now + Utils::GetTimestamp<Code>(c->getObject(), IPGM_TSC).time_since_epoch()));
    }
    break;
  }case Atom::INSTANTIATED_INPUT_LESS_PROGRAM: {
    input_less_ipgm_views_[view->get_oid()] = view;
    InputLessPGMController *c = new InputLessPGMController(view);
    view->controller_ = c;
    if (is_active_pgm(view)) {

      c->gain_activation();
      _Mem::Get()->push_time_job(new InputLessPGMSignalingJob(view, now + Utils::GetTimestamp<Code>(view->object_, IPGM_TSC).time_since_epoch()));
    }
    break;
  }case Atom::MARKER: // the marker has already been added to the mks of its references.
    other_views_[view->get_oid()] = view;
    cov(view);
    break;
  case Atom::OBJECT:
    other_views_[view->get_oid()] = view;
    cov(view);
    break;
  case Atom::COMPOSITE_STATE: {
    OUTPUT_LINE(HLP_INJ, Utils::RelativeTime(Now()) << " cst " << view->object_->get_oid() << " injected");
    ipgm_views_[view->get_oid()] = view;
    CSTController *c = new CSTController(view);
    view->controller_ = c;
    c->set_secondary_host(get_secondary_group());
    if (is_active_pgm(view)) {

      c->gain_activation();
      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
        c->_take_input(*v); // view will be copied.
    }
    break;
  }case Atom::MODEL: {
    OUTPUT_LINE(HLP_INJ, Utils::RelativeTime(Now()) << " mdl " << view->object_->get_oid() << " injected");
    ipgm_views_[view->get_oid()] = view;
    bool inject_in_secondary_group;
    MDLController *c = MDLController::New(view, inject_in_secondary_group);
    view->controller_ = c;
    if (inject_in_secondary_group)
      get_secondary_group()->inject_secondary_mdl_controller(view);
    if (is_active_pgm(view)) {

      c->gain_activation();
      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v)
        c->Controller::_take_input(*v); // view will be copied.
    }
    break;
  }
  }

  if (is_eligible_input(view)) { // have existing programs reduce the new view.

    newly_salient_views_.insert(view);
    inject_reduction_jobs(view);
  }

  leave();
  //if(get_oid()==2)
  // std::cout<<Utils::RelativeTime(Now())<<" stdin <- "<<view->object->get_oid()<<std::endl;
}

void Group::inject_new_object(View *view) { // the view can hold anything but groups and notifications.
//uint64 t0=Now();
  switch (view->object_->code(0).getDescriptor()) {
  case Atom::MARKER: // the marker does not exist yet: add it to the mks of its references.
    for (uint32 i = 0; i < view->object_->references_size(); ++i) {

      Code *ref = view->object_->get_reference(i);
      ref->acq_markers();
      ref->markers_.push_back(view->object_);
      ref->rel_markers();
    }
    break;
  default:
    break;
  }

  inject(view);
  notifyNew(view);
  //uint64 t1=Now();
  //std::cout<<"injection: "<<t1-t0<<std::endl;
}

void Group::inject_existing_object(View *view) { // the view can hold anything but groups and notifications.

  Code *object = view->object_;
  object->acq_views();
  View *existing_view = (View *)object->get_view(this, false);
  if (!existing_view) { // no existing view: add the view to the object's view_map and inject.

    object->views_.insert(view);
    object->rel_views();

    inject(view);
  } else { // call set on the ctrl values of the existing view with the new view's ctrl values, including sync. NB: org left unchanged.

    object->rel_views();

    enter();

    pending_operations_.push_back(new Group::Set(existing_view->get_oid(), VIEW_RES, view->get_res()));
    pending_operations_.push_back(new Group::Set(existing_view->get_oid(), VIEW_SLN, view->get_sln()));
    switch (object->code(0).getDescriptor()) {
    case Atom::INSTANTIATED_PROGRAM:
    case Atom::INSTANTIATED_ANTI_PROGRAM:
    case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
    case Atom::INSTANTIATED_CPP_PROGRAM:
    case Atom::COMPOSITE_STATE:
    case Atom::MODEL:
      pending_operations_.push_back(new Group::Set(existing_view->get_oid(), VIEW_ACT, view->get_act()));
      break;
    }

    existing_view->code(VIEW_SYNC) = view->code(VIEW_SYNC);
    existing_view->set_ijt(Now());

    bool wiew_is_salient = view->get_sln() > get_sln_thr();
    bool wiew_was_salient = existing_view->get_sln() > get_sln_thr();
    bool reduce_view = (!wiew_was_salient && wiew_is_salient);

    // give a chance to ipgms to reduce the new view.
    bool group_is_c_active = update_c_act() > get_c_act_thr();
    bool group_is_c_salient = update_c_sln() > get_c_sln_thr();
    if (group_is_c_active && group_is_c_salient && reduce_view) {

      newly_salient_views_.insert(view);
      inject_reduction_jobs(view);
    }

    leave();
  }
}

void Group::inject_group(View *view) { // the view holds a group.

  enter();

  group_views_[view->get_oid()] = view;

  if (get_c_sln() > get_c_sln_thr() && view->get_sln() > get_sln_thr()) { // group is c-salient and view is salient.

    if (view->get_vis() > get_vis_thr()) // new visible group in a c-active and c-salient host.
      ((Group *)view->object_)->viewing_groups_[this] = view->get_cov();

    inject_reduction_jobs(view);
  }

  leave();

  if (((Group *)view->object_)->get_upr() > 0) // inject the next update job for the group.
    _Mem::Get()->push_time_job(new UpdateJob((Group *)view->object_, ((Group *)view->object_)->get_next_upr_time(Now())));

  notifyNew(view);
}

void Group::inject_notification(View *view, bool lock) {

  if (lock)
    enter();

  notification_views_[view->get_oid()] = view;

  for (uint32 i = 0; i < view->object_->references_size(); ++i) {

    Code *ref = view->object_->get_reference(i);
    ref->acq_markers();
    ref->markers_.push_back(view->object_);
    ref->rel_markers();
  }

  view->set_ijt(Now());

  if (get_c_sln() > get_c_sln_thr() && view->get_sln() > get_sln_thr()) // group is c-salient and view is salient.
    inject_reduction_jobs(view);

  if (lock)
    leave();
}

void Group::inject_reduction_jobs(View *view) { // group is assumed to be c-salient; already protected.

  if (get_c_act() > get_c_act_thr()) { // host is c-active.

      // build reduction jobs from host's own inputs and own overlays.
    FOR_ALL_VIEWS_WITH_INPUTS_BEGIN(this, v)

      if (v->second->get_act() > get_act_thr())//{ // active ipgm/icpp_pgm/rgrp view.
        v->second->controller_->_take_input(view); // view will be copied.
        //std::cout<<std::hex<<(void *)v->second->controller<<std::dec<<" <- "<<view->object->get_oid()<<std::endl;}

    FOR_ALL_VIEWS_WITH_INPUTS_END
  }

  // build reduction jobs from host's own inputs and overlays from viewing groups, if no cov and view is not a notification.
  // NB: visibility is not transitive;
  // no shadowing: if a view already exists in the viewing group, there will be twice the reductions: all of the identicals will be trimmed down at injection time.
  UNORDERED_MAP<Group *, bool>::const_iterator vg;
  for (vg = viewing_groups_.begin(); vg != viewing_groups_.end(); ++vg) {

    if (vg->second || view->is_notification()) // no reduction jobs when cov==true or view is a notification.
      continue;

    FOR_ALL_VIEWS_WITH_INPUTS_BEGIN(vg->first, v)

      if (v->second->get_act() > vg->first->get_act_thr()) // active ipgm/icpp_pgm/rgrp view.
        v->second->controller_->_take_input(view); // view will be copied.

    FOR_ALL_VIEWS_WITH_INPUTS_END
  }
}

void Group::notifyNew(View *view) {

  if (get_ntf_new() == 1) {

    uint16 ntf_grp_count = get_ntf_grp_count();
    for (uint16 i = 1; i <= ntf_grp_count; ++i)
      _Mem::Get()->inject_notification(new NotificationView(this, get_ntf_grp(i), new MkNew(_Mem::Get(), view->object_)), get_ntf_grp(i) != this); // the object appears for the first time in the group: notify.
  }
}

void Group::cov(View *view) {

  UNORDERED_MAP<Group *, bool>::const_iterator vg;
  for (vg = viewing_groups_.begin(); vg != viewing_groups_.end(); ++vg) {

    if (vg->second) // cov==true, viewing group c-salient and c-active (otherwise it wouldn't be a viewing group).
      _Mem::Get()->inject_copy(view, vg->first);
  }
}

void Group::cov() {

  // cov, i.e. injecting now newly salient views in the viewing groups from which the group is visible and has cov.
  // reduction jobs will be added at each of the eligible viewing groups' own update time.
  UNORDERED_MAP<Group *, bool>::const_iterator vg;
  for (vg = viewing_groups_.begin(); vg != viewing_groups_.end(); ++vg) {

    if (vg->second) { // cov==true.

      std::multiset<P<View>, r_code::View::Less>::const_iterator v;
      for (v = newly_salient_views_.begin(); v != newly_salient_views_.end(); ++v) { // no cov for pgm (all sorts), groups, notifications.

        if ((*v)->is_notification())
          continue;
        switch ((*v)->object_->code(0).getDescriptor()) {
        case Atom::GROUP:
        case Atom::NULL_PROGRAM:
        case Atom::INSTANTIATED_PROGRAM:
        case Atom::INSTANTIATED_CPP_PROGRAM:
        case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
        case Atom::INSTANTIATED_ANTI_PROGRAM:
        case Atom::COMPOSITE_STATE:
        case Atom::MODEL:
          break;
        default:
          _Mem::Get()->inject_copy(*v, vg->first); // no need to protect group->newly_salient_views_[i] since the support values for the ctrl values are not even read.
          break;
        }
      }
    }
  }
}

void Group::delete_view(View *v) {

  if (v->is_notification())
    notification_views_.erase(v->get_oid());
  else switch (v->object_->code(0).getDescriptor()) {
  case Atom::NULL_PROGRAM:
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_CPP_PROGRAM:
  case Atom::COMPOSITE_STATE:
  case Atom::MODEL:
    ipgm_views_.erase(v->get_oid());
    break;
  case Atom::INSTANTIATED_ANTI_PROGRAM:
    anti_ipgm_views_.erase(v->get_oid());
    break;
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
    input_less_ipgm_views_.erase(v->get_oid());
    break;
  case Atom::OBJECT:
  case Atom::MARKER:
    other_views_.erase(v->get_oid());
    break;
  case Atom::GROUP:
    group_views_.erase(v->get_oid());
    break;
  }
}

void Group::delete_view(UNORDERED_MAP<uint32, P<View> >::const_iterator &v) {

  if (v->second->is_notification())
    v = notification_views_.erase(v);
  else switch (v->second->object_->code(0).getDescriptor()) {
  case Atom::NULL_PROGRAM:
  case Atom::INSTANTIATED_PROGRAM:
  case Atom::INSTANTIATED_CPP_PROGRAM:
  case Atom::COMPOSITE_STATE:
  case Atom::MODEL:
    v = ipgm_views_.erase(v);
    break;
  case Atom::INSTANTIATED_ANTI_PROGRAM:
    v = anti_ipgm_views_.erase(v);
    break;
  case Atom::INSTANTIATED_INPUT_LESS_PROGRAM:
    v = input_less_ipgm_views_.erase(v);
    break;
  case Atom::OBJECT:
  case Atom::MARKER:
    v = other_views_.erase(v);
    break;
  case Atom::GROUP:
    v = group_views_.erase(v);
    break;
  }
}

Group *Group::get_secondary_group() {

  Group *secondary = NULL;
  r_code::list<Code *>::const_iterator m;
  acq_markers();
  for (m = markers_.begin(); m != markers_.end(); ++m) {

    if ((*m)->code(0).asOpcode() == Opcodes::MkGrpPair) {

      if ((Group *)(*m)->get_reference((*m)->code(GRP_PAIR_FIRST).asIndex()) == this) {

        secondary = (Group *)(*m)->get_reference((*m)->code(GRP_PAIR_SECOND).asIndex());
        break;
      }
    }
  }
  rel_markers();
  return secondary;
}

void Group::load_secondary_mdl_controller(View *view) {

  PrimaryMDLController *p = (PrimaryMDLController *)view->controller_;
  View *_view = new View(view, true);
  _view->code(VIEW_ACT) = Atom::Float(0);
  _view->references_[0] = this;
  ipgm_views_[_view->get_oid()] = _view;
  SecondaryMDLController *s = new SecondaryMDLController(_view);
  _view->controller_ = s;
  view->object_->views_.insert(_view);
  p->set_secondary(s);
  s->set_primary(p);
}

void Group::inject_secondary_mdl_controller(View *view) {

  PrimaryMDLController *p = (PrimaryMDLController *)view->controller_;
  View *_view = new View(view, true);
  _view->code(VIEW_ACT) = Atom::Float(0);
  _view->references_[0] = this;
  ipgm_views_[_view->get_oid()] = _view;
  SecondaryMDLController *s = new SecondaryMDLController(_view);
  _view->controller_ = s;
  view->object_->views_.insert(_view);
  p->set_secondary(s);
  s->set_primary(p);
}

Timestamp Group::get_next_upr_time(Timestamp now) const {

  uint32 __upr = get_upr();
  if (__upr == 0)
    return Utils_MaxTime;
  auto _upr = __upr * Utils::GetBasePeriod();
  auto delta = (now - Utils::GetTimeReference()) % _upr;
  return now - delta + _upr;
}

Timestamp Group::get_prev_upr_time(Timestamp now) const {

  uint32 __upr = get_upr();
  if (__upr == 0)
    return Utils_MaxTime;
  auto _upr = __upr * Utils::GetBasePeriod();
  auto delta = (now - Utils::GetTimeReference()) % _upr;
  return now - delta;
}
}
