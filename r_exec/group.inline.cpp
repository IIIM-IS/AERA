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

namespace r_exec {

inline Group::Group(r_code::Mem *m) : LObject(m), CriticalSection() {

  reset_ctrl_values();
  reset_stats();
  reset_decay_values();
}

inline Group::Group(r_code::SysObject *source) : LObject(source), CriticalSection() {

  reset_ctrl_values();
  reset_stats();
  reset_decay_values();
}

inline Group::~Group() {

  invalidate();
}

inline bool Group::invalidate() {

  if (LObject::invalidate())
    return true;

  // unregister from all groups it views.
  std::unordered_map<uint32, P<View> >::const_iterator gv;
  for (gv = group_views_.begin(); gv != group_views_.end(); ++gv) {

    ((Group *)gv->second->object_)->enter();
    ((Group *)gv->second->object_)->viewing_groups_.erase(this);
    ((Group *)gv->second->object_)->leave();
  }
  /* We keep the group intact: the only thing is now the group will not be updated anymore.
          // remove all views that are hosted by this group.
          FOR_ALL_VIEWS_BEGIN(this,v)

              v->second->object->acq_views();
              v->second->object->views_.erase(v->second); // delete view from object's views.
              v->second->object->rel_views();

          FOR_ALL_VIEWS_END

          notification_views_.clear();
          ipgm_views_.clear();
          anti_ipgm_views_.clear();
          input_less_ipgm_views_.clear();
          other_views_.clear();
          group_views_.clear();
  */
  return false;
}

inline uint32 Group::get_upr() const {

  return (uint32)code(GRP_UPR).asFloat();
}

inline float32 Group::get_sln_thr() const {

  return code(GRP_SLN_THR).asFloat();
}

inline float32 Group::get_act_thr() const {

  return code(GRP_ACT_THR).asFloat();
}

inline float32 Group::get_vis_thr() const {

  return code(GRP_VIS_THR).asFloat();
}

inline float32 Group::get_c_sln_thr() const {

  return code(GRP_C_SLN_THR).asFloat();
}

inline float32 Group::get_c_act_thr() const {

  return code(GRP_C_ACT_THR).asFloat();
}

inline float32 Group::get_c_sln() const {

  return code(GRP_C_SLN).asFloat();
}

inline float32 Group::get_c_act() const {

  return code(GRP_C_ACT).asFloat();
}

inline void Group::mod_sln_thr(float32 value) {

  ++sln_thr_changes_;
  acc_sln_thr_ += value;
}

inline void Group::set_sln_thr(float32 value) {

  ++sln_thr_changes_;
  acc_sln_thr_ += value - get_sln_thr();
}

inline void Group::mod_act_thr(float32 value) {

  ++act_thr_changes_;
  acc_act_thr_ += value;
}

inline void Group::set_act_thr(float32 value) {

  ++act_thr_changes_;
  acc_act_thr_ += value - get_act_thr();
}

inline void Group::mod_vis_thr(float32 value) {

  ++vis_thr_changes_;
  acc_vis_thr_ += value;
}

inline void Group::set_vis_thr(float32 value) {

  ++vis_thr_changes_;
  acc_vis_thr_ += value - get_vis_thr();
}

inline void Group::mod_c_sln(float32 value) {

  ++c_sln_changes_;
  acc_c_sln_ += value;
}

inline void Group::set_c_sln(float32 value) {

  ++c_sln_changes_;
  acc_c_sln_ += value - get_c_sln();
}

inline void Group::mod_c_act(float32 value) {

  ++c_act_changes_;
  acc_c_act_ += value;
}

inline void Group::set_c_act(float32 value) {

  ++c_act_changes_;
  acc_c_act_ += value - get_c_act();
}

inline void Group::mod_c_sln_thr(float32 value) {

  ++c_sln_thr_changes_;
  acc_c_sln_thr_ += value;
}

inline void Group::set_c_sln_thr(float32 value) {

  ++c_sln_thr_changes_;
  acc_c_sln_thr_ += value - get_c_sln_thr();
}

inline void Group::mod_c_act_thr(float32 value) {

  ++c_act_thr_changes_;
  acc_c_act_thr_ += value;
}

inline void Group::set_c_act_thr(float32 value) {

  ++c_act_thr_changes_;
  acc_c_act_thr_ += value - get_c_act_thr();
}

inline float32 Group::get_sln_chg_thr() {

  return code(GRP_SLN_CHG_THR).asFloat();
}

inline float32 Group::get_sln_chg_prd() {

  return code(GRP_SLN_CHG_PRD).asFloat();
}

inline float32 Group::get_act_chg_thr() {

  return code(GRP_ACT_CHG_THR).asFloat();
}

inline float32 Group::get_act_chg_prd() {

  return code(GRP_ACT_CHG_PRD).asFloat();
}

inline float32 Group::get_avg_sln() {

  return code(GRP_AVG_SLN).asFloat();
}

inline float32 Group::get_high_sln() {

  return code(GRP_HIGH_SLN).asFloat();
}

inline float32 Group::get_low_sln() {

  return code(GRP_LOW_SLN).asFloat();
}

inline float32 Group::get_avg_act() {

  return code(GRP_AVG_ACT).asFloat();
}

inline float32 Group::get_high_act() {

  return code(GRP_HIGH_ACT).asFloat();
}

inline float32 Group::get_low_act() {

  return code(GRP_LOW_ACT).asFloat();
}

inline float32 Group::get_high_sln_thr() {

  return code(GRP_HIGH_SLN_THR).asFloat();
}

inline float32 Group::get_low_sln_thr() {

  return code(GRP_LOW_SLN_THR).asFloat();
}

inline float32 Group::get_sln_ntf_prd() {

  return code(GRP_SLN_NTF_PRD).asFloat();
}

inline float32 Group::get_high_act_thr() {

  return code(GRP_HIGH_ACT_THR).asFloat();
}

inline float32 Group::get_low_act_thr() {

  return code(GRP_LOW_ACT_THR).asFloat();
}

inline float32 Group::get_act_ntf_prd() {

  return code(GRP_ACT_NTF_PRD).asFloat();
}

inline float32 Group::get_low_res_thr() {

  return code(GRP_LOW_RES_THR).asFloat();
}

inline float32 Group::get_ntf_new() {

  return code(GRP_NTF_NEW).asFloat();
}

inline uint16 Group::get_ntf_grp_count() {

  return code(code(GRP_NTF_GRPS).asIndex()).getAtomCount();
}

inline Group *Group::get_ntf_grp(uint16 i) {

  if (code(code(GRP_NTF_GRPS).asIndex() + i).readsAsNil())
    return this;

  uint16 index = code(code(GRP_NTF_GRPS).asIndex() + i).asIndex();
  return (Group *)get_reference(index);
}

inline void Group::_mod_0_positive(uint16 member_index, float32 value) {

  float32 v = code(member_index).asFloat() + value;
  if (v < 0)
    v = 0;
  code(member_index) = Atom::Float(v);
}

inline void Group::_mod_0_plus1(uint16 member_index, float32 value) {

  float32 v = code(member_index).asFloat() + value;
  if (v < 0)
    v = 0;
  else if (v > 1)
    v = 1;
  code(member_index) = Atom::Float(v);
}

inline void Group::_mod_minus1_plus1(uint16 member_index, float32 value) {

  float32 v = code(member_index).asFloat() + value;
  if (v < -1)
    v = -1;
  else if (v > 1)
    v = 1;
  code(member_index) = Atom::Float(v);
}

inline void Group::_set_0_positive(uint16 member_index, float32 value) {

  if (value < 0)
    code(member_index) = Atom::Float(0);
  else
    code(member_index) = Atom::Float(value);
}

inline void Group::_set_0_plus1(uint16 member_index, float32 value) {

  if (value < 0)
    code(member_index) = Atom::Float(0);
  else if (value > 1)
    code(member_index) = Atom::Float(1);
  else
    code(member_index) = Atom::Float(value);
}

inline void Group::_set_minus1_plus1(uint16 member_index, float32 value) {

  if (value < -1)
    code(member_index) = Atom::Float(-1);
  else if (value > 1)
    code(member_index) = Atom::Float(1);
  else
    code(member_index) = Atom::Float(value);
}

inline void Group::_set_0_1(uint16 member_index, float32 value) {

  if (value == 0 || value == 1)
    code(member_index) = Atom::Float(value);
}

inline void Group::mod(uint16 member_index, float32 value) {

  switch (member_index) {
  case GRP_UPR:
  case GRP_DCY_PRD:
  case GRP_SLN_CHG_PRD:
  case GRP_ACT_CHG_PRD:
  case GRP_SLN_NTF_PRD:
  case GRP_ACT_NTF_PRD:
    _mod_0_positive(member_index, value);
    return;
  case GRP_SLN_THR:
    mod_sln_thr(value);
    return;
  case GRP_ACT_THR:
    mod_act_thr(value);
    return;
  case GRP_VIS_THR:
    mod_vis_thr(value);
    return;
  case GRP_C_SLN:
    mod_c_sln(value);
    return;
  case GRP_C_SLN_THR:
    mod_c_sln_thr(value);
    return;
  case GRP_C_ACT:
    mod_c_act(value);
    return;
  case GRP_C_ACT_THR:
    mod_c_act_thr(value);
    return;
  case GRP_DCY_PER:
    _mod_minus1_plus1(member_index, value);
    return;
  case GRP_SLN_CHG_THR:
  case GRP_ACT_CHG_THR:
  case GRP_HIGH_SLN_THR:
  case GRP_LOW_SLN_THR:
  case GRP_HIGH_ACT_THR:
  case GRP_LOW_ACT_THR:
  case GRP_LOW_RES_THR:
    _mod_0_plus1(member_index, value);
    return;
  }
}

inline void Group::set(uint16 member_index, float32 value) {

  switch (member_index) {
  case GRP_UPR:
  case GRP_DCY_PRD:
  case GRP_SLN_CHG_PRD:
  case GRP_ACT_CHG_PRD:
  case GRP_SLN_NTF_PRD:
  case GRP_ACT_NTF_PRD:
    _set_0_positive(member_index, value);
    return;
  case GRP_SLN_THR:
    set_sln_thr(value);
    return;
  case GRP_ACT_THR:
    set_act_thr(value);
    return;
  case GRP_VIS_THR:
    set_vis_thr(value);
    return;
  case GRP_C_SLN:
    set_c_sln(value);
    return;
  case GRP_C_SLN_THR:
    set_c_sln_thr(value);
    return;
  case GRP_C_ACT:
    set_c_act(value);
    return;
  case GRP_C_ACT_THR:
    set_c_act_thr(value);
    return;
  case GRP_DCY_PER:
    _set_minus1_plus1(member_index, value);
    return;
  case GRP_SLN_CHG_THR:
  case GRP_ACT_CHG_THR:
  case GRP_HIGH_SLN_THR:
  case GRP_LOW_SLN_THR:
  case GRP_HIGH_ACT_THR:
  case GRP_LOW_ACT_THR:
  case GRP_LOW_RES_THR:
    _set_0_plus1(member_index, value);
    return;
  case GRP_NTF_NEW:
  case GRP_DCY_TGT:
  case GRP_DCY_AUTO:
    _set_0_1(member_index, value);
    return;
  }
}
}
