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

#include "../r_code/utils.h"
#include "opcodes.h"


namespace r_exec {

inline View::View() : r_code::View(), controller_(NULL) {

  code_[VIEW_OID].atom_ = GetOID();
  reset_ctrl_values();
}

inline View::View(r_code::SysView *source, r_code::Code *object) : r_code::View(source, object), controller_(NULL) {

  code_[VIEW_OID].atom_ = GetOID();
  reset();
}

inline View::View(const View *view, bool new_OID) : r_code::View(), controller_(NULL) {

  object_ = view->object_;
  memcpy(code_, view->code_, VIEW_CODE_MAX_SIZE * sizeof(Atom) + 2 * sizeof(r_code::Code *)); // reference_set is contiguous to code; memcpy in one go.
  if (new_OID)
    code_[VIEW_OID].atom_ = GetOID();
  controller_ = NULL; // deprecated: controller=view->controller;
  reset();
}

inline View::View(SyncMode sync,
  Timestamp ijt,
  float32 sln,
  int32 res,
  r_code::Code *destination,
  r_code::Code *origin,
  r_code::Code *object) : r_code::View(), controller_(NULL) {

  code(VIEW_OPCODE) = Atom::SSet(Opcodes::View, VIEW_ARITY);
  init(sync, ijt, sln, res, destination, origin, object);
}

inline View::View(SyncMode sync,
  Timestamp ijt,
  float32 sln,
  int32 res,
  r_code::Code *destination,
  r_code::Code *origin,
  r_code::Code *object,
  float32 act) : r_code::View(), controller_(NULL) {

  code(VIEW_OPCODE) = Atom::SSet(Opcodes::PgmView, PGM_VIEW_ARITY);
  init(sync, ijt, sln, res, destination, origin, object);
  code(VIEW_ACT) = Atom::Float(act);
}

inline void View::init(SyncMode sync,
  Timestamp ijt,
  float32 sln,
  int32 res,
  r_code::Code *destination,
  r_code::Code *origin,
  r_code::Code *object) {

  code_[VIEW_OID].atom_ = GetOID();
  reset_ctrl_values();

  code(VIEW_SYNC) = Atom::Float(sync);
  code(VIEW_IJT) = Atom::IPointer(code(VIEW_OPCODE).getAtomCount() + 1);
  r_code::Utils::SetTimestamp<View>(this, VIEW_IJT, ijt);
  code(VIEW_SLN) = Atom::Float(sln);
  code(VIEW_RES) = res < 0 ? Atom::PlusInfinity() : Atom::Float(res);
  code(VIEW_HOST) = Atom::RPointer(0);
  code(VIEW_ORG) = origin ? Atom::RPointer(1) : Atom::Nil();

  references_[0] = destination;
  references_[1] = origin;

  set_object(object);
}

inline View::~View() {

  if (!!controller_)
    controller_->invalidate();
}

inline void View::reset() {

  reset_ctrl_values();
  reset_init_sln();
  reset_init_act();
}

inline uint32 View::get_oid() const {

  return code_[VIEW_OID].atom_;
}

inline bool View::isNotification() const {

  return false;
}

inline Group *View::get_host() {

  uint32 host_reference = code(VIEW_HOST).asIndex();
  return (Group *)references_[host_reference];
}

inline View::SyncMode View::get_sync() {

  return (SyncMode)(uint32)code(VIEW_SYNC).asFloat();
}

inline float32 View::get_res() {

  return code(VIEW_RES).asFloat();
}

inline float32 View::get_sln() {

  return code(VIEW_SLN).asFloat();
}

inline float32 View::get_act() {

  return code(VIEW_ACT).asFloat();
}

inline float32 View::get_vis() {

  return code(GRP_VIEW_VIS).asFloat();
}

inline bool View::get_cov() {

  if (object_->code(0).getDescriptor() == Atom::GROUP)
    return code(GRP_VIEW_COV).asBoolean();
  return false;
}

inline void View::mod_res(float32 value) {

  if (code(VIEW_RES) == Atom::PlusInfinity())
    return;
  acc_res_ += value;
  ++res_changes_;
}

inline void View::set_res(float32 value) {

  if (code(VIEW_RES) == Atom::PlusInfinity())
    return;
  acc_res_ += value - get_res();
  ++res_changes_;
}

inline void View::mod_sln(float32 value) {

  acc_sln_ += value;
  ++sln_changes_;
}

inline void View::set_sln(float32 value) {

  acc_sln_ += value - get_sln();
  ++sln_changes_;
}

inline void View::mod_act(float32 value) {

  acc_act_ += value;
  ++act_changes_;
}

inline void View::set_act(float32 value) {

  acc_act_ += value - get_act();
  ++act_changes_;
}

inline void View::mod_vis(float32 value) {

  acc_vis_ += value;
  ++vis_changes_;
}

inline void View::set_vis(float32 value) {

  acc_vis_ += value - get_vis();
  ++vis_changes_;
}

inline float32 View::update_sln_delta() {

  float32 delta = get_sln() - initial_sln_;
  initial_sln_ = get_sln();
  return delta;
}

inline float32 View::update_act_delta() {

  float32 act = get_act();
  float32 delta = act - initial_act_;
  initial_act_ = act;
  return delta;
}

inline void View::force_res(float32 value) {

  code(VIEW_RES) = Atom::Float(value);
}

inline void View::mod(uint16 member_index, float32 value) {

  switch (member_index) {
  case VIEW_SLN:
    mod_sln(value);
    break;
  case VIEW_RES:
    mod_res(value);
    break;
  case VIEW_ACT:
    mod_act(value);
    break;
  case GRP_VIEW_VIS:
    mod_vis(value);
    break;
  }
}

inline void View::set(uint16 member_index, float32 value) {

  switch (member_index) {
  case VIEW_SLN:
    set_sln(value);
    break;
  case VIEW_RES:
    set_res(value);
    break;
  case VIEW_ACT:
    set_act(value);
    break;
  case GRP_VIEW_VIS:
    set_vis(value);
    break;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

inline bool NotificationView::isNotification() const {

  return true;
}
}
