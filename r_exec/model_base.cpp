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

#include "model_base.h"
#include "mem.h"

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

uint32 ModelBase::MEntry::_ComputeHashCode(_Fact *component) { // 14 bits: [fact or |fact (1)|type (3)|data (10)].

  uint32 hash_code;
  if (component->is_fact())
    hash_code = 0x00000000;
  else
    hash_code = 0x00002000;
  Code *payload = component->get_reference(0);
  Atom head = payload->code(0);
  uint16 opcode = head.asOpcode();
  switch (head.getDescriptor()) {
  case Atom::OBJECT:
    if (opcode == Opcodes::Cmd) { // type: 2.

      hash_code |= 0x00000800;
      hash_code |= (payload->code(CMD_FUNCTION).asOpcode() & 0x000003FF); // data: function id.
    } else if (opcode == Opcodes::IMdl) { // type 3.

      hash_code |= 0x00000C00;
      hash_code |= (((uint32)payload->get_reference(0)) & 0x000003FF); // data: address of the mdl.
    } else if (opcode == Opcodes::ICst) { // type 4.

      hash_code |= 0x00001000;
      hash_code |= (((uint32)payload->get_reference(0)) & 0x000003FF); // data: address of the cst.
    } else // type: 0.
      hash_code |= (opcode & 0x000003FF); // data: class id.
    break;
  case Atom::MARKER: // type 1.
    hash_code |= 0x00000400;
    hash_code |= (opcode & 0x000003FF); // data: class id.
    break;
  }

  return hash_code;
}

uint32 ModelBase::MEntry::ComputeHashCode(Code *mdl, bool packed) {

  uint32 hash_code = (mdl->code(mdl->code(HLP_TPL_ARGS).asIndex()).getAtomCount() << 28);
  _Fact *lhs;
  _Fact *rhs;
  if (packed) {

    Code *unpacked_mdl = mdl->get_reference(mdl->references_size() - MDL_HIDDEN_REFS);
    lhs = (_Fact *)unpacked_mdl->get_reference(0);
    rhs = (_Fact *)unpacked_mdl->get_reference(1);
  } else {

    lhs = (_Fact *)mdl->get_reference(0);
    rhs = (_Fact *)mdl->get_reference(1);
  }
  hash_code |= (_ComputeHashCode(lhs) << 14);
  hash_code |= _ComputeHashCode(rhs);
  return hash_code;
}

bool ModelBase::MEntry::Match(Code *lhs, Code *rhs) {

  if (lhs->code(0).asOpcode() == Opcodes::Ent || rhs->code(0).asOpcode() == Opcodes::Ent)
    return lhs == rhs;
  if (lhs->code(0).asOpcode() == Opcodes::Ont || rhs->code(0).asOpcode() == Opcodes::Ont)
    return lhs == rhs;
  if (lhs->code_size() != rhs->code_size())
    return false;
  if (lhs->references_size() != rhs->references_size())
    return false;
  for (uint16 i = 0; i < lhs->code_size(); ++i) {

    if (lhs->code(i) != rhs->code(i))
      return false;
  }
  for (uint16 i = 0; i < lhs->references_size(); ++i) {

    if (!Match(lhs->get_reference(i), rhs->get_reference(i)))
      return false;
  }
  return true;
}

ModelBase::MEntry::MEntry() : mdl_(NULL), packed_(false), touch_time_(seconds(0)), hash_code_(0) {
}

ModelBase::MEntry::MEntry(Code *mdl, bool packed) : mdl_(mdl), packed_(packed), touch_time_(Now()), hash_code_(ComputeHashCode(mdl, packed)) {
}

bool ModelBase::MEntry::match(const MEntry &e) const { // at this point both models have the same hash code.

  if (mdl_ == e.mdl_)
    return true;

  // Get the unpacked models.
  Code* mdl_0 = (packed_ ? mdl_->get_reference(mdl_->references_size() - MDL_HIDDEN_REFS) : mdl_);
  Code* mdl_1 = (e.packed_ ? e.mdl_->get_reference(e.mdl_->references_size() - MDL_HIDDEN_REFS) : e.mdl_);

  if (mdl_0->code_size() != mdl_1->code_size())
    return false;
  for (uint16 i = 0; i < mdl_0->code_size(); ++i) { // first check the mdl code: this checks on tpl args and guards.

    if (i == MDL_STRENGTH || i == MDL_CNT || i == MDL_SR || i == MDL_DSR || i == MDL_ARITY) // ignore house keeping data.
      continue;

    if (mdl_0->code(i) != mdl_1->code(i))
      return false;
  }

  Code* f_lhs_0 = mdl_0->get_reference(0);
  Code* f_lhs_1 = mdl_1->get_reference(0);
  if (f_lhs_0->references_size() < 1)
    return false;
  if (f_lhs_1->references_size() < 1)
    return false;
  // Make sure we don't match fact with anti-fact.
  if (f_lhs_0->code(0) != f_lhs_1->code(0))
    return false;
  // Match the payloads of the facts.
  if (!Match(f_lhs_0->get_reference(0), f_lhs_1->get_reference(0)))
    return false;

  Code* f_rhs_0 = mdl_0->get_reference(1);
  Code* f_rhs_1 = mdl_1->get_reference(1);
  // Make sure we don't match fact with anti-fact.
  if (f_rhs_0->code(0) != f_rhs_1->code(0))
    return false;
  // Match the payloads of the facts.
  if (!Match(f_rhs_0->get_reference(0), f_rhs_1->get_reference(0)))
    return false;

  return true;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ModelBase *ModelBase::singleton_ = NULL;

ModelBase *ModelBase::Get() {

  return singleton_;
}

ModelBase::ModelBase() {

  singleton_ = this;
}

void ModelBase::trim_objects() {

  mdlCS_.enter();
  auto now = Now();
  MdlSet::iterator m;
  for (m = black_list_.begin(); m != black_list_.end();) {

    if (now - (*m).touch_time_ >= thz_)
      m = black_list_.erase(m);
    else
      ++m;
  }
  mdlCS_.leave();
}

void ModelBase::load(Code *mdl) { // mdl is already packed.

  MEntry e(mdl, true);
  if (mdl->views_.size() > 0) // no need to lock at load time.
    white_list_.insert(e);
  else
    black_list_.insert(e);
}

void ModelBase::get_models(r_code::list<P<Code> > &models) {

  mdlCS_.enter();
  MdlSet::iterator m;
  for (m = white_list_.begin(); m != white_list_.end(); ++m)
    models.push_back((*m).mdl_);
  for (m = black_list_.begin(); m != black_list_.end(); ++m)
    models.push_back((*m).mdl_);
  mdlCS_.leave();
}

Code *ModelBase::check_existence(Code *mdl) {

  MEntry e(mdl, false);
  MEntry copy;
  mdlCS_.enter();
  MdlSet::iterator m = black_list_.find(e);

  // jm: iterator's on sets are now immutable because of hash keey issues
  // solution is to remove and re-add
  if (m != black_list_.end()) {

    copy = *m;
    copy.touch_time_ = Now();
    black_list_.erase(*m);
    black_list_.insert(copy);

    //(*m).touch_time_=Now();
    mdlCS_.leave();
    return NULL;
  }
  m = white_list_.find(e);
  if (m != white_list_.end())
  {
    copy = *m;
    copy.touch_time_ = Now();
    white_list_.erase(*m);
    white_list_.insert(copy);

    //(*m).touch_time_=Now();   // jm
    mdlCS_.leave();
    return copy.mdl_;
  }
  _Mem::Get()->pack_hlp(e.mdl_);
  e.packed_ = true;
  white_list_.insert(e);
  mdlCS_.leave();
  return mdl;
}

void ModelBase::check_existence(Code *m0, Code *m1, Code *&_m0, Code *&_m1) { // m0 and m1 unpacked.

  MEntry e_m0(m0, false);
  MEntry copy;
  mdlCS_.enter();
  MdlSet::iterator m = black_list_.find(e_m0);
  if (m != black_list_.end())
  {
    copy = *m;
    copy.touch_time_ = Now();
    black_list_.erase(*m);
    black_list_.insert(copy);

    // (*m).touch_time_=Now();  jm
    mdlCS_.leave();
    _m0 = _m1 = NULL;
    return;
  }
  m = white_list_.find(e_m0);
  if (m != white_list_.end()) {

    copy = *m;
    copy.touch_time_ = Now();
    white_list_.erase(*m);
    white_list_.insert(copy);

    //(*m).touch_time_=Now();  //jm
    _m0 = copy.mdl_;
    Code *rhs = m1->get_reference(m1->code(m1->code(MDL_OBJS).asIndex() + 2).asIndex());
    Code *im0 = rhs->get_reference(0);
    im0->set_reference(0, _m0); // change imdl m0 into imdl _m0.
  } else
    _m0 = m0;
  MEntry e_m1(m1, false);
  m = black_list_.find(e_m1);
  if (m != black_list_.end()) {
    copy = *m;
    copy.touch_time_ = Now();
    black_list_.erase(*m);
    black_list_.insert(copy);

    //(*m).touch_time_=Now();
    mdlCS_.leave();
    _m1 = NULL;
    return;
  }
  m = white_list_.find(e_m1);
  if (m != white_list_.end()) {
    copy = *m;
    copy.touch_time_ = Now();
    white_list_.erase(*m);
    white_list_.insert(copy);

    //(*m).touch_time_=Now();  //jm
    mdlCS_.leave();
    _m1 = copy.mdl_;
    return;
  }
  if (_m0 == m0) {

    _Mem::Get()->pack_hlp(m0);
    e_m0.packed_ = true;
    white_list_.insert(e_m0);
  }
  _Mem::Get()->pack_hlp(m1);
  e_m1.packed_ = true;
  white_list_.insert(e_m1);
  mdlCS_.leave();
  _m1 = m1;
}

void ModelBase::register_mdl_failure(Code *mdl) { // mdl is packed.

  MEntry e(mdl, true);
  mdlCS_.enter();
  white_list_.erase(e);
  black_list_.insert(e);
  mdlCS_.leave();
}

void ModelBase::register_mdl_timeout(Code *mdl) { // mdl is packed.

  MEntry e(mdl, true);
  mdlCS_.enter();
  white_list_.erase(e);
  mdlCS_.leave();
}
}
