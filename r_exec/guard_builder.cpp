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

#include "guard_builder.h"

using namespace std::chrono;

namespace r_exec {

GuardBuilder::GuardBuilder() : _Object() {
}

GuardBuilder::~GuardBuilder() {
}

void GuardBuilder::build(Code *mdl, _Fact *premise, _Fact *cause, uint16 &write_index) const {

  mdl->code(MDL_FWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(0);

  mdl->code(MDL_BWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TimingGuardBuilder::TimingGuardBuilder(microseconds period) : GuardBuilder(), period(period) {
}

TimingGuardBuilder::~TimingGuardBuilder() {
}

void TimingGuardBuilder::write_guard(Code *mdl, uint16 l, uint16 r, uint16 opcode, microseconds offset, uint16 &write_index, uint16 &extent_index) const {

  mdl->code(++write_index) = Atom::AssignmentPointer(l, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(opcode, 2); // l:(opcode r offset)
  mdl->code(++extent_index) = Atom::VLPointer(r);
  mdl->code(++extent_index) = Atom::IPointer(extent_index + 1);
  Utils::SetTimestamp(mdl, ++extent_index, Timestamp(offset));
  extent_index += 2;
}

void TimingGuardBuilder::_build(Code *mdl, uint16 t0, uint16 t1, uint16 &write_index) const {

  Code *rhs_val = mdl->get_reference(1);
  uint16 t2 = rhs_val->code(FACT_AFTER).asIndex();
  uint16 t3 = rhs_val->code(FACT_BEFORE).asIndex();

  mdl->code(MDL_FWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(2);

  uint16 extent_index = write_index + 2;

  write_guard(mdl, t2, t0, Opcodes::Add, period, write_index, extent_index);
  write_guard(mdl, t3, t1, Opcodes::Add, period, write_index, extent_index);

  write_index = extent_index;
  mdl->code(MDL_BWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(2);

  extent_index = write_index + 2;

  write_guard(mdl, t0, t2, Opcodes::Sub, period, write_index, extent_index);
  write_guard(mdl, t1, t3, Opcodes::Sub, period, write_index, extent_index);

  write_index = extent_index;
}

void TimingGuardBuilder::build(Code *mdl, _Fact *premise, _Fact *cause, uint16 &write_index) const {

  uint16 t0;
  uint16 t1;
  uint16 tpl_arg_set_index = mdl->code(MDL_TPL_ARGS).asIndex();
  if (mdl->code(tpl_arg_set_index).getAtomCount() == 0) {

    Code *lhs_val = mdl->get_reference(0);
    t0 = lhs_val->code(FACT_AFTER).asIndex();
    t1 = lhs_val->code(FACT_BEFORE).asIndex();
  } else { // use the tpl args.

    t0 = 1;
    t1 = 2;
  }

  _build(mdl, t0, t1, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

SGuardBuilder::SGuardBuilder(microseconds period, microseconds offset) : TimingGuardBuilder(period), offset(offset) {
}

SGuardBuilder::~SGuardBuilder() {
}

void SGuardBuilder::_build(Code *mdl, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const {

  Code *rhs = mdl->get_reference(1);
  uint16 t2 = rhs->code(FACT_AFTER).asIndex();
  uint16 t3 = rhs->code(FACT_BEFORE).asIndex();
  uint16 q1 = rhs->get_reference(0)->code(MK_VAL_VALUE).asIndex();

  Code *lhs = mdl->get_reference(0);
  uint16 speed_t0 = lhs->code(FACT_AFTER).asIndex();
  uint16 speed_t1 = lhs->code(FACT_BEFORE).asIndex();
  uint16 speed_value = lhs->get_reference(0)->code(MK_VAL_VALUE).asIndex();

  mdl->code(MDL_FWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(3);

  uint16 extent_index = write_index + 3;

  write_guard(mdl, t2, t0, Opcodes::Add, period, write_index, extent_index);
  write_guard(mdl, t3, t1, Opcodes::Add, period, write_index, extent_index);

  mdl->code(++write_index) = Atom::AssignmentPointer(q1, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(Opcodes::Add, 2); // q1:(+ q0 (* s period))
  mdl->code(++extent_index) = Atom::VLPointer(q0);
  mdl->code(++extent_index) = Atom::IPointer(extent_index + 1);
  mdl->code(++extent_index) = Atom::Operator(Opcodes::Mul, 2);
  mdl->code(++extent_index) = Atom::VLPointer(speed_value);
  mdl->code(++extent_index) = Atom::IPointer(extent_index + 1);
  Utils::SetTimestamp(mdl, ++extent_index, Timestamp(period));
  extent_index += 2;

  write_index = extent_index;
  mdl->code(MDL_BWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(5);

  extent_index = write_index + 5;

  write_guard(mdl, t0, t2, Opcodes::Sub, period, write_index, extent_index);
  write_guard(mdl, t1, t3, Opcodes::Sub, period, write_index, extent_index);

  write_guard(mdl, speed_t0, t2, Opcodes::Sub, offset, write_index, extent_index);
  write_guard(mdl, speed_t1, t3, Opcodes::Sub, offset, write_index, extent_index);

  mdl->code(++write_index) = Atom::AssignmentPointer(speed_value, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(Opcodes::Div, 2); // s:(/ (- q1 q0) period)
  mdl->code(++extent_index) = Atom::IPointer(extent_index + 2);
  mdl->code(++extent_index) = Atom::IPointer(extent_index + 4);
  mdl->code(++extent_index) = Atom::Operator(Opcodes::Sub, 2);
  mdl->code(++extent_index) = Atom::VLPointer(q1);
  mdl->code(++extent_index) = Atom::VLPointer(q0);
  Utils::SetTimestamp(mdl, ++extent_index, Timestamp(period));
  extent_index += 2;

  write_index = extent_index;
}

void SGuardBuilder::build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  uint16 q0;
  uint16 t0;
  uint16 t1;
  uint16 tpl_arg_set_index = mdl->code(MDL_TPL_ARGS).asIndex();
  if (mdl->code(tpl_arg_set_index).getAtomCount() == 0) {

    q0 = premise_pattern->get_reference(0)->code(MK_VAL_VALUE).asIndex();
    t0 = premise_pattern->code(FACT_AFTER).asIndex();
    t1 = premise_pattern->code(FACT_BEFORE).asIndex();
  } else { // use the tpl args.

    q0 = 0;
    t0 = 1;
    t1 = 2;
  }

  _build(mdl, q0, t0, t1, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

NoArgCmdGuardBuilder::NoArgCmdGuardBuilder(microseconds period, microseconds offset, microseconds cmd_duration) : TimingGuardBuilder(period), offset(offset), cmd_duration(cmd_duration) {
}

NoArgCmdGuardBuilder::~NoArgCmdGuardBuilder() {
}

void NoArgCmdGuardBuilder::_build(Code *mdl, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const {

  Code *rhs = mdl->get_reference(1);
  uint16 t2 = rhs->code(FACT_AFTER).asIndex();
  uint16 t3 = rhs->code(FACT_BEFORE).asIndex();

  Code *lhs = mdl->get_reference(0);
  uint16 cmd_t0 = lhs->code(FACT_AFTER).asIndex();
  uint16 cmd_t1 = lhs->code(FACT_BEFORE).asIndex();

  mdl->code(MDL_FWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(2);

  uint16 extent_index = write_index + 2;

  write_guard(mdl, t2, t0, Opcodes::Add, period, write_index, extent_index);
  write_guard(mdl, t3, t1, Opcodes::Add, period, write_index, extent_index);

  write_index = extent_index;
  mdl->code(MDL_BWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(4);

  extent_index = write_index + 4;

  write_guard(mdl, t0, t2, Opcodes::Sub, period, write_index, extent_index);
  write_guard(mdl, t1, t3, Opcodes::Sub, period, write_index, extent_index);

  write_guard(mdl, cmd_t0, t2, Opcodes::Sub, offset, write_index, extent_index);
  write_guard(mdl, cmd_t1, cmd_t0, Opcodes::Add, cmd_duration, write_index, extent_index);

  write_index = extent_index;
}

void NoArgCmdGuardBuilder::build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  uint16 q0;
  uint16 t0;
  uint16 t1;
  uint16 tpl_arg_set_index = mdl->code(MDL_TPL_ARGS).asIndex();
  if (mdl->code(tpl_arg_set_index).getAtomCount() == 0) {

    q0 = premise_pattern->get_reference(0)->code(MK_VAL_VALUE).asIndex();
    t0 = premise_pattern->code(FACT_AFTER).asIndex();
    t1 = premise_pattern->code(FACT_BEFORE).asIndex();
  } else { // use the tpl args.

    q0 = 0;
    t0 = 1;
    t1 = 2;
  }

  _build(mdl, q0, t0, t1, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CmdGuardBuilder::CmdGuardBuilder(microseconds period, uint16 cmd_arg_index) : TimingGuardBuilder(period), cmd_arg_index(cmd_arg_index) {
}

CmdGuardBuilder::~CmdGuardBuilder() {
}

void CmdGuardBuilder::_build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const {

  Code *rhs = mdl->get_reference(1);
  uint16 t2 = rhs->code(FACT_AFTER).asIndex();
  uint16 t3 = rhs->code(FACT_BEFORE).asIndex();
  uint16 q1 = rhs->get_reference(0)->code(MK_VAL_VALUE).asIndex();

  Code *lhs = mdl->get_reference(0);
  uint16 cmd_t0 = lhs->code(FACT_AFTER).asIndex();
  uint16 cmd_t1 = lhs->code(FACT_BEFORE).asIndex();
  uint16 cmd_arg = lhs->get_reference(0)->code(cmd_arg_index).asIndex();

  mdl->code(MDL_FWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(3);

  uint16 extent_index = write_index + 3;

  write_guard(mdl, t2, t0, Opcodes::Add, period, write_index, extent_index);
  write_guard(mdl, t3, t1, Opcodes::Add, period, write_index, extent_index);

  mdl->code(++write_index) = Atom::AssignmentPointer(q1, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(fwd_opcode, 2); // q1:(fwd_opcode q0 cmd_arg)
  mdl->code(++extent_index) = Atom::VLPointer(q0);
  mdl->code(++extent_index) = Atom::VLPointer(cmd_arg);
  extent_index += 1;

  write_index = extent_index;
  mdl->code(MDL_BWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(5);

  extent_index = write_index + 5;

  write_guard(mdl, t0, t2, Opcodes::Sub, period, write_index, extent_index);
  write_guard(mdl, t1, t3, Opcodes::Sub, period, write_index, extent_index);

  write_guard(mdl, cmd_t0, t2, Opcodes::Sub, period, write_index, extent_index);
  write_guard(mdl, cmd_t1, t3, Opcodes::Sub, period, write_index, extent_index);

  mdl->code(++write_index) = Atom::AssignmentPointer(cmd_arg, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(bwd_opcode, 2); // cmd_arg:(bwd_opcode q1 q0)
  mdl->code(++extent_index) = Atom::VLPointer(q1);
  mdl->code(++extent_index) = Atom::VLPointer(q0);
  extent_index += 1;

  write_index = extent_index;
}

void CmdGuardBuilder::_build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  uint16 q0;
  uint16 t0;
  uint16 t1;
  uint16 tpl_arg_set_index = mdl->code(MDL_TPL_ARGS).asIndex();
  if (mdl->code(tpl_arg_set_index).getAtomCount() == 0) {

    q0 = premise_pattern->get_reference(0)->code(MK_VAL_VALUE).asIndex();
    t0 = premise_pattern->code(FACT_AFTER).asIndex();
    t1 = premise_pattern->code(FACT_BEFORE).asIndex();
  } else { // use the tpl args.

    q0 = 0;
    t0 = 1;
    t1 = 2;
  }

  _build(mdl, fwd_opcode, bwd_opcode, q0, t0, t1, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MCGuardBuilder::MCGuardBuilder(microseconds period, float32 cmd_arg_index) : CmdGuardBuilder(period, cmd_arg_index) {
}

MCGuardBuilder::~MCGuardBuilder() {
}

void MCGuardBuilder::build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  _build(mdl, Opcodes::Mul, Opcodes::Div, premise_pattern, cause_pattern, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ACGuardBuilder::ACGuardBuilder(microseconds period, uint16 cmd_arg_index) : CmdGuardBuilder(period, cmd_arg_index) {
}

ACGuardBuilder::~ACGuardBuilder() {
}

void ACGuardBuilder::build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  _build(mdl, Opcodes::Add, Opcodes::Sub, premise_pattern, cause_pattern, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

ConstGuardBuilder::ConstGuardBuilder(microseconds period, float32 constant, microseconds offset) : TimingGuardBuilder(period), constant(constant), offset(offset) {
}

ConstGuardBuilder::~ConstGuardBuilder() {
}

void ConstGuardBuilder::_build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const {

  Code *rhs = mdl->get_reference(1);
  uint16 t2 = rhs->code(FACT_AFTER).asIndex();
  uint16 t3 = rhs->code(FACT_BEFORE).asIndex();
  uint16 q1 = rhs->get_reference(0)->code(MK_VAL_VALUE).asIndex();

  Code *lhs = mdl->get_reference(1);
  uint16 t4 = lhs->code(FACT_AFTER).asIndex();
  uint16 t5 = lhs->code(FACT_BEFORE).asIndex();

  mdl->code(MDL_FWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(3);

  uint16 extent_index = write_index + 3;

  write_guard(mdl, t2, t0, Opcodes::Add, period, write_index, extent_index);
  write_guard(mdl, t3, t1, Opcodes::Add, period, write_index, extent_index);

  mdl->code(++write_index) = Atom::AssignmentPointer(q1, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(fwd_opcode, 2); // q1:(fwd_opcode q0 constant)
  mdl->code(++extent_index) = Atom::VLPointer(q0);
  mdl->code(++extent_index) = Atom::Float(constant);
  extent_index += 1;

  write_index = extent_index;
  mdl->code(MDL_BWD_GUARDS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(5);

  extent_index = write_index + 5;

  write_guard(mdl, t0, t2, Opcodes::Sub, period, write_index, extent_index);
  write_guard(mdl, t1, t3, Opcodes::Sub, period, write_index, extent_index);

  write_guard(mdl, t4, t2, Opcodes::Sub, offset, write_index, extent_index);
  write_guard(mdl, t5, t3, Opcodes::Sub, offset, write_index, extent_index);

  mdl->code(++write_index) = Atom::AssignmentPointer(q0, ++extent_index);
  mdl->code(extent_index) = Atom::Operator(bwd_opcode, 2); // q0:(bwd_opcode q1 constant)
  mdl->code(++extent_index) = Atom::VLPointer(q1);
  mdl->code(++extent_index) = Atom::Float(constant);
  extent_index += 1;

  write_index = extent_index;
}

void ConstGuardBuilder::_build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  uint16 q0;
  uint16 t0;
  uint16 t1;
  uint16 tpl_arg_set_index = mdl->code(MDL_TPL_ARGS).asIndex();
  if (mdl->code(tpl_arg_set_index).getAtomCount() == 0) {

    q0 = premise_pattern->get_reference(0)->code(MK_VAL_VALUE).asIndex();
    t0 = premise_pattern->code(FACT_AFTER).asIndex();
    t1 = premise_pattern->code(FACT_BEFORE).asIndex();
  } else { // use the tpl args.

    q0 = 0;
    t0 = 1;
    t1 = 2;
  }

  _build(mdl, fwd_opcode, bwd_opcode, q0, t0, t1, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

MGuardBuilder::MGuardBuilder(microseconds period, float32 constant, microseconds offset) : ConstGuardBuilder(period, constant, offset) {
}

MGuardBuilder::~MGuardBuilder() {
}

void MGuardBuilder::build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  _build(mdl, Opcodes::Mul, Opcodes::Div, premise_pattern, cause_pattern, write_index);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AGuardBuilder::AGuardBuilder(microseconds period, float32 constant, microseconds offset) : ConstGuardBuilder(period, constant, offset) {
}

AGuardBuilder::~AGuardBuilder() {
}

void AGuardBuilder::build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const {

  _build(mdl, Opcodes::Add, Opcodes::Sub, premise_pattern, cause_pattern, write_index);
}
}
