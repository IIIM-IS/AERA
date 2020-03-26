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

#ifndef guard_builder_h
#define guard_builder_h

#include "factory.h"


namespace r_exec {

class GuardBuilder :
  public _Object {
public:
  GuardBuilder();
  virtual ~GuardBuilder();

  virtual void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// fwd: t2=t0+period, t3=t1+period.
// bwd: t0=t2-period, t1=t3-period.
class TimingGuardBuilder :
  public GuardBuilder {
protected:
  uint64 period;

  void write_guard(Code *mdl, uint16 l, uint16 r, uint16 opcode, uint64 offset, uint16 &write_index, uint16 &extent_index) const;
  void _build(Code *mdl, uint16 t0, uint16 t1, uint16 &write_index) const;
public:
  TimingGuardBuilder(uint64 period);
  virtual ~TimingGuardBuilder();

  virtual void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// fwd: q1=q0+speed*period.
// bwd: speed=(q1-q0)/period, speed.after=q1.after-offset, speed.before=q1.before-offset.
class SGuardBuilder :
  public TimingGuardBuilder {
private:
  uint64 offset; // period-(speed.after-t0).

  void _build(Code *mdl, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const;
public:
  SGuardBuilder(uint64 period, uint64 offset);
  ~SGuardBuilder();

  void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// bwd: cmd.after=q1.after-offset, cmd.before=cmd.after+cmd_duration.
class NoArgCmdGuardBuilder :
  public TimingGuardBuilder {
protected:
  uint64 offset;
  uint64 cmd_duration;

  void _build(Code *mdl, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const;
public:
  NoArgCmdGuardBuilder(uint64 period, uint64 offset, uint64 cmd_duration);
  ~NoArgCmdGuardBuilder();

  void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// bwd: cmd.after=q1.after-period, cmd.before=q1.before-period.
class CmdGuardBuilder :
  public TimingGuardBuilder {
protected:
  uint16 cmd_arg_index;

  void _build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const;
  void _build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;

  CmdGuardBuilder(uint64 period, uint16 cmd_arg_index);
public:
  virtual ~CmdGuardBuilder();
};

// fwd: q1=q0*cmd_arg.
// bwd: cmd_arg=q1/q0.
class MCGuardBuilder :
  public CmdGuardBuilder {
public:
  MCGuardBuilder(uint64 period, float32 cmd_arg_index);
  ~MCGuardBuilder();

  void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// fwd: q1=q0+cmd_arg.
// bwd: cmd_arg=q1-q0.
class ACGuardBuilder :
  public CmdGuardBuilder {
private:

public:
  ACGuardBuilder(uint64 period, uint16 cmd_arg_index);
  ~ACGuardBuilder();

  void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// bwd: cause.after=t2-offset, cause.before=t3-offset.
class ConstGuardBuilder :
  public TimingGuardBuilder {
protected:
  float32 constant;
  uint64 offset;

  void _build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, uint16 q0, uint16 t0, uint16 t1, uint16 &write_index) const;
  void _build(Code *mdl, uint16 fwd_opcode, uint16 bwd_opcode, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;

  ConstGuardBuilder(uint64 period, float32 constant, uint64 offset);
public:
  ~ConstGuardBuilder();
};

// fwd: q1=q0*constant.
// bwd: q0=q1/constant.
class MGuardBuilder :
  public ConstGuardBuilder {
public:
  MGuardBuilder(uint64 period, float32 constant, uint64 offset);
  ~MGuardBuilder();

  void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};

// fwd: q1=q0+constant.
// bwd: q0=q1-constant.
class AGuardBuilder :
  public ConstGuardBuilder {
public:
  AGuardBuilder(uint64 period, float32 constant, uint64 offset);
  ~AGuardBuilder();

  void build(Code *mdl, _Fact *premise_pattern, _Fact *cause_pattern, uint16 &write_index) const;
};
}


#endif
