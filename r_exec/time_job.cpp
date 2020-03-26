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

#include "time_job.h"
#include "pgm_controller.h"
#include "mem.h"


namespace r_exec {

uint32 TimeJob::job_count_ = 0;

TimeJob::TimeJob(uint64 target_time) : _Object(), target_time(target_time) {
  // Increment without thread lock. It's only for tracing.
  job_id_ = ++job_count_;
}

bool TimeJob::is_alive() const {

  return true;
}

void TimeJob::report(int64 lag) const {

  std::cout << "> late generic: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

UpdateJob::UpdateJob(Group *g, uint64 ijt) : TimeJob(ijt) {

  group = g;
}

bool UpdateJob::update(uint64 &next_target) {

  group->update(target_time);
  return true;
}

void UpdateJob::report(int64 lag) const {

  std::cout << "> late update: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

SignalingJob::SignalingJob(View *v, uint64 ijt) : TimeJob(ijt) {

  view = v;
}

bool SignalingJob::is_alive() const {

  return view->controller->is_alive();
}

////////////////////////////////////////////////////////////

AntiPGMSignalingJob::AntiPGMSignalingJob(View *v, uint64 ijt) : SignalingJob(v, ijt) {
}

bool AntiPGMSignalingJob::update(uint64 &next_target) {

  if (is_alive())
    ((AntiPGMController *)view->controller)->signal_anti_pgm();
  return true;
}

void AntiPGMSignalingJob::report(int64 lag) const {

  std::cout << "> late |pgm signaling: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

InputLessPGMSignalingJob::InputLessPGMSignalingJob(View *v, uint64 ijt) : SignalingJob(v, ijt) {
}

bool InputLessPGMSignalingJob::update(uint64 &next_target) {

  if (is_alive())
    ((InputLessPGMController *)view->controller)->signal_input_less_pgm();
  return true;
}

void InputLessPGMSignalingJob::report(int64 lag) const {

  std::cout << "> late input-less pgm signaling: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

InjectionJob::InjectionJob(View *v, uint64 ijt) : TimeJob(ijt) {

  view = v;
}

bool InjectionJob::update(uint64 &next_target) {

  _Mem::Get()->inject(view);
  return true;
}

void InjectionJob::report(int64 lag) const {

  std::cout << "> late injection: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

EInjectionJob::EInjectionJob(View *v, uint64 ijt) : TimeJob(ijt) {

  view = v;
}

bool EInjectionJob::update(uint64 &next_target) {

  _Mem::Get()->inject_existing_object(view, view->object, view->get_host());
  return true;
}

void EInjectionJob::report(int64 lag) const {

  std::cout << "> late injection: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

SaliencyPropagationJob::SaliencyPropagationJob(Code *o, float32 sln_change, float32 source_sln_thr, uint64 ijt) : TimeJob(ijt), sln_change(sln_change), source_sln_thr(source_sln_thr) {

  object = o;
}

bool SaliencyPropagationJob::update(uint64 &next_target) {

  if (!object->is_invalidated())
    _Mem::Get()->propagate_sln(object, sln_change, source_sln_thr);
  return true;
}

void SaliencyPropagationJob::report(int64 lag) const {

  std::cout << "> late sln propagation: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

ShutdownTimeCore::ShutdownTimeCore() : TimeJob(0) {
}

bool ShutdownTimeCore::update(uint64 &next_target) {

  return false;
}

////////////////////////////////////////////////////////////

PerfSamplingJob::PerfSamplingJob(uint64 start, uint32 period) : TimeJob(start), period(period) {
}

bool PerfSamplingJob::update(uint64 &next_target) {

  _Mem::Get()->inject_perf_stats();
  target_time += period;
  next_target = target_time;
  return true;
}

bool PerfSamplingJob::is_alive() const {

  return true;
}
}
