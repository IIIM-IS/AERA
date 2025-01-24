//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2025 Jeff Thompson
//_/_/ Copyright (c) 2018-2025 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2025 Icelandic Institute for Intelligent Machines
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

#include "time_job.h"
#include "pgm_controller.h"
#include "mem.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;

namespace r_exec {

uint32 TimeJob::job_count_ = 0;

TimeJob::TimeJob(Timestamp target_time) : _Object(), target_time_(target_time) {
  // Increment without thread lock. It's only for tracing.
  job_id_ = ++job_count_;
}

bool TimeJob::is_alive() const {

  return true;
}

void TimeJob::report(microseconds lag) const {

  std::cout << "> late generic: " << lag.count() << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

UpdateJob::UpdateJob(Group *g, Timestamp ijt) : TimeJob(ijt) {

  group_ = g;
}

bool UpdateJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " UpdateJob::TimeJob " << get_job_id() <<
    ": group_" << group_->get_oid() << "->update()");
#endif
  group_->update(target_time_);
  return true;
}

void UpdateJob::report(int64 lag) const {

  std::cout << "> late update: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

SignalingJob::SignalingJob(View *v, Timestamp ijt) : TimeJob(ijt) {

  view_ = v;
}

bool SignalingJob::is_alive() const {

  return view_->controller_->is_alive();
}

////////////////////////////////////////////////////////////

AntiPGMSignalingJob::AntiPGMSignalingJob(View *v, Timestamp ijt) : SignalingJob(v, ijt) {
}

bool AntiPGMSignalingJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " AntiPGMSignalingJob::TimeJob " << get_job_id() <<
    ": controller(" << view_->controller_->get_detail_oid() << ")->signal_anti_pgm()");
#endif
  if (is_alive())
    ((AntiPGMController *)view_->controller_)->signal_anti_pgm();
  return true;
}

void AntiPGMSignalingJob::report(int64 lag) const {

  std::cout << "> late |pgm signaling: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

InputLessPGMSignalingJob::InputLessPGMSignalingJob(View *v, Timestamp ijt) : SignalingJob(v, ijt) {
}

bool InputLessPGMSignalingJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " InputLessPGMSignalingJob::TimeJob " << get_job_id() <<
    ": controller(" << view_->controller_->get_detail_oid() << ")->signal_input_less_pgm()");
#endif
  if (is_alive())
    ((InputLessPGMController *)view_->controller_)->signal_input_less_pgm();
  return true;
}

void InputLessPGMSignalingJob::report(int64 lag) const {

  std::cout << "> late input-less pgm signaling: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

InjectionJob::InjectionJob(View *v, Timestamp target_time, bool is_from_io_device) : TimeJob(target_time) {

  view_ = v;
  is_from_io_device_ = is_from_io_device;
}

bool InjectionJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " InjectionJob::TimeJob " << get_job_id() <<
    ": inject(View(fact(" << view_->object_->get_detail_oid() << ")))");
#endif
  _Mem::Get()->inject(view_);
  if (is_from_io_device_)
    // The view injection time may be different than now, so log it too.
    OUTPUT_LINE(IO_DEVICE_INJ_EJT, Utils::RelativeTime(Now()) << " I/O device inject " <<
      view_->object_->get_oid() << ", ijt " << Utils::RelativeTime(view_->get_ijt()));
  return true;
}

void InjectionJob::report(int64 lag) const {

  std::cout << "> late injection: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

EInjectionJob::EInjectionJob(View *v, Timestamp ijt) : TimeJob(ijt) {

  view_ = v;
}

bool EInjectionJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " EInjectionJob::TimeJob " << get_job_id() <<
    ": inject_existing_object(View(fact(" << view_->object_->get_detail_oid() << ")))");
#endif
  _Mem::Get()->inject_existing_object(view_, view_->object_, view_->get_host());
  return true;
}

void EInjectionJob::report(int64 lag) const {

  std::cout << "> late injection: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

SaliencyPropagationJob::SaliencyPropagationJob(Code *o, float32 sln_change, float32 source_sln_thr, Timestamp ijt) : TimeJob(ijt), sln_change_(sln_change), source_sln_thr_(source_sln_thr) {

  object_ = o;
}

bool SaliencyPropagationJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " SaliencyPropagationJob::TimeJob " << get_job_id() <<
    ": propagate_sln(fact(" << object_->get_detail_oid() << "))");
#endif
  if (!object_->is_invalidated())
    _Mem::Get()->propagate_sln(object_, sln_change_, source_sln_thr_);
  return true;
}

void SaliencyPropagationJob::report(int64 lag) const {

  std::cout << "> late sln propagation: " << lag << " us behind." << std::endl;
}

////////////////////////////////////////////////////////////

ShutdownTimeCore::ShutdownTimeCore() : TimeJob(Timestamp(seconds(0))) {
}

bool ShutdownTimeCore::update(Timestamp &next_target) {

  return false;
}

////////////////////////////////////////////////////////////

PerfSamplingJob::PerfSamplingJob(Timestamp start, microseconds period) : TimeJob(start), period_(period) {
}

bool PerfSamplingJob::update(Timestamp &next_target) {

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " PerfSamplingJob::TimeJob " << get_job_id() <<
    ": inject_perf_stats()");
#endif
  _Mem::Get()->inject_perf_stats();
  target_time_ += period_;
  next_target = target_time_;
  return true;
}

bool PerfSamplingJob::is_alive() const {

  return true;
}
}
