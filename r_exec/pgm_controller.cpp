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

#include "pgm_controller.h"
#include "mem.h"

using namespace r_code;

namespace r_exec {

_PGMController::_PGMController(r_code::View *ipgm_view) : OController(ipgm_view) {

  run_once_ = !ipgm_view->object_->code(IPGM_RUN).asBoolean();
}

_PGMController::~_PGMController() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

InputLessPGMController::InputLessPGMController(r_code::View *ipgm_view) : _PGMController(ipgm_view) {

  overlays_.push_back(new InputLessPGMOverlay(this));
}

InputLessPGMController::~InputLessPGMController() {
}

void InputLessPGMController::signal_input_less_pgm() { // next job will be pushed by the rMem upon processing the current signaling job, i.e. right after exiting this function.

  reductionCS_.enter();
  if (overlays_.size()) {

    InputLessPGMOverlay *overlay = (InputLessPGMOverlay *)overlays_.front();
    overlay->inject_productions();
    overlay->reset();

    if (!run_once_) {

      if (is_alive()) {

        Group *host = get_view()->get_host();
        host->enter();
        if (host->get_c_act() > host->get_c_act_thr() && // c-active group.
          host->get_c_sln() > host->get_c_sln_thr()) { // c-salient group.

          host->leave();

          TimeJob *next_job = new InputLessPGMSignalingJob((r_exec::View*)view_, Now() + time_scope_);
          _Mem::Get()->push_time_job(next_job);
        } else
          host->leave();
      }
    }
  }
  reductionCS_.leave();

  if (run_once_)
    invalidate();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PGMController::PGMController(r_code::View *ipgm_view) : _PGMController(ipgm_view) {

  overlays_.push_back(new PGMOverlay(this));
}

PGMController::~PGMController() {
}

void PGMController::notify_reduction() {

  if (run_once_)
    invalidate();
}

void PGMController::take_input(r_exec::View *input) {

  Controller::__take_input<PGMController>(input);
}

void PGMController::reduce(r_exec::View *input) {

  r_code::list<P<Overlay> >::const_iterator o;
  uint32 oid = input->object_->get_oid();
  //uint64 t=Now()-Utils::GetTimeReference();
  //std::cout<<Time::ToString_seconds(t)<<" got "<<oid_<<" "<<input->get_sync()<<std::endl;
  if (time_scope_.count() > 0) {

    reductionCS_.enter();
    auto now = Now(); // call must be located after the CS.enter() since (*o)->reduce() may update (*o)->birth_time.
    //uint64 t=now-Utils::GetTimeReference();
    for (o = overlays_.begin(); o != overlays_.end();) {

      if ((*o)->is_invalidated())
        o = overlays_.erase(o);
      else {

        auto  birth_time = ((PGMOverlay *)*o)->get_birth_time();
        if (birth_time.time_since_epoch().count() > 0 && now - birth_time > time_scope_) {
          //std::cout<<Time::ToString_seconds(t)<<" kill "<<std::hex<<(void *)*o<<std::dec<<" born: "<<Time::ToString_seconds(birth_time-Utils::GetTimeReference())<<" after "<<Time::ToString_seconds(now-birth_time)<<std::endl;
          //std::cout<<std::hex<<(void *)*o<<std::dec<<" ------------kill "<<input->object->get_oid()<<" ignored "<<std::endl;
          o = overlays_.erase(o);
        } else {
          //void *oo=*o;
          Overlay *offspring = (*o)->reduce(input);
          if (offspring) {
            overlays_.push_front(offspring);
            //std::cout<<Time::ToString_seconds(t)<<" "<<std::hex<<oo<<std::dec<<" born: "<<Time::ToString_seconds(((PGMOverlay *)oo)->get_birth_time()-Utils::GetTimeReference())<<" reduced "<<input->object->get_oid()<<" "<<input->get_sync()<<" offspring: "<<std::hex<<offspring<<std::dec<<std::endl;
            //std::cout<<std::hex<<(void *)oo<<std::dec<<" --------------- reduced "<<input->object->get_oid()<<" "<<input->get_sync()<<std::endl;
          }
          if (!is_alive())
            break;
          ++o;
        }
      }
    }
    reductionCS_.leave();
  } else {

    reductionCS_.enter();
    for (o = overlays_.begin(); o != overlays_.end();) {

      if ((*o)->is_invalidated())
        o = overlays_.erase(o);
      else {

        Overlay *offspring = (*o)->reduce(input);
        if (offspring)
          overlays_.push_front(offspring);
        if (!is_alive())
          break;
        ++o;
      }

    }
    reductionCS_.leave();
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AntiPGMController::AntiPGMController(r_code::View *ipgm_view) : _PGMController(ipgm_view), successful_match_(false) {

  overlays_.push_back(new AntiPGMOverlay(this));
}

AntiPGMController::~AntiPGMController() {
}

void AntiPGMController::take_input(r_exec::View *input) {

  Controller::__take_input<AntiPGMController>(input);
}

void AntiPGMController::reduce(r_exec::View *input) {

  reductionCS_.enter();
  r_code::list<P<Overlay> >::const_iterator o;
  for (o = overlays_.begin(); o != overlays_.end();) {

    if ((*o)->is_invalidated())
      o = overlays_.erase(o);
    else {

      Overlay *offspring = (*o)->reduce(input);
      if (successful_match_) { // the controller has been restarted: reset the overlay and kill all others

        Overlay *overlay = *o;
        overlay->reset();
        overlays_.clear();
        overlays_.push_back(overlay);
        successful_match_ = false;
        break;
      }
      ++o;
      if (offspring)
        overlays_.push_front(offspring);
    }
  }
  reductionCS_.leave();
}

void AntiPGMController::signal_anti_pgm() {

  reductionCS_.enter();
  if (successful_match_) // a signaling job has been spawn in restart(): we are here in an old job during which a positive match occurred: do nothing.
    successful_match_ = false;
  else { // no positive match during this job: inject productions and restart.

    Overlay *overlay = overlays_.front();
    ((AntiPGMOverlay *)overlay)->inject_productions();
    overlay->reset(); // reset the first overlay and kill all others.
    if (!run_once_ && is_alive()) {

      overlays_.clear();
      overlays_.push_back(overlay);
    }
  }
  reductionCS_.leave();

  if (run_once_)
    invalidate();
}

void AntiPGMController::restart() { // one anti overlay matched all its inputs, timings and guards.

  push_new_signaling_job();
  successful_match_ = true;
}

void AntiPGMController::push_new_signaling_job() {

  Group *host = get_view()->get_host();
  host->enter();
  if (get_view()->get_act() > host->get_act_thr() && // active ipgm.
    host->get_c_act() > host->get_c_act_thr() && // c-active group.
    host->get_c_sln() > host->get_c_sln_thr()) { // c-salient group.

    host->leave();
    // The time scope is stored as a timestamp, but it is actually a duration.
    TimeJob *next_job = new AntiPGMSignalingJob((r_exec::View*)view_, Now() + Utils::GetTimestamp<Code>(get_object(), IPGM_TSC).time_since_epoch());
    _Mem::Get()->push_time_job(next_job);
  } else
    host->leave();
}
}
