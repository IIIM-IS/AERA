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

#include "pgm_controller.h"
#include "mem.h"

using namespace r_code;

namespace r_exec {

_PGMController::_PGMController(_View *ipgm_view) : OController(ipgm_view) {

  run_once_ = !ipgm_view->object_->code(IPGM_RUN).asBoolean();
}

_PGMController::~_PGMController() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

InputLessPGMController::InputLessPGMController(_View *ipgm_view) : _PGMController(ipgm_view) {

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

PGMController::PGMController(_View *ipgm_view) : _PGMController(ipgm_view) {

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
  if (time_scope_.count() > 0) {

    reductionCS_.enter();
    auto now = Now(); // call must be located after the CS.enter() since (*o)->reduce() may update (*o)->birth_time.
    for (o = overlays_.begin(); o != overlays_.end();) {

      if ((*o)->is_invalidated())
        o = overlays_.erase(o);
      else {

        auto  birth_time = ((PGMOverlay *)*o)->get_birth_time();
        if (birth_time.time_since_epoch().count() > 0 && now - birth_time > time_scope_) {
          o = overlays_.erase(o);
        } else {
          Overlay *offspring = (*o)->reduce(input);
          if (offspring) {
            overlays_.push_front(offspring);
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

AntiPGMController::AntiPGMController(_View *ipgm_view) : _PGMController(ipgm_view), successful_match_(false) {

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
    TimeJob *next_job = new AntiPGMSignalingJob((r_exec::View*)view_, Now() + Utils::GetDuration<Code>(get_object(), IPGM_TSC));
    _Mem::Get()->push_time_job(next_job);
  } else
    host->leave();
}
}
