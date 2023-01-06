//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
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

#include "time_core.h"
#include "mem.h"
#include "init.h"

using namespace std::chrono;

namespace r_exec {

thread_ret thread_function_call TimeCore::Run(void *args) {

  TimeCore *_this = ((TimeCore *)args);

  bool run = true;
  while (run) {

    P<TimeJob> j = _Mem::Get()->pop_time_job();
    if (j == NULL)
      break;
    if (!j->is_alive()) {

      j = NULL;
      continue;
    }

    Timestamp target = j->target_time_;
    Timestamp next_target(microseconds(0));
    if (target.time_since_epoch().count() == 0) // 0 means ASAP. Control jobs (shutdown) are caught here.
      run = j->update(next_target);
    else {

      auto time_to_wait = duration_cast<microseconds>(target - Now());
      if (time_to_wait.count() == 0) // right on time: do the job.
        run = j->update(next_target);
      else if (time_to_wait.count() > 0) { // early: spawn a delegate to wait for the due time; delegate will die when done.

        DelegatedCore *d = new DelegatedCore(j->target_time_, time_to_wait, j);
        d->start(DelegatedCore::Wait);
        _Mem::Get()->register_time_job_latency(time_to_wait);
        next_target = Timestamp(seconds(0));
      } else { // late: do the job and report.

        run = j->update(next_target);
        j->report(-time_to_wait);
      }
    }

    while (next_target.time_since_epoch().count() && run) {

      if (!j->is_alive())
        break;

      auto time_to_wait = duration_cast<microseconds>(next_target - Now());
      next_target = Timestamp(seconds(0));
      if (time_to_wait.count() == 0) // right on time: do the job.
        run = j->update(next_target);
      else if (time_to_wait.count() > 0) { // early: spawn a delegate to wait for the due time; delegate will die when done.
                                  // the delegate will handle the next target when it is known (call to update()).
        DelegatedCore *d = new DelegatedCore(next_target, time_to_wait, j);
        d->start(DelegatedCore::Wait);
      } else { // late: do the job and report.

        run = j->update(next_target);
        j->report(-time_to_wait);
      }
    }
    j = NULL;
  }

  thread_ret_val(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TimeCore::TimeCore() : Thread() {
}

TimeCore::~TimeCore() {
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

thread_ret thread_function_call DelegatedCore::Wait(void *args) {

  _Mem::Get()->start_core();
  DelegatedCore *_this = ((DelegatedCore *)args);

  auto time_to_wait = _this->time_to_wait_;
  auto target_time = _this->target_time_;

wait: _this->timer_.start(time_to_wait);
  _this->timer_.wait();

  if (!_this->job_->is_alive())
    goto end;

  if (_Mem::Get()->check_state() == _Mem::RUNNING) { // checks for shutdown that could have happened during the wait on timer.

    while (Now() < target_time); // early, we have to wait; on Windows: timers resolution in ms => poll.
    target_time = Timestamp(seconds(0));
    _this->job_->update(target_time);
  }

redo: if (target_time.time_since_epoch().count()) {

  if (!_this->job_->is_alive())
    goto end;
  if (_Mem::Get()->check_state() != _Mem::RUNNING) // checks for shutdown that could have happened during the last update().
    goto end;

  time_to_wait = duration_cast<microseconds>(target_time - Now());
  if (time_to_wait.count() == 0) { // right on time: do the job.

    _this->job_->update(target_time);
    goto redo;
  } else if (time_to_wait.count() < 0) { // late.

    _this->job_->update(target_time);
    _this->job_->report(-time_to_wait);
    goto redo;
  } else
    goto wait;
}

    end: _Mem::Get()->shutdown_core();
      delete _this;
      thread_ret_val(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DelegatedCore::DelegatedCore(Timestamp target_time, microseconds time_to_wait, TimeJob *j) : Thread(), target_time_(target_time), time_to_wait_(time_to_wait), job_(j) {
}

DelegatedCore::~DelegatedCore() {
}
}
