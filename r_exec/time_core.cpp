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

#include "time_core.h"
#include "mem.h"
#include "init.h"


namespace r_exec {

thread_ret thread_function_call TimeCore::Run(void *args) {

  TimeCore *_this = ((TimeCore *)args);

  bool run = true;
  while (run) {

    P<TimeJob> j = _Mem::Get()->popTimeJob();
    if (j == NULL)
      break;
    if (!j->is_alive()) {

      j = NULL;
      continue;
    }

    uint64 target = j->target_time;
    uint64 next_target = 0;
    if (target == 0) // 0 means ASAP. Control jobs (shutdown) are caught here.
      run = j->update(next_target);
    else {

      int64 time_to_wait = target - Now();
      if (time_to_wait == 0) // right on time: do the job.
        run = j->update(next_target);
      else if (time_to_wait > 0) { // early: spawn a delegate to wait for the due time; delegate will die when done.

        DelegatedCore *d = new DelegatedCore(j->target_time, time_to_wait, j);
        d->start(DelegatedCore::Wait);
        _Mem::Get()->register_time_job_latency(time_to_wait);
        next_target = 0;
      } else { // late: do the job and report.

        run = j->update(next_target);
        j->report(-time_to_wait);
      }
    }

    while (next_target && run) {

      if (!j->is_alive())
        break;

      uint64 time_to_wait = next_target - Now();
      next_target = 0;
      if (time_to_wait == 0) // right on time: do the job.
        run = j->update(next_target);
      else if (time_to_wait > 0) { // early: spawn a delegate to wait for the due time; delegate will die when done.
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

  int64 time_to_wait = _this->time_to_wait;
  uint64 target_time = _this->target_time;

wait: _this->timer.start(time_to_wait);
  _this->timer.wait();

  if (!_this->job->is_alive())
    goto end;

  if (_Mem::Get()->check_state() == _Mem::RUNNING) { // checks for shutdown that could have happened during the wait on timer.

    while (Now() < target_time); // early, we have to wait; on Windows: timers resolution in ms => poll.
    target_time = 0;
    _this->job->update(target_time);
  }

redo: if (target_time) {

  if (!_this->job->is_alive())
    goto end;
  if (_Mem::Get()->check_state() != _Mem::RUNNING) // checks for shutdown that could have happened during the last update().
    goto end;

  time_to_wait = target_time - Now();
  if (time_to_wait == 0) { // right on time: do the job.

    _this->job->update(target_time);
    goto redo;
  } else if (time_to_wait < 0) { // late.

    _this->job->update(target_time);
    _this->job->report(-time_to_wait);
    goto redo;
  } else
    goto wait;
}

    end: _Mem::Get()->shutdown_core();
      delete _this;
      thread_ret_val(0);
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

DelegatedCore::DelegatedCore(uint64 target_time, uint64 time_to_wait, TimeJob *j) : Thread(), target_time(target_time), time_to_wait(time_to_wait), job(j) {
}

DelegatedCore::~DelegatedCore() {
}
}
