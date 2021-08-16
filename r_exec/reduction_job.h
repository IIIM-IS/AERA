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

#ifndef reduction_job_h
#define reduction_job_h

#include "overlay.h"
#include "object.h"


namespace r_exec {

class r_exec_dll _ReductionJob :
  public _Object {
protected:
  _ReductionJob();
public:
  Timestamp ijt_; // time of injection of the job in the pipe.
  virtual bool update(Timestamp now) = 0; // return false to shutdown the reduction core.
  virtual void debug() {}
  // Temporary until a strong requirement can retroactively invalidate. See https://github.com/IIIM-IS/AERA/pull/174
  bool is_for_strong_requirement_;
  uint32 get_job_id() { return job_id_; }
private:
  static uint32 job_count_;
  int job_id_;
};

template<class _P> class ReductionJob :
  public _ReductionJob {
public:
  P<View> input_;
  P<_P> processor_;
  ReductionJob(View *input, _P *processor) : _ReductionJob(), input_(input), processor_(processor) {}
  bool update(Timestamp now) {

    _Mem::Get()->register_reduction_job_latency(now - ijt_);
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(now) << " ReductionJob " << get_job_id() <<
      ": controller(" << processor_->get_detail_oid() << ")->reduce(View(fact_" << 
      input_->object_->get_oid() << "))");
#endif
    processor_->reduce(input_);
    return true;
  }
  void debug() {

    processor_->debug(input_);
  }
};

template<class _P, class T, class C> class BatchReductionJob :
  public _ReductionJob {
public:
  P<_P> processor_; // the controller that will process the job.
  P<T> trigger_; // the event that triggered the job.
  P<C> controller_; // the controller that produced the job.
  BatchReductionJob(_P *processor, T *trigger, C *controller) : _ReductionJob(), processor_(processor), trigger_(trigger), controller_(controller) {}
  bool update(Timestamp now) {

    _Mem::Get()->register_reduction_job_latency(now - ijt_);
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(now) << " BatchReductionJob " << get_job_id() <<
      ": controller(" << controller_->get_detail_oid() << "), trigger fact(" << 
      trigger_->get_detail_oid() << ")");
#endif
    processor_->reduce_batch(trigger_, controller_);
    return true;
  }
};

class r_exec_dll ShutdownReductionCore :
  public _ReductionJob {
public:
  bool update(Timestamp now);
};

class r_exec_dll AsyncInjectionJob :
  public _ReductionJob {
public:
  P<View> input_;
  AsyncInjectionJob(View *input) : _ReductionJob(), input_(input) {}
  bool update(Timestamp now);
};
}


#endif
