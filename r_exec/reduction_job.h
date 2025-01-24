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

#ifndef reduction_job_h
#define reduction_job_h

#include "../r_code/utils.h"
#include "object.h"
#include "mem_output.h"


namespace r_exec {

class r_exec_dll _ReductionJob :
  public _Object {
protected:
  _ReductionJob();
  void register_latency(Timestamp now);
public:
  Timestamp ijt_; // time of injection of the job in the pipe.
  virtual bool update(Timestamp now) = 0; // return false to shutdown the reduction core.
  virtual void debug() {}
  uint32 get_job_id() const { return job_id_; }
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
  bool update(Timestamp now) override {

    register_latency(now);
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, r_code::Utils::RelativeTime(now) << " ReductionJob " << get_job_id() <<
      ": controller(" << processor_->get_detail_oid() << ")->reduce(View(fact_" << 
      input_->object_->get_oid() << "))");
#endif
    processor_->reduce(input_);
    return true;
  }
  void debug() override {

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
  bool update(Timestamp now) override {

    register_latency(now);
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, r_code::Utils::RelativeTime(now) << " BatchReductionJob " << get_job_id() <<
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
  bool update(Timestamp now) override;
};

class r_exec_dll AsyncInjectionJob :
  public _ReductionJob {
public:
  P<View> input_;
  AsyncInjectionJob(View *input) : _ReductionJob(), input_(input) {}
  bool update(Timestamp now) override;
};
}


#endif
