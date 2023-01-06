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

#ifndef time_job_h
#define time_job_h

#include "group.h"
#include "pgm_overlay.h"


namespace r_exec {

class r_exec_dll TimeJob :
  public _Object {
protected:
  TimeJob(Timestamp target_time);
public:
  Timestamp target_time_; // absolute deadline; 0 means ASAP.
  virtual bool update(Timestamp &next_target) = 0; // next_target: absolute deadline; 0 means no more waiting; return false to shutdown the time core.
  virtual bool is_alive() const;
  virtual void report(std::chrono::microseconds lag) const;
  uint32 get_job_id() const { return job_id_; }

  /**
   * Compare P<TimeJob> based only on target_time_.
   */
  class Compare {
  public:
    bool
      operator()
      (const P<TimeJob>& x, const P<TimeJob>& y) const
    {
      return x->target_time_ < y->target_time_;
    }
  };

private:
  static uint32 job_count_;
  int job_id_;
};

class r_exec_dll UpdateJob :
  public TimeJob {
public:
  P<Group> group_;
  UpdateJob(Group *g, Timestamp ijt);
  bool update(Timestamp &next_target) override;
  void report(int64 lag) const;
};

class r_exec_dll SignalingJob :
  public TimeJob {
protected:
  SignalingJob(View *v, Timestamp ijt);
public:
  P<View> view_;
  bool is_alive() const override;
};

class r_exec_dll AntiPGMSignalingJob :
  public SignalingJob {
public:
  AntiPGMSignalingJob(View *v, Timestamp ijt);
  bool update(Timestamp &next_target) override;
  void report(int64 lag) const;
};

class r_exec_dll InputLessPGMSignalingJob :
  public SignalingJob {
public:
  InputLessPGMSignalingJob(View *v, Timestamp ijt);
  bool update(Timestamp &next_target) override;
  void report(int64 lag) const;
};

/**
 * InjectionJob extends TimeJob to inject a View at a later time.
 */
class r_exec_dll InjectionJob :
  public TimeJob {
public:
  P<View> view_;
  /**
   * Create an InjectionJob to call  _Mem::Get()->inject(view_) at the target_time.
   * \param v The View for calling inject.
   * \param target_time The target time for the TimeJob.
   * \param is_from_io_device True if this is called from inject_from_io_device().
   * This is only needed so that this will log the I/O device inject.
   */
  InjectionJob(View *v, Timestamp target_time, bool is_from_io_device);
  bool update(Timestamp &next_target) override;
  void report(int64 lag) const;

  bool is_from_io_device_;
};

class r_exec_dll EInjectionJob :
  public TimeJob {
public:
  P<View> view_;
  EInjectionJob(View *v, Timestamp ijt);
  bool update(Timestamp &next_target) override;
  void report(int64 lag) const;
};

class r_exec_dll SaliencyPropagationJob :
  public TimeJob {
public:
  P<r_code::Code> object_;
  float32 sln_change_;
  float32 source_sln_thr_;
  SaliencyPropagationJob(r_code::Code *o, float32 sln_change, float32 source_sln_thr, Timestamp ijt);
  bool update(Timestamp &next_target) override;
  void report(int64 lag) const;
};

class r_exec_dll ShutdownTimeCore :
  public TimeJob {
public:
  ShutdownTimeCore();
  bool update(Timestamp &next_target) override;
};

template<class M> class MonitoringJob :
  public TimeJob {
public:
  P<M> monitor_;
  MonitoringJob(M *monitor, Timestamp deadline) : TimeJob(deadline), monitor_(monitor) {
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, "  make MonitoringJob::TimeJob " << get_job_id() <<
      "(" << get_detail_oid() << ") for monitor(" << monitor_->get_detail_oid() << "), deadline " <<
      Utils::RelativeTime(deadline));
#endif
  }
  bool update(Timestamp &next_target) override {

#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " MonitoringJob::TimeJob " << get_job_id() <<
      ": monitor(" << monitor_->get_detail_oid() << ")->update()");
#endif
    monitor_->update(next_target);
    return true;
  }
  bool is_alive() const override {

    return monitor_->is_alive();
  }
  void report(std::chrono::microseconds lag) const override {

    std::cout << "> late monitoring: " << lag.count() << " us behind." << std::endl;
  }
};

class r_exec_dll PerfSamplingJob :
  public TimeJob {
public:
  std::chrono::microseconds period_;
  PerfSamplingJob(Timestamp start, std::chrono::microseconds period);
  bool is_alive() const override;
  bool update(Timestamp &next_target) override;
};
}


#endif
