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

#ifndef hlp_controller_h
#define hlp_controller_h

#include "overlay.h"
#include "binding_map.h"
#include "g_monitor.h"
#include "group.h"


namespace r_exec {

typedef enum {
  WEAK_REQUIREMENT_DISABLED = 0,
  STRONG_REQUIREMENT_DISABLED_NO_WEAK_REQUIREMENT = 1,
  STRONG_REQUIREMENT_DISABLED_WEAK_REQUIREMENT = 2,
  WEAK_REQUIREMENT_ENABLED = 3,
  NO_REQUIREMENT = 4
}ChainingStatus;

class HLPController :
  public OController {
private:
  uint32 strong_requirement_count; // number of active strong requirements in the same group; updated dynamically.
  uint32 weak_requirement_count; // number of active weak requirements in the same group; updated dynamically.
  uint32 requirement_count; // sum of the two above.
protected:
  class EvidenceEntry { // evidences.
  private:
    void load_data(_Fact *evidence);
  public:
    P<_Fact> evidence;
    Timestamp after;
    Timestamp before;
    float32 confidence;

    EvidenceEntry();
    EvidenceEntry(_Fact *evidence);
    EvidenceEntry(_Fact *evidence, _Fact *payload);

    bool is_too_old(Timestamp now) const { return (evidence->is_invalidated() || before < now); }
  };

  class PredictedEvidenceEntry : // predicted evidences.
    public EvidenceEntry {
  public:
    PredictedEvidenceEntry();
    PredictedEvidenceEntry(_Fact *evidence);
  };

  template<class E> class Cache {
  public:
    CriticalSection CS;
    r_code::list<E> evidences;
  };

  Cache<EvidenceEntry> evidences;
  Cache<PredictedEvidenceEntry> predicted_evidences;

  template<class E> void _store_evidence(Cache<E> *cache, _Fact *evidence) {

    E e(evidence);
    cache->CS.enter();
    auto now = Now();
    r_code::list<E>::const_iterator _e;
    for (_e = cache->evidences.begin(); _e != cache->evidences.end();) {

      if ((*_e).is_too_old(now)) // garbage collection.
        _e = cache->evidences.erase(_e);
      else
        ++_e;
    }
    cache->evidences.push_front(e);
    cache->CS.leave();
  }

  P<HLPBindingMap> bindings;

  bool evaluate_bwd_guards(HLPBindingMap *bm);

  MatchResult check_evidences(_Fact *target, _Fact *&evidence); // evidence with the match (positive or negative), get_absentee(target) otherwise.
  MatchResult check_predicted_evidences(_Fact *target, _Fact *&evidence); // evidence with the match (positive or negative), NULL otherwise.

  bool _has_tpl_args;
  uint32 ref_count; // used to detect _Object::refCount dropping down to 1 for hlp with tpl args.
  bool is_orphan(); // true when there are tpl args and no requirements: the controller cannot execute anymore.

  std::vector<P<HLPController> > controllers; // all controllers for models/states instantiated in the patterns; case of models: [0]==lhs, [1]==rhs.
  Timestamp last_match_time; // last time a match occurred (fwd), regardless of its outcome.
  bool become_invalidated(); // true if one controller is invalidated or if all controllers pointing to this are invalidated.
  virtual void kill_views() {}
  virtual void check_last_match_time(bool match) = 0;

  HLPController(r_code::View *view);
public:
  virtual ~HLPController();

  void invalidate();

  Code *get_core_object() const { return getObject(); } // cst or mdl.
  Code *get_unpacked_object() const { // the unpacked version of the core object.

    Code *core_object = get_core_object();
    return core_object->get_reference(core_object->references_size() - MDL_HIDDEN_REFS);
  }

  void add_requirement(bool strong);
  void remove_requirement(bool strong);

  uint32 get_requirement_count(uint32 &weak_requirement_count, uint32 &strong_requirement_count);
  uint32 get_requirement_count();

  void store_evidence(_Fact *evidence) { _store_evidence<EvidenceEntry>(&evidences, evidence); }
  void store_predicted_evidence(_Fact *evidence) { _store_evidence <PredictedEvidenceEntry>(&predicted_evidences, evidence); }

  virtual Fact *get_f_ihlp(HLPBindingMap *bindings, bool wr_enabled) const = 0;

  uint16 get_out_group_count() const;
  Code *get_out_group(uint16 i) const; // i starts at 1.
  Group *get_host() const;
  bool has_tpl_args() const { return _has_tpl_args; }

  void inject_prediction(Fact *prediction, float32 confidence) const; // for simulated predictions.
};
}


#endif
