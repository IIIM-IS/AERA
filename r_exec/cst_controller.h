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

#ifndef cst_controller_h
#define cst_controller_h

#include "hlp_overlay.h"
#include "hlp_controller.h"
#include "factory.h"


namespace r_exec {

// All inputs are expected to be synchronized (within the time tolerance).
// The time value of each fact is therefore the same (within the time tolerance).
// Therefore, said time value is not listed in the argument list of the icst.
// It is held by the fact holding said icst.
// Inputs synchronized on state are treated as if they were produced continuously, i.e. Now().
// The confidence value for an icst is the lowest value taken from the matched inputs.
// No cmds or imdls in a cst.
//
// Forward chaining:
// output a prediction of icst instead of icst if at least one input is a prediction.
// output as many predictions of icst as we got simulated predictions for different goals.
class CSTOverlay :
  public HLPOverlay {
protected:
  Timestamp match_deadline_; // before deadline after the last match.
  float32 lowest_cfd_; // among the inputs (forward chaining).

  std::vector<P<_Fact> > inputs_;

  UNORDERED_SET<P<_Fact>, r_code::PHash<_Fact> > predictions_; // f0->pred->f1->obj.
  UNORDERED_SET<P<Sim>, r_code::PHash<Sim> > simulations_;
  uint32 original_patterns_size_;

  void inject_production();
  void update(HLPBindingMap *map, _Fact *input, _Fact *bound_pattern);
  CSTOverlay *get_offspring(HLPBindingMap *map, _Fact *input, _Fact *bound_pattern);

  CSTOverlay(const CSTOverlay *original);
public:
  CSTOverlay(Controller *c, HLPBindingMap *bindings);
  ~CSTOverlay();

  bool reduce(View *input, CSTOverlay *&offspring);

  void load_patterns();

  bool can_match(Timestamp now) const;

  bool is_simulated() { return simulations_.size() > 0; }

  /**
   * Find the first _Fact in patterns_ which matches the input, and update
   * the binding map.
   * \param input The input _Fact to match against a pattern.
   * \param map The binding map for calling match_fwd_strict and has the bindings
   * if this returns a pattern.
   * \return The matching pattern from patterns_, or NULL if not found.
   */
  _Fact* CSTOverlay::bindPattern(_Fact *input, HLPBindingMap* map);
};

// Backward chaining:
// if there are requirements, do nothing: these requirements will get the goal and abduce.
// else
// bind all patterns and look in the cache for positive evidences; for all bound patterns not matched in the cache, output a sub-goal (simulated or not, depending on the super-goal).
class CSTController :
  public HLPController {
private:
  Group *secondary_host_;

  void abduce(HLPBindingMap *bm, Fact *f_super_goal); // f_super_goal is f0->g->f1->icst or f0->g->|f1->icst.
  void inject_goal(HLPBindingMap *bm,
    Fact *f_super_goal, // f0->g->f1->icst or f0->g->|f1->icst.
    _Fact *sub_goal_target, // f1.
    Sim *sim,
    Timestamp now,
    float32 confidence,
    r_code::Code *group) const;

  void kill_views();
  void check_last_match_time(bool match); // kill if no match after primary_thz;
public:
  CSTController(r_code::View *view);
  ~CSTController();

  void take_input(r_exec::View *input);
  void reduce(r_exec::View *input);

  Fact *get_f_ihlp(HLPBindingMap *bindings, bool wr_enabled) const;
  Fact *get_f_icst(HLPBindingMap *bindings, std::vector<P<_Fact> > *inputs) const;

  void inject_icst(Fact *production, float32 confidence, std::chrono::microseconds time_to_live) const; // here, resilience=time to live, in us.
  void inject_icst(Fact *production, float32 confidence, Timestamp::duration time_to_live) const {
    inject_icst(production, confidence, std::chrono::duration_cast<std::chrono::microseconds>(time_to_live));
  }
  bool inject_prediction(Fact *prediction, float32 confidence, std::chrono::microseconds time_to_live) const; // here, resilience=time to live, in us; returns true if the prediction has actually been injected.
  bool inject_prediction(Fact *prediction, float32 confidence, Timestamp::duration time_to_live) const {
    return inject_prediction(prediction, confidence, std::chrono::duration_cast<std::chrono::microseconds>(time_to_live));
  }

  void set_secondary_host(Group *host);
  Group *get_secondary_host() const;
};
}


#endif
