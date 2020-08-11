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

#ifndef factory_h
#define factory_h

#include "../r_code/utils.h"
#include "binding_map.h"
#include "overlay.h"
#include "dll.h"


namespace r_exec {

// No instances of the following classes can transmited; for now: facts and icst will be.

// Notification markers are not put in their references marker sets.
// They are not used to propagate saliency changes.
// They are encoded as Atom::Object instead of Atom::Marker.

class r_exec_dll MkNew :
  public LObject {
public:
  MkNew(r_code::Mem *m, r_code::Code *object);
};

class r_exec_dll MkLowRes :
  public LObject {
public:
  MkLowRes(r_code::Mem *m, r_code::Code *object);
};

class r_exec_dll MkLowSln :
  public LObject {
public:
  MkLowSln(r_code::Mem *m, r_code::Code *object);
};

class r_exec_dll MkHighSln :
  public LObject {
public:
  MkHighSln(r_code::Mem *m, r_code::Code *object);
};

class r_exec_dll MkLowAct :
  public LObject {
public:
  MkLowAct(r_code::Mem *m, r_code::Code *object);
};

class r_exec_dll MkHighAct :
  public LObject {
public:
  MkHighAct(r_code::Mem *m, r_code::Code *object);
};

class r_exec_dll MkSlnChg :
  public LObject {
public:
  MkSlnChg(r_code::Mem *m, r_code::Code *object, float32 value);
};

class r_exec_dll MkActChg :
  public LObject {
public:
  MkActChg(r_code::Mem *m, r_code::Code *object, float32 value);
};

class Pred;
class Goal;

class r_exec_dll _Fact :
  public LObject {
private:
  static bool MatchAtom(Atom lhs, Atom rhs);
  static bool MatchStructure(const r_code::Code *lhs, uint16 lhs_base_index, uint16 lhs_index, const r_code::Code *rhs, uint16 rhs_index);
  static bool Match(const r_code::Code *lhs, uint16 lhs_base_index, uint16 lhs_index, const r_code::Code *rhs, uint16 rhs_index, uint16 lhs_arity);
  static bool CounterEvidence(const r_code::Code *lhs, const r_code::Code *rhs);
protected:
  _Fact();
  _Fact(r_code::SysObject *source);
  _Fact(_Fact *f);
  _Fact(uint16 opcode, r_code::Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr);
public:
  static bool MatchObject(const r_code::Code *lhs, const r_code::Code *rhs);

  virtual bool is_invalidated();

  bool is_fact() const { return (code(0).asOpcode() == Opcodes::Fact); }
  bool is_anti_fact() const { return (code(0).asOpcode() == Opcodes::AntiFact); }
  void set_opposite() const {
    if (is_fact())
      code(0) = Atom::Object(Opcodes::AntiFact, FACT_ARITY);
    else
      code(0) = Atom::Object(Opcodes::Fact, FACT_ARITY);
  }
  _Fact *get_absentee() const;

  bool match_timings_sync(const _Fact *evidence) const;
  bool match_timings_overlap(const _Fact *evidence) const;
  bool match_timings_inclusive(const _Fact *evidence) const;

  MatchResult is_evidence(const _Fact *target) const;
  MatchResult is_timeless_evidence(const _Fact *target) const;

  bool has_after() const { return r_code::Utils::HasTimestamp<r_code::Code>(this, FACT_AFTER); }
  bool has_before() const { return r_code::Utils::HasTimestamp<r_code::Code>(this, FACT_BEFORE); }
  Timestamp get_after() const;
  Timestamp get_before() const;
  float32 get_cfd() const { return code(FACT_CFD).asFloat(); }

  void set_cfd(float32 cfd) { code(FACT_CFD) = Atom::Float(cfd); }

  Pred *get_pred() const {
    Code *pred = get_reference(0);
    if (pred->code(0).asOpcode() == Opcodes::Pred)
      return (Pred *)pred;
    return NULL;
  }
  Goal *get_goal() const {
    Code *goal = get_reference(0);
    if (goal->code(0).asOpcode() == Opcodes::Goal)
      return (Goal *)goal;
    return NULL;
  }

  void trace() const;
};

typedef enum {
  SIM_ROOT = 0,
  SIM_OPTIONAL = 1,
  SIM_MANDATORY = 2
}SimMode;

class r_exec_dll Sim :
  public LObject {
public:
  Sim(Sim *s); // is_requirement=false (not copied).
  // For SIM_MANDATORY or SIM_OPTIONAL, provide sol, sol_cfd and sol_before. Otherwise, defaults for SIM_ROOT.
  Sim(SimMode mode, std::chrono::microseconds thz, Fact *super_goal, bool opposite, Controller *root, float32 psln_thr, Controller *sol_controller = NULL, float32 sol_cfd = 0, Timestamp sol_before = Timestamp(std::chrono::seconds(0)));
  bool invalidate();
  bool is_invalidated();
  // If SIM_MANDATORY or SIM_OPTIONAL: qualifies a sub-goal of the branch's root.
  SimMode get_mode() const { return (SimMode)(int)code(SIM_MODE).asFloat(); }
  // simulation time allowance (this is not the goal deadline); 0 indicates no time for simulation.
  std::chrono::microseconds get_thz() const { 
    // The time horizon is stored as a timestamp, but it is actually a duration.
    return std::chrono::duration_cast<std::chrono::microseconds>(r_code::Utils::GetTimestamp<Code>(this, SIM_THZ).time_since_epoch());
  }

  bool is_requirement_;

  bool opposite_; // of the goal the sim is attached to, i.e. the result of the match during controller->reduce(); the confidence is in the goal target.

  P<Fact> super_goal_; // of the goal the sim is attached to.
  P<Controller> root_; // controller that produced the simulation branch root (SIM_ROOT): identifies the branch.
  P<Controller> sol_controller_; // controller that produced a sub-goal of the branch's root: identifies the model that can be a solution for the super-goal.
  float32 sol_cfd_; // confidence of the solution goal.
  Timestamp sol_before_; // deadline of the solution goal.
};

// Caveat: instances of Fact can becone instances of AntiFact (set_opposite() upon MATCH_SUCCESS_NEGATIVE during backward chaining).
// In particular, is_fact() and is_anti_fact() are based on the opcode, not on the class.
// Do not store any data in this class.
class r_exec_dll Fact :
  public _Fact {
public:
  void *operator new(size_t s);
  Fact();
  Fact(r_code::SysObject *source);
  Fact(Fact *f);
  Fact(r_code::Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr);
};

// Caveat: as for Fact.
class r_exec_dll AntiFact :
  public _Fact {
public:
  void *operator new(size_t s);
  AntiFact();
  AntiFact(r_code::SysObject *source);
  AntiFact(AntiFact *f);
  AntiFact(r_code::Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr);
};

// Goals and predictions:
// When positive evidences are found for a goal/prediction, said object is invalidated: in g-monitors and p-monitors, respectively.
// Idem for negative evidences In such cases, an absentee (absence of the expected fact) is injected.
//
// A negative evidence is either:
// (a) a fact that is either the absence of the target (absentee) or a fact asserting a state for the target which is not the expected one (it is assumed that an object can be in only one state WRT a given attribute; and recurse in icst) or,
// (b) a prediction holding a fact that is a counter evidence for the target with a confidence higher than said target.
//
// When a decision ground is invalidated, the subsequent is also invalidated: said invalidation is performed in the p-monitors and g-monitors.
// When a super-goal is invalidated (simulated or actual), sub-goals are also invalidated: said invalidation is performed in the g-monitors of the sub-goals.
//
// Invalidation checks are performed at both _take_input() time and reduce() time as the invalidation may occur during the transit in the pipe.

class r_exec_dll Pred :
  public LObject {
public:
  Pred();
  Pred(r_code::SysObject *source);
  Pred(_Fact *target, float32 psln_thr);

  bool is_invalidated();
  bool grounds_invalidated(_Fact *evidence);

  _Fact *get_target() const { return (_Fact *)get_reference(0); }

  std::vector<P<_Fact> > grounds_; // f1->obj; predictions that were used to build this predictions (i.e. antecedents); empty if simulated.
  std::vector<P<Sim> > simulations_;

  bool is_simulation() const { return simulations_.size() > 0; }
  Sim *get_simulation(Controller *root) const; // return true if there is a simulation for the goal.
};

class r_exec_dll Goal :
  public LObject {
public:
  Goal();
  Goal(r_code::SysObject *source);
  Goal(_Fact *target, r_code::Code *actor, Sim* sim, float32 psln_thr);

  bool invalidate();
  bool is_invalidated();
  bool ground_invalidated(_Fact *evidence);

  bool is_requirement() const;

  bool is_self_goal() const;
  bool is_drive() const { return (!has_sim() && is_self_goal()); }

  _Fact *get_target() const { return (_Fact *)get_reference(0); }
  _Fact *get_super_goal() const { return get_sim()->super_goal_; }
  r_code::Code *get_actor() const { return get_reference(code(GOAL_ACTR).asIndex()); }

  /**
   * Check if this Goal has a Sim object.
   * @return True if this Goal has a Sim object, otherwise false.
   */
  bool has_sim() const { return code(GOAL_SIM).getDescriptor() == Atom::R_PTR; }

  /**
   * Get the Sim object.
   * @return The Sim object, or NULL if this Goal does not have a Sim object.
   */
  // Debug: Define _Sim because if a replicode file defines (sim ...) it won't have all the fields.
  Sim* get_sim() const { return has_sim() ? (Sim*)get_reference(code(GOAL_SIM).asIndex()) : NULL; }

  /**
   * Set the Sim object for this Goal. If this Goal already has a Sim object, print an error and
   * do nothing. (Normally, the Sim object is given to the Goal constructor. If code needs to set
   * the Sim object, it should be on a Goal that was just constructed with a NULL Sim object.
   * @param sim The Sim object.
   */
  void set_sim(Sim* sim) {
    if (has_sim()) {
      std::cerr << "Goal::set_sim: Error: The goal already has a Sim. Ignoring the new sim." << std::endl;
      return;
    }

    code(GOAL_SIM) = Atom::RPointer(references_size());
    add_reference(sim);
  }

  /**
   * Check if this goal is a simulation, specifically if there is a simulation object whose
   * time horizon is not zero.
   * @return True if this is a simulation.
   */
  bool is_simulation() const {
    Sim* sim = get_sim();
    return sim && sim->get_thz() != seconds(0);
  }

  P<_Fact> ground_; // f->p->f->imdl (weak requirement) that allowed backward chaining, if any.

  // goal->target->cfd/(before-now).
  float32 get_strength(Timestamp now) const {
    _Fact *target = get_target();
    return target->get_cfd() / std::chrono::duration_cast<std::chrono::microseconds>(target->get_before() - now).count();
  }
};

class r_exec_dll MkRdx :
  public LObject {
public:
  MkRdx();
  MkRdx(r_code::SysObject *source);
  MkRdx(r_code::Code *imdl_fact, r_code::Code *input, r_code::Code *output, float32 psln_thr, BindingMap *binding_map); // for mdl.
  MkRdx(r_code::Code *imdl_fact, r_code::Code *input1, r_code::Code *input2, r_code::Code *output, float32 psln_thr, BindingMap *binding_map); // for mdl.

  P<BindingMap> bindings_; // NULL when produced by programs.
};

class r_exec_dll Success :
  public LObject {
public:
  Success();
  Success(_Fact *object, _Fact *evidence, float32 psln_thr);
};

class r_exec_dll Perf :
  public LObject {
public:
  Perf();
  Perf(std::chrono::microseconds reduction_job_avg_latency, std::chrono::microseconds d_reduction_job_avg_latency, std::chrono::microseconds time_job_avg_latency, std::chrono::microseconds d_time_job_avg_latency);
};

class r_exec_dll ICST :
  public LObject {
public:
  ICST();
  ICST(r_code::SysObject *source);

  bool is_invalidated();

  bool contains(_Fact *component, uint16 &component_index) const;

  P<BindingMap> bindings_;
  std::vector<P<_Fact> > components_; // the inputs that triggered the building of the icst.
};
}


#endif
