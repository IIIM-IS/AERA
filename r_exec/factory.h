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
class Success;

class r_exec_dll _Fact :
  public LObject {
protected:
  _Fact();
  _Fact(r_code::SysObject *source);
  _Fact(_Fact *f);
  _Fact(uint16 opcode, r_code::Code *object, Timestamp after, Timestamp before, float32 confidence, float32 psln_thr);
public:
  static bool MatchAtom(Atom lhs, Atom rhs);
  static bool MatchStructure(const r_code::Code* lhs, uint16 lhs_base_index, uint16 lhs_index, const r_code::Code* rhs, uint16 rhs_index, bool same_binding_state = false);
  static bool Match(const r_code::Code* lhs, uint16 lhs_base_index, uint16 lhs_index, const r_code::Code* rhs, uint16 rhs_index, uint16 lhs_arity, bool same_binding_state = false);
  static bool CounterEvidence(const r_code::Code* lhs, const r_code::Code* rhs);
  /**
   * Check if lhs matches rhs.
   * \param same_binding_state (optional) If false, then allow an unbound variable to match another
   * bound or unbound object. If true, then only allow an unbound variable to match another
   * unbound variable. If omitted, use false.
   */
  static bool MatchObject(const r_code::Code *lhs, const r_code::Code *rhs, bool same_binding_state = false);

  bool is_invalidated() override;

  bool is_fact() const { return (code(0).asOpcode() == Opcodes::Fact); }
  bool is_anti_fact() const { return (code(0).asOpcode() == Opcodes::AntiFact); }
  void set_opposite() {
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
  Success *get_success() const {
    Code *success = get_reference(0);
    if (success->code(0).asOpcode() == Opcodes::Success)
      return (Success *)success;
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
  // For SIM_MANDATORY or SIM_OPTIONAL, provide solution_controller, solution_cfd and solution_before. Otherwise, defaults for SIM_ROOT.
  // For SIM_ROOT, solution_before is unused so use Utils::GetTimeReference() which is 0s:0ms:0us in the decompiled output.
  Sim(SimMode mode, std::chrono::microseconds thz, Fact *super_goal, bool opposite, Controller *root, float32 psln_thr, Controller *solution_controller = NULL, float32 solution_cfd = 0, Timestamp solution_before = r_code::Utils::GetTimeReference());
  bool invalidate() override;
  bool is_invalidated() override;
  // If SIM_MANDATORY or SIM_OPTIONAL: qualifies a sub-goal of the branch's root.
  SimMode get_mode() const { return (SimMode)(int)code(SIM_MODE).asFloat(); }
  // simulation time allowance (this is not the goal deadline); 0 indicates no time for simulation.
  std::chrono::microseconds get_thz() const {
    return r_code::Utils::GetDuration<Code>(this, SIM_THZ);
  }

  /**
   * Get the (fact of the) super goal of the goal the sim is attached to.
   */
  Fact* get_f_super_goal() const { return (Fact*)get_reference(0); }

  /**
   * Get the opposite flag of the goal the sim is attached to, i.e. the result of the
   * match during controller->reduce(); the confidence is in the goal target.
   */
  bool get_opposite() const { return code(SIM_OPPOSITE).asBoolean(); }

  /**
   * Get the confidence of the solution goal.
   */
  float32 get_solution_cfd() const { return code(SIM_SOLUTION_CFD).asFloat(); }

  /**
   * Get the  deadline of the solution goal.
   */
  Timestamp get_solution_before() const { return r_code::Utils::GetTimestamp<Code>(this, SIM_SOLUTION_BEFORE); }

  /**
   * Set the  deadline of the solution goal.
   */
  void set_solution_before(Timestamp before) { return r_code::Utils::SetTimestamp<Code>(this, SIM_SOLUTION_BEFORE, before); }

  /**
   * Recursively search this or parent Sim objects to get the Sim object where get_mode() == SIM_ROOT.
   * \return The root Sim object, or NULL if not found (which shouldn't happen).
   */
  Sim* get_root_sim();

  /**
   * If f_obj does not match any of the goal targets stored in the root Sim object, then store it in the root Sim object
   * for future checks and return true. (In this case, you should inject it as a goal.) Otherwise if the goal target was already
   * registered then return false, meaning that it does not need to be made into a goal again.
   * \param f_obj The (fact of the) object, such as (fact (mk_val ...)).
   * \return True if f_obj has been registered and should be injected as a goal, false if it is already registered and should not
   * be injected as a goal.
   */
  bool register_goal_target(_Fact* f_obj);

  bool is_requirement_;

  P<Controller> root_; // controller that produced the simulation branch root (SIM_ROOT): identifies the branch.
  P<Controller> solution_controller_; // controller that produced a sub-goal of the branch's root: identifies the model that can be a solution for the super-goal.

  /**
   * A DefeasiblePromotedFact holds the defeasible fact that was promoted to the next frame in this Sim,
   * plus the DefeasibleValidity which is attached to its prediction and which is invalidated if a actual
   * predicted fact is injected. It also holds the original fact that was promoted, for preventing repeated promotion.
   */
  class DefeasiblePromotedFact {
  public:
    DefeasiblePromotedFact()
      : promoted_fact_(NULL), original_fact_(NULL), defeasible_validity_(NULL)
    {}

    /**
     * Create a DefeasiblePromotedFact with the given values.
     * \param original_fact The original fact that was promoted, for preventing repeated promotion.
     * \param promoted_fact The promoted fact which will be checked against actual predicted facts.
     * \param defeasible_validity The DefeasibleValidity object that is attached to the predicted promoted fact.
     */
    DefeasiblePromotedFact(_Fact* original_fact, _Fact* promoted_fact, DefeasibleValidity* defeasible_validity)
      : original_fact_(original_fact), promoted_fact_(promoted_fact), defeasible_validity_(defeasible_validity)
    {}

    /**
     * Check if the list has a DefeasiblePromotedFact with the given original_fact. Use pointer equality (not a deep match).
     * \param list The list to check.
     * \param original_fact The original fact to match.
     * \return True if the list has a DefeasiblePromotedFact with the given original_fact.
     */
    static bool has_original_fact(const r_code::list<DefeasiblePromotedFact>& list, const _Fact* original_fact);

    P<_Fact> original_fact_;
    P<_Fact> promoted_fact_;
    P<DefeasibleValidity> defeasible_validity_;
  };
  CriticalSectionList<DefeasiblePromotedFact> defeasible_promoted_facts_;

  // The list of actual (non-icst) facts in this Sim which are able to (or already have) defeat a fact in defeasible_promoted_facts_ .
  // (For a critical section, expect to use defeasible_promoted_facts_.CS_ .)
  r_code::list<P<_Fact> > defeating_facts_;

  // A list of (fact (pred (fact (cmd ::)))) to check if a command has already been signalled in this sim.
  std::vector<P<_Fact> > already_signalled_;

private:
  std::vector<P<_Fact> > goalTargets_;
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
  /**
   * Create a Pred with the given values.
   * \param target The prediction target.
   * \param simulations Copy the list of simulations.
   * \param psln_thr The propagation of saliency threshold.
   */
  Pred(_Fact *target, const std::vector<P<Sim> >& simulations, float32 psln_thr) : LObject() {
    construct(target, simulations, psln_thr);
  }
  /**
   * Create a Pred with the given values and no simulations.
   * \param target The prediction target.
   * \param psln_thr The propagation of saliency threshold.
   */
  Pred(_Fact *target, float32 psln_thr) : LObject() {
    construct(target, std::vector<P<Sim>>(), psln_thr);
  }
  /**
   * Create a Pred with the given values.
   * \param target The prediction target.
   * \param sim The one simulation for the list of simulations.
   * \param psln_thr The propagation of saliency threshold.
   */
  Pred(_Fact *target, Sim* sim, float32 psln_thr) : LObject() {
    std::vector<P<Sim>> simulations;
    simulations.push_back(sim);
    construct(target, simulations, psln_thr);
  }
  /**
   * Create a Pred with the given values.
   * \param target The prediction target.
   * \param simulations_source The Pred with the list of simulations to copy. This also copies the
   * set of DefeasibleValidity from simulations_source to the new Pred.
   * \param psln_thr The propagation of saliency threshold.
   */
  Pred(_Fact *target, const Pred* simulations_source, float32 psln_thr);

  /**
   * Check if this Pred is invalidated and also recursively call is_invalidated() for the target of
   * this Pred and the list of simulations, grounds and defeasible validities. If any is invalidated then
   * call invalidate() and return true.
   * \return true if this is invalidated.
   */
  bool is_invalidated() override;
  bool grounds_invalidated(_Fact *evidence);

  _Fact *get_target() const { return (_Fact *)get_reference(0); }

  std::vector<P<_Fact> > grounds_; // f1->obj; predictions that were used to build this predictions (i.e. antecedents); empty if simulated.
  // The set of DefeasibleValidity which is copied to each Pred made from this one. See is_invalidated().
  std::set<P<DefeasibleValidity> > defeasible_validities_;
  // This is set true when a DefeasibleValidity is added for a promoted prediction.
  bool is_promoted_;

  bool is_simulation() const { return get_simulations_size() > 0; }

  /**
   * Get the size of the list of simulations.
   * \return The size of the list of simulations.
   */
  uint16 get_simulations_size() const {
    auto sim_set_index = code(PRED_SIMS).asIndex();
    return code(sim_set_index).getAtomCount();
  }

  /**
   * Get the simulation in the list of simulations at index i.
   * \param i The index of the simulation, from 0 to   get_simulations_size() - 1.
   * \return The simulation at index i.
   */
  Sim* get_simulation(uint16 i) const {
    auto sim_set_index = code(PRED_SIMS).asIndex();
    auto atom = code(sim_set_index + i);
    return (Sim*)get_reference(atom.asIndex());
  }

  /**
   * Get the first simulation whose root is the given root Controller.
   * \param root The root Controller to search for.
   * \return The first simulation with the root, or NULL if not found.
   */
  Sim *get_simulation(Controller *root) const;

  /**
   * Check if the list of simulations has the given sim.
   * \param sim The Sim to check for.
   * \return True if the list of simulations has sim.
   */
  bool has_simulation(Sim* sim) const;

  /**
   * Check if this Pred has a defeasible consequence.
   * \return True if this has a defeasible consequence.
   */
  bool has_defeasible_consequence() const { return !!defeasible_consequence_; }

  /**
   * Get this Pred's defeasible consequence, creating one if it doesn't already exist. If you don't want to
   * create a defeasible consequence , check has_defeasible_consequence() first.
   * \return This Pred's defeasible_consequence_.
   */
  DefeasibleValidity* get_defeasible_consequence() {
    if (!defeasible_consequence_)
      defeasible_consequence_ = new DefeasibleValidity();
    return defeasible_consequence_;
  }

private:
  /**
   * defeasible_consequence_ is a unique DefeasibleValidity for this Pred which is attached to predicted consequences so
   * that they can be invalidated if this Pred is defeated. (However, invalidating the DefeasibleValidity does not
   * invalidate this Pred.)
   */
  P<DefeasibleValidity> defeasible_consequence_;

  void construct(_Fact *target, const std::vector<P<Sim> >& simulations, float32 psln_thr);
};

class r_exec_dll Goal :
  public LObject {
public:
  Goal();
  Goal(r_code::SysObject *source);
  Goal(_Fact *target, r_code::Code *actor, Sim* sim, float32 psln_thr);

  bool invalidate() override;
  bool is_invalidated() override;
  bool ground_invalidated(_Fact *evidence);

  bool is_requirement() const;

  bool is_self_goal() const;
  bool is_drive() const { return (!has_sim() && is_self_goal()); }

  _Fact *get_target() const { return (_Fact *)get_reference(0); }
  _Fact *get_super_goal() const { return get_sim()->get_f_super_goal(); }
  r_code::Code *get_actor() const { return get_reference(code(GOAL_ACTR).asIndex()); }

  /**
   * Check if this Goal has a Sim object.
   * \return True if this Goal has a Sim object, otherwise false.
   */
  bool has_sim() const { return code(GOAL_SIM).getDescriptor() == Atom::R_PTR; }

  /**
   * Get the Sim object.
   * \return The Sim object, or NULL if this Goal does not have a Sim object.
   */
  // Debug: Define _Sim because if a replicode file defines (sim ...) it won't have all the fields.
  Sim* get_sim() const { return has_sim() ? (Sim*)get_reference(code(GOAL_SIM).asIndex()) : NULL; }

  /**
   * Set the Sim object for this Goal. If this Goal already has a Sim object, print an error and
   * do nothing. (Normally, the Sim object is given to the Goal constructor. If code needs to set
   * the Sim object, it should be on a Goal that was just constructed with a NULL Sim object.
   * \param sim The Sim object.
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
   * \return True if this is a simulation.
   */
  bool is_simulation() const {
    Sim* sim = get_sim();
    return sim && sim->get_thz() != std::chrono::seconds(0);
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

  r_code::Code* get_first_input() {
    return get_reference(code(code(MK_RDX_INPUTS).asIndex() + 1).asIndex());
  }

  r_code::Code* get_first_production() {
    return get_reference(code(code(MK_RDX_PRODS).asIndex() + 1).asIndex());
  }

  P<BindingMap> bindings_; // NULL when produced by programs.
};

class r_exec_dll Success :
  public LObject {
public:
  Success();
  Success(_Fact *object, _Fact *evidence, float32 psln_thr);

  /**
   * Get the object of this Success.
   * \return The object of this Success as a _Fact.
   */
  _Fact *get_object() const { return (_Fact*)get_reference(code(SUCCESS_OBJ).asIndex()); }

  /**
   * Check if this Success has an evidence object.
   * \return True if this Success has an evidence object, otherwise false.
   */
  bool has_evidence() const { return code(SUCCESS_EVD).getDescriptor() == Atom::R_PTR; }

  /**
   * Get the evidence object.
   * \return The evidence object as a _Fact, or NULL if this Success does not have an evidence object.
   */
  _Fact* get_evidence() const { return has_evidence() ? (_Fact*)get_reference(code(SUCCESS_EVD).asIndex()) : NULL; }
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

  bool is_invalidated() override;

  /**
   * Check if the components_ contains the given component. This does not recurse.
   * \param component The component to search for. This checks for the exact object (not a match).
   * \param component_index If this returns true, set component_index to the index of the found component.
   * \return True if found the component.
   */
  bool contains(const _Fact *component, uint16 &component_index) const;

  /**
   * Check if the components_ or any sub icst contains the give component. (If a component is an f_icst, this
   * recursively calls itself.)
   * \param component The component to search for. This checks for the exact object (not a match).
   * \return True if found the component.
   */
  bool r_contains(const _Fact* component) const;

  P<BindingMap> bindings_;
  std::vector<P<_Fact> > components_; // the inputs that triggered the building of the icst.
};
}


#endif
