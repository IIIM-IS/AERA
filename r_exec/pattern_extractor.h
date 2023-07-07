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

#ifndef pattern_extractor_h
#define pattern_extractor_h

#include "../r_code/time_buffer.h"

#include "binding_map.h"
#include "guard_builder.h"
#include "cst_controller.h"


namespace r_exec {

class AutoFocusController;

class Input {
public:
  P<BindingMap> bindings_; // contains the values for the abstraction.
  P<_Fact> abstraction_;
  P<_Fact> input_;
  bool eligible_cause_;
  Timestamp ijt_; // injection time.

  Input(View *input, _Fact *abstraction, BindingMap *bindings) : input_(input->object_), ijt_(input->get_ijt()), eligible_cause_(IsEligibleCause(input)), abstraction_(abstraction), bindings_(bindings) {}
  Input() : input_(NULL), eligible_cause_(false), abstraction_(NULL), bindings_(NULL), ijt_(std::chrono::seconds(0)) {}
  Input(const Input &original) : input_(original.input_), eligible_cause_(original.eligible_cause_), abstraction_(original.abstraction_), bindings_(original.bindings_), ijt_(original.ijt_) {}

  static bool IsEligibleCause(r_exec::View *view);

  class IsInvalidated { // for storage in time_buffers.
  public:
    bool operator ()(Input &i, Timestamp time_reference, std::chrono::microseconds thz) const {

      return (time_reference - i.ijt_ > thz);
    }
  };
};

class CInput { // cached inputs.
public:
  P<BindingMap> bindings_; // contains the values for the abstraction.
  P<_Fact> abstraction_;
  P<View> input_;
  bool injected_;
  Timestamp ijt_; // injection time.
  CInput(View *input, _Fact *abstraction, BindingMap *bindings) : input_(input), abstraction_(abstraction), bindings_(bindings), injected_(false), ijt_(input->get_ijt()) {}
  CInput() : input_(NULL), abstraction_(NULL), bindings_(NULL), injected_(false), ijt_(std::chrono::seconds(0)) {}

  bool operator ==(const CInput &i) const { return input_ == i.input_; }

  class IsInvalidated { // for storage in time_buffers.
  public:
    bool operator ()(CInput &i, Timestamp time_reference, std::chrono::microseconds thz) const {

      return (time_reference - i.ijt_ > thz);
    }
  };
};

// Targeted Pattern eXtractor.
// Does nothing.
// Used for wr_enabled productions, for well-rated productions or when model acqusiiton is disabled.
class r_exec_dll TPX :
  public _Object {
protected:
  AutoFocusController *auto_focus_;
  P<_Fact> target_; // goal or prediction target, or premise (CTPX); abstraction: lhs of a mdl for goals, rhs for predictions, premise for CTPX.
  P<BindingMap> target_bindings_;
  P<_Fact> abstracted_target_;

  P<CSTController> cst_hook_; // in case the target is an icst.

  std::vector<P<BindingMap> > new_maps_; // acquired (in the case the target's bm is not fully specified) while matching the target's bm with inputs.

  bool filter(View *input, _Fact *abstracted_input, BindingMap *bm);

  TPX(AutoFocusController *auto_focus, _Fact *target);
public:
  TPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings);
  virtual ~TPX();

  _Fact *get_pattern() const { return abstracted_target_; }
  BindingMap *get_bindings() const { return target_bindings_; }

  virtual bool take_input(View *view, _Fact *abstracted_input, BindingMap *bm);
  virtual void signal(View *input) const;
  virtual void ack_pred_success(_Fact *predicted_f);
};

class ICST;

class r_exec_dll _TPX :
  public TPX {
private:
  static const uint32 InputsInitialSize = 16;
protected:
  class Component { // for building csts.
  public:
    _Fact *object;
    bool discarded;
    Component() {}
    Component(_Fact *object) : object(object), discarded(false) {}
  };

  /**
   * See find_f_icst.
   */
  class FindFIcstResult {
  public:
    FindFIcstResult(_Fact* f_icst, _Fact* component_pattern)
    {
      this->f_icst = f_icst;
      this->component_pattern = component_pattern;
    }

    P<_Fact> f_icst;
    _Fact* component_pattern;
  };

  r_code::list<Input> inputs_; // time-controlled buffer (inputs older than tpx_time_horizon from now are discarded).
  std::vector<P<r_code::Code> > mdls_; // new mdls.
  std::vector<P<r_code::Code> > csts_; // new csts.
  std::vector<P<_Fact> > f_icsts_; // facts of new icsts.

  void filter_icst_components(ICST *icst, uint32 icst_index, std::vector<Component> &components);

  /**
   * If the fact is a (fact (icst ...)) then search the icst for the component. If not found, then
   * recursively search all members which are (fact (icst ...)).
   * \param fact The fact to search for the component. If fact->get_reference(0) is not an icst, then return NULL.
   * \param component The component to search for by being the same object (not matching).
   * \param max_depth (optional) The maximum recursion depth for when this calls itself to search members.
   * If this is zero, then don't recurse. If omitted, use a default.
   * \return The Code pattern in the unpacked cst that matches the component, if found. NULL if not found.
   */
  static _Fact* find_f_icst_component(_Fact* fact, const _Fact* component, int max_depth = 3);

  /**
   * Find an f_icst for the component by looking in inputs_ and f_icsts_.
   * \param component The cst component to search for.
   * \param results Call this with an empty vector<FindFIcstResult>. If no f_icst is found, then this is empty. Otherwise an
   * entry has the found f_icst and the Code pattern in the unpacked cst that matches the given component.
   * \param find_multiple If true then add an entry to results for each f_icst found in  inputs_ and f_icsts_ . If false, then
   * results has at most one entry.
   */
  void _find_f_icst(_Fact *component, std::vector<FindFIcstResult>& results, bool find_multiple);

  /**
   * Find an f_icst for the component by looking in inputs_ and f_icsts_.
   * \param component The cst component to search for.
   * \param results Call this with an empty vector<FindFIcstResult>. If no f_icst is found, then this is empty. Otherwise an
   * entry has the found f_icst and the Code pattern in the unpacked cst that matches the given component.
   * \param find_multiple (optional) If true then add an entry to results for each f_icst found in  inputs_ and f_icsts_ .
   * If omitted or false, then results has at most one entry.
   */
  void find_f_icst(_Fact *component, std::vector<FindFIcstResult>& results, bool find_multiple = false);

  /**
   * Find an f_icst for the component by looking in inputs_ and f_icsts_, or if not found then try to make one with a new cst.
   * \param component The cst component to search for.
   * \param results Call this with an empty vector<FindFIcstResult>. If no f_icst is found, then this is empty. Otherwise an
   * entry has the found f_icst and the Code pattern in the unpacked cst that matches the given component.
   * \param new_cst If no existing f_icst is found, then this sets new_cst to a new cst and results has one entry with the
   * new f_icst.
   * \param find_multiple (optional) If true then add an entry to results for each f_icst found in  inputs_ and f_icsts_ .
   * If omitted or false, then results has at most one entry.
   */
  void find_f_icst(_Fact *component, std::vector<FindFIcstResult>& results, P<r_code::Code> &new_cst, bool find_multiple = false);

  _Fact *make_f_icst(_Fact *component, _Fact*& component_pattern, P<r_code::Code> &new_cst);
  r_code::Code *build_cst(const std::vector<Component> &components, BindingMap *bm, _Fact *main_component);

  r_code::Code *build_mdl_head(HLPBindingMap *bm, uint16 tpl_arg_count, _Fact *lhs, _Fact *rhs, uint16 &write_index, bool allow_shared_timing_vars = true);
  r_code::Code* build_mdl_head_from_abstract(uint16 tpl_arg_count, r_code::Code* lhs, r_code::Code* rhs, uint16& write_index);
  void build_mdl_tail(r_code::Code *mdl, uint16 write_index);

  void inject_hlps() const;
  void inject_hlps(Timestamp analysis_starting_time);

  virtual std::string get_header() const = 0;

  _TPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings);
  _TPX(AutoFocusController *auto_focus, _Fact *target);
public:
  virtual ~_TPX();

  void debug(View* /* input */) {};
};

// Pattern extractor targeted at goal successes.
// Possible causes are younger than the production of the goal.
// Models produced are of the form: M1[cause -> goal_target], where cause can be an imdl and goal_target can be an imdl.
// M1 does not have template arguments.
// Commands are ignored (CTPX' job).
class r_exec_dll GTPX : // target is the goal target.
  public _TPX {
private:
  P<Fact> f_imdl_; // that produced the goal.

  std::vector<P<_Fact> > predictions_; // successful predictions that may invalidate the need for model building.

  bool build_mdl(_Fact *cause, _Fact *consequent, GuardBuilder *guard_builder, std::chrono::microseconds period);
  bool build_mdl(_Fact *f_icst, _Fact *cause_pattern, _Fact *consequent, GuardBuilder *guard_builder, std::chrono::microseconds period, r_code::Code *new_cst);

  std::string get_header() const override;
public:
  GTPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings, Fact *f_imdl);
  ~GTPX();

  bool take_input(View *input, _Fact *abstracted_input, BindingMap *bm) override;
  void signal(View *input) const override;
  void ack_pred_success(_Fact *predicted_f) override;
  void reduce(View *input); // input is v->f->success(target,input) or v->|f->success(target,input).
};

// Pattern extractor targeted at prediciton failures.
// Possible causes are older than the production of the prediction.
// Models produced are of the form: M1[cause -> |imdl M0] where M0 is the model that produced the failed prediction and cause can be an imdl.
// M1 does not have template arguments.
// Commands are ignored (CTPX' job).
class r_exec_dll PTPX : // target is the prediction.
  public _TPX {
private:
  P<Fact> f_imdl_; // that produced the prediction (and for which the PTPX will find strong requirements).

  bool build_mdl(_Fact *cause, _Fact *consequent, GuardBuilder *guard_builder, std::chrono::microseconds period);
  bool build_mdl(_Fact *f_icst, _Fact *cause_pattern, _Fact *consequent, GuardBuilder *guard_builder, std::chrono::microseconds period, r_code::Code *new_cst);

  std::string get_header() const override;
public:
  PTPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings, Fact *f_imdl);
  ~PTPX();

  void signal(View *input) const override;
  void reduce(View *input); // input is v->f->success(target,input) or v->|f->success(target,input).
};

// Pattern extractor targeted at changes of repeated input facts (SYMC_PERIODIC or SYNC_HOLD).
// Models produced are of the form: [premise -> [cause -> consequent]], i.e. M1:[premise -> imdl M0], M0:[cause -> consequent].
// M0 has template args, i.e the value of the premise and its after timestamp.
// N.B.: before-after=upr of the group the input comes from.
// The Consequent is a value different from the expected repetition of premise.
// The premise is an icst assembled from inputs synchronous with the input expected to repeat.
// Guards on values (not only on timings) are computed: this is the only TPX that does so.
// Inputs with SYNC_HOLD: I/O devices are expected to send changes on such inputs as soon as possible.
class CTPX :
  public _TPX {
private:
  bool stored_premise_;
  P<View> premise_;

  GuardBuilder *get_default_guard_builder(_Fact *cause, _Fact *consequent, std::chrono::microseconds period);
  GuardBuilder *find_guard_builder(_Fact *cause, _Fact *consequent, std::chrono::microseconds period);

  bool build_mdl(_Fact *cause, _Fact *consequent, GuardBuilder *guard_builder, std::chrono::microseconds period);
  bool build_mdl(_Fact *f_icst, _Fact *cause_pattern, _Fact *consequent, GuardBuilder *guard_builder, std::chrono::microseconds period);

  bool build_requirement(HLPBindingMap *bm, r_code::Code *m0, std::chrono::microseconds period);

  std::string get_header() const override;
public:
  CTPX(AutoFocusController *auto_focus, View *premise);
  ~CTPX();

  void store_input(r_exec::View *input);
  void reduce(r_exec::View *input); // asynchronous: build models of value change if not aborted asynchronously by ASTControllers.
  void signal(r_exec::View *input); // spawns mdl/cst building (reduce()).
};
}


#endif
