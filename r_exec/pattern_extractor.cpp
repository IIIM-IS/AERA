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

#include "auto_focus.h"
#include "reduction_job.h"
#include "mem.h"
#include "model_base.h"
#include "mdl_controller.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;

namespace r_exec {

bool Input::IsEligibleCause(r_exec::View *view) {

  switch (view->get_sync()) {
  case View::SYNC_AXIOM:
  case View::SYNC_ONCE_AXIOM:
    return false;
  default:
    return true;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

TPX::TPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings) : _Object(), auto_focus_(auto_focus), target_(target), abstracted_target_(pattern), target_bindings_(bindings), cst_hook_(NULL) { // called by GTPX and PTPX's ctor.

  if (bindings->is_fully_specified()) { // get a hook on a cst controller so we get icsts from it: this is needed if the target is an underspecified icst.

    Code *target_payload = target->get_reference(0)->get_reference(0)->get_reference(0);
    if (target_payload->code(0).asOpcode() == Opcodes::ICst) {

      Code *cst = target_payload->get_reference(0);
      cst_hook_ = (CSTController *)((r_exec::View *)cst->get_view(auto_focus->get_primary_group(), true))->controller_;
    }
  }
}

TPX::TPX(AutoFocusController *auto_focus, _Fact *target) : _Object(), auto_focus_(auto_focus) { // called by CTPX's ctor.

  P<BindingMap> bm = new BindingMap();
  abstracted_target_ = (_Fact *)bm->abstract_object(target, false);
  target_ = target;
  target_bindings_ = bm;
}

TPX::~TPX() {
}

bool TPX::take_input(View *input, _Fact *abstracted_input, BindingMap *bm) {

  return filter(input, abstracted_input, bm);
}

void TPX::signal(View *input) const { // input->object is f->success or|f->success.
}

void TPX::ack_pred_success(_Fact *predicted_f) {
}

bool TPX::filter(View *input, _Fact *abstracted_input, BindingMap *bm) {

  if (input->object_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst) // if we get an icst we are called by auto_focus::dispatch_no_inject: the input is irrelevant.
    return false;
  if (target_bindings_->intersect(bm)) {
    return true; }
  if (target_bindings_->is_fully_specified())
    return false;
  for (uint32 i = 0; i < new_maps_.size(); ++i)
    if (new_maps_[i]->intersect(bm)) {
      return true; }

  P<BindingMap> _bm = new BindingMap(target_bindings_);
  _bm->reset_fwd_timings(input->object_);
  time_buffer<CInput, CInput::IsInvalidated> &cache = auto_focus_->get_cache();
  if (_bm->match_fwd_strict(input->object_, (_Fact *)target_->get_reference(0)->get_reference(0))) { // both GTPX and PTPX' target are f0->g/p->f1: we need to match on f1.
    new_maps_.push_back(_bm);
    time_buffer<CInput, CInput::IsInvalidated>::iterator i;
    auto now = Now();
    for (i = cache.begin(now); i != cache.end(); ++i) {

      if (i->injected_) {
        continue; }
      if (_bm->intersect(i->bindings_)) {
        i->injected_ = true;
        auto_focus_->inject_input(i->input_, i->abstraction_, i->bindings_);
      }
    }
    return true;
  } else {

    if (cst_hook_ != NULL)
      cst_hook_->take_input(input);
    CInput ci(input, abstracted_input, bm);
    time_buffer<CInput, CInput::IsInvalidated>::iterator i = cache.find(Now(), ci);
    if (i != cache.end()) { // input already cached.

      if (i->ijt_ < ci.ijt_) // older view (happens for sync_axiom and sync_old).
        cache.erase(i);
      else
        return false;
    }
    cache.push_back(ci);
    return false;
  }
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

_TPX::_TPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings) : TPX(auto_focus, target, pattern, bindings) {

  inputs_.reserve(InputsInitialSize);
}

_TPX::_TPX(AutoFocusController *auto_focus, _Fact *target) : TPX(auto_focus, target) {
}

_TPX::~_TPX() {
}

void _TPX::filter_icst_components(ICST *icst, uint32 icst_index, vector<Component> &components) {

  uint32 found_component_count = 0;
  uint32 *found = new uint32[icst->components_.size()];
  for (uint32 j = 0; j < components.size(); ++j) {

    for (uint32 i = 0; i < icst->components_.size(); ++i) {

      if (components[j].discarded)
        continue;
      if (components[j].object == icst->components_[i]) {

        found[found_component_count] = j;
        ++found_component_count;
      }
    }
  }

  if (found_component_count > 0) { // some of the icst components are already in the inputs: discard said components, keep the icst.

    for (uint32 i = 0; i < found_component_count; ++i)
      components[found[i]].discarded = true;
  } else // none of the icst components are in the inputs; this can only happen because the icst shares one timestamp with the TPX's target: discard the icst.
    components[icst_index].discarded = true;

  delete[] found;
}

_Fact* _TPX::find_f_icst_component(_Fact* fact, const _Fact *component, int max_depth) {
  Code* candidate = fact->get_reference(0);
  if (candidate->code(0).asOpcode() == Opcodes::ICst) {
    ICST* icst = (ICST*)candidate;
    uint16 component_index;
    if (icst->contains(component, component_index)) {
      Code* cst = candidate->get_reference(0);
      // The cst is packed, retrieve the pattern from the unpacked code.
      Code* unpacked_cst = cst->get_reference(cst->references_size() - CST_HIDDEN_REFS);
      return (_Fact*)unpacked_cst->get_reference(component_index);
    }

    if (max_depth > 0) {
      // The component was not found, so recurse to search each member.
      for (uint32 i = 0; i < icst->components_.size(); ++i) {
        // This will check if the component is an icst.
        _Fact* result = find_f_icst_component(icst->components_[i], component, max_depth - 1);
        if (result)
          return result;
      }
    }
  }

  return NULL;
}

void _TPX::_find_f_icst(_Fact *component, vector<FindFIcstResult>& results, bool find_multiple) {

  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end(); ++i) {
    _Fact* component_pattern = find_f_icst_component(i->input_, component);
    if (component_pattern) {
      results.push_back(FindFIcstResult(i->input_, component_pattern));
      if (!find_multiple)
        return;
    }
  }

  vector<P<_Fact> >::const_iterator f_icst;
  for (f_icst = f_icsts_.begin(); f_icst != f_icsts_.end(); ++f_icst) {

    _Fact* component_pattern = find_f_icst_component(*f_icst, component);
    if (component_pattern) {
      results.push_back(FindFIcstResult(*f_icst, component_pattern));
      if (!find_multiple)
        return;
    }
  }
}

void _TPX::find_f_icst(_Fact *component, vector<FindFIcstResult>& results, bool find_multiple) {

  uint16 opcode = component->get_reference(0)->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl) // cmds/imdls cannot be components of a cst.
    return;

  _find_f_icst(component, results, find_multiple);
}

void _TPX::find_f_icst(_Fact *component, vector<FindFIcstResult>& results, P<Code> &new_cst, bool find_multiple) {

  uint16 opcode = component->get_reference(0)->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl)
    // cmds/imdls cannot be components of a cst.
    return;

  _find_f_icst(component, results, find_multiple);
  if (results.size() > 0)
    return;

  _Fact* component_pattern;
  _Fact* f_icst = make_f_icst(component, component_pattern, new_cst);
  if (f_icst)
    results.push_back(FindFIcstResult(f_icst, component_pattern));
}

_Fact *_TPX::make_f_icst(_Fact *component, _Fact*& component_pattern, P<Code> &new_cst) {

  uint16 opcode = component->get_reference(0)->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl)
    // cmds/imdls cannot be components of a cst.
    return NULL;

  vector<Component> components; // no icst found, try to identify components to assemble a cst.
  vector<uint32> icst_components;

  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end(); ++i) {

    if (component == i->input_) {

      components.push_back(Component(component));
    } else if (component->match_timings_sync(i->input_)) {

      Code *icst = i->input_->get_reference(0);
      if (icst->code(0).asOpcode() == Opcodes::ICst)
        icst_components.push_back(components.size());
      components.push_back(Component(i->input_));
    }
  }

  for (uint32 j = 0; j < icst_components.size(); ++j) {

    ICST *icst = (ICST *)components[icst_components[j]].object->get_reference(0);
    filter_icst_components(icst, j, components);
  }

  uint32 actual_size = 0;
  for (uint32 j = 0; j < components.size(); ++j) {

    if (components[j].discarded)
      continue;
    else
      ++actual_size;
  }

  if (actual_size <= 1)
    // contains at most only the provided component.
    return NULL;

  r_code::list<Input>::iterator _i;
  for (_i = inputs_.begin(); _i != inputs_.end(); ++_i) { // flag the components so the tpx does not try them again.

    for (uint32 j = 0; j < components.size(); ++j) {

      if (_i->input_ == components[j].object)
        _i->eligible_cause_ = false;
    }
  }

  P<HLPBindingMap> bm = new HLPBindingMap();
  new_cst = build_cst(components, bm, component);
  // build_cst adds the abstract object of the component as the first reference.
  component_pattern = (_Fact*)new_cst->get_reference(0);
  _Fact *f_icst = bm->build_f_ihlp(new_cst, Opcodes::ICst, false);
  // build_f_ihlp can leave some variables pointing into bm, but we need everything valuated.
  f_icst->set_reference(0, bm->bind_pattern(f_icst->get_reference(0)));
  f_icsts_.push_back(f_icst); // the f_icst can be reused in subsequent model building attempts.
  return f_icst;
}

Code *_TPX::build_cst(const vector<Component> &components, BindingMap *bm, _Fact *main_component) {

  _Fact *abstracted_component = (_Fact *)bm->abstract_object(main_component, false);

  Code *cst = _Mem::Get()->build_object(Atom::CompositeState(Opcodes::Cst, CST_ARITY));

  uint16 actual_component_count = 0;
  // Add the main_component first since its variables are assigned first.
  for (uint16 i = 0; i < components.size(); ++i) {
    if (components[i].discarded)
      continue;
    if (components[i].object == main_component) {
      cst->add_reference(abstracted_component);
      ++actual_component_count;
      break;
    }
  }

  for (uint16 i = 0; i < components.size(); ++i) { // reference patterns;

    if (components[i].discarded)
      continue;
    if (components[i].object == main_component)
      // Already added.
      continue;
    else
      cst->add_reference(bm->abstract_object(components[i].object, true));
    ++actual_component_count;
  }

  uint16 extent_index = CST_ARITY;

  cst->code(CST_TPL_ARGS) = Atom::IPointer(++extent_index);
  cst->code(extent_index) = Atom::Set(0); // no tpl args.

  cst->code(CST_OBJS) = Atom::IPointer(++extent_index);
  cst->code(extent_index) = Atom::Set(actual_component_count);
  for (uint16 i = 0; i < actual_component_count; ++i)
    cst->code(++extent_index) = Atom::RPointer(i);

  cst->code(CST_FWD_GUARDS) = Atom::IPointer(++extent_index);
  cst->code(extent_index) = Atom::Set(0); // no fwd guards.

  cst->code(CST_BWD_GUARDS) = Atom::IPointer(++extent_index);
  cst->code(extent_index) = Atom::Set(0); // no bwd guards.

  cst->code(CST_OUT_GRPS) = Atom::IPointer(++extent_index);
  cst->code(extent_index) = Atom::Set(1); // only one output group: the one the tpx lives in.
  cst->code(++extent_index) = Atom::RPointer(cst->references_size());

  cst->code(CST_ARITY) = Atom::Float(1); // psln_thr.

  cst->add_reference(auto_focus_->get_view()->get_host()); // reference the output group.

  return cst;
}

Code *_TPX::build_mdl_head(HLPBindingMap *bm, uint16 tpl_arg_count, _Fact *lhs, _Fact *rhs, uint16 &write_index, bool allow_shared_timing_vars) {

  Code *mdl = _Mem::Get()->build_object(Atom::Model(Opcodes::Mdl, MDL_ARITY));

  Code* abstract_lhs = bm->abstract_object(lhs, false, allow_shared_timing_vars ? 0 : -1);
  mdl->add_reference(abstract_lhs); // reference lhs.

  int rhs_first_search_index = (allow_shared_timing_vars ? 0 : -1);
  if (abstract_lhs->get_reference(0)->code(0).asOpcode() == Opcodes::IMdl) {
    Code* imdl = abstract_lhs->get_reference(0);
    // This is a reuse model. When matching the timings of the RHS fact, first try the last two
    // exposed args which came from the RHS timings of the imdl being reused.
    auto exposed_args_index = imdl->code(I_HLP_EXPOSED_ARGS).asIndex();
    auto exposed_args_count = imdl->code(exposed_args_index).getAtomCount();
    auto exposed_after_index = exposed_args_index + (exposed_args_count - 1);
    if (exposed_args_count >= 2 &&
        imdl->code(exposed_after_index).getDescriptor() == Atom::VL_PTR)
      rhs_first_search_index = imdl->code(exposed_after_index).asIndex();
  }
  mdl->add_reference(bm->abstract_object(rhs, false, rhs_first_search_index)); // reference rhs.

  write_index = MDL_ARITY;

  mdl->code(MDL_TPL_ARGS) = Atom::IPointer(++write_index);
  if (tpl_arg_count >= 2) {
    // Assume the last two template args are a time interval.
    mdl->code(write_index) = Atom::Set(tpl_arg_count - 1);
    // Write the template args before the time interval.
    for (uint16 i = 0; i < tpl_arg_count - 2; ++i)
      mdl->code(++write_index) = Atom::VLPointer(i);
    ++write_index;
    mdl->code(write_index) = Atom::IPointer(write_index + 1);
    ++write_index;
    // Make the (ti : :) .
    mdl->code(write_index) = Atom::Object(Opcodes::TI, 2);
    mdl->code(++write_index) = Atom::VLPointer(tpl_arg_count - 2);
    mdl->code(++write_index) = Atom::VLPointer(tpl_arg_count - 1);
  }
  else {
    mdl->code(write_index) = Atom::Set(tpl_arg_count);
    for (uint16 i = 0; i < tpl_arg_count; ++i)
      mdl->code(++write_index) = Atom::VLPointer(i);
  }

  mdl->code(MDL_OBJS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(2);
  mdl->code(++write_index) = Atom::RPointer(0);
  mdl->code(++write_index) = Atom::RPointer(1);

  return mdl;
}

void _TPX::build_mdl_tail(Code *mdl, uint16 write_index) {

  mdl->code(MDL_OUT_GRPS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(1); // only one group: the one the tpx lives in.
  mdl->code(++write_index) = Atom::RPointer(2);

  mdl->code(MDL_STRENGTH) = Atom::Float(0);
  mdl->code(MDL_CNT) = Atom::Float(1);
  mdl->code(MDL_SR) = Atom::Float(1);
  mdl->code(MDL_DSR) = Atom::Float(1);
  mdl->code(MDL_ARITY) = Atom::Float(1); // psln_thr.

  mdl->add_reference(auto_focus_->get_view()->get_host()); // reference the output group.
}

void _TPX::inject_hlps() const {

  vector<P<Code> >::const_iterator c;
  for (c = csts_.begin(); c != csts_.end(); ++c)
    _Mem::Get()->pack_hlp(*c);
  auto_focus_->inject_hlps(csts_);
  auto_focus_->inject_hlps(mdls_);
}

void _TPX::inject_hlps(Timestamp analysis_starting_time) {

  if (auto_focus_->decompile_models()) {

    r_code::list<P<Code> > tmp;
    r_code::list<Input>::const_iterator i;
    for (i = inputs_.begin(); i != inputs_.end(); ++i)
      tmp.push_back((Code *)i->input_);

    std::string header("> from buffer -------------------\n\n");

    P<TDecompiler> td = new TDecompiler(1, header);
    td->add_objects(tmp);
    td->decompile();

    auto analysis_end = Now();
    auto d = duration_cast<microseconds>(analysis_end - analysis_starting_time);
    char _timing[255];
    itoa(d.count(), _timing, 10);
    header = Utils::ToString_s_ms_us(Now(), Utils::GetTimeReference());
    std::string s0 = (" > ");
    s0 += get_header() + std::string(":production [");
    std::string timing(_timing);
    std::string s1("us] -------------------\n\n");
    header += s0 + timing + s1;

    td = new TDecompiler(1, header);
    td->add_objects(mdls_);
    inject_hlps();
    td->decompile();
  } else
    inject_hlps();

  csts_.clear();
  mdls_.clear();
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

GTPX::GTPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings, Fact *f_imdl) : _TPX(auto_focus, target, pattern, bindings), f_imdl_(f_imdl) {
}

GTPX::~GTPX() {
}

bool GTPX::take_input(View *input, _Fact *abstracted_input, BindingMap *bm) { // push new input in the time-controlled buffer; old inputs are in front.

  if (!filter(input, abstracted_input, bm))
    return false;

  inputs_.push_back(Input(input, abstracted_input, bm));
  return true;
}

void GTPX::signal(View *input) const { // will be erased from the AF map upon return. P<> kept in reduction job.

  if (!auto_focus_->gtpx_on())
    return;

  if (((_Fact *)input->object_)->is_fact()) { // goal success.

    ReductionJob<GTPX> *j = new ReductionJob<GTPX>(new View(input), (GTPX *)this);
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, "  make ReductionJob<GTPX> " << j->get_job_id() << "(" << j->get_detail_oid() <<
      "): controller(" << get_detail_oid() << ")->reduce(View(fact_" << input->object_->get_oid() << "))");
#endif
    _Mem::Get()->push_reduction_job(j);
  }
}

void GTPX::ack_pred_success(_Fact *predicted_f) { // successful prediction: store; at reduce() time, check if the target was successfully predicted and if so, abort mdl building.

  predictions_.push_back(predicted_f);
}

void GTPX::reduce(r_exec::View *input) { // input->object: f->success.

  _Fact *consequent = (_Fact *)input->object_->get_reference(0)->get_reference(1);
  P<BindingMap> consequent_bm = new BindingMap();
  {
    // Call abstract_object only to update the binding map.
    P<Code> unused = consequent_bm->abstract_object(consequent, false);
  }

  for (uint32 i = 0; i < predictions_.size(); ++i) { // check if some models have successfully predicted the target: if so, abort.

    P<BindingMap> bm = new BindingMap(consequent_bm);
    bm->reset_fwd_timings(predictions_[i]);
    if (bm->match_fwd_strict(predictions_[i], consequent))
      return;
  }

  auto analysis_starting_time = Now();

  if (target_->get_reference(0)->code(0).asOpcode() == Opcodes::MkVal)
    return; // this case will be handled by CTPXs.

  P<GuardBuilder> guard_builder;

  microseconds period;
  microseconds lhs_duration;
  microseconds rhs_duration;

  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end();) {

    if (i->input_->get_after() >= consequent->get_after()) { // discard inputs not younger than the consequent.

      i = inputs_.erase(i);
      continue;
    }

    if (i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst) {

      ++i;
      continue; // components will be evaluated first, then the icst will be identified.
    }

    Input cause = *i;

    if (!cause.eligible_cause_) {

      ++i;
      continue;
    }

    if (Utils::Synchronous(cause.input_->get_after(), target_->get_after())) { // cause in sync with the premise: ignore.

      ++i;
      continue;
    }

    guard_builder = new TimingGuardBuilder(period);// TODO: use the durations.

    period = duration_cast<microseconds>(consequent->get_after() - cause.input_->get_after());
    lhs_duration = duration_cast<microseconds>(cause.input_->get_before() - cause.input_->get_after());
    rhs_duration = duration_cast<microseconds>(consequent->get_before() - consequent->get_after());

    vector<FindFIcstResult> results;
    P<Code> new_cst;
    find_f_icst(cause.input_, results, new_cst);
    if (results.size() == 0) {

      if (build_mdl(cause.input_, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    } else {

      if (build_mdl(results[0].f_icst, results[0].component_pattern, consequent, guard_builder, period, new_cst))
        inject_hlps(analysis_starting_time);
    }
    ++i;
  }
}

bool GTPX::build_mdl(_Fact *cause, _Fact *consequent, GuardBuilder *guard_builder, microseconds period) {

  P<BindingMap> bm = new BindingMap();

  uint16 write_index;
  P<Code> m0 = build_mdl_head(bm, 0, cause, consequent, write_index);
  guard_builder->build(m0, NULL, cause, write_index);
  build_mdl_tail(m0, write_index);

  Code *_m0 = ModelBase::Get()->check_existence(m0);
  if (_m0 == NULL)
    return false;
  else if (_m0 == m0) {

    mdls_.push_back(m0);
    return true;
  } else
    return false;
}

bool GTPX::build_mdl(_Fact *f_icst, _Fact *cause_pattern, _Fact *consequent, GuardBuilder *guard_builder, microseconds period, Code *new_cst) {

  P<BindingMap> bm = new BindingMap();

  uint16 write_index;
  P<Code> m0 = build_mdl_head(bm, 0, f_icst, consequent, write_index);
  guard_builder->build(m0, NULL, cause_pattern, write_index);
  build_mdl_tail(m0, write_index);

  Code *_m0 = ModelBase::Get()->check_existence(m0);
  if (_m0 == NULL)
    return false;
  else if (_m0 == m0) {

    if (new_cst)
      csts_.push_back(new_cst);
    mdls_.push_back(m0);
    return true;
  } else // if m0 already exist, new_cst==NULL.
    return false;
}

std::string GTPX::get_header() const {

  return std::string("GTPX");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PTPX::PTPX(AutoFocusController *auto_focus, _Fact *target, _Fact *pattern, BindingMap *bindings, Fact *f_imdl) : _TPX(auto_focus, target, pattern, bindings), f_imdl_(f_imdl) {
}

PTPX::~PTPX() {
}

void PTPX::signal(View *input) const { // will be erased from the AF map upon return. P<> kept in reduction job.

  if (!auto_focus_->ptpx_on())
    return;

  if (((_Fact *)input->object_)->is_anti_fact()) { // prediction failure.

    ReductionJob<PTPX> *j = new ReductionJob<PTPX>(new View(input), (PTPX *)this);
#ifdef WITH_DETAIL_OID
    OUTPUT_LINE((TraceLevel)0, "  make ReductionJob<PTPX> " << j->get_job_id() << "(" << j->get_detail_oid() <<
      "): controller(" << get_detail_oid() << ")->reduce(View(fact_" << input->object_->get_oid() << "))");
#endif
    _Mem::Get()->push_reduction_job(j);
  }
}

void PTPX::reduce(r_exec::View *input) {

  auto_focus_->copy_cross_buffer(inputs_); // the cause of the prediction failure comes before the prediction.

  auto analysis_starting_time = Now();

  // input->object is the prediction failure: ignore and consider |f->imdl instead.
  Fact* f_imdl = (Fact *)f_imdl_;
  Timestamp f_imdl_after = f_imdl->get_after();
  Timestamp f_imdl_before = f_imdl->get_before();
  // Use the timestamps in the template parameters, similar to PrimaryMDLController::abduce_imdl.
  // This is to make it symmetric with the timestamp in the forward chaining requirement.
  MDLController::get_imdl_template_timings(f_imdl->get_reference(0), f_imdl_after, f_imdl_before);
  P<_Fact> consequent = new Fact(f_imdl->get_reference(0), f_imdl_after, f_imdl_before, f_imdl->get_cfd(), f_imdl->get_psln_thr());
  consequent->set_opposite();

  P<BindingMap> end_bm = new BindingMap();
  {
    // Call abstract_object only to update the binding map.
    P<Code> unused = end_bm->abstract_object(consequent, false);
  }
  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end();) { // filter out inputs irrelevant for the prediction.

    if (i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::Cmd) // no cmds as req lhs (because no bwd-operational); prefer: cmd->effect, effect->imdl.
      i = inputs_.erase(i);
    else if (!end_bm->intersect(i->bindings_) || // discard inputs that do not share values with the consequent.
             i->input_->get_after() > consequent->get_after()) // discard inputs that started after the consequent started.
      i = inputs_.erase(i);
    else
      ++i;
  }

  P<GuardBuilder> guard_builder;
  microseconds period;
  microseconds lhs_duration;
  microseconds rhs_duration;

  for (i = inputs_.begin(); i != inputs_.end(); ++i) {

    if (i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst)
      continue; // components will be evaluated first, then the icst will be identified.

    Input cause = *i;

    if (!cause.eligible_cause_)
      continue;

    period = duration_cast<microseconds>(consequent->get_after() - cause.input_->get_after());
    lhs_duration = duration_cast<microseconds>(cause.input_->get_before() - cause.input_->get_after());
    rhs_duration = duration_cast<microseconds>(consequent->get_before() - consequent->get_after());
    if (period.count() == 0 && lhs_duration == rhs_duration)
      // The LHS and RHS timings are the same, so we don't need guards. This often happens if the timings of the
      // RHS imdl came from an icst.
      guard_builder = new GuardBuilder();
    else
      guard_builder = new TimingGuardBuilder(period); // TODO: use the durations.

    vector<FindFIcstResult> results;
    P<Code> new_cst;
    find_f_icst(cause.input_, results, new_cst, true);
    if (results.size() == 0) {

      if (build_mdl(cause.input_, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    } else {

      for (size_t i = 0; i < results.size(); ++i) {
        if (build_mdl(results[i].f_icst, results[i].component_pattern, consequent, guard_builder, period, new_cst))
          inject_hlps(analysis_starting_time);
      }

      if (!new_cst) {
        // find_f_icst returned results without making a new cst. Check if we can make one.
        _Fact* cause_pattern;
        P<_Fact> f_icst = make_f_icst(cause.input_, cause_pattern, new_cst);
        if (!!f_icst) {
          if (build_mdl(f_icst, cause_pattern, consequent, guard_builder, period, new_cst))
            inject_hlps(analysis_starting_time);
        }
      }
    }
  }
}

bool PTPX::build_mdl(_Fact *cause, _Fact *consequent, GuardBuilder *guard_builder, microseconds period) {

  P<BindingMap> bm = new BindingMap();

  uint16 write_index;
  P<Code> m0 = build_mdl_head(bm, 0, cause, consequent, write_index);
  guard_builder->build(m0, NULL, cause, write_index);
  build_mdl_tail(m0, write_index);

  Code *_m0 = ModelBase::Get()->check_existence(m0);
  if (_m0 == NULL)
    return false;
  else if (_m0 == m0) {

    mdls_.push_back(m0);
    return true;
  } else
    return false;
}

bool PTPX::build_mdl(_Fact *f_icst, _Fact *cause_pattern, _Fact *consequent, GuardBuilder *guard_builder, microseconds period, Code *new_cst) {

  P<BindingMap> bm = new BindingMap();

  uint16 write_index;
  P<Code> m0 = build_mdl_head(bm, 0, f_icst, consequent, write_index);
  guard_builder->build(m0, NULL, cause_pattern, write_index);
  build_mdl_tail(m0, write_index);

  Code *_m0 = ModelBase::Get()->check_existence(m0);
  if (_m0 == NULL)
    return false;
  else if (_m0 == m0) {

    if (new_cst)
      csts_.push_back(new_cst);
    mdls_.push_back(m0);
    return true;
  } else // if m0 already exist, new_cst==NULL.
    return false;
}

std::string PTPX::get_header() const {

  return std::string("PTPX");
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

CTPX::CTPX(AutoFocusController *auto_focus, View *premise) : _TPX(auto_focus, premise->object_), stored_premise_(false), premise_(premise) {
}

CTPX::~CTPX() {
}

void CTPX::store_input(r_exec::View *input) {

  _Fact *input_object = (_Fact *)input->object_;
  P<BindingMap> bm = new BindingMap();
  _Fact *abstracted_input = (_Fact *)bm->abstract_object(input_object, false);
  Input i(input, abstracted_input, bm);
  inputs_.push_front(i);
  if (input_object == target_)
    stored_premise_ = true;
}

void CTPX::signal(r_exec::View *input) {

  View *_view = new View(input); // controller not copied.
  ReductionJob<CTPX> *j = new ReductionJob<CTPX>(_view, this); // holds a reference to this.
#ifdef WITH_DETAIL_OID
  OUTPUT_LINE((TraceLevel)0, "  make ReductionJob " << j->get_job_id() << "(" << j->get_detail_oid() <<
    "): CTPX(" << get_detail_oid() << ")->reduce(View(fact_" << input->object_->get_oid() << "))");
#endif
  _Mem::Get()->push_reduction_job(j);
}

void CTPX::reduce(r_exec::View *input) {

  auto analysis_starting_time = Now();

  if (!stored_premise_)
    inputs_.push_back(Input(premise_, abstracted_target_, target_bindings_));

  _Fact *consequent = (_Fact *)input->object_; // counter-evidence for the premise.

  P<BindingMap> end_bm = new BindingMap();
  {
    // Call abstract_object only to update the binding map.
    P<Code> unused = end_bm->abstract_object(consequent, false);
  }

  bool have_delta = false;
  BindingMap bm_with_delta;
  if (target_->get_reference(0)->code(0).asOpcode() == Opcodes::MkVal) {
    // This is the same as the searched_for delta in find_guard_builder.
    float32 delta = consequent->get_reference(0)->code(MK_VAL_VALUE).asFloat() -
      target_->get_reference(0)->code(MK_VAL_VALUE).asFloat();
    if (delta != 0) {
      have_delta = true;
      P<Code> delta_code(new LObject());
      delta_code->code(0) = Atom::Float(delta);
      bm_with_delta.init(delta_code, 0);
    }
  }

  // Add an imdl to this set if the loop would discard it for not sharing values with
  // the target or consequent, but it does share delta. 
  set<_Fact*> only_intersects_delta_imdls;

  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end();) {

    if (i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst) {
      // Require each icst component to share values with the target or consequent.
      bool icstIsOK = true;
      ICST *icst = (ICST*)i->input_->get_reference(0);
      for (uint32 j = 0; j < icst->components_.size(); ++j) {
        P<BindingMap> component_bm = new BindingMap();
        {
          // Call abstract_object only to update the binding map.
          P<Code> unused = component_bm->abstract_object(icst->components_[j], false);
        }
        if (!(target_bindings_->intersect(component_bm) || end_bm->intersect(component_bm))) {
          icstIsOK = false;
          break;
        }
      }

      if (!icstIsOK ||
        i->input_->get_after() >= consequent->get_after()) // discard inputs not younger than the consequent.
        i = inputs_.erase(i);
      else
        ++i;
      continue;
    }

    if (i->input_->get_after() >= consequent->get_after())
      // Discard inputs not younger than the consequent.
      i = inputs_.erase(i);
    else if (!(target_bindings_->intersect(i->bindings_) || end_bm->intersect(i->bindings_))) {
      // Discard inputs that do not share values with the target or consequent.
      // However, allow an imdl which has the delta value.
      if (have_delta && i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::IMdl &&
          bm_with_delta.intersect(i->bindings_)) {
        only_intersects_delta_imdls.insert(i->input_);
        ++i;
      }
      else
        i = inputs_.erase(i);
    }
    else
      ++i;
  }

  bool need_guard = false;
  if (target_->get_reference(0)->code(0).asOpcode() == Opcodes::MkVal) {
    Atom target_val = target_->get_reference(0)->code(MK_VAL_VALUE);
    if (target_val.isFloat())
      need_guard = true;
  }

  auto period = duration_cast<microseconds>(Utils::GetTimestamp<Code>(consequent, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER)); // sampling period.
  P<GuardBuilder> guard_builder;

  for (i = inputs_.begin(); i != inputs_.end(); ++i) {

    if (target_ == i->input_)
      continue;

    if (i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst)
      continue; // components will be evaluated first, then the icst will be identified.

    Input cause = *i;

    if (!cause.eligible_cause_)
      continue;

    if (Utils::Synchronous(cause.input_->get_after(), target_->get_after())) // cause in sync with the premise: ignore.
      continue;

    // If the LHS cause is an imdl, assume that the default guard builder will be sufficient.
    // However, if the cause is an imdl where a value is delta (see above), call find_guard_builder.
    if (need_guard && (cause.input_->get_reference(0)->code(0).asOpcode() != Opcodes::IMdl ||
          only_intersects_delta_imdls.find(cause.input_) != only_intersects_delta_imdls.end())) {

      if ((guard_builder = find_guard_builder(cause.input_, consequent, period)) == NULL)
        continue;
    } else
      guard_builder = get_default_guard_builder(cause.input_, consequent, period);

    vector<FindFIcstResult> results;
    find_f_icst(cause.input_, results);
    if (results.size() == 0) { // m0:[premise.value premise.after premise.before][cause->consequent] and m1:[lhs1->imdl m0[...][...]] with lhs1 either the premise or an icst containing the premise.

      if (build_mdl(cause.input_, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    } else {

      // m0:[premise.value premise.after premise.before][icst->consequent] and m1:[lhs1->imdl m0[...][...]]
      // with lhs1 either the premise or an icst containing the premise.
      if (build_mdl(results[0].f_icst, results[0].component_pattern, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    }
  }
}

GuardBuilder *CTPX::get_default_guard_builder(_Fact *cause, _Fact *consequent, microseconds period) {

  Code *cause_payload = cause->get_reference(0);
  uint16 opcode = cause_payload->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl) {

    auto offset = duration_cast<microseconds>(consequent->get_after() - cause->get_after());
    auto cmd_duration = duration_cast<microseconds>(cause->get_before() - cause->get_after());
    return new NoArgCmdGuardBuilder(period, offset, cmd_duration);
  }

  return new TimingGuardBuilder(period);
}

// Forms:
// 1A - q1=q0+cmd_arg (if the cause is a cmd)
// 1B - q1=q0*cmd_arg with q0 != 0 (if the cause is a cmd)
// 2  - q1=q0+speed*period, with q1=consequent.value, q0=premise.value, speed=cause.value
// 3A - q1=q0*constant with q0 != 0.
// 3B - q1=q0+constant
GuardBuilder *CTPX::find_guard_builder(_Fact *cause, _Fact *consequent, microseconds period) {

  Code *cause_payload = cause->get_reference(0);
  uint16 opcode = cause_payload->code(0).asOpcode();
  if (opcode == Opcodes::Cmd) {
    uint16 cmd_arg_set_index = cause_payload->code(CMD_ARGS).asIndex();
    uint16 cmd_arg_count = cause_payload->code(cmd_arg_set_index).getAtomCount();
    Atom target_val = target_->get_reference(0)->code(MK_VAL_VALUE);
    Atom consequent_val = consequent->get_reference(0)->code(MK_VAL_VALUE);

    // Form 1
    float32 q0 = target_val.asFloat();
    float32 q1 = consequent_val.asFloat();

    // Form 1A
    float32 searched_for = q1 - q0;
    for (uint16 i = 1; i <= cmd_arg_count; ++i) {

      Atom s = cause_payload->code(cmd_arg_set_index + i);
      if (!s.isFloat())
        continue;
      float32 _s = s.asFloat();
      if (Utils::Equal(_s, searched_for)) {
        auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
        return new ACGuardBuilder(period, period - offset, cmd_arg_set_index + i);
      }
    }

    if (q0 != 0) {
      // Form 1B
      searched_for = q1 / q0;
      for (uint16 i = cmd_arg_set_index + 1; i <= cmd_arg_count; ++i) {

        Atom s = cause_payload->code(i);
        if (!s.isFloat())
          continue;
        float32 _s = s.asFloat();
        if (Utils::Equal(_s, searched_for)) {
          auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
          return new MCGuardBuilder(period, period - offset, i);
        }
      }
    }
  }

  else if (opcode == Opcodes::IMdl) {
    // Form 1
    float32 q0 = target_->get_reference(0)->code(MK_VAL_VALUE).asFloat();
    float32 q1 = consequent->get_reference(0)->code(MK_VAL_VALUE).asFloat();

    // Form 1A
    float32 searched_for = q1 - q0;
    Code* imdl = cause->get_reference(0);
    uint16 imdl_exposed_args_index = imdl->code(I_HLP_EXPOSED_ARGS).asIndex();
    uint16 imdl_exposed_args_count = imdl->code(imdl_exposed_args_index).getAtomCount();

    for (uint16 i = 1; i <= imdl_exposed_args_count; ++i) {

      Atom s = imdl->code(imdl_exposed_args_index + i);
      if (!s.isFloat())
        continue;
      float32 _s = s.asFloat();
      if (Utils::Equal(_s, searched_for)) {
        auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
        // Use the exposed args index in what will be the abstracted imdl.
        return new ACGuardBuilder(period, period - offset, BindingMap::get_abstracted_ihlp_exposed_args_index(imdl) + i);
      }
    }

    if (q0 != 0) {
      // Form 1B
      searched_for = q1 / q0;
      for (uint16 i = imdl_exposed_args_index + 1; i <= imdl_exposed_args_count; ++i) {

        Atom s = imdl->code(i);
        if (!s.isFloat())
          continue;
        float32 _s = s.asFloat();
        if (Utils::Equal(_s, searched_for)) {
          auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
          // Use the exposed args index in what will be the abstracted imdl.
          return new MCGuardBuilder(period, period - offset,
            (i - imdl_exposed_args_index) + BindingMap::get_abstracted_ihlp_exposed_args_index(imdl));
        }
      }
    }
  }
  
  else if (opcode == Opcodes::MkVal) {
    // Forms 2 and 3
    Atom s = cause_payload->code(MK_VAL_VALUE);
    if (s.isFloat()) {

      float32 _s = s.asFloat();
      float32 q0 = target_->get_reference(0)->code(MK_VAL_VALUE).asFloat();
      float32 q1 = consequent->get_reference(0)->code(MK_VAL_VALUE).asFloat();

      // Form 2
      float32 searched_for = (q1 - q0) / period.count();
      if (Utils::Equal(_s, searched_for)) {

        auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
        return new SGuardBuilder(period, period - offset);
      }

      if (q0 != 0) {
        // Form 3A
        auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
        return new MGuardBuilder(period, q1 / q0, offset);
      }

      // Form 3B
      auto offset = duration_cast<microseconds>(Utils::GetTimestamp<Code>(cause, FACT_AFTER) - Utils::GetTimestamp<Code>(target_, FACT_AFTER));
      return new AGuardBuilder(period, q1 - q0, offset);
    }
  }

  return NULL;
}

// m0:[premise.value premise.after premise.before][cause->consequent].
// m1:[icst->imdl m0[...][...]] with icst containing the premise.
bool CTPX::build_mdl(_Fact *cause, _Fact *consequent, GuardBuilder *guard_builder, microseconds period) {

  P<HLPBindingMap> bm = new HLPBindingMap();
  bm->init(target_->get_reference(0), MK_VAL_VALUE);
  bm->init(target_, FACT_AFTER);
  bm->init(target_, FACT_BEFORE);

  uint16 write_index;
  // Set allow_shared_timing_vars false. See BindingMap::abstract_fact .
  // NOTE: This solution is strictly temporary. We should have a heuristic for comparing time values which allows deviations. See https://github.com/IIIM-IS/replicode/pull/110 .
  P<Code> m0 = build_mdl_head(bm, 3, cause, consequent, write_index, false);
  guard_builder->build(m0, NULL, cause, write_index);
  build_mdl_tail(m0, write_index);
  return build_requirement(bm, m0, period); // existence checks performed there.
}

// m0:[premise.value premise.after premise.before][icst->consequent] with icst containing the cause.
// m1:[icst->imdl m0[...][...]] with icst containing the premise.
bool CTPX::build_mdl(_Fact *f_icst, _Fact *cause_pattern, _Fact *consequent, GuardBuilder *guard_builder, microseconds period) {

  P<BindingMap> bm = new BindingMap();
  bm->init(target_->get_reference(0), MK_VAL_VALUE);
  bm->init(target_, FACT_AFTER);
  bm->init(target_, FACT_BEFORE);

  uint16 write_index;
  Code *m0 = build_mdl_head(bm, 3, f_icst, consequent, write_index);
  guard_builder->build(m0, NULL, cause_pattern, write_index);
  build_mdl_tail(m0, write_index);

  return build_requirement(bm, m0, period); // existence checks performed there.
}

bool CTPX::build_requirement(HLPBindingMap *bm, Code *m0, microseconds period) { // check for mdl existence at the same time (ModelBase::mdlCS_-wise).

  vector<FindFIcstResult> results;
  P<Code> new_cst;
  find_f_icst(target_, results, new_cst);
  if (results.size() == 0)
    return false;

  _Fact* f_icst = results[0].f_icst;
  _Fact* premise_pattern = results[0].component_pattern;
  P<Fact> f_im0 = bm->build_f_ihlp(m0, Opcodes::IMdl, false);
  // build_f_ihlp can leave some variables pointing into bm, but abstract_object needs everything valuated.
  f_im0->set_reference(0, bm->bind_pattern(f_im0->get_reference(0)));
  Utils::SetTimestamp<Code>(f_im0, FACT_AFTER, f_icst->get_after());
  Utils::SetTimestamp<Code>(f_im0, FACT_BEFORE, f_icst->get_before());

  P<BindingMap> _bm = new BindingMap();

  uint16 write_index;
  P<Code> m1 = build_mdl_head(_bm, 0, f_icst, f_im0, write_index);
  P<GuardBuilder> guard_builder = new GuardBuilder();
  guard_builder->build(m1, premise_pattern, NULL, write_index);
  build_mdl_tail(m1, write_index);

  Code *_m0;
  Code *_m1;
  ModelBase::Get()->check_existence(m0, m1, _m0, _m1);
  if (_m1 == NULL)
    return false;
  else if (_m1 == m1) {

    if (_m0 == NULL)
      return false;
    else if (_m0 == m0)
      mdls_.push_back(m0);
    if (!!new_cst)
      csts_.push_back(new_cst);
    mdls_.push_back(m1);
  } // if m1 alrady exists, new_cst==NULL.
  return true;
}

std::string CTPX::get_header() const {

  return std::string("CTPX");
}
}
