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

#include "auto_focus.h"
#include "reduction_job.h"
#include "mem.h"
#include "model_base.h"

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
    //std::cout<<Utils::RelativeTime(Now())<<" "<<input->object->get_reference(0)->get_reference(1)->get_oid()<<": end of focus["<<target->get_oid()<<"]\n";
}

void TPX::ack_pred_success(_Fact *predicted_f) {
}

bool TPX::filter(View *input, _Fact *abstracted_input, BindingMap *bm) {

  if (input->object_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst) // if we get an icst we are called by auto_focus::dispatch_no_inject: the input is irrelevant.
    return false;
  //std::cout<<Utils::RelativeTime(Now())<<" tpx ["<<target->get_oid()<<"] <- "<<input->object->get_oid();
  if (target_bindings_->intersect(bm)) {//std::cout<<" lvl0"<<std::endl;
    return true; }
  if (target_bindings_->is_fully_specified())
    return false;
  for (uint32 i = 0; i < new_maps_.size(); ++i)
    if (new_maps_[i]->intersect(bm)) {//std::cout<<" lvl1"<<std::endl;
      return true; }

  P<BindingMap> _bm = new BindingMap(target_bindings_);
  _bm->reset_fwd_timings(input->object_);
  time_buffer<CInput, CInput::IsInvalidated> &cache = auto_focus_->get_cache();
  if (_bm->match_fwd_strict(input->object_, (_Fact *)target_->get_reference(0)->get_reference(0))) { // both GTPX and PTPX' target are f0->g/p->f1: we need to match on f1.
//std::cout<<" match";
    new_maps_.push_back(_bm);
    time_buffer<CInput, CInput::IsInvalidated>::iterator i;
    auto now = Now();
    for (i = cache.begin(now); i != cache.end(); ++i) {

      if (i->injected_) {//std::cout<<" ?"<<(*i).input->object->get_oid()<<" ("<<Utils::RelativeTime(i->ijt)<<") skip\n";
        continue; }
      if (_bm->intersect(i->bindings_)) {
        i->injected_ = true;
        auto_focus_->inject_input(i->input_, i->abstraction_, i->bindings_);
        //std::cout<<" ?"<<(*i).input->object->get_oid()<<" ("<<Utils::RelativeTime(i->ijt)<<") success\n";
      }//else std::cout<<" ?"<<(*i).input->object->get_oid()<<" ("<<Utils::RelativeTime(i->ijt)<<") failure\n";
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
    //std::cout<<" cached"<<" ("<<Utils::RelativeTime(input->get_ijt())<<")\n";
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

void _TPX::filter_icst_components(ICST *icst, uint32 icst_index, std::vector<Component> &components) {

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

_Fact *_TPX::_find_f_icst(_Fact *component, uint16 &component_index) {

  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end(); ++i) {

    Code *candidate = i->input_->get_reference(0);
    if (candidate->code(0).asOpcode() == Opcodes::ICst) {

      ICST *icst = (ICST *)candidate;
      if (icst->contains(component, component_index))
        return i->input_;
    }
  }

  std::vector<P<_Fact> >::const_iterator f_icst;
  for (f_icst = icsts_.begin(); f_icst != icsts_.end(); ++f_icst) {

    ICST *icst = (ICST *)(*f_icst)->get_reference(0);
    if (icst->contains(component, component_index))
      return (*f_icst);
  }

  return NULL;
}

_Fact *_TPX::find_f_icst(_Fact *component, uint16 &component_index) {

  uint16 opcode = component->get_reference(0)->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl) // cmds/imdls cannot be components of a cst.
    return NULL;

  return _find_f_icst(component, component_index);
}

_Fact *_TPX::find_f_icst(_Fact *component, uint16 &component_index, P<Code> &new_cst) {

  uint16 opcode = component->get_reference(0)->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl)
    // cmds/imdls cannot be components of a cst.
    return NULL;

  _Fact *f_icst = _find_f_icst(component, component_index);
  if (f_icst != NULL)
    return f_icst;

  return make_f_icst(component, component_index, new_cst);
}

_Fact *_TPX::make_f_icst(_Fact *component, uint16 &component_index, P<Code> &new_cst) {

  uint16 opcode = component->get_reference(0)->code(0).asOpcode();
  if (opcode == Opcodes::Cmd || opcode == Opcodes::IMdl)
    // cmds/imdls cannot be components of a cst.
    return NULL;

  std::vector<Component> components; // no icst found, try to identify components to assemble a cst.
  std::vector<uint32> icst_components;

  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end(); ++i) {

    if (component == i->input_) {

      component_index = components.size();
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
  _Fact *f_icst = bm->build_f_ihlp(new_cst, Opcodes::ICst, false);
  icsts_.push_back(f_icst); // the f_icst can be reused in subsequent model building attempts.
  return f_icst;
}

Code *_TPX::build_cst(const std::vector<Component> &components, BindingMap *bm, _Fact *main_component) {

  _Fact *abstracted_component = (_Fact *)bm->abstract_object(main_component, false);

  Code *cst = _Mem::Get()->build_object(Atom::CompositeState(Opcodes::Cst, CST_ARITY));

  uint16 actual_component_count = 0;
  for (uint16 i = 0; i < components.size(); ++i) { // reference patterns;

    if (components[i].discarded)
      continue;
    if (components[i].object == main_component)
      cst->add_reference(abstracted_component);
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

  cst->add_reference(auto_focus_->getView()->get_host()); // reference the output group.

  return cst;
}

Code *_TPX::build_mdl_head(HLPBindingMap *bm, uint16 tpl_arg_count, _Fact *lhs, _Fact *rhs, uint16 &write_index, bool allow_shared_timing_vars) {

  Code *mdl = _Mem::Get()->build_object(Atom::Model(Opcodes::Mdl, MDL_ARITY));

  mdl->add_reference(bm->abstract_object(lhs, false, allow_shared_timing_vars)); // reference lhs.
  mdl->add_reference(bm->abstract_object(rhs, false, allow_shared_timing_vars)); // reference rhs.

  write_index = MDL_ARITY;

  mdl->code(MDL_TPL_ARGS) = Atom::IPointer(++write_index);
  mdl->code(write_index) = Atom::Set(tpl_arg_count);
  for (uint16 i = 0; i < tpl_arg_count; ++i)
    mdl->code(++write_index) = Atom::VLPointer(i);

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

  mdl->add_reference(auto_focus_->getView()->get_host()); // reference the output group.
}

void _TPX::inject_hlps() const {

  std::vector<P<Code> >::const_iterator c;
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
    header = Time::ToString_seconds(Now() - Utils::GetTimeReference());
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
  _Fact *abstracted_consequent = (_Fact *)consequent_bm->abstract_object(consequent, false);

  for (uint32 i = 0; i < predictions_.size(); ++i) { // check if some models have successfully predicted the target: if so, abort.

    P<BindingMap> bm = new BindingMap(consequent_bm);
    bm->reset_fwd_timings(predictions_[i]);
    if (bm->match_fwd_strict(predictions_[i], consequent))
      return;
  }

  auto analysis_starting_time = Now();

  bool need_guard;
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

    uint16 cause_index;
    P<Code> new_cst;
    _Fact *f_icst = find_f_icst(cause.input_, cause_index, new_cst);
    if (f_icst == NULL) {

      if (build_mdl(cause.input_, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    } else {

      Code *unpacked_cst;
      if (!new_cst) {

        Code *cst = f_icst->get_reference(0)->get_reference(0);
        unpacked_cst = cst->get_reference(cst->references_size() - CST_HIDDEN_REFS); // the cst is packed, retrieve the pattern from the unpacked code.
      } else
        unpacked_cst = new_cst;

      _Fact *cause_pattern = (_Fact *)unpacked_cst->get_reference(cause_index);
      if (build_mdl(f_icst, cause_pattern, consequent, guard_builder, period, new_cst))
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

  _Fact *consequent = new Fact((Fact *)f_imdl_); // input->object is the prediction failure: ignore and consider |f->imdl instead.
  consequent->set_opposite();

  P<BindingMap> end_bm = new BindingMap();
  P<_Fact> abstract_input = (_Fact *)end_bm->abstract_object(consequent, false);
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

    uint16 cause_index;
    P<Code> new_cst;
    _Fact *f_icst = find_f_icst(cause.input_, cause_index, new_cst);
    if (f_icst == NULL) {

      if (build_mdl(cause.input_, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    } else {

      Code *unpacked_cst;
      if (!new_cst) {

        Code *cst = f_icst->get_reference(0)->get_reference(0);
        unpacked_cst = cst->get_reference(cst->references_size() - CST_HIDDEN_REFS); // the cst is packed, retrieve the pattern from the unpacked code.
      } else
        unpacked_cst = new_cst;

      _Fact *cause_pattern = (_Fact *)unpacked_cst->get_reference(cause_index);
      if (build_mdl(f_icst, cause_pattern, consequent, guard_builder, period, new_cst))
        inject_hlps(analysis_starting_time);
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
  P<_Fact> abstract_input = (_Fact *)end_bm->abstract_object(consequent, false);
  r_code::list<Input>::const_iterator i;
  for (i = inputs_.begin(); i != inputs_.end();) {

    if (i->input_->get_reference(0)->code(0).asOpcode() == Opcodes::ICst) {
      // Require each icst component to share values with the target or consequent.
      bool icstIsOK = true;
      ICST *icst = (ICST*)i->input_->get_reference(0);
      for (uint32 j = 0; j < icst->components_.size(); ++j) {
        P<BindingMap> component_bm = new BindingMap();
        P<_Fact> unused = (_Fact *)component_bm->abstract_object(icst->components_[j], false);
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
    if (!(target_bindings_->intersect(i->bindings_) || end_bm->intersect(i->bindings_)) || // discard inputs that do not share values with the target or consequent.
      i->input_->get_after() >= consequent->get_after()) // discard inputs not younger than the consequent.
      i = inputs_.erase(i);
    else
      ++i;
  }

  bool need_guard;
  if (target_->get_reference(0)->code(0).asOpcode() == Opcodes::MkVal)
    need_guard = target_->get_reference(0)->code(MK_VAL_VALUE).isFloat();
  else
    need_guard = false;

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

    if (need_guard) {

      if ((guard_builder = find_guard_builder(cause.input_, consequent, period)) == NULL)
        continue;
    } else
      guard_builder = get_default_guard_builder(cause.input_, consequent, period);

    uint16 cause_index;
    _Fact *f_icst = find_f_icst(cause.input_, cause_index);
    if (f_icst == NULL) { // m0:[premise.value premise.after premise.before][cause->consequent] and m1:[lhs1->imdl m0[...][...]] with lhs1 either the premise or an icst containing the premise.

      if (build_mdl(cause.input_, consequent, guard_builder, period))
        inject_hlps(analysis_starting_time);
    } else {

      Code *cst = f_icst->get_reference(0)->get_reference(0); // cst is packed.
      Code *unpacked_cst = cst->get_reference(cst->references_size() - CST_HIDDEN_REFS); // get the unpacked code to retreive the pattern.
      _Fact *cause_pattern = (_Fact *)cst->get_reference(cause_index);
      if (build_mdl(f_icst, cause_pattern, consequent, guard_builder, period)) // m0:[premise.value premise.after premise.before][icst->consequent] and m1:[lhs1->imdl m0[...][...]] with lhs1 either the premise or an icst containing the premise.
        inject_hlps(analysis_starting_time);
    }
  }
}

GuardBuilder *CTPX::get_default_guard_builder(_Fact *cause, _Fact *consequent, microseconds period) {

  Code *cause_payload = cause->get_reference(0);
  uint16 opcode = cause_payload->code(0).asOpcode();
  if (opcode == Opcodes::Cmd) {

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
    // Form 1
    float32 q0 = target_->get_reference(0)->code(MK_VAL_VALUE).asFloat();
    float32 q1 = consequent->get_reference(0)->code(MK_VAL_VALUE).asFloat();

    // Form 1A
    float32 searched_for = q1 - q0;
    uint16 cmd_arg_set_index = cause_payload->code(CMD_ARGS).asIndex();
    uint16 cmd_arg_count = cause_payload->code(cmd_arg_set_index).getAtomCount();
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
  } else if (opcode == Opcodes::MkVal) {
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
  //std::cout<<Utils::RelativeTime(Now())<<" found --------------------- M0\n";
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

  uint16 premise_index;
  P<Code> new_cst;
  _Fact *f_icst = find_f_icst(target_, premise_index, new_cst);
  if (f_icst == NULL) {//std::cout<<Utils::RelativeTime(Now())<<" failed xxxxxxxxx M1 / 0\n";
    return false; }

  P<Fact> f_im0 = bm->build_f_ihlp(m0, Opcodes::IMdl, false);
  Utils::SetTimestamp<Code>(f_im0, FACT_AFTER, f_icst->get_after());
  Utils::SetTimestamp<Code>(f_im0, FACT_BEFORE, f_icst->get_before());

  Code *unpacked_cst;
  if (!new_cst) {

    Code *cst = f_icst->get_reference(0)->get_reference(0);
    unpacked_cst = cst->get_reference(cst->references_size() - CST_HIDDEN_REFS); // the cst is packed, retrieve the pattern from the unpacked code.
  } else
    unpacked_cst = new_cst;
  _Fact *premise_pattern = (_Fact *)unpacked_cst->get_reference(premise_index);

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
  //std::cout<<Utils::RelativeTime(Now()<<" found --------------------- M1\n";
  return true;
}

std::string CTPX::get_header() const {

  return std::string("CTPX");
}
}
