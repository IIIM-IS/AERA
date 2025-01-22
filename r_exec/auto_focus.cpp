//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2025 Jeff Thompson
//_/_/ Copyright (c) 2018-2025 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2025 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2018 Jacqueline Clare Mallett
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

#include "mem.h"
#include "auto_focus.h"
#include "ast_controller.h"

using namespace std;
using namespace r_code;

namespace r_exec {

AutoFocusController::AutoFocusController(_View *view) : Controller(view) {

  // Load arguments: pass_through, acquire_models, decompile_models, list of output groups: 1st must be the primary, 2nd the secondary, then other groups.
  Code *icpp_pgm = get_object();
  uint16 arg_set_index = icpp_pgm->code(ICPP_PGM_ARGS).asIndex();
  uint16 arg_count = icpp_pgm->code(arg_set_index).getAtomCount();
  uint8 i = 1;
  pass_through_ = icpp_pgm->code(arg_set_index + i++).asBoolean();
  ctpx_on_ = icpp_pgm->code(arg_set_index + i++).asBoolean();
  gtpx_on_ = icpp_pgm->code(arg_set_index + i++).asBoolean();
  ptpx_on_ = icpp_pgm->code(arg_set_index + i++).asBoolean();
  trace_injections_ = icpp_pgm->code(arg_set_index + i++).asBoolean();
  decompile_models_ = icpp_pgm->code(arg_set_index + i).asBoolean();
  for (uint16 j = i; j < arg_count; ++j)
    output_groups_.push_back((Group *)icpp_pgm->get_reference(j - i));

  cross_buffer_.set_thz(_Mem::Get()->get_tpx_time_horizon());
  cross_buffer_.reserve(CrossBufferInitialSize);
  auto thz = 2 * ((r_exec::View*)view)->get_host()->get_upr()*Utils::GetBasePeriod(); // thz==2*sampling period.
  cache_.set_thz(thz);
  cache_.reserve(CacheInitialSize);

#ifdef WITH_DETAIL_OID
  OUTPUT_LINE(AUTO_FOCUS, "new controller(" << get_detail_oid() << ") for auto_focus " << view->object_->get_oid());
#endif
}

AutoFocusController::~AutoFocusController() {
}

Code *AutoFocusController::get_core_object() const {

  return get_object(); // icpp_pgm.
}

inline void AutoFocusController::inject_input(View *input, uint32 start) {

  Group *origin = input->get_host();
  for (uint16 i = start; i < output_groups_.size(); ++i) {

    Group *output_group = output_groups_[i];
    View *view = new View(input, true);
    view->references_[0] = output_group;
    view->code(VIEW_RES) = Atom::Float(Utils::GetResilience(view->code(VIEW_RES).asFloat(), origin->get_upr(), output_group->get_upr()));
    _Mem::Get()->inject(view);
  }
}

inline void AutoFocusController::inject_input(View *input, _Fact *abstract_input, BindingMap *bm) {

  View *primary_view = inject_input(input);
  cross_buffer_.push_back(Input(primary_view, abstract_input, bm));
}

inline View *AutoFocusController::inject_input(View *input) {

  _Fact *input_fact = (_Fact *)input->object_;

  Group *origin = input->get_host();
  Group *ref_group = output_groups_[0];

  auto now = Now();

  View *primary_view;
  _Fact *copy;
  switch (input->get_sync()) {
  case View::SYNC_ONCE: // no copy, morph res; N.B.: cmds are sync_once.
    for (uint16 i = 0; i < output_groups_.size(); ++i) {

      Group *output_group = output_groups_[i];
      View *view = new View(input, true);
      view->references_[0] = output_group;
      view->references_[1] = input->references_[0];
      view->code(VIEW_RES) = Atom::Float(Utils::GetResilience(view->code(VIEW_RES).asFloat(), origin->get_upr(), output_group->get_upr()));
      _Mem::Get()->inject(view);
      if (i == 0)
        primary_view = view;
    }
    break;
  case View::SYNC_PERIODIC: // inject a copy, morph res, add a controller.
    if (input_fact->is_anti_fact())
      copy = new AntiFact(input_fact->get_reference(0), ref_group->get_prev_upr_time(now), ref_group->get_next_upr_time(now), 1, 1);
    else
      copy = new Fact(input_fact->get_reference(0), ref_group->get_prev_upr_time(now), ref_group->get_next_upr_time(now), 1, 1);
    for (uint16 i = 0; i < output_groups_.size(); ++i) {

      Group *output_group = output_groups_[i];
      View *view = new View(input, true);
      view->references_[0] = output_group;
      view->references_[1] = input->references_[0];
      view->code(VIEW_RES) = Atom::Float(Utils::GetResilience(view->code(VIEW_RES).asFloat(), origin->get_upr(), output_group->get_upr()));
      view->object_ = copy;
      _Mem::Get()->inject(view);
      if (i == 0) {

        primary_view = view;
        // Don't use CTPX to model changes in anti-facts.
        if (ctpx_on_ && input_fact->is_fact())
          // Set time_to_live to 2 frame periods so that PASTController stays valid during the whole frame when CTPX may build models.
          _Mem::Get()->inject_null_program(new PASTController(this, view), output_group, 2 * output_group->get_upr()*Utils::GetBasePeriod(), true);
      }
    }
    break;
  case View::SYNC_HOLD: { // inject a copy, add a controller, sync_once, morph res, after=now+time_tolerance (de-sync as it can have the same effect as a cmd), before=now+output_grp.upr+time_tolerance.
    auto offset = 2 * Utils::GetTimeTolerance();
    if (input_fact->is_anti_fact())
      copy = new AntiFact(input_fact->get_reference(0), now + offset, now + ref_group->get_upr()*Utils::GetBasePeriod(), 1, 1);
    else
      copy = new Fact(input_fact->get_reference(0), now + offset, now + ref_group->get_upr()*Utils::GetBasePeriod(), 1, 1);
    for (uint16 i = 0; i < output_groups_.size(); ++i) {

      Group *output_group = output_groups_[i];
      View *view = new View(input, true);
      view->references_[0] = output_group;
      view->references_[1] = input->references_[0];
      view->code(VIEW_SYNC) = Atom::Float(View::SYNC_ONCE);
      view->code(VIEW_RES) = Atom::Float(Utils::GetResilience(view->code(VIEW_RES).asFloat(), origin->get_upr(), output_group->get_upr()));
      view->object_ = copy;
      _Mem::Get()->inject(view);
      if (i == 0) {

        primary_view = view;
        if (ctpx_on_)
          _Mem::Get()->inject_null_program(new HASTController(this, view, input_fact), output_group, output_group->get_upr()*Utils::GetBasePeriod(), true);
      }
    }
    break;
  }case View::SYNC_AXIOM: // inject a copy, sync_once, res=1, fact.before=next output_grp upr.
    if (input_fact->is_anti_fact())
      copy = new AntiFact(input_fact->get_reference(0), ref_group->get_prev_upr_time(now), ref_group->get_next_upr_time(now), 1, 1);
    else
      copy = new Fact(input_fact->get_reference(0), ref_group->get_prev_upr_time(now), ref_group->get_next_upr_time(now), 1, 1);
    for (uint16 i = 0; i < output_groups_.size(); ++i) {

      Group *output_group = output_groups_[i];
      View *view = new View(input, true);
      view->references_[0] = output_group;
      view->references_[1] = input->references_[0];
      view->code(VIEW_SYNC) = Atom::Float(View::SYNC_ONCE_AXIOM);
      view->code(VIEW_RES) = Atom::Float(1);
      view->object_ = copy;
      _Mem::Get()->inject(view);
      if (i == 0)
        primary_view = view;
    }
    break;
  }

  if (trace_injections_) {

    const char* syncString = "";
    switch (input->get_sync()) {
    case View::SYNC_HOLD: syncString = "(HOLD)"; break;
    case View::SYNC_ONCE: syncString = "(ONCE)"; break;
    case View::SYNC_PERIODIC: syncString = "(PERIODIC)"; break;
    case View::SYNC_AXIOM: syncString = "(AXIOM)"; break;
    }
    OUTPUT_LINE(AUTO_FOCUS, Utils::RelativeTime(Now()) << " A/F -> " << input->object_->get_oid() << "|" <<
      primary_view->object_->get_oid() << " " << syncString);
  }

  return primary_view;
}

inline void AutoFocusController::notify(_Fact *target, View *input, TPXMap &map) {

  TPXMap::const_iterator m = map.find(target);
  if (m != map.end()) { // shall always be the case.

    m->second->signal(input); // will spawn a ReductionJob holding a P<> on m->second.
    map.erase(m);
  }
}

inline void AutoFocusController::dispatch_pred_success(Success* success, TPXMap &map) {

  TPXMap::const_iterator m;
  for (m = map.begin(); m != map.end(); ++m)
    m->second->ack_pred_success(success);
}

inline void AutoFocusController::dispatch(View *input, _Fact *abstract_input, BindingMap *bm, bool &injected, TPXMap &map) {

  TPXMap::const_iterator m;
  for (m = map.begin(); m != map.end(); ++m) {

    if (m->second->take_input(input, abstract_input, bm)) {

      if (!injected) {

        injected = true;
        inject_input(input, abstract_input, bm);
      }
    }
  }
}

inline void AutoFocusController::dispatch_no_inject(View *input, _Fact *abstract_input, BindingMap *bm, TPXMap &map) {

  TPXMap::const_iterator m;
  for (m = map.begin(); m != map.end(); ++m)
    m->second->take_input(input, abstract_input, bm);
}

void AutoFocusController::take_input(r_exec::View *input) {

  if (is_invalidated())
    return;
  if (input->object_->code(0).asOpcode() == Opcodes::Fact ||
    input->object_->code(0).asOpcode() == Opcodes::AntiFact ||
    input->object_->code(0).asOpcode() == Opcodes::MkRdx) // discard everything but facts, |facts and mk.rdx.
    Controller::__take_input<AutoFocusController>(input);
}

void AutoFocusController::reduce(r_exec::View *input) {

  Code *input_object = input->object_;
  uint16 opcode = input_object->code(0).asOpcode();

  reductionCS_.enter();

  if (opcode == Opcodes::MkRdx) {

    Fact *f_ihlp = (Fact *)input_object->get_reference(MK_RDX_IHLP_REF);
    BindingMap *bm = ((MkRdx *)input_object)->bindings_;
    if (f_ihlp->get_reference(0)->code(0).asOpcode() == Opcodes::IMdl) { // handle new goals/predictions as new targets.

      Code *mdl = f_ihlp->get_reference(0)->get_reference(0);
      Code *unpacked_mdl = mdl->get_reference(mdl->references_size() - MDL_HIDDEN_REFS);
      uint16 obj_set_index = unpacked_mdl->code(MDL_OBJS).asIndex();

      // Get the first production in the set of productions.
      uint16 production_set_index = input_object->code(MK_RDX_PRODS).asIndex();
      uint16 production_reference_index = input_object->code(production_set_index + 1).asIndex();
      Code *production = input_object->get_reference(production_reference_index);

      _Fact *pattern;
      TPX *tpx;
      Goal *goal = ((_Fact *)production)->get_goal();
      if (goal != NULL) { // build a tpx to find models like M:[A -> B] where B is the goal target.

        pattern = (_Fact *)unpacked_mdl->get_reference(unpacked_mdl->code(obj_set_index + 1).asIndex()); // lhs.
        tpx = build_tpx<GTPX>((_Fact *)production, pattern, bm, goal_ratings_, f_ihlp, f_ihlp->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED).asBoolean());
        goals_.insert(std::make_pair((_Fact *)production, tpx));
      } else {

        Pred *pred = ((_Fact *)production)->get_pred();
        if (pred != NULL) { // build a tpx to find models like M:[A -> |imdl M0] where M0 is the model that produced the prediction.

          pattern = (_Fact *)unpacked_mdl->get_reference(unpacked_mdl->code(obj_set_index + 2).asIndex()); // rhs.
          tpx = build_tpx<PTPX>((_Fact *)production, pattern, bm, prediction_ratings_, f_ihlp, f_ihlp->get_reference(0)->code(I_HLP_WEAK_REQUIREMENT_ENABLED).asBoolean());
          predictions_.insert(std::make_pair((_Fact *)production, tpx));
        }
      }
    }
  } else {

    bool success = (opcode == Opcodes::Fact);
    if (success || opcode == Opcodes::AntiFact) { // discard everything but facts.

      Code *payload = input_object->get_reference(0);
      uint16 opcode = payload->code(0).asOpcode();
      if (opcode == Opcodes::Success) { // input_object is f->success->payload, where payload is f->g or f->p; trim down the target list, rate targets, signal tpx.

        _Fact *target = (_Fact *)payload->get_reference(0);
        Goal *goal = target->get_goal();
        if (goal != NULL) {

          notify(target, input, goals_);
        } else { // prediction.

          notify(target, input, predictions_);
          if (success) // a mdl has correctly predicted a GTPX's target: the GTPX shall not produce anything: we need to pass the prediction to all GTPX.
            dispatch_pred_success((Success*)payload, goals_);
        }
      } else if (opcode == Opcodes::Perf)
        inject_input(input, 2); // inject in all output groups but the primary and secondary.
      else { // filter according to targets: inject (once) when possible and pass to TPX if any.

        if (pass_through_) {

          View* primary_view = NULL;
          if (opcode != Opcodes::ICst) // don't inject again (since it comes from inside).
            primary_view = inject_input(input);

          if (ptpx_on_) {
            // The PTPX may need the input. Save it in cross_buffer_ up to tpx_time_horizon.
            if (opcode == Opcodes::ICst) {
              P<BindingMap> bm = ((ICST *)payload)->bindings_;
              _Fact *abstract_f_ihlp = bm->abstract_f_ihlp((_Fact *)input_object);
              cross_buffer_.push_back(Input(input, abstract_f_ihlp, bm));
            }
            else {
              P<BindingMap> bm = new BindingMap();
              P<_Fact> abstract_input = (_Fact *)bm->abstract_object(input_object, false);
              cross_buffer_.push_back(Input(primary_view, abstract_input, bm));
            }
          }
        } else {

          P<BindingMap> bm = new BindingMap();
          if (opcode == Opcodes::ICst) { // dispatch but don't inject again (since it comes from inside).

            bm = ((ICST *)payload)->bindings_;
            _Fact *abstract_f_ihlp = bm->abstract_f_ihlp((_Fact *)input_object);
            dispatch_no_inject(input, abstract_f_ihlp, bm, goals_);
            dispatch_no_inject(input, abstract_f_ihlp, bm, predictions_);
            cross_buffer_.push_back(Input(input, abstract_f_ihlp, bm));
          } else {

            P<_Fact> abstract_input = (_Fact *)bm->abstract_object(input_object, false);
            bool injected = false;
            dispatch(input, abstract_input, bm, injected, goals_);
            dispatch(input, abstract_input, bm, injected, predictions_);
          }
        }
      }
    }
  }

  reductionCS_.leave();
}

void AutoFocusController::inject_hlps(const vector<P<Code> > &hlps) const { // inject in the primary group; models will be injected in the secondary group automatically.

  vector<View *> views;

  auto now = Now();

  vector<P<Code> >::const_iterator hlp;
  for (hlp = hlps.begin(); hlp != hlps.end(); ++hlp) {

    View *view = new View(View::SYNC_ONCE, now, 0, -1, output_groups_[0], NULL, *hlp, 1); // SYNC_ONCE,sln=0,res=forever,act=1.
    view->references_[0] = output_groups_[0];
    views.push_back(view);
  }

  _Mem::Get()->inject_hlps(views, output_groups_[0]);
}

void AutoFocusController::copy_cross_buffer(r_code::list<Input> &destination) { // copy inputs so they can be flagged independently by the tpxs that share the cross buffer.

  reductionCS_.enter();
  time_buffer<Input, Input::IsInvalidated>::iterator i;
  for (i = cross_buffer_.begin(Now()); i != cross_buffer_.end(); ++i)
    destination.push_back(Input(*i));
  reductionCS_.leave();
}
}
