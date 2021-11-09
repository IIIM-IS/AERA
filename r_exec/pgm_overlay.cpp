//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2021 Jeff Thompson
//_/_/ Copyright (c) 2018-2021 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018 Thor Tomasarson
//_/_/ Copyright (c) 2018-2021 Icelandic Institute for Intelligent Machines
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

#include "pgm_overlay.h"
#include "pgm_controller.h"
#include "mem.h"
#include "group.h"
#include "opcodes.h"
#include "context.h"
#include "callbacks.h"

using namespace std;
using namespace r_code;

// pgm layout:
//
// index content
//
// PGM_TPL_ARGS >iptr to the tpl args set
// PGM_INPUTS >iptr to the pattern set
// PGM_GUARDS >iptr to the guard set
// PGM_PRODS >iptr to the production set
// pgm_code[PGM_TPL_ARGS] >tpl arg set #n0
// pgm_code[PGM_TPL_ARGS]+1 >iptr to first tpl pattern
// ... >...
// pgm_code[PGM_TPL_ARGS]+n0 >iptr to last tpl pattern
// pgm_code[pgm_code[PGM_TPL_ARGS]+1] >opcode of the first tpl pattern
// ... >...
// pgm_code[pgm_code[PGM_TPL_ARGS]+n0] >opcode of the last tpl pattern
// pgm_code[PGM_INPUTS] >input pattern set #n1
// pgm_code[PGM_INPUTS]+1 >iptr to first input pattern
// ... >...
// pgm_code[PGM_INPUTS]+n1 >iptr to last input pattern
// pgm_code[pgm_code[PGM_INPUTS]+1] >opcode of the first input pattern
// ... >...
// pgm_code[pgm_code[PGM_INPUTS]+n1] >opcode of the last input pattern
// ... >...
// pgm_code[PGM_GUARDS] >guard set #n2
// pgm_code[PGM_GUARDS]+1 >iptr to first guard
// ... >...
// pgm_code[PGM_GUARDS]+n2 >iptr to last guard
// pgm_code[pgm_code[PGM_GUARDS]+1] >opcode of the first guard
// ... >...
// pgm_code[pgm_code[PGM_GUARDS]+n2] >opcode of the last guard
// ... >...
// pgm_code[PGM_PRODS] >production set #n3
// pgm_code[PGM_PRODS]+1 >iptr to first production
// ... >...
// pgm_code[PGM_PRODS]+n3 >iptr to last production
// pgm_code[pgm_code[PGM_PRODS]+1] >opcode of the first production
// ... >...
// pgm_code[pgm_code[PGM_PRODS]+n3] >opcode of the last production
// ... >...

using namespace std::chrono;

namespace r_exec {

InputLessPGMOverlay::InputLessPGMOverlay() : Overlay() { // used for constructing PGMOverlay offsprings.
}

InputLessPGMOverlay::InputLessPGMOverlay(Controller *c) : Overlay(c) {

  patch_tpl_args();
}

InputLessPGMOverlay::~InputLessPGMOverlay() {
}

void InputLessPGMOverlay::reset() {

  Overlay::reset();

  patch_tpl_args();

  patch_indices_.clear();
  value_commit_index_ = 0;
  values_.as_std()->clear();
  productions_.clear();
}

bool InputLessPGMOverlay::evaluate(uint16 index) {

  IPGMContext c(get_object()->get_reference(0), get_view(), code_, index, this);
  return c.evaluate();
}

void InputLessPGMOverlay::patch_tpl_args() { // no rollback on that part of the code.
    //get_object()->trace();
  uint16 tpl_arg_set_index = code_[PGM_TPL_ARGS].asIndex(); // index to the set of all tpl patterns.
  uint16 arg_count = code_[tpl_arg_set_index].getAtomCount();
  uint16 ipgm_arg_set_index = get_object()->code(IPGM_ARGS).asIndex(); // index to the set of all ipgm tpl args.
  for (uint16 i = 1; i <= arg_count; ++i) { // pgm_code[tpl_arg_set_index+i] is an iptr to a pattern.

    Atom &skel_iptr = code_[code_[tpl_arg_set_index + i].asIndex() + 1];
    uint16 pgm_code_index = code_[tpl_arg_set_index + i].asIndex();

    patch_tpl_code(pgm_code_index, get_object()->code(ipgm_arg_set_index + i).asIndex());
    skel_iptr = Atom::IPGMPointer(ipgm_arg_set_index + i); // patch the pgm code with ptrs to the tpl args' actual location in the ipgm code.
  }
  //Atom::Trace(code, get_object()->get_reference(0)->code_size());
}

void InputLessPGMOverlay::patch_tpl_code(uint16 pgm_code_index, uint16 ipgm_code_index) { // patch recursively : in pgm_code[index] with IPGM_PTRs until ::.

  uint16 atom_count = code_[pgm_code_index].getAtomCount();
  for (uint16 j = 1; j <= atom_count; ++j) {

    switch (code_[pgm_code_index + j].getDescriptor()) {
    case Atom::WILDCARD:
      code_[pgm_code_index + j] = Atom::IPGMPointer(ipgm_code_index + j);
      break;
    case Atom::T_WILDCARD: // leave as is and stop patching.
      return;
    case Atom::I_PTR:
      patch_tpl_code(code_[pgm_code_index + j].asIndex(), get_object()->code(ipgm_code_index + j).asIndex());
      break;
    default: // leave as is.
      break;
    }
  }
}

void InputLessPGMOverlay::patch_input_code(uint16 pgm_code_index, uint16 input_index, uint16 input_code_index, int16 parent_index) {
}

bool InputLessPGMOverlay::inject_productions() {

  auto now = Now();

  bool in_red = false; // if prods are computed by red, we have to evaluate the expression; otherwise, we have to evaluate the prods in the set one by one to be able to reference new objects in this->productions.
  IPGMContext prods(get_object()->get_reference(0), get_view(), code_, code_[PGM_PRODS].asIndex(), this);
  if (prods[0].getDescriptor() != Atom::SET) { // prods[0] is not a set: it is assumed to be an expression lead by red.

    in_red = true;
    if (!prods.evaluate()) {

      rollback();
      productions_.clear();
      return false;
    }
    prods = prods.dereference();
  }
  //prods.trace();
  uint16 production_count = prods.get_children_count();
  uint16 cmd_count = 0; // cmds to the executive (excl. mod/set) and external devices.
  for (uint16 i = 1; i <= production_count; ++i) {

    IPGMContext cmd = prods.get_child_deref(i);
    if (!in_red && !cmd.evaluate()) {

      rollback();
      productions_.clear();
      return false;
    }//cmd.trace();
    IPGMContext function = cmd.get_child_deref(1);

    // layout of a command:
    // 0 >icmd opcode
    // 1 >function
    // 2 >iptr to the set of arguments
    // 3 >set
    // 4 >first arg
    // or:
    // 0 >cmd opcode
    // 1 >function
    // 2 >iptr to the set of arguments
    // 3 >psln_thr
    // 4 >set
    // 5 >first arg

    // identify the production of new objects.
    IPGMContext args = cmd.get_child_deref(2);
    if (cmd[0].asOpcode() == Opcodes::ICmd) {

      if (function[0].asOpcode() == Opcodes::Inject ||
        function[0].asOpcode() == Opcodes::Eject) { // args:[object view]; create an object if not a reference.

        Code *object;
        IPGMContext arg1 = args.get_child(1);
        uint16 index = arg1.getIndex();
        arg1 = arg1.dereference();
        // arg1.trace();
        if (arg1.is_reference())
          productions_.push_back(arg1.get_object());
        else {

          object = _Mem::Get()->build_object(arg1[0]);
          // arg1.trace();
          arg1.copy(object, 0);
          // arg1.trace();
          // object->trace();
          productions_.push_back(_Mem::Get()->check_existence(object));
        }
        patch_code(index, Atom::ProductionPointer(productions_.size() - 1));

        ++cmd_count;
      } else if (function[0].asOpcode() != Opcodes::Mod &&
        function[0].asOpcode() != Opcodes::Set &&
        function[0].asOpcode() != Opcodes::Prb)
        ++cmd_count;
    } else {
      ++cmd_count;
      if (cmd[0].asOpcode() == Opcodes::Cmd)
        // We will also add the fact of the ejection to the set of productions.
        ++cmd_count;
    }
  }

  Code *mk_rdx = NULL;
  uint16 ntf_grp_count = get_view()->get_host()->get_ntf_grp_count();

  uint16 write_index;
  uint16 mk_rdx_prod_index;
  uint16 extent_index;
  if (ntf_grp_count && cmd_count && (get_object()->code(IPGM_NFR).asBoolean())) { // the productions are command objects (cmd); only injections/ejections and cmds to external devices are notified.

    mk_rdx = get_mk_rdx(write_index);
    mk_rdx_prod_index = write_index;
    mk_rdx->code(write_index++) = Atom::Set(cmd_count);
    extent_index = write_index + cmd_count;
  }

  // all productions have evaluated correctly; now we can execute the commands one by one.
  for (uint16 i = 1; i <= production_count; ++i) {

    IPGMContext cmd = prods.get_child_deref(i);
    IPGMContext function = cmd.get_child_deref(1);

    // call device functions.
    IPGMContext args = cmd.get_child_deref(2);
    if (cmd[0].asOpcode() == Opcodes::ICmd) { // command to the executive.

      if (function[0].asOpcode() == Opcodes::Inject) { // args:[object view]; retrieve the object and create a view.

        IPGMContext arg1 = args.get_child(1);
        arg1.dereference_once();
        //arg1.trace();
        Code *object = args.get_child_deref(1).get_object();
        //object->trace();
        IPGMContext _view = args.get_child_deref(2);
        if (_view[0].getAtomCount() != 0) { // regular view (i.e. not |[]).

          View *view = new View();
          _view.copy(view, 0);
          view->set_object(object);

          view->references_[1] = get_view()->get_host();
          view->code(VIEW_ORG) = Atom::RPointer(1);

          _Mem::Get()->inject(view);

          if (mk_rdx) {

            mk_rdx->code(write_index++) = Atom::IPointer(extent_index);
            prods.get_child_deref(i).copy(mk_rdx, extent_index, extent_index);
          }
        } else // this allows building objects with no view (case in point: fact on object: only the fact needs to be injected).
          --cmd_count;
      } else if (function[0].asOpcode() == Opcodes::Eject) { // args:[object view destination_node]; view.grp=destination grp (stdin ot stdout); retrieve the object and create a view.

        Code *object = args.get_child_deref(1).get_object();

        IPGMContext _view = args.get_child_deref(2);
        View *view = new View();
        _view.copy(view, 0);
        view->set_object(object);

        IPGMContext node = args.get_child_deref(3);

        _Mem::Get()->eject(view, node[0].getNodeID());

        if (mk_rdx) {

          mk_rdx->code(write_index++) = Atom::IPointer(extent_index);
          prods.get_child_deref(i).copy(mk_rdx, extent_index, extent_index);
        }
      } else if (function[0].asOpcode() == Opcodes::Mod) { // args:[iptr-to-cptr value_].

        void *object;
        IPGMContext::ObjectType object_type;
        int16 member_index;
        uint32 view_oid;
        args.get_child(1).getMember(object, view_oid, object_type, member_index); // args.get_child(1) is an iptr.

        if (object) {

          float32 value = args.get_child_deref(2)[0].asFloat();
          switch (object_type) {
          case IPGMContext::TYPE_VIEW: { // add the target and value to the group's pending operations.

            Group *g = (Group *)object;
            g->enter();
            g->pending_operations_.push_back(new Group::Mod(view_oid, member_index, value));
            g->leave();
            break;
          }case IPGMContext::TYPE_OBJECT:
            ((Code *)object)->mod(member_index, value); // protected internally.
            break;
          case IPGMContext::TYPE_GROUP:
            ((Group *)object)->enter();
            ((Group *)object)->mod(member_index, value);
            ((Group *)object)->leave();
            break;
          default:
            rollback();
            productions_.clear();
            return false;
          }
        }
      } else if (function[0].asOpcode() == Opcodes::Set) { // args:[iptr-to-cptr value_].

        void *object;
        IPGMContext::ObjectType object_type;
        int16 member_index;
        uint32 view_oid;
        args.get_child(1).getMember(object, view_oid, object_type, member_index); // args.get_child(1) is an iptr.

        if (object) {

          float32 value = args.get_child_deref(2)[0].asFloat();
          switch (object_type) {
          case IPGMContext::TYPE_VIEW: { // add the target and value to the group's pending operations.

            Group *g = (Group *)object;
            g->enter();
            g->pending_operations_.push_back(new Group::Set(view_oid, member_index, value));
            g->leave();
            break;
          }case IPGMContext::TYPE_OBJECT:
            ((Code *)object)->set(member_index, value); // protected internally.
            break;
          case IPGMContext::TYPE_GROUP:
            ((Group *)object)->enter();
            ((Group *)object)->set(member_index, value);
            ((Group *)object)->leave();
            break;
          }
        }
      } else if (function[0].asOpcode() == Opcodes::NewClass) { // TODO

      } else if (function[0].asOpcode() == Opcodes::DelClass) { // TODO

      } else if (function[0].asOpcode() == Opcodes::LDC) { // TODO

      } else if (function[0].asOpcode() == Opcodes::Swap) { // TODO

      } else if (function[0].asOpcode() == Opcodes::Prb) { // args:[probe_level,callback_name,msg,set of objects].

        float32 probe_lvl = args.get_child_deref(1)[0].asFloat();
        if (probe_lvl < _Mem::Get()->get_probe_level()) {

          std::string callback_name = Utils::GetString(&args.get_child_deref(2)[0]);

          Callbacks::Callback callback = Callbacks::Get(callback_name);
          if (callback) {

            std::string msg = Utils::GetString(&args.get_child_deref(3)[0]);
            IPGMContext _objects = args.get_child_deref(4);

            uint8 object_count = _objects[0].getAtomCount();
            Code **objects = NULL;
            if (object_count) {

              objects = new Code *[object_count];
              for (uint8 i = 1; i <= object_count; ++i)
                objects[i - 1] = _objects.get_child_deref(i).get_object();
            }

            callback(duration_cast<microseconds>(now - Utils::GetTimeReference()), false, msg.c_str(), object_count, objects);
            if (object_count)
              delete[] objects;
          }
        }
      } else if (function[0].asOpcode() == Opcodes::Stop) { // no args.

        _Mem::Get()->stop();
      } else { // unknown function.

        rollback();
        productions_.clear();
        return false;
      }
    } else if (cmd[0].asOpcode() == Opcodes::Cmd) { // command to an external device, build a cmd object and send it.

      P<Code> command = _Mem::Get()->build_object(cmd[0]);
      cmd.copy((Code*)command, 0);

      Code* executed_command = _Mem::Get()->eject(command);

      // Build a fact of the command and inject it in stdin. Give the fact an uncertainty range since we don't know when
      // it will be executed. Otherwise a fact with zero duration may not overlap a fact, making predictions fail.
      // We offset the beginning of the uncertainty range at a minimum by 2*GetTimeTolerance() from the frame start (the same as SYNC_HOLD)
      // so that CTPX::reduce will not fail due to "cause in sync with the premise".
      auto relative_time = duration_cast<microseconds>(now - Utils::GetTimeReference());
      auto frame_start = now - (relative_time % _Mem::Get()->get_sampling_period());
      auto after = max(now, frame_start + 2 * Utils::GetTimeTolerance());
      auto before = frame_start + _Mem::Get()->get_sampling_period();
      P<Code> fact;
      if (executed_command) {
        // Set fact to the efferent copy of the command and inject it.
        fact = new Fact(executed_command, after, before, 1, 1);
        View *view = new View(View::SYNC_ONCE, now, 1, 1, _Mem::Get()->get_stdin(), get_view()->get_host(), fact); // SYNC_ONCE, sln=1, res=1,
        _Mem::Get()->inject(view);
        string mk_rdx_info = "";
#ifdef WITH_DETAIL_OID
        if (mk_rdx)
          // We don't know the mk.rdx OID yet, so use the detail OID.
          mk_rdx_info = " mk.rdx(" + to_string(mk_rdx->get_detail_oid()) + "):";
#endif
        OUTPUT_LINE(IO_DEVICE_INJ_EJT, Utils::RelativeTime(Now()) << mk_rdx_info << " I/O device eject " << fact->get_oid());
      }
      else
        // The command wasn't executed. Set fact to an anti-fact of the original command and record in the mk_rdx.
        fact = new AntiFact(command, after, before, 1, 1);

      if (mk_rdx) {

        // Add the original command.
        mk_rdx->code(write_index++) = Atom::IPointer(extent_index);
        prods.get_child_deref(i).copy(mk_rdx, extent_index, extent_index);
        // Add the fact of the injected command that we just made.
        mk_rdx->code(write_index++) = Atom::RPointer(mk_rdx->references_size());
        mk_rdx->add_reference(fact);
      }
    }
  }

  if (mk_rdx) {

    mk_rdx->code(mk_rdx_prod_index) = Atom::Set(cmd_count);
    for (uint16 i = 1; i <= ntf_grp_count; ++i) {

      NotificationView *v = new NotificationView(get_view()->get_host(), get_view()->get_host()->get_ntf_grp(i), mk_rdx);
      _Mem::Get()->inject_notification(v, true);
    }

    OUTPUT_LINE((TraceLevel)0, Utils::RelativeTime(Now()) << " pgm " << controller_->get_object()->get_oid() <<
      " -> mk.rdx " << mk_rdx->get_oid());
  }

  return true;
}

Code *InputLessPGMOverlay::get_mk_rdx(uint16 &extent_index) const {

  uint16 write_index = 0;
  extent_index = MK_RDX_ARITY + 1;

  Code *mk_rdx = new r_exec::LObject(_Mem::Get());

  mk_rdx->code(write_index++) = Atom::Marker(Opcodes::MkRdx, MK_RDX_ARITY);
  mk_rdx->code(write_index++) = Atom::RPointer(0); // code.
  mk_rdx->add_reference(get_object());
  mk_rdx->code(write_index++) = Atom::IPointer(extent_index); // inputs.
  mk_rdx->code(extent_index++) = Atom::Set(0);
  mk_rdx->code(write_index++) = Atom::IPointer(extent_index); // productions.
  mk_rdx->code(write_index++) = Atom::Float(1); // psln_thr.

  return mk_rdx;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

PGMOverlay::PGMOverlay(Controller *c) : InputLessPGMOverlay(c) {

  is_volatile_ = c->get_object()->code(IPGM_RES).asBoolean();
  init();
}

PGMOverlay::PGMOverlay(PGMOverlay *original, uint16 last_input_index, uint16 value_commit_index) : InputLessPGMOverlay() {

  controller_ = original->controller_;

  input_pattern_indices_ = original->input_pattern_indices_;
  input_pattern_indices_.push_back(last_input_index); // put back the last original's input index.
  for (uint16 i = 0; i < original->input_views_.size() - 1; ++i) // ommit the last original's input view.
    input_views_.push_back(original->input_views_[i]);

  code_size_ = original->code_size_;
  code_ = new r_code::Atom[code_size_];
  memcpy(code_, original->code_, code_size_ * sizeof(r_code::Atom)); // copy patched code.

  Atom *original_code = &get_object()->get_reference(0)->code(0);
  for (uint16 i = 0; i < original->patch_indices_.size(); ++i) // unpatch code.
    code_[original->patch_indices_[i]] = original_code[original->patch_indices_[i]];

  value_commit_index_ = value_commit_index;
  for (uint16 i = 0; i < value_commit_index; ++i) // copy values up to the last commit index.
    values_.push_back(original->values_[i]);

  is_volatile_ = original->is_volatile_;
  birth_time_ = original->birth_time_;
}

PGMOverlay::~PGMOverlay() {
}

void PGMOverlay::init() {

  // init the list of pattern indices.
  uint16 pattern_set_index = code_[PGM_INPUTS].asIndex();
  uint16 pattern_count = code_[pattern_set_index].getAtomCount();
  for (uint16 i = 1; i <= pattern_count; ++i)
    input_pattern_indices_.push_back(code_[pattern_set_index + i].asIndex());

  birth_time_ = Timestamp(seconds(0));
}

bool PGMOverlay::is_invalidated() {

  if (is_volatile_) {

    for (uint32 i = 0; i < input_views_.size(); ++i) {

      if (input_views_[i]->object_->is_invalidated())
        return (invalidated_ = 1);
    }
  }

  return invalidated_ == 1;
}

Code *PGMOverlay::dereference_in_ptr(Atom a) {

  switch (a.getDescriptor()) {
  case Atom::IN_OBJ_PTR:
    return getInputObject(a.asInputIndex());
  case Atom::D_IN_OBJ_PTR: {
    Atom ptr = code_[a.asRelativeIndex()]; // must be either an IN_OBJ_PTR or a D_IN_OBJ_PTR.
    Code *parent = dereference_in_ptr(ptr);
    return parent->get_reference(parent->code(ptr.asIndex()).asIndex());
  }default: // shall never happen.
    return NULL;
  }
}

void PGMOverlay::patch_input_code(uint16 pgm_code_index, uint16 input_index, uint16 input_code_index, int16 parent_index) { // patch recursively : in pgm_code[pgm_code_index] with (D_)IN_OBJ_PTRs until ::.

  uint16 atom_count = code_[pgm_code_index].getAtomCount();

  // Replace the head of a structure by a ptr to the input object.
  Atom head;
  if (parent_index < 0)
    head = code_[pgm_code_index] = Atom::InObjPointer(input_index, input_code_index);
  else
    head = code_[pgm_code_index] = Atom::DInObjPointer(parent_index, input_code_index);
  patch_indices_.push_back(pgm_code_index);

  // Proceed with the structure's members.
  for (uint16 j = 1; j <= atom_count; ++j) {

    uint16 patch_index = pgm_code_index + j;
    switch (code_[patch_index].getDescriptor()) {
    case Atom::T_WILDCARD: // leave as is and stop patching.
      return;
    case Atom::WILDCARD:
      if (parent_index < 0)
        code_[patch_index] = Atom::InObjPointer(input_index, input_code_index + j);
      else
        code_[patch_index] = Atom::DInObjPointer(parent_index, input_code_index + j);
      patch_indices_.push_back(patch_index);
      break;
    case Atom::I_PTR: { // sub-structure: go one level deeper in the pattern.
      uint16 indirection = code_[patch_index].asIndex(); // save the indirection before patching.

      if (parent_index < 0)
        code_[patch_index] = Atom::InObjPointer(input_index, input_code_index + j);
      else
        code_[patch_index] = Atom::DInObjPointer(parent_index, input_code_index + j);
      patch_indices_.push_back(patch_index);
      switch (dereference_in_ptr(head)->code(j).getDescriptor()) { // caution: the pattern points to sub-structures using iptrs. However, the input object may have a rptr instead of an iptr: we have to disambiguate.
      case Atom::I_PTR: // dereference and recurse.
        patch_input_code(indirection, input_index, dereference_in_ptr(head)->code(j).asIndex(), parent_index);
        break;
      case Atom::R_PTR: // do not dereference and recurse.
        patch_input_code(indirection, input_index, 0, patch_index);
        break;
      default: // shall never happen.
        break;
      }
      break;
    }default: // leave as is.
      break;
    }
  }
}

Overlay *PGMOverlay::reduce(r_exec::View *input) {

  uint16 input_index;
  switch (match(input, input_index)) {
  case SUCCESS:
    if (input_pattern_indices_.size() == 0) { // all patterns matched.

      if (check_guards() && inject_productions()) {

        ((PGMController *)controller_)->notify_reduction();
        PGMOverlay *offspring = new PGMOverlay(this, input_index, value_commit_index_);
        invalidate();
        return offspring;
      } else {

        PGMOverlay *offspring = new PGMOverlay(this, input_index, value_commit_index_);
        invalidate();
        return offspring;
      }
    } else { // create an overlay in a state where the last input is not matched: this overlay will be able to catch other candidates for the input patterns that have already been matched.

      PGMOverlay *offspring = new PGMOverlay(this, input_index, value_commit_index_);
      commit();
      if (birth_time_.time_since_epoch().count() == 0)
        birth_time_ = Now();
      return offspring;
    }
  case FAILURE: // just rollback: let the overlay match other inputs.
    rollback();
  case IMPOSSIBLE:
    return NULL;
  }
}

PGMOverlay::MatchResult PGMOverlay::match(r_exec::View *input, uint16 &input_index) {

  input_views_.push_back(input);
  bool failed = false;
  r_code::list<uint16>::const_iterator it;
  for (it = input_pattern_indices_.begin(); it != input_pattern_indices_.end(); ++it) {

    MatchResult r = _match(input, *it);
    switch (r) {
    case SUCCESS:
      input_index = *it;
      input_pattern_indices_.erase(it);
      return r;
    case FAILURE:
      failed = true;
      rollback(); // to try another pattern on a clean basis.
    case IMPOSSIBLE:
      break;
    }
  }
  input_views_.pop_back();
  return failed ? FAILURE : IMPOSSIBLE;
}

PGMOverlay::MatchResult PGMOverlay::_match(r_exec::View *input, uint16 pattern_index) {

  if (code_[pattern_index].asOpcode() == Opcodes::AntiPtn) {

    IPGMContext input_object = IPGMContext::GetContextFromInput(input, this);
    IPGMContext pattern_skeleton(get_object()->get_reference(0), get_view(), code_, code_[pattern_index + 1].asIndex(), this); // pgm_code[pattern_index] is the first atom of the pattern; pgm_code[pattern_index+1] is an iptr to the skeleton.
    if (!pattern_skeleton.match(input_object))
      return SUCCESS;
    MatchResult r = __match(input, pattern_index);
    switch (r) {
    case IMPOSSIBLE:
    case FAILURE:
      return SUCCESS;
    case SUCCESS:
      return FAILURE;
    }
  } else if (code_[pattern_index].asOpcode() == Opcodes::Ptn) {

    IPGMContext input_object = IPGMContext::GetContextFromInput(input, this);
    IPGMContext pattern_skeleton(get_object()->get_reference(0), get_view(), code_, code_[pattern_index + 1].asIndex(), this); // pgm_code[pattern_index] is the first atom of the pattern; pgm_code[pattern_index+1] is an iptr to the skeleton.
    if (!pattern_skeleton.match(input_object))
      return IMPOSSIBLE;
    return __match(input, pattern_index);
  }
  return IMPOSSIBLE;
}

PGMOverlay::MatchResult PGMOverlay::__match(r_exec::View *input, uint16 pattern_index) {
  // The input has just been pushed on input_views_ (see match).
  // pgm_code[pattern_index+1].asIndex() is the structure pointed by the pattern's skeleton.
  patch_input_code(code_[pattern_index + 1].asIndex(), input_views_.size() - 1, 0);
  // match: evaluate the set of guards.
  uint16 guard_set_index = code_[pattern_index + 2].asIndex();
  // Get the IPGMContext like in InputLessPGMOverlay::evaluate.
  IPGMContext c(get_object()->get_reference(0), get_view(), code_, guard_set_index, this);
  if (!c.evaluate())
    return FAILURE;
  if (c.dereference()[0].isBooleanFalse())
    // The boolean guard is false.
    return FAILURE;
  return SUCCESS;
}

bool PGMOverlay::check_guards() {

  uint16 guard_set_index = code_[PGM_GUARDS].asIndex();
  uint16 guard_count = code_[guard_set_index].getAtomCount();
  for (uint16 i = 1; i <= guard_count; ++i) {

    // Get the IPGMContext like in InputLessPGMOverlay::evaluate.
    IPGMContext c(get_object()->get_reference(0), get_view(), code_, guard_set_index + i, this);
    if (!c.evaluate())
      return false;
    if (c.dereference()[0].isBooleanFalse())
      // The boolean guard is false.
      return false;
  }
  return true;
}

Code *PGMOverlay::get_mk_rdx(uint16 &extent_index) const {

  uint16 write_index = 0;
  extent_index = MK_RDX_ARITY + 1;

  Code *mk_rdx = new r_exec::LObject(_Mem::Get());

  mk_rdx->code(write_index++) = Atom::Marker(Opcodes::MkRdx, MK_RDX_ARITY);
  mk_rdx->code(write_index++) = Atom::RPointer(0); // code.
  mk_rdx->add_reference(get_object());
  mk_rdx->code(write_index++) = Atom::IPointer(extent_index); // inputs.
  mk_rdx->code(extent_index++) = Atom::Set(input_views_.size());
  for (uint16 i = 0; i < input_views_.size(); ++i) {

    mk_rdx->code(extent_index++) = Atom::RPointer(i + 1);
    mk_rdx->add_reference(input_views_[i]->object_);
  }
  mk_rdx->code(write_index++) = Atom::IPointer(extent_index); // productions.
  mk_rdx->code(write_index++) = Atom::Float(1); // psln_thr.

  return mk_rdx;
}

////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

AntiPGMOverlay::~AntiPGMOverlay() {
}

Overlay *AntiPGMOverlay::reduce(r_exec::View *input) {

  uint16 input_index;
  switch (match(input, input_index)) {
  case SUCCESS:
    if (input_pattern_indices_.size() == 0) { // all patterns matched.

      if (check_guards()) {

        ((AntiPGMController *)controller_)->restart();
        return NULL;
      } else {
        rollback();
        return NULL;
      }
    } else {

      AntiPGMOverlay *offspring = new AntiPGMOverlay(this, input_index, value_commit_index_);
      commit();
      return offspring;
    }
  case FAILURE: // just rollback: let the overlay match other inputs.
    rollback();
  case IMPOSSIBLE:
    return NULL;
  }
}
}
