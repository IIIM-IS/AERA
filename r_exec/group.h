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

#ifndef group_h
#define group_h

#include <set>
#include <unordered_map>

#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
#include "object.h"
#include "view.h"


namespace r_exec {

class _Mem;
class HLPController;

// Shared resources:
// all parameters: accessed by Mem::update and reduction cores (via overlay mod/set).
// all views: accessed by Mem::update and reduction cores.
// viewing_groups_: accessed by Mem::injectNow and Mem::update.
class r_exec_dll Group :
  public LObject,
  public CriticalSection {
private:
  // Ctrl values.
  uint32 sln_thr_changes_;
  float32 acc_sln_thr_;
  uint32 act_thr_changes_;
  float32 acc_act_thr_;
  uint32 vis_thr_changes_;
  float32 acc_vis_thr_;
  uint32 c_sln_changes_;
  float32 acc_c_sln_;
  uint32 c_act_changes_;
  float32 acc_c_act_;
  uint32 c_sln_thr_changes_;
  float32 acc_c_sln_thr_;
  uint32 c_act_thr_changes_;
  float32 acc_c_act_thr_;
  void reset_ctrl_values();

  // Stats.
  float32 avg_sln_;
  float32 high_sln_;
  float32 low_sln_;
  float32 avg_act_;
  float32 high_act_;
  float32 low_act_;
  uint32 sln_updates_;
  uint32 act_updates_;

  // Decay.
  float32 sln_decay_;
  float32 sln_thr_decay_;
  int32 decay_periods_to_go_;
  float32 decay_percentage_per_period_;
  float32 decay_target_; // -1: none, 0: sln, 1:sln_thr
  void reset_decay_values();

  // Notifications.
  int32 sln_change_monitoring_periods_to_go_;
  int32 act_change_monitoring_periods_to_go_;

  void _mod_0_positive(uint16 member_index, float32 value);
  void _mod_0_plus1(uint16 member_index, float32 value);
  void _mod_minus1_plus1(uint16 member_index, float32 value);
  void _set_0_positive(uint16 member_index, float32 value);
  void _set_0_plus1(uint16 member_index, float32 value);
  void _set_minus1_plus1(uint16 member_index, float32 value);
  void _set_0_1(uint16 member_index, float32 value);

  bool is_active_pgm(View *view);
  bool is_eligible_input(View *view);

  void inject(View *view);

  void notifyNew(View *view);
  void cov(View *view);

  class GroupState {
  public:
    float32 former_sln_thr;
    bool was_c_active;
    bool is_c_active;
    bool was_c_salient;
    bool is_c_salient;
    GroupState(float32 former_sln_thr,
      bool was_c_active,
      bool is_c_active,
      bool was_c_salient,
      bool is_c_salient) : former_sln_thr(former_sln_thr), was_c_active(was_c_active), is_c_active(is_c_active), was_c_salient(was_c_salient), is_c_salient(is_c_salient) {}
  };

  void _update_saliency(GroupState *state, View *view);
  void _update_activation(GroupState *state, View *view);
  void _update_visibility(GroupState *state, View *view);

  void _initiate_sln_propagation(Code *object, float32 change, float32 source_sln_thr) const;
  void _initiate_sln_propagation(Code *object, float32 change, float32 source_sln_thr, std::vector<Code *> &path) const;
  void _propagate_sln(Code *object, float32 change, float32 source_sln_thr, std::vector<Code *> &path) const;
public:
  // xxx_views are meant for erasing views with res==0. They are specialized by type to ease update operations.
  // Active overlays are to be found in xxx_ipgm_views.
  std::unordered_map<uint32, P<View> > ipgm_views_;
  std::unordered_map<uint32, P<View> > anti_ipgm_views_;
  std::unordered_map<uint32, P<View> > input_less_ipgm_views_;
  std::unordered_map<uint32, P<View> > notification_views_;
  std::unordered_map<uint32, P<View> > group_views_;
  std::unordered_map<uint32, P<View> > other_views_;

  // Defined to create reduction jobs in the viewing groups from the viewed group.
  // Empty when the viewed group is invisible (this means that visible groups can be non c-active or non c-salient).
  // Maintained by the viewing groups (at update time).
  // Viewing groups are c-active and c-salient. the bool is the cov.
  std::unordered_map<Group *, bool> viewing_groups_;

  // Populated within update; ordered by increasing ijt; cleared at the beginning of update.
  std::multiset<P<View>, r_code::_View::Less> newly_salient_views_;

  // Populated upon ipgm injection; used at update time; cleared afterward.
  std::vector<Controller *> new_controllers_;

  class Operation {
  protected:
    Operation(uint32 oid) : oid_(oid) {}
  public:
    const uint32 oid_; // of the view.
    virtual void execute(Group *g) const = 0;
  };

  class ModSet :
    public Operation {
  protected:
    ModSet(uint32 oid, uint16 member_index, float32 value) : Operation(oid), member_index_(member_index), value_(value) {}
    const uint16 member_index_;
    const float32 value_;
  };

  class Mod :
    public ModSet {
  public:
    Mod(uint32 oid, uint16 member_index, float32 value) : ModSet(oid, member_index, value) {}
    void execute(Group *g) const {

      View *v = g->get_view(oid_);
      if (v)
        v->mod(member_index_, value_);
    }
  };

  class Set :
    public ModSet {
  public:
    Set(uint32 oid, uint16 member_index, float32 value) : ModSet(oid, member_index, value) {}
    void execute(Group *g) const {

      View *v = g->get_view(oid_);
      if (v)
        v->set(member_index_, value_);
    }
  };

  // Pending mod/set operations on the group's view, exploited and cleared at update time.
  std::vector<Operation *> pending_operations_;

  Group(r_code::Mem *m = NULL);
  Group(r_code::SysObject *source);
  virtual ~Group();

  bool invalidate(); // removes all views of itself and of any other object.

  bool all_views_cond(uint8 &selector, std::unordered_map<uint32, P<View> >::const_iterator &it, std::unordered_map<uint32, P<View> >::const_iterator &end) {
    while (it == end) {
      switch (selector++) {
      case 0:
        it = anti_ipgm_views_.begin();
        end = anti_ipgm_views_.end();
        break;
      case 1:
        it = input_less_ipgm_views_.begin();
        end = input_less_ipgm_views_.end();
        break;
      case 2:
        it = notification_views_.begin();
        end = notification_views_.end();
        break;
      case 3:
        it = group_views_.begin();
        end = group_views_.end();
        break;
      case 4:
        it = other_views_.begin();
        end = other_views_.end();
        break;
      case 5:
        selector = 0;
        return false;
      }
    }
    return true;
  }

#define FOR_ALL_VIEWS_BEGIN(g,it) { \
    uint8 selector; \
    std::unordered_map<uint32,P<View> >::const_iterator it=g->ipgm_views_.begin(); \
    std::unordered_map<uint32,P<View> >::const_iterator end=g->ipgm_views_.end(); \
    for(selector=0;g->all_views_cond(selector,it,end);++it){

#define FOR_ALL_VIEWS_BEGIN_NO_INC(g,it) { \
    uint8 selector; \
    std::unordered_map<uint32,P<View> >::const_iterator it=g->ipgm_views_.begin(); \
    std::unordered_map<uint32,P<View> >::const_iterator end=g->ipgm_views_.end(); \
    for(selector=0;g->all_views_cond(selector,it,end);){

#define FOR_ALL_VIEWS_END } \
  }

  bool views_with_inputs_cond(uint8 &selector, std::unordered_map<uint32, P<View> >::const_iterator &it, std::unordered_map<uint32, P<View> >::const_iterator &end) {
    while (it == end) {
      switch (selector++) {
      case 0:
        it = anti_ipgm_views_.begin();
        end = anti_ipgm_views_.end();
        break;
      case 1:
        selector = 0;
        return false;
      }
    }
    return true;
  }

#define FOR_ALL_VIEWS_WITH_INPUTS_BEGIN(g,it) { \
    uint8 selector; \
    std::unordered_map<uint32,P<View> >::const_iterator it=g->ipgm_views_.begin(); \
    std::unordered_map<uint32,P<View> >::const_iterator end=g->ipgm_views_.end(); \
    for(selector=0;g->views_with_inputs_cond(selector,it,end);++it){

#define FOR_ALL_VIEWS_WITH_INPUTS_END } \
  }

  bool non_ntf_views_cond(uint8 &selector, std::unordered_map<uint32, P<View> >::const_iterator &it, std::unordered_map<uint32, P<View> >::const_iterator &end) {
    while (it == end) {
      switch (selector++) {
      case 0:
        it = anti_ipgm_views_.begin();
        end = anti_ipgm_views_.end();
        break;
      case 1:
        it = input_less_ipgm_views_.begin();
        end = input_less_ipgm_views_.end();
        break;
      case 2:
        it = group_views_.begin();
        end = group_views_.end();
        break;
      case 3:
        it = other_views_.begin();
        end = other_views_.end();
        break;
      case 4:
        selector = 0;
        return false;
      }
    }
    return true;
  }

#define FOR_ALL_NON_NTF_VIEWS_BEGIN(g,it) { \
    uint8 selector; \
    std::unordered_map<uint32,P<View> >::const_iterator it=g->ipgm_views_.begin(); \
    std::unordered_map<uint32,P<View> >::const_iterator end=g->ipgm_views_.end(); \
    for(selector=0;g->non_ntf_views_cond(selector,it,end);++it){

#define FOR_ALL_NON_NTF_VIEWS_END } \
  }

  View *get_view(uint32 OID);

  uint32 get_upr() const;

  float32 get_sln_thr() const;
  float32 get_act_thr() const;
  float32 get_vis_thr() const;

  float32 get_c_sln() const;
  float32 get_c_act() const;

  float32 get_c_sln_thr() const;
  float32 get_c_act_thr() const;

  void mod_sln_thr(float32 value);
  void set_sln_thr(float32 value);
  void mod_act_thr(float32 value);
  void set_act_thr(float32 value);
  void mod_vis_thr(float32 value);
  void set_vis_thr(float32 value);
  void mod_c_sln(float32 value);
  void set_c_sln(float32 value);
  void mod_c_act(float32 value);
  void set_c_act(float32 value);
  void mod_c_sln_thr(float32 value);
  void set_c_sln_thr(float32 value);
  void mod_c_act_thr(float32 value);
  void set_c_act_thr(float32 value);

  float32 update_sln_thr(); // computes and applies decay on sln thr if any.
  float32 update_act_thr();
  float32 update_vis_thr();
  float32 update_c_sln();
  float32 update_c_act();
  float32 update_c_sln_thr();
  float32 update_c_act_thr();

  float32 get_sln_chg_thr();
  float32 get_sln_chg_prd();
  float32 get_act_chg_thr();
  float32 get_act_chg_prd();

  float32 get_avg_sln();
  float32 get_high_sln();
  float32 get_low_sln();
  float32 get_avg_act();
  float32 get_high_act();
  float32 get_low_act();

  float32 get_high_sln_thr();
  float32 get_low_sln_thr();
  float32 get_sln_ntf_prd();
  float32 get_high_act_thr();
  float32 get_low_act_thr();
  float32 get_act_ntf_prd();
  float32 get_low_res_thr();

  float32 get_ntf_new();

  uint16 get_ntf_grp_count();
  Group *get_ntf_grp(uint16 i); // i starts at 1.

  // Delegate to views; update stats and notifies.
  float32 update_res(View *v);
  float32 update_sln(View *v); // applies decay if any.
  float32 update_act(View *v);

  // Target upr, spr, c_sln, c_act, sln_thr, act_thr, vis_thr, c_sln_thr, c_act_thr, sln_chg_thr,
  // sln_chg_prd, act_chg_thr, act_chg_prd, high_sln_thr, low_sln_thr, sln_ntf_prd, high_act_thr, low_act_thr, act_ntf_prd, low_res_thr, res_ntf_prd, ntf_new,
  // dcy_per, dcy-tgt, dcy_prd.
  void mod(uint16 member_index, float32 value);
  void set(uint16 member_index, float32 value);

  void reset_stats(); // called at the begining of an update.
  void update_stats(); // at the end of an update; may produce notifcations.

  bool load(View *view, Code *object);

  // Called at each update period.
  // - set the final resilience value_, if 0, delete.
  // - set the final saliency.
  // - set the final activation.
  // - set the final visibility, cov.
  // - propagate saliency changes.
  // - inject next update job for the group.
  // - inject new signaling jobs if act pgm with no input or act |pgm.
  // - notify high and low values.
  void update(Timestamp planned_time);

  void inject_new_object(View *view);
  void inject_existing_object(View *view);
  void inject_group(View *view);
  void inject_notification(View *view, bool lock);
  void inject_hlps(std::vector<View *> &views);
  void inject_reduction_jobs(View *view);

  void cov();

  class Hash {
  public:
    size_t operator ()(Group *g) const {
      return (size_t)g;
    }
  };

  class Equal {
  public:
    bool operator ()(const Group *lhs, const Group *rhs) const {
      return lhs == rhs;
    }
  };

  void delete_view(View *v);
  void delete_view(std::unordered_map<uint32, P<View> >::const_iterator &v);

  Group *get_secondary_group();
  void load_secondary_mdl_controller(View *view);
  void inject_secondary_mdl_controller(View *view);

  Timestamp get_next_upr_time(Timestamp now) const;
  Timestamp get_prev_upr_time(Timestamp now) const;
};
}


#include "object.tpl.cpp"
#include "group.inline.cpp"


#endif
