//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2021 Jeff Thompson
//_/_/ Copyright (c) 2018-2021 Kristinn R. Thorisson
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

#ifndef view_h
#define view_h

#include "../r_code/object.h"
#include "overlay.h"
#include "dll.h"


namespace r_exec {

class Group;
class LObject;

// OID is hidden at code_[VIEW_OID].
// Shared resources:
// none: all mod/set operations are pushed on the group and executed at update time.
class r_exec_dll View :
  public r_code::_View {
private:
  static uint32 lastOID_;
  static uint32 GetOID();

  // Ctrl values.
  uint32 sln_changes_;
  float32 acc_sln_;
  uint32 act_changes_;
  float32 acc_act_;
  uint32 vis_changes_;
  float32 acc_vis_;
  uint32 res_changes_;
  float32 acc_res_;
  void reset_ctrl_values();

  // Monitoring
  float32 initial_sln_;
  float32 initial_act_;

  void init(r_code::_View::SyncMode sync,
    Timestamp ijt,
    float32 sln,
    int32 res,
    r_code::Code *host,
    r_code::Code *origin,
    r_code::Code *object);
protected:
  void reset_init_sln();
  void reset_init_act();
public:
  static uint16 ViewOpcode_;

  P<Controller> controller_; // built upon injection of the view (if the object is an ipgm/icpp_pgm/cst/mdl).

  static float32 MorphValue(float32 value, float32 source_thr, float32 destination_thr);
  static float32 MorphChange(float32 change, float32 source_thr, float32 destination_thr);

  uint32 periods_at_low_sln_;
  uint32 periods_at_high_sln_;
  uint32 periods_at_low_act_;
  uint32 periods_at_high_act_;

  View();
  View(r_code::SysView *source, r_code::Code *object);
  View(View *view, Group *group); // copy the view and assigns it to the group (used for cov); morph ctrl values.
  View(const View *view, bool new_OID = false); // simple copy.
  View(r_code::_View::SyncMode sync,
    Timestamp ijt,
    float32 sln,
    int32 res,
    r_code::Code *host,
    r_code::Code *origin,
    r_code::Code *object); // regular view; res set to -1 means forever.
  View(r_code::_View::SyncMode sync,
    Timestamp ijt,
    float32 sln,
    int32 res,
    r_code::Code *host,
    r_code::Code *origin,
    r_code::Code *object,
    float32 act); // pgm/mdl view; res set to -1 means forever.
  ~View();

  void reset();
  void set_object(r_code::Code *object);

  uint32 get_oid() const;

  virtual bool is_notification() const;

  Group *get_host();

  r_code::_View::SyncMode get_sync();
  float32 get_res();
  float32 get_sln();
  float32 get_act();
  bool get_cov();
  float32 get_vis();
  uint32 &ctrl0() { return code_[VIEW_CTRL_0].atom_; } // use only for non-group views.
  uint32 &ctrl1() { return code_[VIEW_CTRL_1].atom_; } // idem.

  void mod_res(float32 value);
  void set_res(float32 value);
  void mod_sln(float32 value);
  void set_sln(float32 value);
  void mod_act(float32 value);
  void set_act(float32 value);
  void mod_vis(float32 value);
  void set_vis(float32 value);

  float32 update_res();
  float32 update_sln(float32 low, float32 high);
  float32 update_act(float32 low, float32 high);
  float32 update_vis();

  float32 update_sln_delta();
  float32 update_act_delta();

  void force_res(float32 value); // unmediated.

  // Target res, sln, act, vis.
  void mod(uint16 member_index, float32 value);
  void set(uint16 member_index, float32 value);

  void delete_from_object();
  void delete_from_group();
};

class r_exec_dll NotificationView :
  public View {
public:
  NotificationView(r_code::Code *origin, r_code::Code *destination, r_code::Code *marker); // res=1, sln=1.

  bool is_notification() const;
};
}


#include "view.inline.cpp"


#endif
