//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2025 Jeff Thompson
//_/_/ Copyright (c) 2018-2025 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2025 Icelandic Institute for Intelligent Machines
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

#ifndef auto_focus_h
#define auto_focus_h

#include "../r_code/time_buffer.h"

#include "overlay.h"
#include "group.h"
#include "pattern_extractor.h"
#include "mem.h"


namespace r_exec {

class r_exec_dll AutoFocusController :
  public Controller {
private:
  // icpp_pgm parameters.
  bool pass_through_;
  bool ctpx_on_;
  bool gtpx_on_;
  bool ptpx_on_;
  bool trace_injections_;
  bool decompile_models_;
  std::vector<Group *> output_groups_; // 1st is the primary, 2nd the secondary, followed by other groups if any.

  class Rating {
  public:
    uint32 evidences_;
    uint32 positive_evidences_;
    float32 success_rate_;
    float32 delta_success_rate_;

    static bool DeltaSuccessRate(float32 delta_success_rate) {

      return delta_success_rate > 0 && delta_success_rate < _Mem::Get()->get_tpx_dsr_thr();
    }

    Rating() : evidences_(0), positive_evidences_(0), success_rate_(0), delta_success_rate_(1) {}

    void add_evidence(bool success) {

      ++evidences_;
      if (success)
        ++positive_evidences_;
      delta_success_rate_ = success_rate_;
      success_rate_ = positive_evidences_ / evidences_;
      delta_success_rate_ = success_rate_ - delta_success_rate_;
    }
  };

  typedef std::unordered_map<P<_Fact>, P<TPX>, r_code::PHash<_Fact> > TPXMap;

  TPXMap goals_; // f->g->f->target.
  TPXMap predictions_; // f->p->f->target.

  typedef std::unordered_map<P<_Fact>, Rating, r_code::PHash<_Fact> > RatingMap;

  // entries are patterns, i.e. abstract targets.
  RatingMap goal_ratings_;
  RatingMap prediction_ratings_;

  static const uint32 CacheInitialSize = 128;
  static const uint32 CrossBufferInitialSize = 1024;

  r_code::time_buffer<CInput, CInput::IsInvalidated> cache_; // contains all inputs we don't know yet if they are relevant or not; thz==sampling period.
  r_code::time_buffer<Input, Input::IsInvalidated> cross_buffer_; // contains all relevant inputs.

  void notify(_Fact *target, View *input, TPXMap &map);
  void dispatch_pred_success(Success* success, TPXMap &map);
  void dispatch(View *input, _Fact *abstract_input, BindingMap *bm, bool &injected, TPXMap &map);
  void dispatch_no_inject(View *input, _Fact *abstract_input, BindingMap *bm, TPXMap &map);
  template<class T> TPX *build_tpx(_Fact *target, _Fact *pattern, BindingMap *bm, RatingMap &map, Fact *f_imdl, bool wr_enabled) {

    if (!gtpx_on_ && !ptpx_on_)
      return new TPX(this, target, pattern, bm);

    if (wr_enabled)
      return new TPX(this, target, pattern, bm);

    RatingMap::const_iterator r = map.find(pattern);
    if (r != map.end()) {

      if (Rating::DeltaSuccessRate(r->second.delta_success_rate_)) // target for which we don't see much improvement over time.
        return new TPX(this, target, pattern, bm);
      else
        return new T(this, target, pattern, bm, f_imdl);
    } else
      return new T(this, target, pattern, bm, f_imdl);
  }
public:
  AutoFocusController(r_code::_View *view);
  ~AutoFocusController();

  r_code::Code *get_core_object() const override;

  void take_input(r_exec::View *input) override;
  void reduce(r_exec::View *input);

  View *inject_input(View *input); // inject a filtered input into the output groups starting from 0; return the view injected in the primary group.
  void inject_input(View *input, uint32 start); // inject an unfiltered input into the output groups starting from start.
  void inject_input(View *input, _Fact *abstract_input, BindingMap *bm); // inject a filtered input into the output groups.
  void inject_hlps(const std::vector<P<r_code::Code> > &hlps) const; // called by TPX; hlp is a mdl or a cst.

  bool decompile_models() const { return decompile_models_; }
  bool gtpx_on() const { return gtpx_on_; }
  bool ptpx_on() const { return ptpx_on_; }
  Group *get_primary_group() const { return output_groups_[0]; }

  void copy_cross_buffer(r_code::list<Input> &destination);
  r_code::time_buffer<CInput, CInput::IsInvalidated> &get_cache() { return cache_; }
};
}


#endif
