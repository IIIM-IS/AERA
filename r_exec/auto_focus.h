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
  bool _pass_through;
  bool _ctpx_on;
  bool _gtpx_on;
  bool _ptpx_on;
  bool _trace_injections;
  bool _decompile_models;
  std::vector<Group *> output_groups; // 1st is the primary, 2nd the secondary, followed by other groups if any.

  class Rating {
  public:
    uint32 evidences;
    uint32 positive_evidences;
    float32 success_rate;
    float32 delta_success_rate;

    static bool DeltaSuccessRate(float32 delta_success_rate) {

      return delta_success_rate > 0 && delta_success_rate < _Mem::Get()->get_tpx_dsr_thr();
    }

    Rating() : evidences(0), positive_evidences(0), success_rate(0), delta_success_rate(1) {}

    void add_evidence(bool success) {

      ++evidences;
      if (success)
        ++positive_evidences;
      delta_success_rate = success_rate;
      success_rate = positive_evidences / evidences;
      delta_success_rate = success_rate - delta_success_rate;
    }
  };

  typedef UNORDERED_MAP<P<_Fact>, P<TPX>, PHash<_Fact> > TPXMap;

  TPXMap goals; // f->g->f->target.
  TPXMap predictions; // f->p->f->target.

  typedef UNORDERED_MAP<P<_Fact>, Rating, PHash<_Fact> > RatingMap;

  // entries are patterns, i.e. abstract targets.
  RatingMap goal_ratings;
  RatingMap prediction_ratings;

  static const uint32 CacheInitialSize = 128;
  static const uint32 CrossBufferInitialSize = 1024;

  time_buffer<CInput, CInput::IsInvalidated> cache; // contains all inputs we don't no yet if they are relevant or not; thz==sampling period.
  time_buffer<Input, Input::IsInvalidated> cross_buffer; // contains all relevant inputs.

  void notify(_Fact *target, View *input, TPXMap &map);
  void dispatch_pred_success(_Fact *predicted_f, TPXMap &map);
  void dispatch(View *input, _Fact *abstract_input, BindingMap *bm, bool &injected, TPXMap &map);
  void dispatch_no_inject(View *input, _Fact *abstract_input, BindingMap *bm, TPXMap &map);
  template<class T> TPX *build_tpx(_Fact *target, _Fact *pattern, BindingMap *bm, RatingMap &map, Fact *f_imdl, bool wr_enabled) {

    if (!_gtpx_on && !_ptpx_on)
      return new TPX(this, target, pattern, bm);

    if (wr_enabled)
      return new TPX(this, target, pattern, bm);

    RatingMap::const_iterator r = map.find(pattern);
    if (r != map.end()) {

      if (Rating::DeltaSuccessRate(r->second.delta_success_rate)) // target for which we don't see much improvement over time.
        return new TPX(this, target, pattern, bm);
      else
        return new T(this, target, pattern, bm, f_imdl);
    } else
      return new T(this, target, pattern, bm, f_imdl);
  }
  void rate(_Fact *target, bool success, TPXMap &map, RatingMap &ratings);
public:
  AutoFocusController(r_code::View *view);
  ~AutoFocusController();

  Code *get_core_object() const;

  void take_input(r_exec::View *input);
  void reduce(r_exec::View *input);

  View *inject_input(View *input); // inject a filtered input into the output groups starting from 0; return the view injected in the primary group.
  void inject_input(View *input, uint32 start); // inject an unfiltered input into the output groups starting from start.
  void inject_input(View *input, _Fact *abstract_input, BindingMap *bm); // inject a filtered input into the output groups.
  void inject_hlps(const std::vector<P<Code> > &hlps) const; // called by TPX; hlp is a mdl or a cst.

  bool decompile_models() const { return _decompile_models; }
  bool gtpx_on() const { return _gtpx_on; }
  bool ptpx_on() const { return _ptpx_on; }
  Group *get_primary_group() const { return output_groups[0]; }

  void copy_cross_buffer(r_code::list<Input> &destination);
  time_buffer<CInput, CInput::IsInvalidated> &get_cache() { return cache; }
};
}


#endif
