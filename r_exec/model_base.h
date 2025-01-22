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

#ifndef model_base_h
#define model_base_h

#include "factory.h"


namespace r_exec {

class _Mem;

// TPX guess models: this list is meant for TPXs to (a) avoid re-guessing known failed models and,
// (b) avoid producing the same models in case they run concurrently.
// The black list contains bad models (models that were killed). This list is trimmed down on a time basis (black_thz) by the garbage collector.
// Each bad model is tagged with the last time it was successfully compared to. GC is performed by comparing this time to the thz.
// The white list contains models that are still alive and is trimmed down when models time out.
// Models are packed before insertion in the white list.
class ModelBase {
  friend class _Mem;
private:
  static ModelBase *singleton_;

  CriticalSection mdlCS_;

  std::chrono::microseconds thz_;

  class MEntry {
  private:
    static bool Match(r_code::Code *lhs, r_code::Code *rhs);
    static uint32 _ComputeHashCode(_Fact *component); // use for lhs/rhs.
  public:
    static uint32 ComputeHashCode(r_code::Code *mdl, bool packed);

    MEntry();
    MEntry(r_code::Code *mdl, bool packed);

    P<r_code::Code> mdl_;
    bool packed_;
    Timestamp touch_time_; // last time the mdl was successfully compared to.
    uint32 hash_code_;

    bool match(const MEntry &e) const;

    class Hash {
    public:
      size_t operator ()(const MEntry& e) const { return e.hash_code_; }
    };

    class Equal {
    public:
      bool operator ()(const MEntry& lhs, const MEntry& rhs) const { return lhs.match(rhs); }
    };
  };

  typedef std::unordered_set<MEntry, typename MEntry::Hash, typename MEntry::Equal> MdlSet;

  MdlSet black_list_; // mdls are already packed when inserted (they come from the white list).
  MdlSet white_list_; // mdls are packed just before insertion.

  void set_thz(std::chrono::microseconds thz) { thz_ = thz; } // called by _Mem::start(); set to secondary_thz.
  void trim_objects(); // called by _Mem::GC().

  ModelBase();
public:
  static ModelBase *Get();

  void load(r_code::Code *mdl); // called by _Mem::load(); models with no views go to the black_list_.
  void get_models(r_code::list<P<r_code::Code> > &models); // white_list_ first, black_list_ next.

  r_code::Code *check_existence(r_code::Code *mdl); // caveat: mdl is unpacked; return (a) NULL if the model is in the black list, (b) a model in the white list if the mdl has been registered there or (c) the mdl itself if not in the model base, in which case the mdl is added to the white list.
  void check_existence(r_code::Code *m0, r_code::Code *m1, r_code::Code *&_m0, r_code::Code *&_m1); // m1 is a requirement on m0; _m0 and _m1 are the return values as defined above; m0 added only if m1 is not black listed.
  void register_mdl_failure(r_code::Code *mdl); // moves the mdl from the white to the black list; happens to bad models.
  void register_mdl_timeout(r_code::Code *mdl); // deletes the mdl from the white list; happen to models that have been unused for primary_thz.
};
}


#endif
