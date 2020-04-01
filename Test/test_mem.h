//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode Test
//_/_/
//_/_/ Jeff Thompson
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

#ifndef test_mem_h
#define test_mem_h

#include "../r_exec/mem.h"

/**
 * TestMem extends Mem so that we can override load and eject, and to implement
 * the behavior of the external environment for pong.2.goal.replicode, etc.
 * If a program does not call a (cmd ...) for the implemented external environment,
 * then it does nothing. (If this external environment is not needed, then main()
 * can just create the parent class r_exec::Mem .)
 */
template<class O, class S> class TestMem :
  public r_exec::Mem<O, S> {
public:
  TestMem();

  ~TestMem();

  /**
   * Call the parent class load(), then set up the objects for the external environment.
   */
  virtual bool load(std::vector<r_code::Code *> *objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid);

  /**
   * Override eject to check for (cmd set_speed_y ...) and other implemented commands.
   */
  virtual void eject(Code *command);

  /**
   * This is called when runInDiagnosticTime() updates the tickTime. Just call
   * onTimeTick(), because there is no real-time timer thread to call it.
   */
  virtual void onDiagnosticTimeTick() { onTimeTick(); }

protected:
  class _Thread : public Thread {
  };

  /**
   * Find the object in r_exec::Seed and objects with the given name.
   * @param objects The objects array from load().
   * @param name The name of the symbol.
   * @return The object, or NULL if not found.
   */
  static Code* findObject
  (std::vector<r_code::Code *> *objects, const char* name);

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [syncMode after 1 1 group nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Atom val, Timestamp after, Timestamp before,
    r_exec::View::SyncMode syncMode, Code* group);

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [syncMode after 1 1 stdin nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Atom val, Timestamp after, Timestamp before,
    r_exec::View::SyncMode syncMode)
  {
    return injectMarkerValue(obj, prop, val, after, before, syncMode, get_stdin());
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 group nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Atom val, Timestamp after, Timestamp before, Code* group)
  {
    return injectMarkerValue
    (obj, prop, val, after, before, r_exec::View::SYNC_PERIODIC, group);
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 stdin nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Atom val, Timestamp after, Timestamp before)
  {
    return injectMarkerValue
    (obj, prop, val, after, before, r_exec::View::SYNC_PERIODIC, get_stdin());
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [syncMode after 1 1 group nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Code* val, Timestamp after, Timestamp before,
    r_exec::View::SyncMode syncMode, Code* group);

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [syncMode after 1 1 stdin nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Code* val, Timestamp after, Timestamp before,
    r_exec::View::SyncMode syncMode)
  {
    return injectMarkerValue(obj, prop, val, after, before, syncMode, get_stdin());
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 group nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Code* val, Timestamp after, Timestamp before, Code* group)
  {
    return injectMarkerValue
    (obj, prop, val, after, before, r_exec::View::SYNC_PERIODIC, group);
  }

  /**
   * Inject (fact (mk.val obj prop val 1) after before 1 1)
   * [SYNC_PERIODIC after 1 1 stdin nil]
   * where val is a simple Atom.
   * @return The created View.
   */
  r_exec::View* injectMarkerValue
  (Code* obj, Code* prop, Code* val, Timestamp after, Timestamp before)
  {
    return injectMarkerValue
    (obj, prop, val, after, before, r_exec::View::SYNC_PERIODIC, get_stdin());
  }

  /**
   * Inject (fact object after before 1 1)
   * [syncMode after 1 1 group nil]
   * @return The created View.
   */
  r_exec::View* injectFact
  (Code* object, Timestamp after, Timestamp before, r_exec::View::SyncMode syncMode,
    Code* group);

  /**
   * Inject (fact object after before 1 1)
   * [SYNC_PERIODIC after 1 1 group nil]
   */
  r_exec::View* injectFact(Code* object, Timestamp after, Timestamp before, Code* group) {
    return injectFact
    (object, after, before, r_exec::View::SYNC_PERIODIC, group);
  }

  void onTimeTick();

  /**
   * If not running in diagnostic time, start the timeTickThread_.
   * If it is already started, do nothing.
   */
  void startTimeTickThread();

  /**
   * This runs in the timeTickThread_ to periodicaly call onTimeTick().
   * (Only used if not running in diagnostic time.)
   */
  static thread_ret thread_function_call timeTickRun(void *args);

  Thread* timeTickThread_;
  Timestamp lastInjectTime_;
  float speed_y_;
  float position_y_;
  Code* position_y_obj_;
  Code* position_property_;
  Code* position_y_property_;
  Code* speed_y_property_;
  Code* primary_group_;
  uint16 set_speed_y_opcode_;
  uint16 move_y_plus_opcode_;
  uint16 move_y_minus_opcode_;
  Timestamp lastCommandTime_;

  Code* yEnt_[10];
  Code* discretePositionObj_;
  Code* discretePosition_;
  Code* nextDiscretePosition_;
  int babbleState_;
};

#endif
