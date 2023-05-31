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

#include "usr_operators.h"

#include "../r_exec/init.h"
#include "auto_focus_callback.h"

#include <iostream>
#include <cmath>


void Init(OpcodeRetriever r) {

  Operators::Init(r);

  std::cout << "> usr operators initialized" << std::endl;
}

uint16 GetOperatorCount() {

  return 4;
}

void GetOperatorName(char *op_name) {

  static uint16 op_index = 0;

  if (op_index == 0) {

    std::string s = "add";
    memcpy(op_name, s.c_str(), s.length());
    ++op_index;
    return;
  }

  if (op_index == 1) {

    std::string s = "sub";
    memcpy(op_name, s.c_str(), s.length());
    ++op_index;
    return;
  }

  if (op_index == 2) {

    std::string s = "mul";
    memcpy(op_name, s.c_str(), s.length());
    ++op_index;
    return;
  }

  if (op_index == 3) {

    std::string s = "dis";
    memcpy(op_name, s.c_str(), s.length());
    ++op_index;
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////

uint16 GetProgramCount() {

  return 2;
}

void GetProgramName(char *pgm_name) {

  static uint16 pgm_index = 0;

  if (pgm_index == 0) {

    std::string s = "test_program";
    memcpy(pgm_name, s.c_str(), s.length());
    ++pgm_index;
    return;
  }
  /*
      if(pgm_index==1){

          std::string s="correlator";
          memcpy(pgm_name,s.c_str(),s.length());
          ++pgm_index;
          return;
      }
  */
  if (pgm_index == 1) {

    std::string s = "auto_focus";
    memcpy(pgm_name, s.c_str(), s.length());
    ++pgm_index;
    return;
  }
}

////////////////////////////////////////////////////////////////////////////////

uint16 GetCallbackCount() {

  return 1;
}

void GetCallbackName(char *callback_name) {

  static uint16 callback_index = 0;

  if (callback_index == 0) {

    std::string s = "print";
    memcpy(callback_name, s.c_str(), s.length());
    ++callback_index;
    return;
  }
}

void* GetUserOperatorFunction(const char* function_name) {
  if (strcmp(function_name, "Init") == 0)
    return &Init;
  else if (strcmp(function_name, "GetOperatorCount") == 0)
    return &GetOperatorCount;
  else if (strcmp(function_name, "GetOperatorName") == 0)
    return &GetOperatorName;
  else if (strcmp(function_name, "add") == 0)
    return &usr_operators::add;
  else if (strcmp(function_name, "sub") == 0)
    return &usr_operators::sub;
  else if (strcmp(function_name, "mul") == 0)
    return &usr_operators::mul;
  else if (strcmp(function_name, "dis") == 0)
    return &usr_operators::dis;
  else if (strcmp(function_name, "GetProgramCount") == 0)
    return &GetProgramCount;
  else if (strcmp(function_name, "GetProgramName") == 0)
    return &GetProgramName;
  else if (strcmp(function_name, "test_program") == 0)
    return &test_program;
  else if (strcmp(function_name, "auto_focus") == 0)
    return &auto_focus;
  else if (strcmp(function_name, "GetCallbackCount") == 0)
    return &GetCallbackCount;
  else if (strcmp(function_name, "GetCallbackName") == 0)
    return &GetCallbackName;
  else if (strcmp(function_name, "print") == 0)
    return &usr_operators::print;
  else
    return NULL;
}
