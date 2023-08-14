//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2023 Leonard Eberding
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

#include "operators.h"

#include "../r_exec/init.h"
#include "../r_exec/mem.h"

static uint16 Vec3Opcode;
static uint16 Vec2Opcode;
static uint16 VecOpcode;
static uint16 QuatOpcode;

namespace usr_operators {

////////////////////////////////////////////////////////////////////////////////

bool add(const r_exec::Context &context) {

  r_exec::Context lhs = *context.get_child(1);
  r_exec::Context rhs = *context.get_child(2);

  if (lhs[0].asOpcode() == Vec3Opcode && rhs[0].asOpcode() == Vec3Opcode) {

    context.setCompoundResultHead(Atom::Object(Vec3Opcode, 3));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() + (*rhs.get_child(1))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() + (*rhs.get_child(2))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(3))[0].asFloat() + (*rhs.get_child(3))[0].asFloat()));
    return true;
  }

  if (lhs[0].asOpcode() == Vec2Opcode && rhs[0].asOpcode() == Vec2Opcode) {

    context.setCompoundResultHead(Atom::Object(Vec2Opcode, 2));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() + (*rhs.get_child(1))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() + (*rhs.get_child(2))[0].asFloat()));
    return true;
  }


  if (lhs[0].asOpcode() == VecOpcode && rhs[0].asOpcode() == VecOpcode) {

    r_exec::Context lhs_vals = *lhs.get_child(1);
    r_exec::Context rhs_vals = *rhs.get_child(1);
    if (lhs_vals.get_children_count() == rhs_vals.get_children_count()) {

      int dimensionality = rhs_vals.get_children_count();
      uint16 vals_index = 2 + context.setCompoundResultHead(Atom::Object(VecOpcode, 1));
      context.addCompoundResultPart(Atom::IPointer(vals_index));

      context.addCompoundResultPart(Atom::Set(dimensionality));
      for (int i = 1; i <= dimensionality; ++i) {
        context.addCompoundResultPart(Atom::Float((*lhs_vals.get_child(i))[0].asFloat() + (*rhs_vals.get_child(i))[0].asFloat()));
      }
      return true;
    }
  }

  if (lhs[0].asOpcode() == QuatOpcode && rhs[0].asOpcode() == QuatOpcode) {

    context.setCompoundResultHead(Atom::Object(QuatOpcode, 4));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() + (*rhs.get_child(1))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() + (*rhs.get_child(2))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(3))[0].asFloat() + (*rhs.get_child(3))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(4))[0].asFloat() + (*rhs.get_child(4))[0].asFloat()));
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool sub(const r_exec::Context &context) {

  r_exec::Context lhs = *context.get_child(1);
  r_exec::Context rhs = *context.get_child(2);

  if (lhs[0].asOpcode() == Vec3Opcode && rhs[0].asOpcode() == Vec3Opcode) {

    context.setCompoundResultHead(Atom::Object(Vec3Opcode, 3));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() - (*rhs.get_child(1))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() - (*rhs.get_child(2))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(3))[0].asFloat() - (*rhs.get_child(3))[0].asFloat()));
    return true;
  }

  if (lhs[0].asOpcode() == Vec2Opcode && rhs[0].asOpcode() == Vec2Opcode) {

    context.setCompoundResultHead(Atom::Object(Vec2Opcode, 2));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() - (*rhs.get_child(1))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() - (*rhs.get_child(2))[0].asFloat()));
    return true;
  }

  if (lhs[0].asOpcode() == VecOpcode && rhs[0].asOpcode() == VecOpcode) {

    r_exec::Context lhs_vals = *lhs.get_child(1);
    r_exec::Context rhs_vals = *rhs.get_child(1);
    if (lhs_vals.get_children_count() == rhs_vals.get_children_count()) {

      int dimensionality = rhs_vals.get_children_count();
      uint16 vals_index = 2 + context.setCompoundResultHead(Atom::Object(VecOpcode, 1));
      context.addCompoundResultPart(Atom::IPointer(vals_index));

      context.addCompoundResultPart(Atom::Set(dimensionality));
      for (int i = 1; i <= dimensionality; ++i) {
        context.addCompoundResultPart(Atom::Float((*lhs_vals.get_child(i))[0].asFloat() - (*rhs_vals.get_child(i))[0].asFloat()));
      }
      return true;
    }
  }

  if (lhs[0].asOpcode() == QuatOpcode && rhs[0].asOpcode() == QuatOpcode) {

    context.setCompoundResultHead(Atom::Object(QuatOpcode, 4));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() - (*rhs.get_child(1))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() - (*rhs.get_child(2))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(3))[0].asFloat() - (*rhs.get_child(3))[0].asFloat()));
    context.addCompoundResultPart(Atom::Float((*lhs.get_child(4))[0].asFloat() - (*rhs.get_child(4))[0].asFloat()));
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool mul(const r_exec::Context &context) {

  r_exec::Context lhs = *context.get_child(1);
  r_exec::Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].asOpcode() == Vec3Opcode) {

      context.setCompoundResultHead(Atom::Object(Vec3Opcode, 3));
      context.addCompoundResultPart(Atom::Float(lhs[0].asFloat()*(*rhs.get_child(1))[0].asFloat()));
      context.addCompoundResultPart(Atom::Float(lhs[0].asFloat()*(*rhs.get_child(2))[0].asFloat()));
      context.addCompoundResultPart(Atom::Float(lhs[0].asFloat()*(*rhs.get_child(3))[0].asFloat()));
      return true;
    }

    if (rhs[0].asOpcode() == Vec2Opcode) {

      context.setCompoundResultHead(Atom::Object(Vec2Opcode, 2));
      context.addCompoundResultPart(Atom::Float(lhs[0].asFloat() * (*rhs.get_child(1))[0].asFloat()));
      context.addCompoundResultPart(Atom::Float(lhs[0].asFloat() * (*rhs.get_child(2))[0].asFloat()));
      return true;
    }

    if(rhs[0].asOpcode() == VecOpcode) {

      r_exec::Context rhs_vals = *rhs.get_child(1);
      int dimensionality = rhs_vals.get_children_count();
      uint16 vals_index = 2 + context.setCompoundResultHead(Atom::Object(VecOpcode, 1));
      context.addCompoundResultPart(Atom::IPointer(vals_index));

      context.addCompoundResultPart(Atom::Set(dimensionality));
      for (int i = 1; i <= dimensionality; ++i) {
        context.addCompoundResultPart(Atom::Float(lhs[0].asFloat() * (*rhs_vals.get_child(i))[0].asFloat()));
      }
      return true;
    }
  }
  else if (lhs[0].asOpcode() == Vec3Opcode) {

    if (rhs[0].isFloat()) {

      context.setCompoundResultHead(Atom::Object(Vec3Opcode, 3));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat()*rhs[0].asFloat()));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat()*rhs[0].asFloat()));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(3))[0].asFloat()*rhs[0].asFloat()));
      return true;
    }
  }
  else if (lhs[0].asOpcode() == Vec2Opcode) {

    if (rhs[0].isFloat()) {

      context.setCompoundResultHead(Atom::Object(Vec2Opcode, 2));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() * rhs[0].asFloat()));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() * rhs[0].asFloat()));
      return true;
    }
  }
  else if (lhs[0].asOpcode() == VecOpcode) {

    if (rhs[0].isFloat()) {

      r_exec::Context lhs_vals = *lhs.get_child(1);
      int dimensionality = lhs_vals.get_children_count();
      uint16 vals_index = 2 + context.setCompoundResultHead(Atom::Object(VecOpcode, 1));
      context.addCompoundResultPart(Atom::IPointer(vals_index));

      context.addCompoundResultPart(Atom::Set(dimensionality));
      for (int i = 1; i <= dimensionality; ++i) {
        context.addCompoundResultPart(Atom::Float((*lhs_vals.get_child(i))[0].asFloat() * rhs[0].asFloat()));
      }
      return true;
    }
  }
  else if (lhs[0].asOpcode() == QuatOpcode && rhs[0].asOpcode() == QuatOpcode) {

    float32 w1 = (*lhs.get_child(1))[0].asFloat();
    float32 x1 = (*lhs.get_child(2))[0].asFloat();
    float32 y1 = (*lhs.get_child(3))[0].asFloat();
    float32 z1 = (*lhs.get_child(4))[0].asFloat();

    float32 w2 = (*rhs.get_child(1))[0].asFloat();
    float32 x2 = (*rhs.get_child(2))[0].asFloat();
    float32 y2 = (*rhs.get_child(3))[0].asFloat();
    float32 z2 = (*rhs.get_child(4))[0].asFloat();

    float32 w_new = round((-x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2) * 100.) / 100.;
    float32 x_new = round(( x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2) * 100.) / 100.;
    float32 y_new = round((-x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2) * 100.) / 100.;
    float32 z_new = round(( x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2) * 100.) / 100.;

    context.setCompoundResultHead(Atom::Object(QuatOpcode, 4));
    context.addCompoundResultPart(Atom::Float(w_new));
    context.addCompoundResultPart(Atom::Float(x_new));
    context.addCompoundResultPart(Atom::Float(y_new));
    context.addCompoundResultPart(Atom::Float(z_new));

    return true;
  }
  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool div(const r_exec::Context& context) {

  r_exec::Context lhs = *context.get_child(1);
  r_exec::Context rhs = *context.get_child(2);

  if (lhs[0].asOpcode() == Vec3Opcode) {

    if (rhs[0].isFloat() && rhs[0].asFloat() != 0) {

      context.setCompoundResultHead(Atom::Object(Vec3Opcode, 3));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() / rhs[0].asFloat()));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() / rhs[0].asFloat()));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(3))[0].asFloat() / rhs[0].asFloat()));
      return true;
    }
  }
  else if (lhs[0].asOpcode() == Vec2Opcode) {

    if (rhs[0].isFloat() && rhs[0].asFloat() != 0) {

      context.setCompoundResultHead(Atom::Object(Vec2Opcode, 2));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(1))[0].asFloat() / rhs[0].asFloat()));
      context.addCompoundResultPart(Atom::Float((*lhs.get_child(2))[0].asFloat() / rhs[0].asFloat()));
      return true;
    }
  }
  else if (lhs[0].asOpcode() == VecOpcode) {

    if (rhs[0].isFloat() && rhs[0].asFloat() != 0) {

      r_exec::Context lhs_vals = *lhs.get_child(1);
      int dimensionality = lhs_vals.get_children_count();
      uint16 vals_index = 2 + context.setCompoundResultHead(Atom::Object(VecOpcode, 1));
      context.addCompoundResultPart(Atom::IPointer(vals_index));

      context.addCompoundResultPart(Atom::Set(dimensionality));
      for (int i = 1; i <= dimensionality; ++i) {
        context.addCompoundResultPart(Atom::Float((*lhs_vals.get_child(i))[0].asFloat() / rhs[0].asFloat()));
      }
      return true;
    }
  }
  else if (lhs[0].asOpcode() == QuatOpcode && rhs[0].asOpcode() == QuatOpcode) {

    //taking the conjugate here.
    float32 w1 = (*lhs.get_child(1))[0].asFloat();
    float32 x1 = -(*lhs.get_child(2))[0].asFloat();
    float32 y1 = -(*lhs.get_child(3))[0].asFloat();
    float32 z1 = -(*lhs.get_child(4))[0].asFloat();

    float32 w2 = (*rhs.get_child(1))[0].asFloat();
    float32 x2 = (*rhs.get_child(2))[0].asFloat();
    float32 y2 = (*rhs.get_child(3))[0].asFloat();
    float32 z2 = (*rhs.get_child(4))[0].asFloat();

    // And take the conjugate again
    float32 w_new = round((-x1 * x2 - y1 * y2 - z1 * z2 + w1 * w2) * 100.) / 100.;
    float32 x_new = -round((x1 * w2 + y1 * z2 - z1 * y2 + w1 * x2) * 100.) / 100.;
    float32 y_new = -round((-x1 * z2 + y1 * w2 + z1 * x2 + w1 * y2) * 100.) / 100.;
    float32 z_new = -round((x1 * y2 - y1 * x2 + z1 * w2 + w1 * z2) * 100.) / 100.;

    float32 _1 = w_new < 0 ? -1. : 1.;

    context.setCompoundResultHead(Atom::Object(QuatOpcode, 4));
    context.addCompoundResultPart(Atom::Float(_1 * w_new));
    context.addCompoundResultPart(Atom::Float(_1 * x_new));
    context.addCompoundResultPart(Atom::Float(_1 * y_new));
    context.addCompoundResultPart(Atom::Float(_1 * z_new));

    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool dis(const r_exec::Context &context) {

  r_exec::Context lhs = *context.get_child(1);
  r_exec::Context rhs = *context.get_child(2);

  if (lhs[0].asOpcode() == Vec3Opcode && rhs[0].asOpcode() == Vec3Opcode) {

    float32 d1 = (*lhs.get_child(1))[0].asFloat() - (*rhs.get_child(1))[0].asFloat();
    float32 d2 = (*lhs.get_child(2))[0].asFloat() - (*rhs.get_child(2))[0].asFloat();
    float32 d3 = (*lhs.get_child(3))[0].asFloat() - (*rhs.get_child(3))[0].asFloat();

    float32 norm2 = d1 * d1 + d2 * d2 + d3 * d3;
    context.setAtomicResult(Atom::Float(sqrt(norm2)));
    return true;
  }

  if (lhs[0].asOpcode() == Vec2Opcode && rhs[0].asOpcode() == Vec2Opcode) {

    float32 d1 = (*lhs.get_child(1))[0].asFloat() - (*rhs.get_child(1))[0].asFloat();
    float32 d2 = (*lhs.get_child(2))[0].asFloat() - (*rhs.get_child(2))[0].asFloat();

    float32 norm2 = d1 * d1 + d2 * d2;
    context.setAtomicResult(Atom::Float(sqrt(norm2)));
    return true;
  }

  if (lhs[0].asOpcode() == VecOpcode && rhs[0].asOpcode() == VecOpcode) {

    r_exec::Context lhs_vals = *lhs.get_child(1);
    r_exec::Context rhs_vals = *rhs.get_child(1);
    if (lhs_vals.get_children_count() == rhs_vals.get_children_count()) {

      int dimensionality = lhs_vals.get_children_count();
      float32 norm2 = 0;
      for (int i = 1; i <= dimensionality; ++i) {
        float32 d = (*lhs_vals.get_child(i))[0].asFloat() - (*rhs_vals.get_child(i))[0].asFloat();
        norm2 += (d * d);
      }
      context.setAtomicResult(Atom::Float(sqrt(norm2)));
      return true;
    }
  }

  if (lhs[0].asOpcode() == QuatOpcode && rhs[0].asOpcode() == QuatOpcode) {


    float32 w1 = (*lhs.get_child(1))[0].asFloat();
    float32 x1 = (*lhs.get_child(2))[0].asFloat();
    float32 y1 = (*lhs.get_child(3))[0].asFloat();
    float32 z1 = (*lhs.get_child(4))[0].asFloat();

    float32 w2 = (*rhs.get_child(1))[0].asFloat();
    float32 x2 = (*rhs.get_child(2))[0].asFloat();
    float32 y2 = (*rhs.get_child(3))[0].asFloat();
    float32 z2 = (*rhs.get_child(4))[0].asFloat();

    float32 inner_product = x1 * x2 + y1 * y2 + z1 * z2 + w1 * w2;
    float32 dis = acos(2 * inner_product * inner_product - 1);

    context.setAtomicResult(Atom::Float(dis));

  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

}

r_code::resized_vector<uint16> Operators::Init(OpcodeRetriever r) {

  const char* vec3 = "vec3";
  const char* vec2 = "vec2";
  const char* vec = "vec";
  const char* quat = "quat";

  r_code::resized_vector<uint16> initialized_opcodes;
  initialized_opcodes.push_back(Vec3Opcode = r(vec3));
  initialized_opcodes.push_back(Vec2Opcode = r(vec2));
  initialized_opcodes.push_back(VecOpcode = r(vec));
  initialized_opcodes.push_back(QuatOpcode = r(quat));

  return initialized_opcodes;
}
