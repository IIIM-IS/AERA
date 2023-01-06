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

#include "operator.h"
#include "context.h"
#include "mem.h"
#include "init.h"
#include "opcodes.h"
#include "group.h"
#include "../submodules/CoreLibrary/CoreLibrary/utils.h"
#include "../r_code/utils.h"
#include <math.h>

using namespace std::chrono;
using namespace r_code;

namespace r_exec {

resized_vector<Operator> Operator::Operators_;

void Operator::Register(uint16 opcode, bool(*o)(const Context &)) {

  if (Operators_[opcode].operator_)
    Operators_[opcode].setOverload(o);
  else
    Operators_[opcode] = Operator(o);
}

////////////////////////////////////////////////////////////////////////////////

bool now(const Context &context) {

  context.setTimestampResult(Now());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool rnd(const Context &context) {

  Context range = *context.get_child(1);

  if (!range[0].isFloat()) {

    context.setAtomicResult(Atom::Nil());
    return false;
  }

  /*Random r;float32 rng=range[0].asFloat();
  float32 result=r(range[0].asFloat());
  result/=ULONG_MAX;*/
  float32 result = (((float32)(rand() % 100)) / 100)*range[0].asFloat();
  context.setAtomicResult(Atom::Float(result));
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool equ(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  bool r = (lhs == rhs);
  context.setAtomicResult(Atom::Boolean(r));
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool neq(const Context &context) {

  bool r = *context.get_child(1) != *context.get_child(2);
  context.setAtomicResult(Atom::Boolean(r));
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool gtr(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() > rhs[0].asFloat();
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) > Utils::GetTimestamp(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      bool r = Utils::GetDuration(&lhs[0]) > Utils::GetDuration(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }

  context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool lsr(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() < rhs[0].asFloat();
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) < Utils::GetTimestamp(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      bool r = Utils::GetDuration(&lhs[0]) < Utils::GetDuration(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }

  context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool gte(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() >= rhs[0].asFloat();
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) >= Utils::GetTimestamp(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      bool r = Utils::GetDuration(&lhs[0]) >= Utils::GetDuration(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }

  context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool lse(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() <= rhs[0].asFloat();
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) <= Utils::GetTimestamp(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      bool r = Utils::GetDuration(&lhs[0]) <= Utils::GetDuration(&rhs[0]);
      context.setAtomicResult(Atom::Boolean(r));
      return true;
    }
  }

  context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool add(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (lhs[0] == Atom::PlusInfinity()) {

        context.setAtomicResult(Atom::PlusInfinity());
        return true;
      }

      if (rhs[0] == Atom::PlusInfinity()) {

        context.setAtomicResult(Atom::PlusInfinity());
        return true;
      }

      context.setAtomicResult(Atom::Float(lhs[0].asFloat() + rhs[0].asFloat()));
      return true;
    } else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      if (lhs[0] != Atom::PlusInfinity()) {

        context.setTimestampResult(Utils::GetTimestamp(&rhs[0]) + microseconds((int64)lhs[0].asFloat()));
        return true;
      }
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      if (lhs[0] != Atom::PlusInfinity()) {
        context.setDurationResult(Utils::GetDuration(&rhs[0]) + microseconds((int64)lhs[0].asFloat()));
        return true;
      }
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].isFloat()) {

      if (rhs[0] != Atom::PlusInfinity()) {

        context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) + microseconds((int64)rhs[0].asFloat()));
        return true;
      }
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) + Utils::GetDuration(&rhs[0]));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      context.setDurationResult(Utils::GetDuration(&lhs[0]) + Utils::GetDuration(&rhs[0]));
      return true;
    }
    else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {
      context.setTimestampResult(Utils::GetDuration(&lhs[0]) + Utils::GetTimestamp(&rhs[0]));
      return true;
    }
    else if (rhs[0].isFloat()) {
      if (rhs[0] != Atom::PlusInfinity()) {
        context.setDurationResult(Utils::GetDuration(&lhs[0]) + microseconds((int64)rhs[0].asFloat()));
        return true;
      }
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool sub(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (lhs[0] == Atom::PlusInfinity()) {

        context.setAtomicResult(Atom::PlusInfinity());
        return true;
      }

      if (rhs[0] == Atom::PlusInfinity()) {

        context.setAtomicResult(Atom::Float(0));
        return true;
      }

      context.setAtomicResult(Atom::Float(lhs[0].asFloat() - rhs[0].asFloat()));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      context.setDurationResult(duration_cast<microseconds>(Utils::GetTimestamp(&lhs[0]) - Utils::GetTimestamp(&rhs[0])));
      return true;
    } else if (rhs[0].isFloat()) {

      if (rhs[0] != Atom::PlusInfinity()) {

        context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) - microseconds((int64)rhs[0].asFloat()));
        return true;
      }
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) - Utils::GetDuration(&rhs[0]));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      context.setDurationResult(Utils::GetDuration(&lhs[0]) - Utils::GetDuration(&rhs[0]));
      return true;
    }
    else if (rhs[0].isFloat()) {
      if (rhs[0] != Atom::PlusInfinity()) {
        context.setDurationResult(Utils::GetDuration(&lhs[0]) - microseconds((int64)rhs[0].asFloat()));
        return true;
      }
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool mul(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (lhs[0] == Atom::PlusInfinity()) {

        if (rhs[0] == Atom::PlusInfinity()) {

          context.setAtomicResult(Atom::PlusInfinity());
          return true;
        }

        if (rhs[0].asFloat() > 0) {

          context.setAtomicResult(Atom::PlusInfinity());
          return true;
        }

        if (rhs[0].asFloat() <= 0) {

          context.setAtomicResult(Atom::Float(0));
          return true;
        }
      }

      if (rhs[0] == Atom::PlusInfinity()) {

        if (lhs[0].asFloat() > 0) {

          context.setAtomicResult(Atom::PlusInfinity());
          return true;
        }

        if (lhs[0].asFloat() <= 0) {

          context.setAtomicResult(Atom::Float(0));
          return true;
        }
      }

      context.setAtomicResult(Atom::Float(lhs[0].asFloat()*rhs[0].asFloat()));
      return true;
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      if (lhs[0] != Atom::PlusInfinity()) {
        context.setAtomicResult(Atom::Float(lhs[0].asFloat() * (float32)Utils::GetDuration(&rhs[0]).count()));
        return true;
      }
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].isFloat()) {
      if (rhs[0] != Atom::PlusInfinity()) {
        float64 lhs_float = (float64)Utils::GetDuration(&lhs[0]).count();
        context.setDurationResult(microseconds((int64)(lhs_float * rhs[0].asFloat())));
        return true;
      }
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      // Counter-intuitive, but is the existing behavior.
      context.setAtomicResult(Atom::Float((float32)Utils::GetDuration(&lhs[0]).count() * (float32)Utils::GetDuration(&rhs[0]).count()));
      return true;
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool div(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (rhs[0].asFloat() != 0) {

        if (lhs[0] == Atom::PlusInfinity()) {

          if (rhs[0] == Atom::PlusInfinity()) {

            context.setAtomicResult(Atom::PlusInfinity());
            return true;
          }

          if (rhs[0].asFloat() > 0) {

            context.setAtomicResult(Atom::PlusInfinity());
            return true;
          }

          if (rhs[0].asFloat() <= 0) {

            context.setAtomicResult(Atom::Float(0));
            return true;
          }
        }

        if (rhs[0] == Atom::PlusInfinity()) {

          if (lhs[0].asFloat() > 0) {

            context.setAtomicResult(Atom::PlusInfinity());
            return true;
          }

          if (lhs[0].asFloat() <= 0) {

            context.setAtomicResult(Atom::Float(0));
            return true;
          }
        }

        context.setAtomicResult(Atom::Float(lhs[0].asFloat() / rhs[0].asFloat()));
        return true;
      }
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      if (lhs[0] != Atom::PlusInfinity()) {
        context.setAtomicResult(Atom::Float(lhs[0].asFloat() / (float32)Utils::GetDuration(&rhs[0]).count()));
        return true;
      }
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].isFloat()) {
      if (rhs[0] != Atom::PlusInfinity()) {
        float64 lhs_float = (float64)Utils::GetDuration(&lhs[0]).count();
        context.setDurationResult(microseconds((int64)(lhs_float / rhs[0].asFloat())));
        return true;
      }
    }
    else if (rhs[0].getDescriptor() == Atom::DURATION) {
      // Counter-intuitive, but is the existing behavior.
      context.setAtomicResult(Atom::Float((float32)Utils::GetDuration(&lhs[0]).count() / (float32)Utils::GetDuration(&rhs[0]).count()));
      return true;
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool dis(const Context &context) {

  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      context.setAtomicResult(Atom::Float(abs(lhs[0].asFloat() - rhs[0].asFloat())));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      context.setDurationResult(abs(duration_cast<microseconds>(Utils::GetTimestamp(&lhs[0]) - Utils::GetTimestamp(&rhs[0]))));
      return true;
    }
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION) {
    if (rhs[0].getDescriptor() == Atom::DURATION) {
      context.setDurationResult(abs(Utils::GetDuration(&lhs[0]) - Utils::GetDuration(&rhs[0])));
      return true;
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool ln(const Context &context) {

  Context arg = *context.get_child(1);

  if (arg[0].isFloat()) {

    if (arg[0].asFloat() != 0) {

      context.setAtomicResult(Atom::Float(::log(arg[0].asFloat())));
      return true;
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool exp(const Context &context) {

  Context arg = *context.get_child(1);

  if (arg[0].isFloat()) {

    context.setAtomicResult(Atom::Float(::exp(arg[0].asFloat())));
    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool log(const Context &context) {

  Context arg = *context.get_child(1);

  if (arg[0].isFloat()) {

    if (arg[0].asFloat() != 0) {

      context.setAtomicResult(Atom::Float(log10(arg[0].asFloat())));
      return true;
    }
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool e10(const Context &context) {

  Context arg = *context.get_child(1);

  if (arg[0].isFloat()) {

    context.setAtomicResult(Atom::Float(pow(10, arg[0].asFloat())));
    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool syn(const Context &context) {

  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool ins(const Context &context) {

  return IPGMContext::Ins(*(IPGMContext *)context.get_implementation());
}

////////////////////////////////////////////////////////////////////////////////

bool red(const Context &context) {

  return IPGMContext::Red(*(IPGMContext *)context.get_implementation());
}

////////////////////////////////////////////////////////////////////////////////

bool fvw(const Context &context) {

  return IPGMContext::Fvw(*(IPGMContext *)context.get_implementation());
}

////////////////////////////////////////////////////////////////////////////////

bool is_sim(const Context &context) {

  const IPGMContext &ipgm_context = *(IPGMContext *)context.get_implementation();
  IPGMContext arg = ipgm_context.get_child_deref(1);
  Code* obj = arg.get_object();

  bool result = false;
  if (obj->code(0).asOpcode() == Opcodes::Goal)
    result = ((Goal*)obj)->is_simulation();
  else if (obj->code(0).asOpcode() == Opcodes::Pred)
    result = ((Pred*)obj)->is_simulation();

  context.setAtomicResult(Atom::Boolean(result));
  return true;
}

bool minimum(const Context &context) {
  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat() && rhs[0].isFloat()) {
    if (lhs[0] == Atom::MinusInfinity() || rhs[0] == Atom::MinusInfinity()) {
      context.setAtomicResult(Atom::MinusInfinity());
      return true;
    }

    if (lhs[0] == Atom::PlusInfinity()) {
      context.setAtomicResult(Atom::Float(rhs[0].asFloat()));
      return true;
    }
    if (rhs[0] == Atom::PlusInfinity()) {
      context.setAtomicResult(Atom::Float(lhs[0].asFloat()));
      return true;
    }

    context.setAtomicResult(Atom::Float(min(lhs[0].asFloat(), rhs[0].asFloat())));
    return true;
  }
  else if (lhs[0].getDescriptor() == Atom::TIMESTAMP && rhs[0].getDescriptor() == Atom::TIMESTAMP) {
    context.setTimestampResult(min(Utils::GetTimestamp(&lhs[0]), Utils::GetTimestamp(&rhs[0])));
    return true;
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION && rhs[0].getDescriptor() == Atom::DURATION) {
    context.setDurationResult(min(Utils::GetDuration(&lhs[0]), Utils::GetDuration(&rhs[0])));
    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

bool maximum(const Context &context) {
  Context lhs = *context.get_child(1);
  Context rhs = *context.get_child(2);

  if (lhs[0].isFloat() && rhs[0].isFloat()) {
    if (lhs[0] == Atom::PlusInfinity() || rhs[0] == Atom::PlusInfinity()) {
      context.setAtomicResult(Atom::PlusInfinity());
      return true;
    }

    if (lhs[0] == Atom::MinusInfinity()) {
      context.setAtomicResult(Atom::Float(rhs[0].asFloat()));
      return true;
    }
    if (rhs[0] == Atom::MinusInfinity()) {
      context.setAtomicResult(Atom::Float(lhs[0].asFloat()));
      return true;
    }

    context.setAtomicResult(Atom::Float(max(lhs[0].asFloat(), rhs[0].asFloat())));
    return true;
  }
  else if (lhs[0].getDescriptor() == Atom::TIMESTAMP && rhs[0].getDescriptor() == Atom::TIMESTAMP) {
    context.setTimestampResult(max(Utils::GetTimestamp(&lhs[0]), Utils::GetTimestamp(&rhs[0])));
    return true;
  }
  else if (lhs[0].getDescriptor() == Atom::DURATION && rhs[0].getDescriptor() == Atom::DURATION) {
    context.setDurationResult(max(Utils::GetDuration(&lhs[0]), Utils::GetDuration(&rhs[0])));
    return true;
  }

  context.setAtomicResult(Atom::Nil());
  return false;
}

}
