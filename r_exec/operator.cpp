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

r_code::vector<Operator> Operator::Operators_;

void Operator::Register(uint16 opcode, bool(*o)(const Context &, uint16 &index)) {

  if (Operators_[opcode].operator_)
    Operators_[opcode].setOverload(o);
  else
    Operators_[opcode] = Operator(o);
}

////////////////////////////////////////////////////////////////////////////////

bool now(const Context &context, uint16 &index) {

  index = context.setTimestampResult(Now());
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool rnd(const Context &context, uint16 &index) {

  Context range = *context.getChild(1);

  if (!range[0].isFloat()) {

    index = context.setAtomicResult(Atom::Nil());
    return false;
  }

  /*Random r;float32 rng=range[0].asFloat();
  float32 result=r(range[0].asFloat());
  result/=ULONG_MAX;*/
  float32 result = (((float32)(rand() % 100)) / 100)*range[0].asFloat();
  index = context.setAtomicResult(Atom::Float(result));
  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool equ(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  bool r = (lhs == rhs);
  index = context.setAtomicResult(Atom::Boolean(r));
  return r;
}

////////////////////////////////////////////////////////////////////////////////

bool neq(const Context &context, uint16 &index) {

  bool r = *context.getChild(1) != *context.getChild(2);
  index = context.setAtomicResult(Atom::Boolean(r));
  return r;
}

////////////////////////////////////////////////////////////////////////////////

bool gtr(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() > rhs[0].asFloat();
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) > Utils::GetTimestamp(&rhs[0]);
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  }

  index = context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool lsr(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() < rhs[0].asFloat();
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) < Utils::GetTimestamp(&rhs[0]);
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  }

  index = context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool gte(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() >= rhs[0].asFloat();
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) >= Utils::GetTimestamp(&rhs[0]);
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  }

  index = context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool lse(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      bool r = lhs[0].asFloat() <= rhs[0].asFloat();
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      bool r = Utils::GetTimestamp(&lhs[0]) <= Utils::GetTimestamp(&rhs[0]);
      index = context.setAtomicResult(Atom::Boolean(r));
      return r;
    }
  }

  index = context.setAtomicResult(Atom::UndefinedBoolean());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool add(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (lhs[0] == Atom::PlusInfinity()) {

        index = context.setAtomicResult(Atom::PlusInfinity());
        return true;
      }

      if (rhs[0] == Atom::PlusInfinity()) {

        index = context.setAtomicResult(Atom::PlusInfinity());
        return true;
      }

      index = context.setAtomicResult(Atom::Float(lhs[0].asFloat() + rhs[0].asFloat()));
      return true;
    } else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      if (lhs[0] != Atom::PlusInfinity()) {

        index = context.setTimestampResult(Utils::GetTimestamp(&rhs[0]) + microseconds((int64)lhs[0].asFloat()));
        return true;
      }
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      index = context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) + Utils::GetTimestamp(&rhs[0]).time_since_epoch());
      return true;
    } else if (rhs[0].isFloat()) {

      if (rhs[0] != Atom::PlusInfinity()) {

        index = context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) + microseconds((int64)rhs[0].asFloat()));
        return true;
      }
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool sub(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (lhs[0] == Atom::PlusInfinity()) {

        index = context.setAtomicResult(Atom::PlusInfinity());
        return true;
      }

      if (rhs[0] == Atom::PlusInfinity()) {

        index = context.setAtomicResult(Atom::Float(0));
        return true;
      }

      index = context.setAtomicResult(Atom::Float(lhs[0].asFloat() - rhs[0].asFloat()));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      index = context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) - Utils::GetTimestamp(&rhs[0]).time_since_epoch());
      return true;
    } else if (rhs[0].isFloat()) {

      if (rhs[0] != Atom::PlusInfinity()) {

        index = context.setTimestampResult(Utils::GetTimestamp(&lhs[0]) - microseconds((int64)rhs[0].asFloat()));
        return true;
      }
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool mul(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (lhs[0] == Atom::PlusInfinity()) {

        if (rhs[0] == Atom::PlusInfinity()) {

          index = context.setAtomicResult(Atom::PlusInfinity());
          return true;
        }

        if (rhs[0].asFloat() > 0) {

          index = context.setAtomicResult(Atom::PlusInfinity());
          return true;
        }

        if (rhs[0].asFloat() <= 0) {

          index = context.setAtomicResult(Atom::Float(0));
          return true;
        }
      }

      if (rhs[0] == Atom::PlusInfinity()) {

        if (lhs[0].asFloat() > 0) {

          index = context.setAtomicResult(Atom::PlusInfinity());
          return true;
        }

        if (lhs[0].asFloat() <= 0) {

          index = context.setAtomicResult(Atom::Float(0));
          return true;
        }
      }

      index = context.setAtomicResult(Atom::Float(lhs[0].asFloat()*rhs[0].asFloat()));
      return true;
    } else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      index = context.setAtomicResult(Atom::Float(Utils::GetMicrosecondsSinceEpoch(&rhs[0]).count() * lhs[0].asFloat()));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].isFloat()) {

      index = context.setTimestampResult(Timestamp(microseconds((int64)(Utils::GetMicrosecondsSinceEpoch(&lhs[0]).count() * rhs[0].asFloat()))));
      return true;
    } else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      index = context.setAtomicResult(Atom::Float(Utils::GetMicrosecondsSinceEpoch(&lhs[0]).count() * Utils::GetMicrosecondsSinceEpoch(&lhs[0]).count()));
      return true;
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool div(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      if (rhs[0].asFloat() != 0) {

        if (lhs[0] == Atom::PlusInfinity()) {

          if (rhs[0] == Atom::PlusInfinity()) {

            index = context.setAtomicResult(Atom::PlusInfinity());
            return true;
          }

          if (rhs[0].asFloat() > 0) {

            index = context.setAtomicResult(Atom::PlusInfinity());
            return true;
          }

          if (rhs[0].asFloat() <= 0) {

            index = context.setAtomicResult(Atom::Float(0));
            return true;
          }
        }

        if (rhs[0] == Atom::PlusInfinity()) {

          if (lhs[0].asFloat() > 0) {

            index = context.setAtomicResult(Atom::PlusInfinity());
            return true;
          }

          if (lhs[0].asFloat() <= 0) {

            index = context.setAtomicResult(Atom::Float(0));
            return true;
          }
        }

        index = context.setAtomicResult(Atom::Float(lhs[0].asFloat() / rhs[0].asFloat()));
        return true;
      }
    } else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      float64 rhs_t = (float64)Utils::GetMicrosecondsSinceEpoch(&rhs[0]).count();
      if (rhs_t != 0) {

        index = context.setAtomicResult(Atom::Float(lhs[0].asFloat() / rhs_t));
        return true;
      }
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].isFloat()) {

      if (rhs[0].asFloat() != 0) {

        float64 lhs_t = (float64)Utils::GetMicrosecondsSinceEpoch(&lhs[0]).count();
        index = context.setTimestampResult(Timestamp(microseconds((int64)(lhs_t / rhs[0].asFloat()))));
        return true;
      }
    } else if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      float64 rhs_t = (float64)Utils::GetMicrosecondsSinceEpoch(&rhs[0]).count();
      if (rhs_t != 0) {

        float64 lhs_t = (float64)Utils::GetMicrosecondsSinceEpoch(&lhs[0]).count();
        index = context.setAtomicResult(Atom::Float(lhs_t / rhs_t));
        return true;
      }
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool dis(const Context &context, uint16 &index) {

  Context lhs = *context.getChild(1);
  Context rhs = *context.getChild(2);

  if (lhs[0].isFloat()) {

    if (rhs[0].isFloat()) {

      index = context.setAtomicResult(Atom::Float(abs(lhs[0].asFloat() - rhs[0].asFloat())));
      return true;
    }
  } else if (lhs[0].getDescriptor() == Atom::TIMESTAMP) {

    if (rhs[0].getDescriptor() == Atom::TIMESTAMP) {

      auto lhs_t = Utils::GetMicrosecondsSinceEpoch(&lhs[0]).count();
      auto rhs_t = Utils::GetMicrosecondsSinceEpoch(&rhs[0]).count();
      index = context.setTimestampResult(Timestamp(microseconds(abs(lhs_t - rhs_t))));
      return true;
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool ln(const Context &context, uint16 &index) {

  Context arg = *context.getChild(1);

  if (arg[0].isFloat()) {

    if (arg[0].asFloat() != 0) {

      index = context.setAtomicResult(Atom::Float(::log(arg[0].asFloat())));
      return true;
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool exp(const Context &context, uint16 &index) {

  Context arg = *context.getChild(1);

  if (arg[0].isFloat()) {

    index = context.setAtomicResult(Atom::Float(::exp(arg[0].asFloat())));
    return true;
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool log(const Context &context, uint16 &index) {

  Context arg = *context.getChild(1);

  if (arg[0].isFloat()) {

    if (arg[0].asFloat() != 0) {

      index = context.setAtomicResult(Atom::Float(log10(arg[0].asFloat())));
      return true;
    }
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool e10(const Context &context, uint16 &index) {

  Context arg = *context.getChild(1);

  if (arg[0].isFloat()) {

    index = context.setAtomicResult(Atom::Float(pow(10, arg[0].asFloat())));
    return true;
  }

  index = context.setAtomicResult(Atom::Nil());
  return false;
}

////////////////////////////////////////////////////////////////////////////////

bool syn(const Context &context, uint16 &index) {

  return true;
}

////////////////////////////////////////////////////////////////////////////////

bool ins(const Context &context, uint16 &index) {

  return IPGMContext::Ins(*(IPGMContext *)context.get_implementation(), index);
}

////////////////////////////////////////////////////////////////////////////////

bool red(const Context &context, uint16 &index) {

  return IPGMContext::Red(*(IPGMContext *)context.get_implementation(), index);
}

////////////////////////////////////////////////////////////////////////////////

bool fvw(const Context &context, uint16 &index) {

  return IPGMContext::Fvw(*(IPGMContext *)context.get_implementation(), index);
}

////////////////////////////////////////////////////////////////////////////////

bool is_sim(const Context &context, uint16 &index) {

  const IPGMContext &ipgm_context = *(IPGMContext *)context.get_implementation();
  IPGMContext arg = *ipgm_context.getChild(1);
  Code* obj = arg.getObject();

  bool result = false;
  if (obj->code(0).asOpcode() == Opcodes::Goal)
    result = ((Goal*)obj)->is_simulation();
  else if (obj->code(0).asOpcode() == Opcodes::Pred)
    result = ((Pred*)obj)->is_simulation();

  index = context.setAtomicResult(Atom::Boolean(result));
  return true;
}

}
