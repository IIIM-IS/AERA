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

#ifndef replicode_defs_h
#define replicode_defs_h


#define EXECUTIVE_DEVICE 0xA1000000


#define VIEW_CODE_MAX_SIZE 13 // size of the code of the largest view (grp view) + 1 (oid used by rMems); view set opcode's index is 0.

#define VIEW_OPCODE 0
#define VIEW_SYNC 1
#define VIEW_IJT 2 // iptr to timestamp (+3 atoms)
#define VIEW_SLN 3
#define VIEW_RES 4
#define VIEW_HOST 5
#define VIEW_ORG 6
#define VIEW_ACT 7
#define GRP_VIEW_COV 7
#define GRP_VIEW_VIS 8
#define VIEW_CTRL_0 10 // for nong-group views, this uint32 (not atom) may hold control data (ex: cache status).
#define VIEW_CTRL_1 11 // idem.
#define VIEW_OID 12

#define VIEW_ARITY 6
#define PGM_VIEW_ARITY 7


#define OBJECT_CLASS 0


#define GRP_UPR 1
#define GRP_SLN_THR 2
#define GRP_ACT_THR 3
#define GRP_VIS_THR 4
#define GRP_C_SLN 5
#define GRP_C_SLN_THR 6
#define GRP_C_ACT 7
#define GRP_C_ACT_THR 8
#define GRP_DCY_PER 9
#define GRP_DCY_TGT 10
#define GRP_DCY_PRD 11
#define GRP_DCY_AUTO 12
#define GRP_SLN_CHG_THR 13
#define GRP_SLN_CHG_PRD 14
#define GRP_ACT_CHG_THR 15
#define GRP_ACT_CHG_PRD 16
#define GRP_AVG_SLN 17
#define GRP_HIGH_SLN 18
#define GRP_LOW_SLN 19
#define GRP_AVG_ACT 20
#define GRP_HIGH_ACT 21
#define GRP_LOW_ACT 22
#define GRP_HIGH_SLN_THR 23
#define GRP_LOW_SLN_THR 24
#define GRP_SLN_NTF_PRD 25
#define GRP_HIGH_ACT_THR 26
#define GRP_LOW_ACT_THR 27
#define GRP_ACT_NTF_PRD 28
#define GRP_NTF_NEW 29
#define GRP_LOW_RES_THR 30
#define GRP_NTF_GRPS 31
#define GRP_ARITY 32


#define PGM_TPL_ARGS 1
#define PGM_INPUTS 2
#define PGM_GUARDS 3
#define PGM_PRODS 4
#define PGM_ARITY 5


#define IPGM_PGM 1
#define IPGM_ARGS 2
#define IPGM_RUN 3
#define IPGM_TSC 4
#define IPGM_RES 5
#define IPGM_NFR 6
#define IPGM_ARITY 7


#define ICPP_PGM_NAME 1
#define ICPP_PGM_ARGS 2
#define ICPP_PGM_RUN 3
#define ICPP_PGM_TSC 4
#define ICPP_PGM_RES 5
#define ICPP_PGM_NFR 6
#define ICPP_PGM_ARITY 7


#define MK_RDX_CODE 1
#define MK_RDX_INPUTS 2
#define MK_RDX_PRODS 3
#define MK_RDX_ARITY 4

#define MK_RDX_IHLP_REF 0


#define CMD_FUNCTION 1
#define CMD_ARGS 2
#define CMD_ARITY 3


#define VAL_HLD_ARITY 2


#define MK_VAL_OBJ  1
#define MK_VAL_ATTR  2
#define MK_VAL_VALUE  3
#define MK_VAL_ARITY  4


#define CST_TPL_ARGS 1
#define CST_OBJS 2
#define CST_FWD_GUARDS 3
#define CST_BWD_GUARDS 4
#define CST_OUT_GRPS 5
#define CST_ARITY 6

#define CST_HIDDEN_REFS 1


#define MDL_TPL_ARGS 1
#define MDL_OBJS 2
#define MDL_FWD_GUARDS 3
#define MDL_BWD_GUARDS 4
#define MDL_OUT_GRPS 5
#define MDL_STRENGTH 6
#define MDL_CNT 7
#define MDL_SR 8
#define MDL_DSR 9
#define MDL_ARITY 10

#define MDL_HIDDEN_REFS 1

#define HLP_HIDDEN_REFS 1


#define HLP_TPL_ARGS 1
#define HLP_OBJS 2
#define HLP_FWD_GUARDS 3
#define HLP_BWD_GUARDS 4
#define HLP_OUT_GRPS 5


#define I_HLP_OBJ 1
#define I_HLP_TPL_ARGS 2
#define I_HLP_EXPOSED_ARGS 3
#define I_HLP_WEAK_REQUIREMENT_ENABLED 4
#define I_HLP_ARITY 5


#define FACT_OBJ 1
#define FACT_AFTER 2
#define FACT_BEFORE 3
#define FACT_CFD 4
#define FACT_ARITY 5

#define FACT_OBJ_REF 0


#define PRED_TARGET 1
#define PRED_SIMS 2
#define PRED_ARITY 3

#define PRED_TARGET_REF 0


#define GOAL_TARGET 1
#define GOAL_ACTR 2
#define GOAL_SIM 3
#define GOAL_ARITY 4


#define SUCCESS_OBJ 1
#define SUCCESS_EVD 2
#define SUCCESS_ARITY 3


#define GRP_PAIR_FIRST 1
#define GRP_PAIR_SECOND 2
#define GRP_PAIR_ARITY 3


#define PERF_RDX_LTCY 1
#define PERF_D_RDX_LTCY 2
#define PERF_TIME_LTCY 3
#define PERF_D_TIME_LTCY 4
#define PERF_ARITY 5

#define SIM_MODE 1
#define SIM_THZ 2
#define SIM_F_SUPER_GOAL 3
#define SIM_OPPOSITE 4
#define SIM_ROOT_MODEL 5
#define SIM_SOLUTION_MODEL 6
#define SIM_SOLUTION_CFD 7
#define SIM_SOLUTION_BEFORE 8
#define SIM_ARITY 9

#define UNDEFINED_OID 0xFFFFFFFF

#endif
