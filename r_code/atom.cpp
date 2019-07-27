//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode r_Code
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

#include	"atom.h"

#include	<iostream>
#include	<set>


namespace	r_code{

	uint8	Atom::Members_to_go=0;
	uint8	Atom::Timestamp_data=0;
	uint8	Atom::String_data=0;
	uint8	Atom::Char_count=0;
	void	Atom::trace()	const{

		write_indents();
		switch(getDescriptor()){
		case	NIL:					std::cout<<"nil";return;
		case	BOOLEAN_:				std::cout<<"bl: "<<std::boolalpha<<asBoolean();return;
		case	WILDCARD:				std::cout<<":";return;
		case	T_WILDCARD:				std::cout<<"::";return;
		case	I_PTR:					std::cout<<"iptr: "<<std::dec<<asIndex();return;
		case	VL_PTR:					std::cout<<"vlptr: "<<std::dec<<asIndex();return;
		case	R_PTR:					std::cout<<"rptr: "<<std::dec<<asIndex();return;
		case	IPGM_PTR:				std::cout<<"ipgm_ptr: "<<std::dec<<asIndex();return;
		case	IN_OBJ_PTR:				std::cout<<"in_obj_ptr: "<<std::dec<<(uint32)asInputIndex()<<" "<<asIndex();return;
		case	D_IN_OBJ_PTR:			std::cout<<"d_in_obj_ptr: "<<std::dec<<(uint32)asRelativeIndex()<<" "<<asIndex();return;
		case	OUT_OBJ_PTR:			std::cout<<"out_obj_ptr: "<<std::dec<<asIndex();return;
		case	VALUE_PTR:				std::cout<<"value_ptr: "<<std::dec<<asIndex();return;
		case	PROD_PTR:				std::cout<<"prod_ptr: "<<std::dec<<asIndex();return;
		case	ASSIGN_PTR:				std::cout<<"assign_ptr: "<<std::dec<<(uint16)asAssignmentIndex()<<" "<<asIndex();return;
		case	THIS:					std::cout<<"this";return;
		case	VIEW:					std::cout<<"view";return;
		case	MKS:					std::cout<<"mks";return;
		case	VWS:					std::cout<<"vws";return;
		case	NODE:					std::cout<<"nid: "<<std::dec<<(uint32)getNodeID();return;
		case	DEVICE:					std::cout<<"did: "<<std::dec<<(uint32)getNodeID()<<" "<<(uint32)getClassID()<<" "<<(uint32)getDeviceID();return;
		case	DEVICE_FUNCTION:		std::cout<<"fid: "<<std::dec<< asOpcode() << " (" << GetOpcodeName(asOpcode()).c_str() << ")";return;
		case	C_PTR:					std::cout<<"cptr: "<<std::dec<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	SET:					std::cout<<"set: "<<std::dec<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	OBJECT:					std::cout<<"obj: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	S_SET:					std::cout<<"s_set: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	MARKER:					std::cout<<"mk: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	OPERATOR:				std::cout<<"op: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	STRING:					std::cout<<"st: "<<std::dec<<(uint16)getAtomCount();Members_to_go=String_data=getAtomCount();Char_count=(atom	&	0x000000FF);return;
		case	TIMESTAMP:				std::cout<<"us";Members_to_go=Timestamp_data=2;return;
		case	GROUP:					std::cout<<"grp: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	INSTANTIATED_PROGRAM:
		case	INSTANTIATED_ANTI_PROGRAM:
		case	INSTANTIATED_INPUT_LESS_PROGRAM:
										std::cout<<"ipgm: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	COMPOSITE_STATE:		std::cout<<"cst: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	MODEL:					std::cout<<"mdl: "<<std::dec<<asOpcode()<<" ("<<GetOpcodeName(asOpcode()).c_str()<<") "<<(uint16)getAtomCount();Members_to_go=getAtomCount();return;
		case	NULL_PROGRAM:			std::cout<<"null pgm "<<takesPastInputs()?"all inputs":"new inputs";return;
		default:
			if(Timestamp_data){
				
				--Timestamp_data;
				std::cout<<atom;
			}else	if(String_data){

				--String_data;
				std::string	s;
				char	*content=(char	*)&atom;
				for(uint8	i=0;i<4;++i){

					if(Char_count-->0)
						s+=content[i];
					else
						break;
				}
				std::cout<<s.c_str();
			}else	if(isFloat()){

				std::cout<<"nb: "<<std::scientific<<asFloat();
				return;
			}else
				std::cout<<"undef";
			return;
		}
	}

	void	Atom::write_indents()	const{

		if(Members_to_go){

			std::cout<<"   ";
			--Members_to_go;
		}
	}

	void	Atom::Trace(Atom	*base,uint16	count){

		std::cout<<"--------\n";
		for(uint16	i=0;i<count;++i){

			std::cout<<i<<"\t";
			base[i].trace();
			std::cout<<std::endl;
		}
	}

	// These are filled by r_exec::Init().
	UNORDERED_MAP<uint16, std::set<std::string>> OpcodeNames;

	std::string GetOpcodeName(uint16 opcode) {
		UNORDERED_MAP<uint16, std::set<std::string>>::iterator names = 
		  OpcodeNames.find(opcode);
		if (names == OpcodeNames.end())
			return "unknown";

		std::string result;
		for (std::set<std::string>::iterator it = names->second.begin();
			 it != names->second.end(); ++it) {
			if (result.size() != 0)
				result += "/";
			result += *it;
		}

		return result;
	}

	void AddOpcodeName(uint16 opcode, const char* name) {
		UNORDERED_MAP<uint16, std::set<std::string>>::iterator it = 
		  OpcodeNames.find(opcode);
		if (it == OpcodeNames.end())
			// No existing entry for the opcode.
			OpcodeNames[opcode] = std::set<std::string>();

		// This copies the char* .
		OpcodeNames[opcode].insert(name);
	}
}
