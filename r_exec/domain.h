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

#ifndef	domain_h
#define	domain_h

#include	"binding_map.h"


namespace	r_exec{
/*
	class	Range:
	public	_Object{
	public:
		virtual	void	add(BindingMap	*bm,uint32	i)=0;
		virtual	void	remove(BindingMap	*bm,uint32	i)=0;
		virtual	bool	contains(BindingMap	*bm,uint32	i)	const=0;
	};

	// Discrete range.
	// Use for bool, std::string, P<Code>.
	template<typename	T>	class	DRange:
	public	Range{
	private:
		UNORDERED_SET<T>	values;

		void	add(T	value){
			values.insert(value);
		}
		void	remove(T	value){
			values.erase(value);
		}
		bool	contains(T	value)	const{
			return	values.find(value)!=values.end();
		}
	public:
		void	add(BindingMap	*bm,uint32	i){}
		void	remove(BindingMap	*bm,uint32	i){}
		bool	contains(BindingMap	*bm,uint32	i)	const{	return	false;	}
	};

	// Continuous range.
	// Use for numerical intervals (float32 or uint64).
	template<typename	T>	class	CRange:
	public	Range{
	private:
		std::list<std::pair<T,T> >	values;

		void	add(T	value){

			std::list<std::pair<T,T> >::const_iterator	i;
			for(i=values.begin();i!=values.end();++i){

				
				
			}
		}
		void	remove(T	value){

			std::list<std::pair<T,T> >::const_iterator	i;
			for(i=values.begin();i!=values.end();++i){


			}
		}
		bool	contains(T	value)	const{

			std::list<std::pair<T,T> >::const_iterator	i;
			for(i=values.begin();i!=values.end();++i){

				if((*i).first<=value	&&	(*i).second>=value)
					return	true;
			}

			return	false;
		}
	public:
		void	add(BindingMap	*bm,uint32	i){}
		void	remove(BindingMap	*bm,uint32	i){}
		bool	contains(BindingMap	*bm,uint32	i)	const{	return	false;	}
	};

	// Last hidden reference of a model.
	class	Domain:
	public	Code{
	private:
		std::vector<P<Range> >	ranges;	// same indices as in a BindingMap.
	public:
		void	add(BindingMap	*bm);
		void	remove(BindingMap	*bm);
		bool	contains(BindingMap	*bm)	const;
	};*/
}


#endif
