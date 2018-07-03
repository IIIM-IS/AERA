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

#ifndef r_code_image_h
#define	r_code_image_h

#include	"../submodules/CoreLibrary/CoreLibrary/types.h"

#include	<fstream>


using	namespace	std;
using	namespace	core;

namespace	r_comp{
	class	Image;
}

namespace	r_code{

	//	An image contains the following:
	//		- sizes of what follows
	//		- object map: list of indexes (4 bytes) of objects in the code segment
	//		- code segment: list of objects
	//			- object:
	//				- size of the code (number of atoms)
	//				- size of the reference set (number of pointers)
	//				- size of the marker set (number of pointers)
	//				- size of the view set (number of views)
	//				- code: indexes in internal pointers are relative to the beginning of the object, indexes in reference pointers are relative to the beginning of the reference set
	//				- reference set:
	//					- number of pointers
	//					- pointers to the relocation segment, i.e. indexes of relocation entries
	//				- marker set:
	//					- number of pointers
	//					- pointers to the relocation segment, i.e. indexes of relocation entries
	//				- view set: list of views 
	//					- view:
	//						- size of the code (number of atoms)
	//						- size of the reference set (number of pointers)
	//						- list of atoms
	//						- reference set:
	//							- pointers to the relocation segment, i.e. indexes of relocation entries
	//
	//	RAM layout of Image::data [sizes in word32]:
	//
	//		data[0]:								index of first object in data (i.e. def_size+map_size) [1]
	//		...										...
	//		data[map_size-1]:						index of last object in data [1]
	//		data[map_size]:							first word32 of code segment [1]
	//		...										...
	//		data[map_size+code_size-1]:				last word32 of code segment [1]
	//
	//	I is the implementation class; prototype:
	//	class	ImageImpl{
	//	protected:
	//		uin64	get_timestamp()	const;
	//		uint32	map_size()		const;
	//		uint32	code_size()		const;
	//		word32	*data();	//	[object map|code segment|reloc segment]
	//	public:
	//		void	*operator	new(size_t,uint32	data_size);
	//		ImageImpl(uint32	map_size,uint32	code_size);
	//		~ImageImpl();
	//	};

	template<class	I>	class	Image:
	public	I{
	friend	class r_comp::Image;
	public:
		static	Image<I>	*Build(uint64	timestamp,uint32	map_size,uint32	code_size,uint32	names_size);
		//	file IO
		static	Image<I>	*Read(ifstream &stream);
		static	void		Write(Image<I>	*image,ofstream &stream);

		Image();
		~Image();

		uint32	get_size()	const;				//	size of data in word32
		uint32	getObjectCount()	const;
		word32	*getObject(uint32	i);			//	points to the code size of the object; the first atom is at getObject()+2
		word32	*getCodeSegment();				//	equals getObject(0)
		uint32	getCodeSegmentSize()	const;
		
		void	trace()	const;
	};

	//	utilities
	uint32	dll_export	GetSize(const	std::string	&s);	//	returns the number of word32 needed to encode the string
	void	dll_export	Write(word32	*data,const	std::string	&s);
	void	dll_export	Read(word32	*data,std::string	&s);
}


#include	"image.tpl.cpp"


#endif
