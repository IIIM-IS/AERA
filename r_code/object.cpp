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

#include	"object.h"
#include	"replicode_defs.h"

#include	<iostream>


namespace	r_code{

	SysView::SysView(){
	}

	SysView::SysView(View	*source){

		for(uint32	i=0;i<VIEW_CODE_MAX_SIZE;++i)
			code[i]=source->code(i);

		for(uint32	i=0;i<2;++i)	//	to get the right size in Image::add_object().
			if(source->references[i])
				references.push_back(0);
	}

	void	SysView::write(word32	*data){

		data[0]=code.size();
		data[1]=references.size();
		uint32	i=0;
		for(;i<code.size();++i)
			data[2+i]=code[i].atom;
		for(uint32	j=0;j<references.size();++j)
			data[2+i+j]=references[j];
	}

	void	SysView::read(word32	*data){

		uint32	code_size=data[0];
		uint32	reference_set_size=data[1];
		uint32	i;
		uint32	j;
		for(i=0;i<code_size;++i)
			code.push_back(Atom(data[2+i]));
		for(j=0;j<reference_set_size;++j)
			references.push_back(data[2+i+j]);
	}

	uint32	SysView::get_size()	const{

		return	2+code.size()+references.size();
	}

	void	SysView::trace(std::ostream& out){

		out<<" code size: "<<code.size()<<std::endl;
		out<<" reference set size: "<<references.size()<<std::endl;
		out<<"---code---"<<std::endl;
		Atom::TraceContext context;
		for(uint32	i=0;i<code.size();++i){

			code[i].trace(context, out);
			out<<std::endl;
		}
		out<<"---reference set---"<<std::endl;
		for(uint32	i=0;i<references.size();++i)
			out<<references[i]<<std::endl;
	}

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	uint32	SysObject::LastOID=0;

	SysObject::SysObject():oid(LastOID++){
#ifdef WITH_DEBUG_OID
      debug_oid = 0;
#endif
	}

	SysObject::SysObject(Code	*source){

		uint32	i;
		for(i=0;i<source->code_size();++i)
			code[i]=source->code(i);

		UNORDERED_SET<View	*,View::Hash,View::Equal>::const_iterator	v;
		source->acq_views();
		for(i=0,v=source->views.begin();v!=source->views.end();++i,++v)
			views[i]=new	SysView(*v);
		source->rel_views();

		oid=source->get_oid();
#ifdef WITH_DEBUG_OID
		debug_oid = source->get_debug_oid();
#endif

		for(i=0;i<source->references_size();++i)	//	to get the right size in Image::add_object().
			references.push_back(0);
	}

	SysObject::~SysObject(){

		for(uint32	i=0;i<views.size();++i)
			delete	views[i];
	}

	void	SysObject::write(word32	*data){

		data[0]=oid;
		data[1]=code.size();
		data[2]=references.size();
		data[3]=markers.size();
		data[4]=views.size();
		uint32	i;
		uint32	j;
		uint32	k;
		uint32	l;
		for(i=0;i<code.size();++i)
			data[5+i]=code[i].atom;
		for(j=0;j<references.size();++j)
			data[5+i+j]=references[j];
		for(k=0;k<markers.size();++k)
			data[5+i+j+k]=markers[k];
		uint32	offset=0;
		for(l=0;l<views.size();++l){

			views[l]->write(data+5+i+j+k+offset);
			offset+=views[l]->get_size();
		}
	}

	void	SysObject::read(word32	*data){

		oid=data[0];
		uint32	code_size=data[1];
		uint32	reference_set_size=data[2];
		uint32	marker_set_size=data[3];
		uint32	view_set_size=data[4];
		uint32	i;
		uint32	j;
		uint32	k;
		uint32	l;
		for(i=0;i<code_size;++i)
			code.push_back(Atom(data[5+i]));
		for(j=0;j<reference_set_size;++j)
			references.push_back(data[5+i+j]);
		for(k=0;k<marker_set_size;++k)
			markers.push_back(data[5+i+j+k]);
		uint32	offset=0;
		for(l=0;l<view_set_size;++l){

			SysView	*v=new	SysView();
			v->read(data+5+i+j+k+offset);
			views.push_back(v);
			offset+=v->get_size();
		}
	}

	uint32	SysObject::get_size(){

		uint32	view_set_size=0;
		for(uint32	i=0;i<views.size();++i)
			view_set_size+=views[i]->get_size();
		return	5+code.size()+references.size()+markers.size()+view_set_size;
	}

	void	SysObject::trace(std::ostream& out){

		out<<"\n---object---\n";
		out<<oid<<std::endl;
		out<<"code size: "<<code.size()<<std::endl;
		out<<"reference set size: "<<references.size()<<std::endl;
		out<<"marker set size: "<<markers.size()<<std::endl;
		out<<"view set size: "<<views.size()<<std::endl;
		out<<"\n---code---\n";
		uint32	i;
		Atom::TraceContext context;
		for(i=0;i<code.size();++i){

			out<<i<<" ";
			code[i].trace(context, out);
			out<<std::endl;
		}
		out<<"\n---reference set---\n";
		for(i=0;i<references.size();++i)
			out<<i<<" "<<references[i]<<std::endl;
		out<<"\n---marker set---\n";
		for(i=0;i<markers.size();++i)
			out<<i<<" "<<markers[i]<<std::endl;
		out<<"\n---view set---\n";
		for(uint32	k=0;k<views.size();++k){

			out<<"view["<<k<<"]"<<std::endl;
			out<<"reference set size: "<<views[k]->references.size()<<std::endl;
			out<<"-code-"<<std::endl;
			uint32	j;
			for(j=0;j<views[k]->code.size();++i,++j){

				out<<j<<" ";
				views[k]->code[j].trace(context, out);
				out<<std::endl;
			}
			out<<"-reference set-"<<std::endl;
			for(j=0;j<views[k]->references.size();++i,++j)
				out<<j<<" "<<views[k]->references[j]<<std::endl;
		}
	}

	void	SysObject::trace() { trace(std::cout); }

	////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

	Mem	*Mem::Singleton=NULL;

	Mem::Mem(){
		
		Singleton=this;
	}

	Mem	*Mem::Get(){

		return	Singleton;
	}
}