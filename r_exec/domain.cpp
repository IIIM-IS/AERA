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

#include "domain.h"


namespace r_exec {
/*
    void Domain::add(BindingMap *bm){

        for(uint32 i=0;i<ranges.size();++i)
            ranges[i]->add(bm,i);
    }

    void Domain::remove(BindingMap *bm){

        for(uint32 i=0;i<ranges.size();++i)
            ranges[i]->remove(bm,i);
    }

    bool Domain::contains(BindingMap *bm) const{

        if(ranges.size()==0)
            return false;

        for(uint32 i=0;i<ranges.size();++i){

            if(!ranges[i]->contains(bm,i));
                return false;
        }

        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    template<> void DRange<bool>::add(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            add(a->asBoolean());
    }

    template<> void DRange<bool>::remove(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            remove(a->asBoolean());
    }

    template<> bool DRange<bool>::contains(BindingMap *bm,uint32 i) const{

        Atom *a=bm->get_code(i);
        if(a)
            return contains(a->asBoolean());
        return true;
    }

    template<> void DRange<std::string>::add(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            add(Utils::GetString(a));
    }

    template<> void DRange<std::string>::remove(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            remove(Utils::GetString(a));
    }

    template<> bool DRange<std::string>::contains(BindingMap *bm,uint32 i) const{

        Atom *a=bm->get_code(i);
        if(a)
            return contains(Utils::GetString(a));
        return true;
    }

    template<> void DRange<P<Code> >::add(BindingMap *bm,uint32 i){

        P<Code> o=bm->get_object(i);
        if(!!o)
            add(o);
    }

    template<> void DRange<P<Code> >::remove(BindingMap *bm,uint32 i){

        P<Code> o=bm->get_object(i);
        if(!!o)
            remove(o);
    }

    template<> bool DRange<P<Code> >::contains(BindingMap *bm,uint32 i) const{

        P<Code> o=bm->get_object(i);
        if(!!o)
            return contains(o);
        return true;
    }

    ////////////////////////////////////////////////////////////////////////////////////////////////

    template<> void CRange<float32>::add(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            add(a->asFloat());
    }

    template<> void CRange<float32>::remove(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            remove(a->asFloat());
    }

    template<> bool CRange<float32>::contains(BindingMap *bm,uint32 i) const{

        Atom *a=bm->get_code(i);
        if(a)
            return contains(a->asFloat());
        return true;
    }

    template<> void CRange<uint64>::add(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            add(Utils::GetTimestamp(a));
    }

    template<> void CRange<uint64>::remove(BindingMap *bm,uint32 i){

        Atom *a=bm->get_code(i);
        if(a)
            remove(Utils::GetTimestamp(a));
    }

    template<> bool CRange<uint64>::contains(BindingMap *bm,uint32 i) const{

        Atom *a=bm->get_code(i);
        if(a)
            return contains(Utils::GetTimestamp(a));
        return true;
    }*/
}
