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
//_/_/ Copyright (c) 2012 Jan Koutnik
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

#include <vector>
#include <math.h>

#ifndef LSTMBLOCK_H_
#define LSTMBLOCK_H_

struct LstmBlockState {
  // default constructor creates one cell inside
  LstmBlockState() {
    sc.assign(1, 0.);
    ac.assign(1, 0.);
    bc.assign(1, 0.);
    ec.assign(1, 0.);
    es.assign(1, 0.);
    dc.assign(1, 0.);
    ai = af = ao = bi = bf = bo = di = df = d_o = 0.0; // this is necessary, C++ does not initialize them in vector of states
  }
  LstmBlockState(int nCells) {
    sc.assign(nCells, 0.);
    ac.assign(nCells, 0.);
    bc.assign(nCells, 0.);
    ec.assign(nCells, 0.);
    es.assign(nCells, 0.);
    dc.assign(nCells, 0.);
    ai = af = ao = bi = bf = bo = di = df = d_o = 0.0;
  }
  void addCell() {
    sc.push_back(0.);
    ac.push_back(0.);
    bc.push_back(0.);
    ec.push_back(0.);
    es.push_back(0.);
    dc.push_back(0.);
  }

  // current cell states
  std::vector <double> sc; // cell states
  std::vector <double> ac; // block input activations
  std::vector <double> bc; // memory block output
  double ai; // input gate excitation
  double af; // forget gate excitation
  double ao; // output gate excitation
  double bi; // input gate activation
  double bf; // forget gate activation
  double bo; // output gate activation

  //deltas
  double di; // input gate delta
  double df; // forget gate delta
  double d_o; // output gate delta
  std::vector<double> dc; // cell delta

  //errors
  std::vector<double> ec; // cell outputs errors
  std::vector<double> es; // cell states errors

};

class LstmBlock {
public:
  LstmBlock(int nCells, int nInputs, int nOutputs, int nBlocks);
  void forwardPassStep(int t, std::vector<LstmBlockState>& state, std::vector<double>& x, std::vector<double>& b);
  // the wK vector is a corresponding column from wK matrix stored in ForwardLayer
  // the wH matrix is a part of transposed matrix (columns as rows) for the source cells
  void backwardPassStep(int t, std::vector<LstmBlockState>& state, std::vector< std::vector<double> >& wK,
    std::vector<double>& dk, std::vector< std::vector<double> >& wH, std::vector<double>& db,
    std::vector< std::vector<double> >& gw, std::vector<double>& gd, std::vector<double>& x, std::vector<double>& b);
  void setConstantWeights(int nCells, int nInputs, int nOutputs, int nBlocks, double w);
  void setConstantWeights(double w);
  void setRandomWeights(double halfRange);
  void updateMomentum(double alpha);
  void updateWeights(double eta, double alpha);
  void resetDerivs();

  void printWeights();
  void getSerializedWeights1(std::vector<double> & w);
  void getSerializedWeights2(std::vector<double> & w);
  void getSerializedDerivs1(std::vector<double> & w);
  void getSerializedDerivs2(std::vector<double> & w);
  ~LstmBlock();


  //private:
      //LstmBlockState state;

      // input gate weights, derivatives, momentum weights
  std::vector <double> wIi;
  std::vector <double> wHi;
  std::vector <double> wCi;
  std::vector <double> wIid;
  std::vector <double> wHid;
  std::vector <double> wCid;
  std::vector <double> wIim;
  std::vector <double> wHim;
  std::vector <double> wCim;

  // forget gate weights
  std::vector <double> wIf;
  std::vector <double> wHf;
  std::vector <double> wCf;
  std::vector <double> wIfd;
  std::vector <double> wHfd;
  std::vector <double> wCfd;
  std::vector <double> wIfm;
  std::vector <double> wHfm;
  std::vector <double> wCfm;
  // output gate weights
  std::vector <double> wIo;
  std::vector <double> wHo;
  std::vector <double> wCo;
  std::vector <double> wIod;
  std::vector <double> wHod;
  std::vector <double> wCod;
  std::vector <double> wIom;
  std::vector <double> wHom;
  std::vector <double> wCom;
  // cell weights
  std::vector <std::vector<double> > wIc;
  std::vector<std::vector<double> > wHc;

  std::vector <std::vector<double> > wIcd;
  std::vector<std::vector<double> > wHcd;
  std::vector <std::vector<double> > wIcm;
  std::vector<std::vector<double> > wHcm;

  // bias
  double biasI, biasF, biasO, biasId, biasFd, biasOd, biasIm, biasFm, biasOm;
  std::vector <double> biasC;
  std::vector <double> biasCd;
  std::vector <double> biasCm;

};



inline double fnF(double x) {
  return 1 / (1 + exp(-x));
}
inline double fnFd(double x) {
  double ex = exp(x);
  return ex / ((1 + ex)*(1 + ex));
}

inline double fnG(double x) {
  //return 1/(1+exp(-x));
  return tanh(x);
}
inline double fnGd(double x) {
  //double ex=exp(x);
  //return ex/((1+ex)*(1+ex));
  double th = tanh(x);
  return 1 - th * th;
}

inline double fnH(double x) {
  //return 1/(1+exp(-x));
  return tanh(x);
}
inline double fnHd(double x) {
  //double ex=exp(x);
  //return ex/((1+ex)*(1+ex));
  double th = tanh(x);
  return 1 - th * th;
}

#endif /* LSTMBLOCK_H_ */
