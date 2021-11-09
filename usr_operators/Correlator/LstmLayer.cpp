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
#include <iostream>

#include "LstmLayer.h"
#include "LstmBlock.h"

LstmLayer::LstmLayer(int nCells, int nInputs, int nOutputs, int nBlocks) {
  lstmBlock_.assign(nBlocks, LstmBlock(nCells, nInputs, nOutputs, nBlocks));
}

void LstmLayer::getBlockOutputs(int t, std::vector <std::vector<LstmBlockState> >& state, std::vector<double>& b) {
  int nCells = state[0][0].bc.size();
  for (int i = 0; i < (int)state.size(); i++) {
    for (int j = 0; j < nCells; j++) {
      if (t >= 0 && t < state[0].size()) { // the time is explicitly given from the calling method
        b[i*nCells + j] = state[i][t].bc[j];
      }
    }
  }

}

void LstmLayer::forwardPassStep(int t, std::vector <std::vector<LstmBlockState> >& state, std::vector<std::vector<double> >& x) {
  int nCells = state[0][0].bc.size();
  std::vector<double> b(state.size()*nCells, 0.);
  getBlockOutputs(t - 1, state, b);

  for (int i = 0; i < (int)state.size(); i++) {
    lstmBlock_[i].forwardPassStep(t, state[i], x[t], b);
  }
}

void LstmLayer::backwardPassStep(int t, std::vector <std::vector<LstmBlockState> >& state,
  std::vector< std::vector<double> >& wK,
  std::vector<double>& dk, std::vector<std::vector<double> >& x) {
  int nCells = state[0][0].bc.size();
  int nBlocks = state.size();
  int nOutputs = wK.size();

  std::vector<std::vector<std::vector<double> > > wKtrans(nBlocks, std::vector<std::vector<double> >(nCells, std::vector<double>(nOutputs)));
  for (int i = 0; i < nBlocks; i++) {
    for (int j = 0; j < nCells; j++) {
      for (int k = 0; k < nOutputs; k++) {
        wKtrans[i][j][k] = wK[k][i * nCells + j];
      }
    }
  }
  /*std::cout << "wKtrans = " << std::endl;
  for(int i=0;i<nBlocks;i++){
      for(int j=0;j<nCells;j++){
          for(int k=0;k<nOutputs;k++){
              std::cout << wKtrans[i][j][k] << " ";
          }
          std::cout << std::endl;
      }
      std::cout << std::endl;
  }*/


  std::vector<std::vector<std::vector<double> > > wHtrans(nBlocks, std::vector<std::vector<double> >(nCells, std::vector<double>(nBlocks * nCells)));
  for (int i = 0; i < nBlocks; i++) {
    for (int j = 0; j < nCells; j++) {
      for (int k = 0; k < nCells*nBlocks; k++) {
        wHtrans[i][j][k] = lstmBlock_[k / nCells].wHc[k % nCells][i * nCells + j];
      }
    }
  }
  /*
  std::cout << "wHtrans = " << std::endl;
  for(int i=0;i<nBlocks;i++){
      for(int j=0;j<nCells;j++){
          for(int k=0;k<nCells*nBlocks;k++){
              std::cout << wHtrans[i][j][k] << " ";
          }
          std::cout << std::endl;
      }
      std::cout << std::endl;
  }*/

  std::vector<double> db(nBlocks*nCells, 0.);
  if (t < state[0].size() - 1) {
    for (int i = 0; i < nBlocks * nCells; i++) {
      db[i] = state[i / nCells][t + 1].dc[i % nCells];
    }
  }

  /*for(int i=0;i<db.size();i++){
      std::cout << db[i] << " ";
  }std::cout << std::endl;
  */
  std::vector<double> gd(nBlocks * 3, 0.);
  std::vector<std::vector<std::vector<double> > > gw(nBlocks, std::vector<std::vector<double> >(nCells, std::vector<double>(nBlocks * 3)));
  if (t < state[0].size() - 1) {
    for (int i = 0; i < nBlocks; i++) {
      gd[3 * i] = state[i][t + 1].di;
      gd[3 * i + 1] = state[i][t + 1].df;
      gd[3 * i + 2] = state[i][t + 1].d_o;
    }
  }

  for (int i = 0; i < nBlocks; i++) {
    for (int j = 0; j < nCells; j++) {
      for (int k = 0; k < nBlocks; k++) {
        gw[i][j][3 * k] = lstmBlock_[k].wHi[nCells * i + j];
        gw[i][j][3 * k + 1] = lstmBlock_[k].wHf[nCells * i + j];;
        gw[i][j][3 * k + 2] = lstmBlock_[k].wHo[nCells * i + j];;
      }
    }
  }

  // other block cells activations (needed for weight update)
  std::vector<double> b(state.size()*nCells, 0.);
  getBlockOutputs(t - 1, state, b); // was t-1

  for (int i = 0; i < (int)state.size(); i++) {
    lstmBlock_[i].backwardPassStep(t, state[i], wKtrans[i], dk, wHtrans[i], db, gw[i], gd, x[t], b);  //fix
  }
}

void LstmLayer::updateWeights(double eta, double alpha) {
  for (int i = 0; i < lstmBlock_.size(); i++) {
    lstmBlock_[i].updateWeights(eta, alpha);
  }
}

void LstmLayer::resetDerivs() {
  for (int i = 0; i < lstmBlock_.size(); i++) {
    lstmBlock_[i].resetDerivs();
  }
}

void LstmLayer::printWeights() {
  for (int i = 0; i < lstmBlock_.size(); i++) {
    std::cout << "Debug [LstmLayer, weights]" << std::endl;
    lstmBlock_[i].printWeights();
  }
}

void LstmLayer::setConstantWeights(double w) {
  for (int i = 0; i < lstmBlock_.size(); i++) {
    lstmBlock_[i].setConstantWeights(w);
  }
}

void LstmLayer::setRandomWeights(double halfRange) {
  for (int i = 0; i < lstmBlock_.size(); i++) {
    lstmBlock_[i].setRandomWeights(halfRange);
  }
}

LstmLayer::~LstmLayer() {

}
