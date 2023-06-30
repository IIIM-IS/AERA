//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2021 Leonard Eberding
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

#include "tcp_io_device.h"

using namespace std;
using namespace std::chrono;
using namespace r_code;
using namespace r_exec;


#ifndef ENABLE_PROTOBUF
namespace tcp_io_device {
  template<class O, class S> TcpIoDevice<O, S>::TcpIoDevice() : MemExec<O, S>()
  {
    cout << "\n> ERROR: Trying to use the TcpIoDevice without setting ENABLE_PROTOBUF flag in the beginning of main.cpp" << endl;
  }

  template<class O, class S>
  TcpIoDevice<O, S>::~TcpIoDevice() {}

  template<class O, class S>
  int TcpIoDevice<O, S>::initTCP(string port)
  {
    return 1;
  }

  template class TcpIoDevice<r_exec::LObject, r_exec::MemStatic>;
  template class TcpIoDevice<r_exec::LObject, r_exec::MemVolatile>;
}
#else

#include "AERA_Protobuf/utils.h"
#include "AERA_Protobuf/tcp_data_message.pb.h"

namespace tcp_io_device {

  template<class O, class S> TcpIoDevice<O, S>::TcpIoDevice() : MemExec<O, S>()
  {

    timeTickThread_ = 0;
    lastInjectTime_ = Timestamp(seconds(0));
    lastCommandTime_ = Timestamp(seconds(0));

    receive_queue_ = std::make_shared<SafeQueue>(1);
    send_queue_ = std::make_shared<SafeQueue>(100);

    tcp_connection_ = new TCPConnection(receive_queue_, send_queue_, 8);

    started_ = false;
  }

  template<class O, class S>
  TcpIoDevice<O, S>::~TcpIoDevice() {

    delete tcp_connection_;

    // Not sure if those are needed, I guess the ownership lies somewhere else, right?
    // Delete entities
    /*for (auto it = objects_.begin(); it != objects_.end(); ++it) {
      delete it->second;
    }

    // Delete objects
    for (auto it = entities_.begin(); it != entities_.end(); ++it) {
      delete it->second;
    } */

    if (timeTickThread_) {
      delete timeTickThread_;
    }
  }

  template<class O, class S>
  int TcpIoDevice<O, S>::initTCP(string port)
  {
    int err = tcp_connection_->establishConnection(port);
    tcp_connection_->start();
    // Wait for a SetupMessage from the client.
    while (true) {
      auto msg = std::move(receive_queue_->dequeue());
      if (!msg) {
        continue;
      }
      if (msg->messagetype() != msg->SETUP) {
        continue;
      }
      handleSetupMessage(std::move(msg));
      break;
    }
    return err;
  }


  template<class O, class S>
  bool TcpIoDevice<O, S>::load(const vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid)
  {
    // Call the method in the parent class.
    if (!MemExec<O, S>::load(objects, stdin_oid, stdout_oid, self_oid)) {
      return false;
    }

    // Load entities
    cout << "> Loading entities:" << endl;
    for (auto it = entities_.begin(); it != entities_.end(); ++it) {
      it->second = find_object(objects, &((it->first)[0]));
      cout << it->first << ":\t" << it->second->get_oid() << endl;
    }

    // Load objects
    cout << "> Loading objects:" << endl;
    for (auto it = objects_.begin(); it != objects_.end(); ++it) {
      it->second = find_object(objects, &((it->first)[0]));
      cout << it->first << ":\t" << it->second->get_oid() << endl;
    }

    // Load command op-codes
    cout << "> Loading commands:" << endl;
    for (auto it = commands_.begin(); it != commands_.end(); ++it) {
      it->second = r_exec::GetOpcode(&((it->first)[0]));
      cout << it->first << ":\t" << it->second << endl;
    }

    return true;
  }



  template<class O, class S>
  r_code::Code* TcpIoDevice<O, S>::eject(r_code::Code* command)
  {
    uint16 function = (command->code(CMD_FUNCTION).atom_ >> 8) & 0x000000FF;

    uint16 args_set_index = command->code(CMD_ARGS).asIndex();
    if (function == commands_["ready"]) {
      if (command->code_size() < 2 || command->code(args_set_index + 1).getDescriptor() != Atom::I_PTR ||
        command->code(command->code(args_set_index + 1).asIndex()).getDescriptor() != Atom::STRING) {
        cout << "> WARNING: Cannot get identifier as string" << endl;
        return NULL;
      }

      string identifier = Utils::GetString(&command->code(command->code(args_set_index + 1).asIndex()));
      if (!started_) {
        cout << "I/O device is ready for " << identifier << endl;
        startTimeTickThread();
      }
      return command;
    }
    // Not ready command, therefore go through all commands and find the appropriate one
    for (auto cmd = commands_.begin(); cmd != commands_.end(); ++cmd) {
      if (cmd->second != function) {
        continue;
      }
      Code* obj = command->get_reference(command->code(args_set_index + 1).asIndex());
      for (auto it = entities_.begin(); it != entities_.end(); ++it) {
        if (it->second != obj) {
          continue;
        }
        tcp_io_device::MsgData msg = constructMessageFromCommand(cmd->first, it->first, command);
        if (!msg.isValid()) {
          cout << "Could not create message from ejected command" << endl;
          return NULL;
        }
        sendDataMessage(msg);
        break;
      }
      return command;
    }

    return NULL;
  }

  template<class O, class S>
  tcp_io_device::MsgData TcpIoDevice<O, S>::constructMessageFromCommand(string cmd_identifier, string entity, r_code::Code* cmd)
  {
    // Get the stored meta data received by the SetupMessage during establishConnection with the correct identifier.
    std::map<string, MetaData>::iterator meta_data_it = meta_data_map_.find(cmd_identifier);
    if (meta_data_it == meta_data_map_.end()) {
      cout << "> WARNING: Could not find cmd identifier " << cmd_identifier << " in the MetaData of available commands!" << endl;
      return tcp_io_device::MsgData::invalidMsgData();
    }

    MetaData stored_meta_data = meta_data_it->second;
    uint16 args_set_index = cmd->code(CMD_ARGS).asIndex();

    if (cmd->code(args_set_index).getAtomCount() == 1) {
      // std::cout << "Cmd without values ejected." << std::endl;
      return tcp_io_device::MsgData::createNewMsgData(stored_meta_data);
    }

    tcp_io_device::MsgData msg_data = tcp_io_device::MsgData::invalidMsgData();
    // Default values for OpCodeHandle == "" and dimensionality of 1.
    int start = args_set_index + 2;
    int end = args_set_index + 3;

    // Otherwise retrieve the start and end indices by de-constructing the r_code::Code object.
    if (stored_meta_data.getOpCodeHandle() != "" ||
      (stored_meta_data.getDimensions() != std::vector<uint64_t>({ 1 }) && stored_meta_data.getDimensions() != std::vector<uint64_t>({ 1, 1 }))) {
      if (cmd->code(args_set_index + 2).getDescriptor() != Atom::I_PTR) {
        std::cout << "ERROR: Ejected command with OpCodeHandle " << stored_meta_data.getOpCodeHandle() << " with dimensionality > 1 and r_code object without the necessary nesting." << std::endl;
        return tcp_io_device::MsgData::invalidMsgData();
      }
      int set_index = cmd->code(args_set_index + 2).asIndex();
      // @todo Check whether dimensionality of stored_meta_data fits the atom count of the set of the r_code Object.
      start = set_index + 1;
      end = start + cmd->code(set_index).getAtomCount();
    }

    std::cout << "Eject cmd: " << cmd_identifier << ", entity: " << entity << ", Value(s): "; // << cmd->code(args_set_index + 2).asFloat() << std::endl;
    for (int i = start; i < end; ++i) {
      std::cout << cmd->code(i).asFloat() << " ";
    }
    std::cout << std::endl;

    switch (auto t = stored_meta_data.getType()) {
      case VariableDescription_DataType_BOOL:
      {
        std::vector<bool> data = getDataVec<bool>(cmd, start, end, t);
        msg_data = tcp_io_device::MsgData::createNewMsgData(stored_meta_data, data);
        break;
      }
      case VariableDescription_DataType_INT64:
      {
        std::vector<int64_t> data = getDataVec<int64_t>(cmd, start, end, t);
        msg_data = tcp_io_device::MsgData::createNewMsgData(stored_meta_data, data);
        break;
      }
      case VariableDescription_DataType_DOUBLE:
      {
        std::vector<double> data = getDataVec<double>(cmd, start, end, t);
        msg_data = tcp_io_device::MsgData::createNewMsgData(stored_meta_data, data);
        break;
      }
      case VariableDescription_DataType_COMMUNICATION_ID:
      {
        std::vector<communication_id_t> data = getDataVec<communication_id_t>(cmd, start, end, t);
        msg_data = tcp_io_device::MsgData::createNewMsgData(stored_meta_data, data);
        break;
      }
      case VariableDescription_DataType_STRING:
      {
        // std::vector<bool> data = getDataVec<bool>(cmd, start, end, t);
        // msg_data.push_back(tcp_io_device::MsgData::createNewMsgData(stored_meta_data, data));
        // TODO: Using the .asIndex() function this can be done (See the Utils::GetString())
        cout << "> WARNING: String type not implemented, yet" << endl;
        return tcp_io_device::MsgData::invalidMsgData();
        break;
      }
    }
    return msg_data;
  }

  template<class O, class S>
  template<class T>
  std::vector<T> TcpIoDevice<O, S>::getDataVec(r_code::Code* cmd, int start_index, int end_index, tcp_io_device::VariableDescription_DataType type) {
    std::vector<T> data;
    switch (type) {
    case VariableDescription_DataType_BOOL:
      for (int i = start_index; i < end_index; ++i) {
        data.push_back(cmd->code(i).asBoolean());
      }
      break;
    case VariableDescription_DataType_INT64:
      for (int i = start_index; i < end_index; ++i) {
        data.push_back((int)cmd->code(i).asFloat());
      }
      break;
    case VariableDescription_DataType_DOUBLE:
      for (int i = start_index; i < end_index; ++i) {
        data.push_back(cmd->code(i).asFloat());
        // std::cout << "Debug DataType Double trace:" << cmd->trace_string() << std::endl;
      }
      break;
    case VariableDescription_DataType_COMMUNICATION_ID:
      for (int i = start_index; i < end_index; ++i) {
        // std::cout << "Start: " << start_index << " End:" << end_index << std::endl;
        // std::cout << "Debug test of Communication_id. 1) cmd trace " << cmd->trace_string() << std::endl;
        if (cmd->code(i).getDescriptor() != Atom::R_PTR) {
          // @todo This isn't an R_PTR whysoever...
          std::cout << "WARNING: Got command which should include a communicaiton id which points to another object without R_PTR. Ignoring it." << std::endl;;
          continue;
        }
        r_code::Code* reference = cmd->get_reference(cmd->code(i).asIndex());
        for (auto it = entities_.begin(); it != entities_.end(); ++it) {
          if (it->second != reference) {
            continue;
          }
          // @todo Need to implement this once the proper object is found earlier.
        }
        data.push_back(cmd->code(i).asFloat());
      }
      break;
    case VariableDescription_DataType_STRING:
      // for (int i = start_index; i < end_index; ++i) {
      //   data.push_back(cmd->code(i).asBoolean());
      // }
      break;
    }
    return data;
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::on_time_tick()
  {
    auto now = r_exec::Now();
    if (now < lastInjectTime_ + get_sampling_period() * 8 / 10) {
      return;
    }
    // Dequeue a msg from the receive_queue_.
    auto msg = std::move(receive_queue_->dequeue());
    // If in diagnostic mode wait for a new message to be received.
    if (reduction_core_count_ == 0 && time_core_count_ == 0 && started_) {
      while (!msg) {
        msg = std::move(receive_queue_->dequeue());
      }
    }
    else {
      // If not in diagnostic mode simply step continue if no message was received
      if (!msg) {
        return;
      }
    }
    std::cout << "Received message of type " << msg->messagetype() << std::endl;

    handleMessage(std::move(msg));
    lastInjectTime_ = now;
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::startTimeTickThread()
  {

    // Send start message to environment
    std::unique_ptr<TCPMessage> msg = std::make_unique<TCPMessage>();
    msg->set_messagetype(TCPMessage_Type_START);
    sendMessage(std::move(msg));
    started_ = true;

    if (reduction_core_count_ == 0 && time_core_count_ == 0) {
      // We don't need a timeTickThread for diagnostic time.
      return;
    }
    if (timeTickThread_) {
      cout << "timeTickThread already running" << endl;
      // We already started the thread.
      return;
    }

    // We are running in real time. on_diagnostic_time_tick() will not be called.
    // Set up a timer thread to call on_time_tick().
    timeTickThread_ = Thread::New<_Thread>(timeTickRun, this);
  }

  template<class O, class S>
  thread_ret thread_function_call TcpIoDevice<O, S>::timeTickRun(void* args)
  {
    TcpIoDevice<O, S>* self = (TcpIoDevice*)args;

    auto sampling_period = MemExec::Get()->get_sampling_period();
    auto tickTime = r_exec::Now();
    // Call on_time_tick at the sampling period.
    while (self->state_ == RUNNING) {
      self->on_time_tick();

      tickTime += sampling_period;
      Thread::Sleep(tickTime - r_exec::Now());
    }
    self->tcp_connection_->stop();

    thread_ret_val(0);
  }


  template<class O, class S>
  void TcpIoDevice<O, S>::sendDataMessage(tcp_io_device::MsgData msg_data) {
    std::unique_ptr<tcp_io_device::TCPMessage> msg = std::make_unique<tcp_io_device::TCPMessage>();
    msg->set_messagetype(tcp_io_device::TCPMessage::DATA);
    tcp_io_device::DataMessage* data_msg = msg->mutable_datamessage();
    tcp_io_device::ProtoVariable* var = data_msg->add_variables();
    msg_data.toMutableProtoVariable(var);
    sendMessage(std::move(msg));
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::sendMessage(std::unique_ptr<TCPMessage> msg) {
    // Simply enqueue the message to send and let the TCPConnection do the actual sending.
    std::cout << "Sending Message of type " << msg->messagetype() << std::endl;
    send_queue_->enqueue(std::move(msg));
  }


  template<class O, class S>
  void TcpIoDevice<O, S>::handleMessage(std::unique_ptr<TCPMessage> msg)
  {
    switch (msg->messagetype()) {
    case TCPMessage_Type_SETUP:
      handleSetupMessage(std::move(msg));
      break;
    case TCPMessage_Type_DATA:
      handleDataMessage(std::move(msg));
      break;
    default:
      cout << "> WARNING: Received Message of different type than SETUP or DATA" << endl;
    }
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::handleDataMessage(std::unique_ptr<TCPMessage> data_msg)
  {
    // @todo: Need to change it to actual receive time
    auto now = r_exec::Now();
    auto data = data_msg->release_datamessage();
    // std::cout << "Received data message. Injecting received data:" << std::endl;
    for (int i = 0; i < data->variables_size(); ++i) {
      MsgData var(&(data->variables(i)));
      auto entity = entities_[id_mapping_[var.getMetaData().getEntityID()]];
      auto obj = objects_[id_mapping_[var.getMetaData().getID()]];
      // std::cout << "Variable " << i << ":" << std::endl;
      // std::cout << "Entity: " << id_mapping_[var.getMetaData().getEntityID()] << std::endl;
      // std::cout << "Property: " << id_mapping_[var.getMetaData().getID()] << std::endl;
      // std::cout << "Value(s):" << std::endl;

      if (var.getMetaData().getType() == VariableDescription_DataType_DOUBLE)
      {
        if (var.getMetaData().getOpCodeHandle() == "") {
          injectDefault<double>(entity, obj, var.getData<double>(), now);
          continue;
        }
        else if (var.getMetaData().getOpCodeHandle() == "set") {
          injectSet<double>(entity, obj, var.getData<double>(), now);
          continue;
        }
        else {
          injectOpCode<double>(entity, obj, var.getData<double>(), now, var.getMetaData().getOpCodeHandle());
          continue;
        }
      }
      else if (var.getMetaData().getType() == VariableDescription_DataType_INT64)
      {
        if (var.getMetaData().getOpCodeHandle() == "") {
          injectDefault<int64_t>(entity, obj, var.getData<int64_t>(), now);
          continue;
        }
        else if (var.getMetaData().getOpCodeHandle() == "set") {
          injectSet<int64_t>(entity, obj, var.getData<int64_t>(), now);
          continue;
        }
        else {
          injectOpCode<int64_t>(entity, obj, var.getData<int64_t>(), now, var.getMetaData().getOpCodeHandle());
          continue;
        }
      }
      
      std::vector<Atom> val;
      if (var.getMetaData().getType() == VariableDescription_DataType_DOUBLE)
      {
        std::vector<double> values = var.getData<double>();
        for (auto it = values.begin(); it != values.end(); ++it) {
          val.push_back(Atom::Float(*it));
        }
      }
      else if (var.getMetaData().getType() == VariableDescription_DataType_INT64){
        std::vector<int64_t> values = var.getData<int64_t>();
        for (auto it = values.begin(); it != values.end(); ++it) {
          val.push_back(Atom::Float(*it));
        }
      }
      else if (var.getMetaData().getType() == VariableDescription_DataType_COMMUNICATION_ID) {
        int64_t val = var.getData<int64_t>()[0];
        std::vector<r_code::Code*> value;
        if (val != -1) {
          if (id_mapping_.find(val) == id_mapping_.end())
          {
            std::cout << "WARNING: Received message with unknown Communication ID" << std::endl;
            continue;
          }
          std::string object_entity = id_mapping_[val];
          // std::cout << object_entity << std::endl;
          if (entities_.find(object_entity) == entities_.end())
          {
            std::cout << "WARNING: Received message with uninitalized entity, this should never happen!" << std::endl;
            continue;
          }
          value.push_back(entities_[object_entity]);
        }
        inject_marker_value_from_io_device(entity, obj, value, now, now + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
        continue;
      }
      if (id_mapping_[var.getMetaData().getID()] == "velocity_y") {
        //inject_marker_value_from_io_device(entity, obj, val, now, now + get_sampling_period(), r_exec::View::SYNC_HOLD, get_stdin());
      }
      else {
        //inject_marker_value_from_io_device(entity, obj, val, now, now + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
      }
    }
  }

  template<class O, class S>
  template<class V>
  void TcpIoDevice<O, S>::injectDefault(r_code::Code* entity, r_code::Code* object, std::vector<V> vals, core::Timestamp time) {
    if (vals.size() == 0) {
      // std::cout << "[]" << std::endl;
      inject_marker_value_from_io_device(entity, object, std::vector<r_code::Code*>(), time, time + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
      return;
    }
    if (vals.size() == 1) {
      // std::cout << vals[0] << std::endl;
      inject_marker_value_from_io_device(entity, object, Atom::Float(vals[0]), time, time + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
      return;
    }
    injectSet<V>(entity, object, vals, time);
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::injectDefault(r_code::Code* entity, r_code::Code* object, std::vector<r_code::Code*> vals, core::Timestamp time) {
    if (vals.size() == 0) {
      inject_marker_value_from_io_device(entity, object, std::vector<r_code::Code*>(), time, time + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
      return;
    }
    if (vals.size() == 1) {
      inject_marker_value_from_io_device(entity, object, vals[0], time, time + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
      return;
    }
    injectSet(entity, object, vals, time);
  }

  template<class O, class S>
  template<class V>
  void TcpIoDevice<O, S>::injectSet(r_code::Code* entity, r_code::Code* object, std::vector<V> vals, core::Timestamp time) {
    std::vector<Atom> atom_vals;
    for (auto it = vals.begin(); it != vals.end(); ++it) {
      // std::cout << *it << std::endl;
      atom_vals.push_back(Atom::Float(*it));
    }
    inject_marker_value_from_io_device(entity, object, atom_vals, time, time + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::injectSet(r_code::Code* entity, r_code::Code* object, std::vector<r_code::Code*> vals, core::Timestamp time) {
    inject_marker_value_from_io_device(entity, object, vals, time, time + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
  }

  template<class O, class S>
  template<class V>
  void TcpIoDevice<O, S>::injectOpCode(r_code::Code* entity, r_code::Code* object, std::vector<V> vals, core::Timestamp time, std::string opcode_handle) {
    core::uint16 op_code = r_exec::GetOpcode(opcode_handle.c_str());
    if (op_code == 0xFFFF) {
      std::cout << "ERROR: Received message with unknown opcode handle! Handle: " << opcode_handle << std::endl;
      return;
    }
    std::vector<Atom> atom_vals;
    for (auto it = vals.begin(); it != vals.end(); ++it) {
      // std::cout << *it << std::endl;
      atom_vals.push_back(Atom::Float(*it));
    }
    inject_marker_value_from_io_device(entity, object, op_code, atom_vals, time, time);
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::handleSetupMessage(std::unique_ptr<TCPMessage> setup_msg)
  {
    auto setup_message = setup_msg->release_setupmessage();
    // Initialize all entities and store their communication ids in the id_mapping_ map.
    cout << "Setup message received." << endl;
    cout << "Parsing entities with communication ids:" << endl;
    for (auto it = setup_message->entities().begin(); it != setup_message->entities().end(); ++it) {
      id_mapping_[it->second] = it->first;
      entities_[it->first] = NULL;
      cout << it->first << " : " << it->second << endl;
    }
    // Initialize all commands and store their communication ids in the id_mapping_ map.
    cout << "Parsing commands with communication ids:" << endl;
    for (auto it = setup_message->commands().begin(); it != setup_message->commands().end(); ++it) {
      id_mapping_[it->second] = it->first;
      commands_[it->first] = 0xFFFF;
      cout << it->first << " : " << it->second << endl;
    }
    // Initialize all properties and store their communication ids in the id_mapping_ map.
    cout << "Parsing objects with communication ids:" << endl;
    for (auto it = setup_message->objects().begin(); it != setup_message->objects().end(); ++it) {
      id_mapping_[it->second] = it->first;
      objects_[it->first] = NULL;
      cout << it->first << " : " << it->second << endl;
    }
    // Add the ready-command, if not received from the environment simulation.
    if (commands_.find("ready") == commands_.end()) {
      commands_["ready"] = 0xFFFF;
    }
    // Get all command descriptions (their meta-data) and store them for later access during the eject calls.
    for (int v = 0; v < setup_message->commanddescriptions_size(); ++v) {
      auto command_desscription = setup_message->commanddescriptions(v);
      if (commands_.find(command_desscription.name()) == commands_.end()) {
        cout << "> WARNING: Variable Description found with different name than any commands. This will never be used!" << endl;
        continue;
      }
      meta_data_map_.insert(std::map<string, MetaData>::value_type(command_desscription.name(), MetaData(&command_desscription.description())));
    }
  }

  // Instatiate this template class as needed by main(). (Needs C++11.)
  template class TcpIoDevice<r_exec::LObject, r_exec::MemStatic>;
  template class TcpIoDevice<r_exec::LObject, r_exec::MemVolatile>;

} // namespace tcp_io_device

#endif // !ENABLE_PROTOBUF