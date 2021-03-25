#include "tcp_io_device.h"

using namespace std::chrono;
using namespace r_code;
using namespace r_exec;


#ifndef ENABLE_PROTOBUF
namespace tcp_io_device {
  template<class O, class S> TcpIoDevice<O, S>::TcpIoDevice() : r_exec::Mem<O, S>()
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

#include "Proto/utils.h"
#include "Proto/tcp_data_message.pb.h"

namespace tcp_io_device {

  template<class O, class S> TcpIoDevice<O, S>::TcpIoDevice() : r_exec::Mem<O, S>()
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
  bool TcpIoDevice<O, S>::load(std::vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid)
  {
    // Call the method in the parent class.
    if (!r_exec::Mem<O, S>::load(objects, stdin_oid, stdout_oid, self_oid)) {
      return false;
    }

    // Load entities
    cout << "> Loading entities:" << endl;
    for (auto it = entities_.begin(); it != entities_.end(); ++it) {
      it->second = findObject(objects, &((it->first)[0]));
      cout << it->first << ":\t" << it->second->get_oid() << endl;
    }

    // Load objects
    cout << "> Loading objects:" << endl;
    for (auto it = objects_.begin(); it != objects_.end(); ++it) {
      it->second = findObject(objects, &((it->first)[0]));
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
      for (auto it = entities_.begin(); it != entities_.end(); ++it) {
        if (it->first.compare(identifier) != 0) {
          continue;
        }

        if (!(command->code_size() >= 3 && command->code(args_set_index + 2).getDescriptor() == Atom::R_PTR &&
          command->references_size() > command->code(args_set_index + 2).asIndex())) {
          cout << "> WARNING: Cannot get the object for ready " << identifier << endl;
          return NULL;
        }

        Code* obj = command->get_reference(command->code(args_set_index + 2).asIndex());
        if (!started_) {
          it->second = obj;
          startTimeTickThread();
        }
        else if (it->second != obj) {
          // For now, don't allow tracking multiple objects.
          return NULL;
        }
        return command;
      }
      // None of the entities available have the same name as the one ejected by AERA
      return NULL;
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
        std::unique_ptr<TCPMessage> msg = std::move(constructMessageFromCommand(cmd->first, it->first, command));
        if (!msg) {
          cout << "Could not create message from ejected command" << endl;
          return NULL;
        }
        sendMessage(std::move(msg));
        break;
      }
      return command;
    }

    return NULL;
  }

  template<class O, class S>
  std::unique_ptr< TCPMessage> TcpIoDevice<O, S>::constructMessageFromCommand(string cmd_identifier, string entity, r_code::Code* cmd)
  {
    // Get the stored meta data received by the SetupMessage during establishConnection with the correct identifier.
    std::map<string, MetaData>::iterator stored_meta_data = meta_data_map_.find(cmd_identifier);
    if (stored_meta_data == meta_data_map_.end()) {
      cout << "> WARNING: Could not find cmd identifier " << cmd_identifier << " in the MetaData of available commands!" << endl;
      return NULL;
    }

    // Create a new message to send later
    std::unique_ptr<TCPMessage> msg = std::make_unique<TCPMessage>();
    msg->set_messagetype(TCPMessage::DATA);

    DataMessage* data_msg = msg->mutable_datamessage();

    ProtoVariable* var = data_msg->add_variables();

    VariableDescription* meta_data = var->mutable_metadata();
    meta_data->set_datatype(stored_meta_data->second.getType());

    for (auto it = id_mapping_.begin(); it != id_mapping_.end(); ++it) {
      if (it->second.compare(entity) == 0) {
        meta_data->set_entityid(it->first);
      }
      if (it->second.compare(cmd_identifier) == 0) {
        meta_data->set_id(it->first);
      }
    }

    auto dim = meta_data->mutable_dimensions();
    for (uint64_t i = 0; i < stored_meta_data->second.getDimensions().size(); i++) {
      dim->Add(stored_meta_data->second.getDimensions()[i]);
    }

    uint16 args_set_index = cmd->code(CMD_ARGS).asIndex();

    // @todo: Only a single value implemented. For multiple values (dim != [1]) this needs to be extended.
    switch (stored_meta_data->second.getType()) {
    case VariableDescription_DataType_BOOL:
    {
      char d = (char)cmd->code(args_set_index + 2).asBoolean();
      var->set_data(std::string(&d));
      break;
    }
    case VariableDescription_DataType_INT64:
    {
      // No direct representation as integer, only float32 in AERA
      int val = (int)cmd->code(args_set_index + 2).asFloat();
      char d[8];
      for (int i = 0; i < 8; i++) {
        d[i] = val >> (8 - 1 - i) * 8;
      }
      var->set_data(std::string(d));
      break;
    }
    case VariableDescription_DataType_DOUBLE:
    {
      double val = (double)cmd->code(args_set_index + 2).asFloat();
      char const* d = reinterpret_cast<char const*>(&val);
      var->set_data(std::string(d));
      break;
    }
    case VariableDescription_DataType_STRING:
    {
      // TODO: Using the .asIndex() function this can be done
      cout << "> WARNING: String type not implemented, yet" << endl;
    }
    }
    return msg;
  }

  template<class O, class S>
  r_code::Code* TcpIoDevice<O, S>::findObject(std::vector<r_code::Code*>* objects, const char* name)
  {
    // Find the object OID.
    uint32 oid = r_exec::Seed.object_names_.findSymbol(name);
    if (oid == UNDEFINED_OID)
      return NULL;

    // Find the object. (Imitate the code in _Mem::load.)
    for (uint32 i = 0; i < objects->size(); ++i) {
      Code* object = (*objects)[i];
      if (object->get_oid() == oid)
        return object;
    }

    return NULL;
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::onTimeTick()
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

    // We are running in real time. onDiagnosticTimeTick() will not be called.
    // Set up a timer thread to call onTimeTick().
    timeTickThread_ = Thread::New<_Thread>(timeTickRun, this);
  }

  template<class O, class S>
  thread_ret thread_function_call TcpIoDevice<O, S>::timeTickRun(void* args)
  {
    TcpIoDevice<O, S>* self = (TcpIoDevice*)args;

    auto sampling_period = Mem::Get()->get_sampling_period();
    auto tickTime = r_exec::Now();
    // Call onTimeTick at the sampling period.
    while (self->state_ == RUNNING) {
      self->onTimeTick();

      tickTime += sampling_period;
      Thread::Sleep(tickTime - r_exec::Now());
    }
    self->tcp_connection_->stop();

    thread_ret_val(0);
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::sendMessage(std::unique_ptr<TCPMessage> msg) {
    // Simply enqueue the message to send and let the TCPConnection do the actual sending.
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
    for (int i = 0; i < data->variables_size(); ++i) {
      ProtoVariable v = data->variables(i);
      MsgData var(&v);
      std::string d = v.data();

      auto entity = entities_[id_mapping_[var.getMetaData().getEntityID()]];
      auto obj = objects_[id_mapping_[var.getMetaData().getID()]];
      Atom val = Atom::Float(var.getData<double>()[0]);
      if (id_mapping_[var.getMetaData().getID()] == "velocity_y") {
        injectMarkerValueFromIoDevice(entity, obj, val, now, now + get_sampling_period(), r_exec::View::SYNC_HOLD, get_stdin());
      }
      else {
        injectMarkerValueFromIoDevice(entity, obj, val, now, now + get_sampling_period(), r_exec::View::SYNC_PERIODIC, get_stdin());
      }
    }
  }

  template<class O, class S>
  void TcpIoDevice<O, S>::handleSetupMessage(std::unique_ptr<TCPMessage> setup_msg)
  {
    auto setup_message = setup_msg->release_setupmessage();
    // Initialize all entities and store their communication ids in the id_mapping_ map.
    for (auto it = setup_message->entities().begin(); it != setup_message->entities().end(); ++it) {
      id_mapping_[it->second] = it->first;
      entities_[it->first] = NULL;
    }
    // Initialize all commands and store their communication ids in the id_mapping_ map.
    for (auto it = setup_message->commands().begin(); it != setup_message->commands().end(); ++it) {
      id_mapping_[it->second] = it->first;
      commands_[it->first] = 0xFFFF;
    }
    // Initialize all properties and store their communication ids in the id_mapping_ map.
    for (auto it = setup_message->objects().begin(); it != setup_message->objects().end(); ++it) {
      id_mapping_[it->second] = it->first;
      objects_[it->first] = NULL;
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

} /

#endif // !ENABLE_PROTOBUF/ namespace tcp_io_device