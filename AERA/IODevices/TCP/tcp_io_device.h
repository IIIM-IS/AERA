//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ HUMANOBS - Replicode Test
//_/_/
//_/_/ Leonard Matthias Eberding
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

#ifndef tcp_io_device_h
#define tcp_io_device_h

#pragma comment (lib, "Ws2_32.lib")

#define PROTOBUF_USE_DLLS

#include "../r_exec/mem.h"
#include "Proto/tcp_data_message.pb.h"
#include "tcp_connection.h"
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <bitset>

namespace tcp_io_device {

  /**
  * MetaData is a class to store the meta-data of messages. Especially used for storing of available commands and their descriptions.
  * Additionally gives access to convenience funtions, like VariableDescription message parsing.
  */
  class MetaData {
    friend class MsgData;

  protected:
    int entity_id_;
    int id_;
    VariableDescription_DataType type_;
    size_t type_size_;
    uint64_t data_size_ = 0;
    uint64_t data_length_ = 0;
    std::vector<uint64_t> dimensions_;

  public:
    /**
    * Constructor for MetaData objects. Converts the passed VariableDescription message and creates a MetaData object from it.
    * \param meta_data The VariableDescription used to convert to a MetaData object.
    */
    MetaData(const VariableDescription* meta_data) {
      setMetaData(meta_data->entityid(),
        meta_data->id(),
        meta_data->datatype(),
        std::vector<uint64>(meta_data->dimensions().begin(), meta_data->dimensions().end()));
    }

    /**
    * Returns the id of the entity for which this description is used.
    */
    int getEntityID() {
      return entity_id_;
    }

    /**
    * Returns the id of the property for which this description is used.
    */
    int getID() {
      return id_;
    }

    /**
    * Returns the full length of the data assigned to this MetaData object
    * Number of data parts (e.g. double values) * the data size of each part (e.g. 8)
    */
    uint64_t getDataLength() { return data_length_; }

    /**
    * Returns the number of bytes used to store a data object (e.g. 8 for double)
    */
    uint64_t getDataSize() { return data_size_; }

    /**
    * Returns the dimensions of the data. E.g. [1920, 1080] for a full HD image.
    */
    std::vector<uint64_t> getDimensions() { return dimensions_; }

    /**
    * Returns the type of the corresponding message.
    */
    VariableDescription_DataType getType() { return type_; }


    /**
    * Sets the fields of the MetaData object.
    * \param entity_id The id of the entity as received in the setup message.
    * \param id The id of the property as received in the setup message.
    * \param t The data type of the data (e.g. DOUBLE, INT, or similar).
    * \param d The dimensions of the data.
    */
    void setMetaData(int entity_id, int id, VariableDescription_DataType t, std::vector<uint64_t> d) {
      entity_id_ = entity_id;
      id_ = id;
      type_ = t;
      dimensions_ = d;
      switch (t)
      {
      case 0:
        type_size_ = 8;
        break;
      case 3:
        type_size_ = 8;
        break;
      default:
        type_size_ = 1;
        break;
      }
      data_size_ = 1;
      data_length_ = 1;
      for (int i = 0; i < dimensions_.size(); ++i) {
        data_size_ *= dimensions_[i] * type_size_;
        data_length_ *= dimensions_[i];
      }
    }
  };


  /**
  * Class to store a DataMessage including a MetaData object and a string for the bytes of data.
  */
  class MsgData {
  private:
    MetaData meta_data_;
    string data_;
  public:

    /**
    * Constructor for MsgData objects. Converts a ProtoVariable message to a MsgData object.
    * \param msg The message used to convert and create a MsgData object from.
    */
    MsgData(const ProtoVariable* msg) : meta_data_(&(msg->metadata())) {
      setData(msg->data());
    }


    /**
    * Setter for the data of the message. Data is stored as a byte representation in form of a std::string.
    * \param d The byte representation of the data in form of a std::string.
    */
    void setData(std::string d) {
      data_ = d;
    }


    /**
    * Returns the MetaData object corresponding to this MsgData.
    */
    MetaData getMetaData() {
      return meta_data_;
    }


    /**
    * Casts the data from the byte representation stored as a string to the template type.
    */
    template <typename T> std::vector<T> getData() {
      T a;
      std::vector<T> values;
      for (int i = 0; i < meta_data_.data_size_; i += meta_data_.type_size_) {
        char* pos = &data_[i];
        memcpy(&a, pos, meta_data_.type_size_);
        values.push_back(a);
      }
      return values;
    }
  };



  /**
   * TcpIoDevice extends Mem so that we can override load and eject. It provides an interface
   * to a TCP connection based on the requirements of the SAGE evaluation platform.
   */

  template<class O, class S> class TcpIoDevice :
    public r_exec::Mem<O, S> {

  public:

    TcpIoDevice();
    ~TcpIoDevice();

    /**
    * Initialize the TCP connection and wait for a client to connect to it.
    * \param port the port in which the TCP/IP connection should be communicating with the environment simulation.
    */
    int initTCP(string port);

    /**
     * Call the parent class load(), then set up the objects for the external environment.
     */
    virtual bool load(std::vector<r_code::Code*>* objects, uint32 stdin_oid, uint32 stdout_oid, uint32 self_oid);

    /**
     * Override eject to check for (cmd set_velocity_y ...) and other implemented commands.
     * \param command The command from the Replicode (cmd ...).
     * \return The given command if it is executed as-is, or a new command object of the command
     * that is actually executed. The program controller will make a fact from the command and
     * inject it as the efferent copy. However, if the command is not executed, then return NULL
     * and the program controller will put an anti-fact of the command in the mk.rdx reduction.
     */
    virtual r_code::Code* eject(r_code::Code* command);

    /**
     * This is called when runInDiagnosticTime() updates the tickTime. Just call
     * onTimeTick(), because there is no real - time timer thread to call it.
     */
    virtual void onDiagnosticTimeTick() { onTimeTick(); }


  protected:
    class _Thread : public Thread {
    };

    TCPConnection* tcp_connection_;
    std::shared_ptr<SafeQueue> receive_queue_;
    std::shared_ptr<SafeQueue> send_queue_;

    std::map<int, string> id_mapping_;
    std::map<string, r_code::Code*> entities_;
    std::map<string, uint16> commands_;
    std::map<string, r_code::Code*> objects_;

    std::map<string, MetaData> meta_data_map_;
    std::map<string, MsgData> data_map_;

    Thread* timeTickThread_;
    Timestamp lastInjectTime_;
    Timestamp lastCommandTime_;

    bool started_;


    /**
     * Find the object in r_exec::Seed and objects with the given name.
     * \param objects The objects array from load().
     * \param name The name of the symbol.
     * \return The object, or NULL if not found.
     */
    static r_code::Code* findObject(
      std::vector<r_code::Code*>* objects, const char* name);


    /**
    * If not running in diagnostic time, start the timeTickThread_.
    * If it is already started, do nothing.
    */
    void startTimeTickThread();

    /**
    * Execute a single step including receiving and sending of messages
    */
    void onTimeTick();

    /**
     * This runs in the timeTickThread_ to periodicaly call onTimeTick().
     * (Only used if not running in diagnostic time.)
     * \param args
     * \return
     */
    static thread_ret thread_function_call timeTickRun(void* args);

    /**
    * Creates a TCPMessage from the given command ejected from AERA and returns it.
    * \param cmd_identifier The identifier of the ejected command.
    * \param entity The name of the entity for which the command was ejected.
    * \param cmd the ejected r_code::Code*.
    * \return The constructed TCPMessage which can be sent to the environment simulation.
    */
    std::unique_ptr< TCPMessage> constructMessageFromCommand(string cmd_identifier, string entity, r_code::Code* cmd);

    /**
    * Message handler for incoming messages. Passes them on to specific handlers by the given TCPMessage_Type
    * \param msg The incoming message.
    */
    void handleMessage(std::unique_ptr<TCPMessage> msg);

    /**
    * Message handler for incoming data messages. Creates an r_code::Code object from the message and calls inject to pass the data to AERA.
    * \param data_msg The incoming data message.
    */
    void handleDataMessage(std::unique_ptr<TCPMessage> data_msg);

    /**
    * Message handler for incoming setup messages. Fills the available entities_, objects_, and commands_ maps as well as
    * a mapping of id to string for fast access when receiving messages from the environment simulation.
    * \param setup_msg The incoming message.
    */
    void handleSetupMessage(std::unique_ptr<TCPMessage> setup_msg);

    /**
    * Enqueues messages to the SafeQueues to pass them thread-safe to the TCPConnection which handles the actual sending of the message.
    * \param msg The message to send.
    */
    void sendMessage(std::unique_ptr<TCPMessage> msg);
  };
} // namespace tcp_io_device
#endif
