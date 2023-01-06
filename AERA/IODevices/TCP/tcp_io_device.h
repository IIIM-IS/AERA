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

#ifndef tcp_io_device_h
#define tcp_io_device_h

#pragma comment (lib, "Ws2_32.lib")

#define PROTOBUF_USE_DLLS

#include "../r_exec/mem.h"
#include <map>
#include <windows.h>
#include <winsock2.h>
#include <ws2tcpip.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <bitset>

#ifndef ENABLE_PROTOBUF

namespace tcp_io_device {

  template<class O, class S> class TcpIoDevice :
    public r_exec::MemExec<O, S> {

  public:

    TcpIoDevice();
    ~TcpIoDevice();

    /**
    * Initialize the TCP connection and wait for a client to connect to it.
    * \param port the port in which the TCP/IP connection should be communicating with the environment simulation.
    */
    int initTCP(std::string port);
  };
}

#else

#include "tcp_connection.h"

namespace tcp_io_device {

  class MetaData;
  class MsgData;
  

  /**
   * TcpIoDevice extends Mem so that we can override load and eject. It provides an interface
   * to a TCP connection based on the requirements of the SAGE evaluation platform.
   */

  template<class O, class S> class TcpIoDevice :
    public r_exec::MemExec<O, S> {

  public:

    TcpIoDevice();
    ~TcpIoDevice();

    /**
    * Initialize the TCP connection and wait for a client to connect to it.
    * \param port the port in which the TCP/IP connection should be communicating with the environment simulation.
    */
    int initTCP(std::string port);

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
     * This is called when run_in_diagnostic_time() updates the tickTime. Just call
     * on_time_tick(), because there is no real - time timer thread to call it.
     */
    virtual void on_diagnostic_time_tick() { on_time_tick(); }


  protected:
    class _Thread : public Thread {
    };

    TCPConnection* tcp_connection_;
    std::shared_ptr<SafeQueue> receive_queue_;
    std::shared_ptr<SafeQueue> send_queue_;

    std::map<int, std::string> id_mapping_;
    std::map<std::string, r_code::Code*> entities_;
    std::map<std::string, uint16> commands_;
    std::map<std::string, r_code::Code*> objects_;

    std::map<std::string, MetaData> meta_data_map_;
    std::map<std::string, MsgData> data_map_;

    Thread* timeTickThread_;
    Timestamp lastInjectTime_;
    Timestamp lastCommandTime_;

    bool started_;

    /**
    * If not running in diagnostic time, start the timeTickThread_.
    * If it is already started, do nothing.
    */
    void startTimeTickThread();

    /**
    * Execute a single step including receiving and sending of messages
    */
    void on_time_tick();

    /**
     * This runs in the timeTickThread_ to periodicaly call on_time_tick().
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
    std::unique_ptr< TCPMessage> constructMessageFromCommand(std::string cmd_identifier, std::string entity, r_code::Code* cmd);

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

#endif // !ENABLE_PROTOBUF

#endif
