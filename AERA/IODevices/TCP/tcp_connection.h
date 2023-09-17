//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2023 Jeff Thompson
//_/_/ Copyright (c) 2018-2023 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2023 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2021-2023 Leonard Eberding
//_/_/ http://www.iiim.is
//_/_/
//_/_/ --- Open-Source BSD License, with CADIA Clause v 1.0 ---
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

#pragma once

#pragma comment (lib, "Ws2_32.lib")

#include <queue>
#include <mutex>
#include <condition_variable>
#include <winsock2.h>
#include <windows.h>
#include <ws2tcpip.h>
#include <string>
#include <stdlib.h>
#include <stdio.h>
#include <thread>
#include <bitset>

#include "AERA_Protobuf/tcp_data_message.pb.h"

namespace tcp_io_device {

  /**
  * SafeQueue is a thread-safe queue used to pass data from the TcpIoDevice to the TCPConnection for outgoing
  * and the other way around for incoming messages.
  */
  class SafeQueue
  {
  public:
    /**
    * Constructor with the name of the queue. Defaults the max number of messages in the queue to 1.
    */
    SafeQueue()
      : queue_()
      , mutex_()
    {
      max_elements_ = 1;
    }

    /**
    * Constructor with the name of the queue. Sets the max number of messages in the queue accordingly.
    * \param max_elements The max number of messages in the queue before old messages are deleted if a new one is enqueued.
    */
    SafeQueue(int max_elements)
      : queue_()
      , mutex_()
    {
      max_elements_ = max_elements;
    }

    ~SafeQueue()
    {}

    /**
    * Adds a new element to the queue, also deletes old messages, if the number of messages in the queue is >= max_elements.
    * \param t The message to enqueue.
    */
    void enqueue(std::unique_ptr<TCPMessage> t)
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      while (queue_.size() >= max_elements_) {
        if (queue_.front()->messagetype() == TCPMessage_Type_DATA) {
          queue_.front()->release_datamessage();
        }
        queue_.pop();
      }
      queue_.push(std::move(t));
    }


    /**
    * Returns the front element (oldest) of the queue and deletes it.
    * \return Oldest message in the queue.
    */
    std::unique_ptr<TCPMessage> dequeue()
    {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      if (queue_.empty()) {
        return NULL;
      }
      std::unique_ptr<TCPMessage> val = std::move(queue_.front());
      queue_.pop();
      return val;
    }

    /**
    * Clears all entries of the queue.
    */
    void clear() {
      std::lock_guard<std::recursive_mutex> lock(mutex_);
      while (queue_.size() > 0) {
        if (queue_.front()->messagetype() == TCPMessage_Type_DATA) {
          queue_.front()->release_datamessage();
        }
        queue_.pop();
      }
    }



  private:
    std::queue<std::unique_ptr<TCPMessage>> queue_;
    mutable std::recursive_mutex mutex_;
    int max_elements_;
  };


  /**
  * TCPConnection is used to pass data using Win-Sockets between the environment simulation and the TcpIoDevice object. It runs
  * in a different thread and uses SafeQueues to pass data between the IODevice and the socket connection.
  */
  class TCPConnection {

  public:

    static std::map<int, std::string> type_to_name_map_;

    /**
    * Constructor for the TCPConnection used in a seperate thread to communicate with the environment simulation
    * \param receive_queue The queue used to pass incoming messages to the TcpIoDevice.
    * \param send_queue The queue used to pass outgoing messages from the TcpIoDevice.
    * \param msg_length_buf_size The number of bytes used to store the message length of the serialized protobuf message (should be 8)
    */
    TCPConnection(std::shared_ptr<SafeQueue> receive_queue, std::shared_ptr<SafeQueue> send_queue, uint64_t msg_length_buf_size);
    ~TCPConnection();

    /**
    * Opens a socket to connect to a client on the passed port.
    * \param port The port used to communicate with the client.
    * \return 0 for success, nonzero for error.
    */
    int establishConnection(std::string port);

    /**
    * Starts the communication between environment simulation and the AERA TCPConnection.
    */
    void start();

    /**
    * Stops the communication
    */
    void stop();

    /**
    * Returns true if the TCPConnection is running.
    * \return true if running, false otherwise.
    */
    bool isRunning() { return state_ == RUNNING; }

    /**
    * Check the socket if there is incoming data ready. This does not block.
    * \param fd The socket file descriptor.
    * \return 1 if data is ready, 0 if no data is ready, -1 for error.
    */
    static int receiveIsReady(SOCKET fd);

  protected:

    typedef enum {
      NOT_STARTED = 0,
      RUNNING = 1,
      STOPPED = 2,
    }State;

    State state_;


    std::shared_ptr<std::thread> tcp_background_thread_;

    SOCKET tcp_socket_;

    uint64_t msg_buf_size_;
    uint64_t msg_length_buf_size_;

    std::shared_ptr<SafeQueue> incoming_queue_;
    std::shared_ptr<SafeQueue> outgoing_queue_;

    /**
    * Handles the TCP connection in the background by checking for new outgoing and incoming messages, dequeueing and enqueueing the
    * SafeQueues, respectively. Repeatedly checks for new messages on the socket and parses them to TCPMessage objects. Takes TCPMessage
    * objects from the outgoing_queue_ and parses them into a byte-stream to send to the client.
    */
    void tcpBackgroundHandler();

    /**
    * Method to receive data from the socket and parse them into a TCPMessage
    * \return A unique_ptr to a TCPMessage for enqueuing it into the receive_queue_.
    */
    std::unique_ptr<TCPMessage> receiveMessage();

    /**
    * Converts a message to a byte-stream and sends it to the client.
    * \param msg The TCPMessage to send.
    * \return The number of bytes sent. If <= 0 an error occured while sending the message.
    */
    int sendMessage(std::unique_ptr<TCPMessage> msg);
  };

} // namespace tcp_io_device
