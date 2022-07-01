//_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/_/
//_/_/
//_/_/ AERA
//_/_/ Autocatalytic Endogenous Reflective Architecture
//_/_/ 
//_/_/ Copyright (c) 2018-2022 Jeff Thompson
//_/_/ Copyright (c) 2018-2022 Kristinn R. Thorisson
//_/_/ Copyright (c) 2018-2022 Icelandic Institute for Intelligent Machines
//_/_/ Copyright (c) 2021 Leonard Eberding
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

#ifdef ENABLE_PROTOBUF

#include "tcp_connection.h"

namespace tcp_io_device {

  TCPConnection::TCPConnection(std::shared_ptr<SafeQueue> receive_queue, std::shared_ptr<SafeQueue> send_queue, uint64_t msg_length_buf_size)
  {
    outgoing_queue_ = send_queue;
    incoming_queue_ = receive_queue;
    msg_length_buf_size_ = msg_length_buf_size;
    FD_ZERO(&tcp_client_fd_set_);
    state_ = NOT_STARTED;
  }

  TCPConnection::~TCPConnection()
  {
    std::cout << "> INFO: Shutting down TCP connection" << std::endl;
    // Set state to STOPPED triggers end of while loop in the backgroundHandler.
    // Wait for the background thread to join and close the socket, if necessary
    state_ = STOPPED;
    tcp_background_thread_->join();
    if (tcp_client_socket_ != INVALID_SOCKET) {
      int err = shutdown(tcp_client_socket_, SD_BOTH);
      if (err != 0) {
        std::cout << "ERROR: Shutdown of Client Socket failed with error: " << err << std::endl;
      }
      closesocket(tcp_client_socket_);
      WSACleanup();
    }
  }

  int TCPConnection::establishConnection(std::string port)
  {
    WSADATA wsa_data;
    int err;

    err = WSAStartup(MAKEWORD(2, 2), &wsa_data);
    if (err != 0) {
      std::cout << "ERROR: WSAStartup failed with error: " << err << std::endl;
      return err;
    }
    struct addrinfo* result = NULL;
    struct addrinfo hints;

    ZeroMemory(&hints, sizeof(hints));
    hints.ai_family = AF_INET;
    hints.ai_socktype = SOCK_STREAM;
    hints.ai_protocol = IPPROTO_TCP;
    hints.ai_flags = AI_PASSIVE;

    std::cout << "> INFO: Resolving server address and port" << std::endl;
    // Resolve the server address and port
    err = getaddrinfo(NULL, port.c_str(), &hints, &result);
    if (err != 0) {
      std::cout << "ERROR: getaddrinfo failed with error: " << err << std::endl;
      WSACleanup();
      return err;
    }

    std::cout << "> INFO: Creating socket for connection to server" << std::endl;

    // Create a SOCKET for connecting to server
    SOCKET listen_socket = INVALID_SOCKET;
    listen_socket = ::socket(result->ai_family, result->ai_socktype, result->ai_protocol);
    if (listen_socket == INVALID_SOCKET) {
      std::cout << "ERROR: Socker failed with error: " << WSAGetLastError() << std::endl;
      freeaddrinfo(result);
      WSACleanup();
      return 1;
    }

    std::cout << "> INFO: Setting up TCP listening socket" << std::endl;
    // Setup the TCP listening socket
    err = ::bind(listen_socket, result->ai_addr, (int)result->ai_addrlen);
    if (err == SOCKET_ERROR) {
      std::cout << "ERROR: Bind failed with error: " << WSAGetLastError() << std::endl;
      freeaddrinfo(result);
      closesocket(listen_socket);
      WSACleanup();
      return 1;
    }

    freeaddrinfo(result);

    // Wait for a client to conenct to the socket.
    err = listen(listen_socket, SOMAXCONN);
    if (err == SOCKET_ERROR) {
      std::cout << "ERROR: Listen failed with error: " << WSAGetLastError() << std::endl;
      closesocket(listen_socket);
      WSACleanup();
      return err;
    }


    std::cout << "> INFO: Accepting client socket" << std::endl;
    // Accept a client socket
    tcp_client_socket_ = ::accept(listen_socket, NULL, NULL);
    if (tcp_client_socket_ == INVALID_SOCKET) {
      std::cout << "ERROR: Accepting client failed with error: " << WSAGetLastError() << std::endl;
      closesocket(listen_socket);
      WSACleanup();
      return 1;
    }

    // No longer need server listen socket
    closesocket(listen_socket);

    // Create a FD_SET to check for new messages in the backgroundHandler.
    FD_SET(tcp_client_socket_, &tcp_client_fd_set_);


    std::cout << "> INFO: TCP connection successfully established" << std::endl;

    return 0;
  }

  void TCPConnection::start() {
    // Start the background thread to handle incoming and outgoing messages.
    state_ = RUNNING;
    tcp_background_thread_ = std::make_shared<std::thread>(&TCPConnection::tcpBackgroundHandler, this);
  }

  void TCPConnection::stop()
  {
    state_ = STOPPED;
  }

  void TCPConnection::tcpBackgroundHandler()
  {

    // Wait for the connection to become alive
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    int error_code = 0;
    while (state_ == RUNNING) {
      // First send all data from the queue
      std::unique_ptr<TCPMessage> msg = outgoing_queue_->dequeue();
      while (msg) {
        error_code = sendMessage(std::move(msg));
        if (error_code <= 0) {
          // Error occured while sending message, break the loop and end the thread.
          break;
        }
        msg = std::move(outgoing_queue_->dequeue());
      }
      // Check if new data is on the TCP connection to receive
      timeval tv{ 0, 100000 };
      int rc = 0;
      FD_ZERO(&tcp_client_fd_set_);
      FD_SET(tcp_client_socket_, &tcp_client_fd_set_);
      rc = ::select(0, &tcp_client_fd_set_, NULL, NULL, &tv);
      if (rc == 0) {
        // No messages on the socket, continue the loop.
        continue;
      }
      if (rc == SOCKET_ERROR) {
        // Something went wrong when receiving the message, break the handler, end the thread.
        std::cout << "select() == SOCKET_ERROR error: " << WSAGetLastError() << std::endl;
        break;
      }
      auto in_msg = receiveMessage();
      if (!in_msg) {
        // Something went wrong when receiving the message, break the handler, end the thread.
        break;
      }

      // Add it to the queue, let the main thread handle them
      incoming_queue_->enqueue(std::move(in_msg));
    }
    // Clear all entries of the queues before shutting down.
    incoming_queue_->clear();
    outgoing_queue_->clear();

    // Close the socket
    if (tcp_client_socket_ != INVALID_SOCKET) {
      int err = shutdown(tcp_client_socket_, SD_SEND);
      if (err != 0) {
        std::cout << "ERROR: Shutdown of Client Socket failed with error: " << err << std::endl;
      }
      closesocket(tcp_client_socket_);
      WSACleanup();
    }
    tcp_client_socket_ = INVALID_SOCKET;
  }

  std::unique_ptr<TCPMessage> TCPConnection::receiveMessage()
  {
    // Number of bytes received
    int received_bytes = 0;

    // Length of read bytes in total (used to ensure split messages are read correctly)
    uint64_t len_res = 0;

    // First read the length of the message to expect (8 byte uint64_t)
    std::string tcp_msg_len_recv_buf_;
    // To ensure split message is read correctly
    while (len_res < msg_length_buf_size_) {
      received_bytes = recv(tcp_client_socket_, &(tcp_msg_len_recv_buf_[len_res]), msg_length_buf_size_ - len_res, 0);
      if (received_bytes > 0) {
        // All good
        len_res += received_bytes;
      }
      else if (received_bytes == 0) {
        // Client closed the connection
        len_res = -1;
        std::cout << "Connection closing..." << std::endl;
        return NULL;
      }
      else {
        // Error occured during receiving
        std::cout << "recv failed during recv of data length with error: " << WSAGetLastError() << std::endl;
        closesocket(tcp_client_socket_);
        WSACleanup();
        return NULL;
      }
    }

    // Convert the read bytes to uint64_t. Little Endian! Otherwise invert for loop - not implemented, yet.
    uint64_t msg_len = 0;
    for (int i = msg_length_buf_size_ - 1; i >= 0; --i)
    {
      msg_len <<= 8;
      msg_len |= (unsigned char)tcp_msg_len_recv_buf_[i];
    }

    // Reset read bytes and total read bytes to 0
    received_bytes = 0;
    len_res = 0;
    // Read as many packages as needed to fill the message buffer. Ensures split messages are received correctly.
    char* buf = new char[msg_len]();
    while (len_res < msg_len) {
      received_bytes = recv(tcp_client_socket_, &buf[len_res], msg_len - len_res, 0);
      if (received_bytes > 0) {
        len_res += received_bytes;
      }
      else if (received_bytes == 0) {
        return NULL;
      }
      else {
        std::cout << "recv failed during recv of data message with error: " << WSAGetLastError() << std::endl;
        closesocket(tcp_client_socket_);
        WSACleanup();
        return NULL;
      }
    }

    // Parse the byte-stream into a TCPMessage
    std::unique_ptr<TCPMessage> msg = std::make_unique<TCPMessage>();
    if (!msg->ParseFromArray(buf, msg_len)) {
      std::cout << "ERROR: Parsing Message from String failed" << std::endl;
      return NULL;
    }

    delete buf;

    return msg;
  }

  int TCPConnection::sendMessage(std::unique_ptr<TCPMessage> msg)
  {
    // Serialize the TCPMessage
    std::string out;
    out = msg->SerializeAsString();

    // First put the length of the message in the first 8 bytes of the output stream
    std::string out_buffer = "";
    for (int i = 0; i < 8; ++i) {
      out_buffer += unsigned char((int)(((uint64_t)out.size() >> (i * 8)) & 0xFF));
    }

    // Attach the serialized message to the byte-stream
    out_buffer += out;

    // Send message length + message through the socket.
    int i_send_result = ::send(tcp_client_socket_, &out_buffer[0], out_buffer.size(), 0);
    if (i_send_result == SOCKET_ERROR) {
      std::cout << "SendMessage failed with error: " << WSAGetLastError() << std::endl;
      closesocket(tcp_client_socket_);
      WSACleanup();
    }

    return i_send_result;
  }

} // namespace tcp_io_device

#endif