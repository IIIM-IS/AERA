// File:          aera_hand_robot_controller.cpp
// Date:
// Description:
// Author:
// Modifications:

#define DEBUG_STRING_MESSAGE

#include <string>
#include <sstream>
#include <memory>
#include <cstdint>
#include <webots/Supervisor.hpp>
#include <webots/Motor.hpp>
#include <webots/PositionSensor.hpp>
#include <winsock.h>

using namespace std;
using namespace webots;

// Object at 0: (-0.3, -0.025, 0.01), {0, 0, 1, 1.50014}
// Object at 5: (-0.257, -0.161, 0.01), {0, 0, -1, 1.00257}
// Object at 10: (-0.154177, -0.264699, 0.01), {0, 0, -1, 0.50559}
// Object at 25: {0.246, -0.1775, 0.01}, {0, 0, 1, 0.999859}

typedef string TCPMessage;

static vector<string>
split(const string& input, char separator)
{
  vector<string> result;
  stringstream is(input);
  string str;
  while (getline(is, str, separator))
    result.push_back(str);

  return result;
}

static SOCKET open_socket(const char* host, int port) {
  struct sockaddr_in address;
  struct hostent *server;
  SOCKET fd;
  int rc;

#ifdef _WIN32
  // Initialize the socket API.
  WSADATA info;
  rc = WSAStartup(MAKEWORD(1, 1), &info); /* Winsock 1.1 */
  if (rc != 0) {
    cout << "ERROR: Cannot initialize Winsock" << endl;
    return INVALID_SOCKET;
  }
#endif

  // Create the socket.
  fd = socket(AF_INET, SOCK_STREAM, 0);
  if (fd == INVALID_SOCKET) {
    cout << "ERROR: Cannot create socket" << endl;
    return INVALID_SOCKET;
  }

  // Fill in the socket address.
  memset(&address, 0, sizeof(struct sockaddr_in));
  address.sin_family = AF_INET;
  address.sin_port = htons(port);
  server = gethostbyname(host);

  if (server)
    memcpy((char *)&address.sin_addr.s_addr, (char *)server->h_addr, server->h_length);
  else {
    cout << "ERROR: Cannot resolve server name: " << host << endl;
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
    return INVALID_SOCKET;
  }
  
  // Connect to the server.
  rc = connect(fd, (struct sockaddr *)&address, sizeof(struct sockaddr));
  if (rc == -1) {
    cout << "ERROR: Cannot connect to the server at " << host << ":" << port << endl;
#ifdef _WIN32
    closesocket(fd);
#else
    close(fd);
#endif
    return -1;
  }
  
  cout << "Connected to server at " << host << ":" << port << endl;
  return fd;
}

// Imitate TCPConnection::sendMessage.
static int sendMessage(SOCKET fd, const unique_ptr<TCPMessage>& msg)
{
  // Serialize the TCPMessage
  string out;
#ifdef DEBUG_STRING_MESSAGE
  out = *msg;
#else
  out = msg->SerializeAsString();
#endif

  // First put the length of the message in the first 8 bytes of the output stream
  string out_buffer = "";
  for (int i = 0; i < 8; ++i) {
    out_buffer += (unsigned char)((int)(((uint64_t)out.size() >> (i * 8)) & 0xFF));
  }

  // Attach the serialized message to the byte-stream
  out_buffer += out;

  // Send message length + message through the socket.
  int i_send_result = ::send(fd, &out_buffer[0], out_buffer.size(), 0);
  if (i_send_result == SOCKET_ERROR) {
    cout << "sendMessage failed" << endl;
  }

  return i_send_result;
}

// Imitate TCPConnection::receiveMessage.
static unique_ptr<TCPMessage> receiveMessage(SOCKET fd)
{
  // Number of bytes received
  int received_bytes = 0;

  // Length of read bytes in total (used to ensure split messages are read correctly)
  uint64_t len_res = 0;

  // First read the length of the message to expect (8 byte uint64_t)
  const int msg_length_buf_size = 8;
  string tcp_msg_len_recv_buf;
  tcp_msg_len_recv_buf.reserve(msg_length_buf_size);
  // To ensure split message is read correctly
  while (len_res < msg_length_buf_size) {
    received_bytes = ::recv(fd, &(tcp_msg_len_recv_buf[len_res]), msg_length_buf_size - len_res, 0);
    if (received_bytes > 0) {
      // All good
      len_res += received_bytes;
    }
    else if (received_bytes == 0) {
      // Client closed the connection
      len_res = -1;
      cout << "Connection closing..." << endl;
      return NULL;
    }
    else {
      // Error occured during receiving
      cout << "recv failed during recv of data length" << endl;
      return NULL;
    }
  }

  // Convert the read bytes to uint64_t. Little Endian! Otherwise invert for loop - not implemented, yet.
  uint64_t msg_len = 0;
  for (int i = msg_length_buf_size - 1; i >= 0; --i)
  {
    msg_len <<= 8;
    msg_len |= (unsigned char)tcp_msg_len_recv_buf[i];
  }

  // Reset read bytes and total read bytes to 0
  received_bytes = 0;
  len_res = 0;
  // Read as many packages as needed to fill the message buffer. Ensures split messages are received correctly.
  char* buf = new char[msg_len]();
  while (len_res < msg_len) {
    received_bytes = recv(fd, &buf[len_res], msg_len - len_res, 0);
    if (received_bytes > 0) {
      len_res += received_bytes;
    }
    else if (received_bytes == 0) {
      return NULL;
    }
    else {
      cout << "recv failed during recv of data message" << endl;
      return NULL;
    }
  }

  // Parse the byte-stream into a TCPMessage
  unique_ptr<TCPMessage> msg = make_unique<TCPMessage>();
#ifdef DEBUG_STRING_MESSAGE
  *msg = string(buf, msg_len);
#else
  if (!msg->ParseFromArray(buf, msg_len)) {
    cout << "ERROR: Parsing Message from String failed" << endl;
    return NULL;
  }
#endif

  delete[] buf;

  return msg;
}

// Check if new data is on the TCP connection to receive.
// Return 0 for no, 1 for yes, -1 for error.
static int receiveIsReady(SOCKET fd) {
  timeval tv{ 0, 0 };
  int rc = 0;
  FD_SET tcp_client_fd_set;
  FD_ZERO(&tcp_client_fd_set);
  FD_SET(fd, &tcp_client_fd_set);
  rc = ::select(fd + 1, &tcp_client_fd_set, NULL, NULL, &tv);
  if (rc == 0) {
    // No messages on the socket.
    return 0;
  }
  else if (rc == SOCKET_ERROR) {
    return -1;
  }
  
  return 1;
}

// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  SOCKET aera_fd = open_socket("127.0.0.1", 8080);
  if (aera_fd == INVALID_SOCKET)
    // Already printed the error.
    return -1;

  Supervisor* robot = new Supervisor();
  Node* sphere = robot->getFromDef("sphere");
  Node* cube = robot->getFromDef("cube");

  // Expect timeStep to be 1 ms.
  int timeStep = (int)robot->getBasicTimeStep();

  const double arm_up = 0.65;
  const double arm_down = 0.8;
  const double jaw_open = 0.01;
  const double jaw_closed = -0.0019;
  const double position_offset = -15.0;
  const double position_factor = 0.1;

  Motor* joint_1 = robot->getMotor("joint_1");
  // Decrease the PID gain from 10 so that we keep a grip on the object.
  joint_1->setControlPID(9, 0, 0);
  Motor* joint_2 = robot->getMotor("joint_2");
  Motor* joint_3 = robot->getMotor("joint_3");
  Motor* joint_5 = robot->getMotor("joint_5");
  Motor* joint_6 = robot->getMotor("joint_6");
  Motor* joint_base_to_jaw_1 = robot->getMotor("joint_base_to_jaw_1");
  Motor* joint_base_to_jaw_2 = robot->getMotor("joint_base_to_jaw_2");
  // Increase the strength of the grip.
  joint_base_to_jaw_1->setControlPID(250, 0, 0);
  joint_base_to_jaw_2->setControlPID(250, 0, 0);

  PositionSensor* joint_1_sensor = robot->getPositionSensor("joint_1_sensor");
  joint_1_sensor->enable(timeStep);
  PositionSensor* joint_base_to_jaw_1_sensor = robot->getPositionSensor("joint_base_to_jaw_1_sensor");
  joint_base_to_jaw_1_sensor->enable(timeStep);
  PositionSensor* joint_base_to_jaw_2_sensor = robot->getPositionSensor("joint_base_to_jaw_2_sensor");
  joint_base_to_jaw_2_sensor->enable(timeStep);

  joint_2->setPosition(arm_up);
  joint_3->setPosition(0.32);
  joint_5->setPosition(-0.5);
  joint_6->setPosition(M_PI/2);
  
  joint_base_to_jaw_1->setPosition(jaw_open);
  joint_base_to_jaw_2->setPosition(jaw_open);

  double debug_next_c_position = 10;
  double debug_next_s_position = 5;
  string debug_next_holding = "[]";

  int aera_us = -100;
  int receive_deadline = MAXINT;
  string command;
  double target_h_position = 0;
  int command_time = -1;
  while (robot->step(timeStep) != -1) {
    // aera_time moves at 1/10 the simulation speed.
    aera_us += timeStep * 100;

    if (aera_us == 0) {
      // Set initial condition. 
      command = "move";
      command_time = aera_us;
      target_h_position = 20;
      
      // Send the setup command.
      unique_ptr<TCPMessage> msg = make_unique<TCPMessage>("setup");
      sendMessage(aera_fd, std::move(msg));
    }
    if (aera_us == 1700*1000 + 65000) {
      // After grab failure, release and reset the positions of the sphere and cube.
      joint_1->setPosition((0 + position_offset) * position_factor);
      joint_6->setPosition(M_PI/2);
      joint_base_to_jaw_1->setPosition(jaw_open);
      joint_base_to_jaw_2->setPosition(jaw_open);
      sphere->resetPhysics();
      cube->resetPhysics();
      sphere->getField("rotation")->setSFRotation((const double[]){0, 0, 1, 1.50014});
      sphere->getField("translation")->setSFVec3f((const double[]){-0.3, -0.025, 0.01});
      cube->getField("rotation")->setSFRotation((const double[]){0, 0, -1, 1.00257});
      cube->getField("translation")->setSFVec3f((const double[]){-0.257, -0.161, 0.01});
      debug_next_s_position = 0;
      debug_next_c_position = 5;
    }

    double h_position = joint_1_sensor->getValue() / position_factor - position_offset;
    // Quantize.
    const double bin_size = 2.5;
    h_position = floor((h_position + bin_size/2) / bin_size) * bin_size;

    if (aera_us % 100000 == 0) {
      // Send the current state.
      {
        unique_ptr<TCPMessage> msg = make_unique<TCPMessage>(to_string(aera_us) + " h position " +
          to_string(h_position));
        sendMessage(aera_fd, std::move(msg));
      }
      {
        unique_ptr<TCPMessage> msg = make_unique<TCPMessage>(to_string(aera_us) + " c position " +
          to_string(debug_next_c_position));
        sendMessage(aera_fd, std::move(msg));
      }
      {
        unique_ptr<TCPMessage> msg = make_unique<TCPMessage>(to_string(aera_us) + " s position " +
          to_string(debug_next_s_position));
        sendMessage(aera_fd, std::move(msg));
      }
      {
        unique_ptr<TCPMessage> msg = make_unique<TCPMessage>(to_string(aera_us) + " h holding " +
          debug_next_holding);
        sendMessage(aera_fd, std::move(msg));
      }
      {
        unique_ptr<TCPMessage> msg = make_unique<TCPMessage>(to_string(aera_us) + " end_values");
        sendMessage(aera_fd, std::move(msg));
      }
      
      if (aera_us >= 100000)
        receive_deadline = aera_us + 65000;
    }
    
    if (aera_us >= receive_deadline) {
      // We haven't received the command yet, so wait for it.
      while (receiveIsReady(aera_fd) == 0) {}
      receive_deadline = MAXINT;
    }

    int ready = receiveIsReady(aera_fd);
    if (ready < 0) {
      cout << "select() == SOCKET_ERROR error" << endl;
      break;
    }
    if (command == "" && ready == 1) {
      receive_deadline = MAXINT;

      auto in_msg = receiveMessage(aera_fd);
      cout << "Debug: " << aera_us << " received \"" << *in_msg << "\"" << endl;
      if (!in_msg)
        // Already printed the error.
        break;

      vector<string> fields = split(*in_msg, ' ');
      if (fields.size() == 2 && (fields[0] == "grab" || fields[0] == "release") && fields[1] == "h") {
        command = fields[0];
      }
      else if (fields.size() == 3 && fields[0] == "move" && fields[1] == "h") {
        command = fields[0];
        double delta = std::stof(fields[2]);
        if (delta > 20)
          delta = 20;
        else if (delta < -20)
          delta = -20;
        target_h_position = h_position + delta;
      }
      else
        cout << "ERROR: Unrecognized command " << *in_msg << endl;
          
      if (command != "") {
        // TODO: Get the command time from the message.
        int frame_start_us = (aera_us / 100000) * 100000;
        command_time = frame_start_us + 65000;
        if (command_time < aera_us)
          // We don't expect this.
          command_time = aera_us;
      }
    }

    if (aera_us == 300*1000 + 65000)
      debug_next_holding = "s";
    if (aera_us == 400*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 500*1000 + 65000)
      debug_next_holding = "s";
    if (aera_us == 600*1000 + 65000)
      debug_next_s_position = target_h_position;
    if (aera_us == 700*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 900*1000 + 65000)
      debug_next_holding = "c";
    if (aera_us == 1000*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 1100*1000 + 65000)
      debug_next_holding = "c";
    if (aera_us == 1200*1000 + 65000)
      debug_next_c_position = target_h_position;
    if (aera_us == 1300*1000 + 65000)
      debug_next_c_position = target_h_position;
    if (aera_us == 1400*1000 + 65000)
      debug_next_c_position = target_h_position;
    if (aera_us == 1500*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 1800*1000 + 65000)
      debug_next_holding = "s";
    if (aera_us == 1900*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 2000*1000 + 65000)
      debug_next_holding = "s";
    if (aera_us == 2100*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 2300*1000 + 65000)
      debug_next_holding = "c";
    if (aera_us == 2400*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 2500*1000 + 65000)
      debug_next_holding = "c";
    if (aera_us == 2600*1000 + 65000)
      debug_next_c_position = target_h_position;
    if (aera_us == 2700*1000 + 65000)
      debug_next_holding = "[]";
    if (aera_us == 3000*1000 + 65000)
      debug_next_holding = "s";
    if (aera_us == 3100*1000 + 65000)
      debug_next_s_position = target_h_position;
      
    if (command_time == aera_us) {
      // Execute the command.
      if (command == "grab") {
        joint_2->setPosition(arm_down);
        command = "close_jaw_and_move_up";
        command_time = command_time + 30000;
      }
      else if (command == "close_jaw_and_move_up") {
        joint_base_to_jaw_1->setPosition(jaw_closed);
        joint_base_to_jaw_2->setPosition(jaw_closed);
        command = "move_arm_up";
        command_time = command_time + 4900;
      }
      else if (command == "move_arm_up") {
        joint_2->setPosition(arm_up);
        command = "";
      }
      else if (command == "release") {
        if (aera_us == 1500*1000 + 65000) {
          // Special case: Drop the cube in the same position as the sphere.
          joint_1->setPosition((-0.3 + position_offset) * position_factor);
          joint_2->setPosition(0.73);
          joint_6->setPosition(1);
        }
        else
          joint_2->setPosition(arm_down);
        command = "open_jaw_and_move_up";
        command_time = command_time + 30000;
      }
      else if (command == "open_jaw_and_move_up") {
        joint_base_to_jaw_1->setPosition(jaw_open);
        joint_base_to_jaw_2->setPosition(jaw_open);
        command = "move_arm_up";
        command_time = command_time + 4900;
      }
      else if (command == "move") {
        // Do the inverse calculation of h_position;
        joint_1->setPosition((target_h_position + position_offset) * position_factor);
        command = "";
      }
    }
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
