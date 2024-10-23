cmake_minimum_required(VERSION 3.22)
project(aera
  VERSION 1.0
  DESCRIPTION "aera"
  LANGUAGES CXX)

include_directories(AERA AERA/IODevices/TCP AERA/IODevices/TCP/AERA_Protobuf
  submodules/CoreLibrary/CoreLibrary r_code r_comp r_exec
  usr_operators usr_operators/Operators usr_operators/Callbacks)
add_compile_definitions(EXECUTIVE_EXPORTS CORELIBRARY_EXPORTS WITH_DETAIL_OID)
add_compile_options()
add_executable(aera
  AERA/AERA_main.cpp
  AERA/main.cpp
  AERA/test_mem.cpp
  AERA/IODevices/TCP/tcp_io_device.cpp
  AERA/IODevices/TCP/AERA_Protobuf/tcp_connection.cpp
  AERA/IODevices/TCP/AERA_Protobuf/tcp_data_message.pb.cc
  AERA/IODevices/video_screen/video_screen.cpp
  AERA/IODevices/video_screen/video_screen_io_device.cpp
  
  submodules/CoreLibrary/CoreLibrary/base.cpp
  submodules/CoreLibrary/CoreLibrary/utils.cpp
  submodules/CoreLibrary/CoreLibrary/xml_parser.cpp

  r_code/atom.cpp
  r_code/code_utils.cpp
  r_code/image.cpp
  r_code/image_impl.cpp
  r_code/object.cpp
  r_code/r_code.cpp

  r_comp/class.cpp
  r_comp/compiler.cpp
  r_comp/decompiler.cpp
  r_comp/preprocessor.cpp
  r_comp/r_comp.cpp
  r_comp/segments.cpp
  r_comp/structure_member.cpp

  r_exec/ast_controller.cpp
  r_exec/auto_focus.cpp
  r_exec/binding_map.cpp
  r_exec/callbacks.cpp
  r_exec/_context.cpp
  r_exec/context.cpp
  r_exec/cpp_programs.cpp
  r_exec/cst_controller.cpp
  r_exec/factory.cpp
  r_exec/g_monitor.cpp
  r_exec/group.cpp
  r_exec/guard_builder.cpp
  r_exec/hlp_context.cpp
  r_exec/hlp_controller.cpp
  r_exec/hlp_overlay.cpp
  r_exec/init.cpp
  r_exec/mdl_controller.cpp
  r_exec/mem.cpp
  r_exec/model_base.cpp
  r_exec/monitor.cpp
  r_exec/opcodes.cpp
  r_exec/operator.cpp
  r_exec/overlay.cpp
  r_exec/pattern_extractor.cpp
  r_exec/pgm_controller.cpp
  r_exec/pgm_overlay.cpp
  r_exec/p_monitor.cpp
  r_exec/reduction_core.cpp
  r_exec/reduction_job.cpp
  r_exec/time_core.cpp
  r_exec/time_job.cpp
  r_exec/view.cpp

  usr_operators/auto_focus_callback.cpp
  usr_operators/usr_operators.cpp
  usr_operators/Callbacks/callbacks.cpp
  usr_operators/Operators/operators.cpp
  usr_operators/TestProgram/test_program.cpp
)
set_target_properties(aera PROPERTIES COMPILE_OPTIONS "-m32" LINK_FLAGS "-m32")
target_link_libraries(aera stdc++fs)
