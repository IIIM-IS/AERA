# Installation and usage of Protobuf

## Installation of protobuf and protoc
Install protobuf using vcpkg.
### Install vcpkg:
Follow the instructions on the [official Microsoft website](https://docs.microsoft.com/en-us/cpp/build/install-vcpkg?view=msvc-160&tabs=windows) to install vcpkg. In short:

 1. Clone locally from [https://github.com/Microsoft/vcpkg](https://github.com/Microsoft/vcpkg). It is known to make problems, if there are spaces in the path to vcpkg. So I suggest installing it somewhere like `C:\Users\<username>\vcpkg\`. So for example:
 `C:\Users\Username> git clone https://github.com/Microsoft/vcpkg`
 2. Change to `vcpkg` directory:
  `C:\Users\Username> cd vcpkg`
  3. In the vcpkg root directory, run the vcpkg bootstrapper command:
  `C:\Users\Username\vcpkg> bootstrap-vcpkg.bat`

### Install protobuf using vcpkg:

#### Prerequisites:
In order to use protobuf in the debug version of AERA it needs to be compiled with `ITERATOR_DEBUG_LEVEL = 0`. Do do this you need to change the `x86-windows.cmake` triplets of vcpkg. **Make a backup of the original `x86-windows.cmake` before continuing**.
You can find the cmake-files in `C:\Users\<username>\vcpkg\triplets`. There open the `x86-windows.cmake` with an editor of choice (e.g. Notepad++) and add the following two lines to the end of the file:
```
set(VCPKG_C_FLAGS_DEBUG "/D_ITERATOR_DEBUG_LEVEL=0")
set(VCPKG_CXX_FLAGS_DEBUG "/D_ITERATOR_DEBUG_LEVEL=0")
```
The whole file should look like this:
```
set(VCPKG_TARGET_ARCHITECTURE x86)
set(VCPKG_CRT_LINKAGE dynamic)
set(VCPKG_LIBRARY_LINKAGE dynamic)
set(VCPKG_C_FLAGS_DEBUG "/D_ITERATOR_DEBUG_LEVEL=0")
set(VCPKG_CXX_FLAGS_DEBUG "/D_ITERATOR_DEBUG_LEVEL=0")
```
In order to use the installed protobuf version without including further linking the file name should stay the same (x86-windows.cmake).

#### Installation:
Go into the root directory of vcpkg with a terminal and run the following command:
```
C:\Users\<username>\vcpkg> vcpkg install protobuf protobuf:x86-windows
```
In order to use protobuf include files in Visual Studio without further linking you should additionally run
```
C:\Users\<username>\vcpkg> vcpkg integrate install
```
If it returns with an error message it probably is because of lacking administrator rights. For this open a new terminal as an administrator and rerun the previous command.

## Compile proto files:
**Not necessary for simply building and using the TCP IODevice, skip if not changing the `tcp_data_message.proto` file.**

If you ever change the proto file found in `...\replicode\AERA\IODevice\TCP\proto` you will have to recompile the proto file. For this use protoc which is included in the previously installed vcpkg protobuf installation. If protoc is not in your path run the following from the `...\vcpkg\installed\x86-windows\tools\protobuf\` folder:
```
protoc -I=C:\Path\to\replicode\AERA\IODevices\TCP\Proto --cpp_out=C:\Path\to\replicode\AERA\IODevices\TCP\Proto C:\Path\to\replicode\AERA\IODevices\TCP\Proto\tcp_data_message.proto
```
If protoc is in your path simply `cd` into the `Proto folder` and run:
```
C:\Path\to\replicode\AERA\IODevices\TCP\Proto> protoc -I=. --cpp_out=. tcp_data_message.proto
```

## Add created libraries to your project
Once everything is installed you need to add the created dlls to your project. For this simply copy-paste `libprotobufd.dll` and `libprotobuf-lited.dll` from `C:\Path\to\vcpkg\installed\x86-windows\debug\bin` to `C:\Path\to\replicode\Debug`. And if you have compiled the project as release additionally copy paste `libprotobuf.dll` and `libprotobuf-lite.dll` from `C:\Path\to\vcpkg\installed\x86-windows\bin` to `C:\Path\to\replicode\Release`. Now everything should be set up for running AERA with a TCP connection.

## Enable protobuf in AERA
If you want to use the tcp-connection with AERA you have to enable it by setting the ENABLE_PROTOBUF flag in the AERA project properties. For this right-click on the AERA project (in the solution view on the left), select Properties -> C/C++ -> Preprocessor. There in the first line (Preprocessor Definitions) add a new entry called ENABLE_PROTOBUF.