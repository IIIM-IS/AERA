AERA
====

These are instructions to build and run AERA.

Prerequisites
=============

* Required: The AERA code repository from https://github.com/IIIM-IS/AERA
* Required: Git
* Required (Windows): Visual Studio, Windows 10 SDK, MSVC v141
* Required (Linux): Visual Studio Code, build tools and dependencies (see below)

Following are the detailed steps for each platform to install the prerequisites.

## Windows
To install Visual Studio, download and install Visual Studio Community Edition 2022 from
https://visualstudio.microsoft.com/vs/community . (If you already have Visual Studio 2019 installed,
this also works. Visual Studio 2017 is no longer supported.)
In the installer, under "Desktop development with C++", check "Windows 10 SDK (10.018362.0)" and
  "MSVC v141 - VS 2017 C++ build tools".

To install Git, download and install GitHub for Desktop from https://desktop.github.com .

To get the AERA code repository, launch GitHub for Desktop and sign in to GitHub. In the File menu, 
click "Clone a Repository". Click the URL tab and enter `https://github.com/IIIM-IS/AERA` . 
It should be a recursive clone (which is the default).

## Linux
These instructions have been tested on Ubuntu 24.04.4 LTS (Noble) but can likely be adapted for other
distros.

1. Download Visual Studio Code from https://code.visualstudio.com/download and install as required. For example, if downloading the DEB file, it can be installed with `sudo apt install ./FILENAME.deb`.
2. Open Visual Studio Code to the extensions tab and install `C/C++ Extension Pack` by Microsoft.
3. Install build dependencies. If using apt, run the following:
```
sudo dpkg --add-architecture i386
sudo apt update
sudp apt install build-essential cmake gcc-multilib g++-multilib git libprotobuf-dev:i386 libprotobuf-dev protobuf-compiler
```
4. Clone this repository to your directory of choice with `git clone --recursive https://github.com/IIIM-IS/AERA` or `git clone --recursive --branch dev https://github.com/IIIM-IS/AERA` if working on the development branch.


Build
=====
Once AERA has been installed, you're ready for the first build. As before, separate instructions are provided for
Windows and Linux platforms.

## Windows
Launch Visual Studio and open the project `AERA.sln` from the cloned repository. E.g.:
`C:\Users\Alice\Documents\GitHub\replicode\AERA.sln` .
If a dialog box appears asking to retarget the Windows version, click cancel. 

### `WITH_DETAIL_OID`

To work with the AERA Visualizer, we must enable `WITH_DETAIL_OID` as follows. In the Visual Studio Solution Explorer,
open the section for `CoreLibrary`. Double-click `base.h` . Uncomment the define for `WITH_DETAIL_OID`, so that
the line is:

    #define WITH_DETAIL_OID // Enable get_detail_oid() in every object.

### Compile

In the Solution Configurations drop-down, make sure you select Release (unless you plan to debug AERA).

In the Solution Options drop-down, make sure you select Win32.

On the Build menu, click Build Solution. (Don't worry about all the compiler warnings.)

## Linux
Launch Visual Studio Code, go to `File > Open Folder`, and select the root folder of the cloned repository. Bring
up the command palette with `Ctrl`+`Shift`+`P` and select `CMake: Configure`. When prompted, choose
the `GCC 13.3.0 x86_64-linux-gnu` preset.

### A brief hack
At the moment, support for Linux is incomplete and a couple lines of code need to be commented out. In the Explorer tab,
open the folder `submodules/CoreLibrary/CoreLibrary`. Double-click `utils.cpp` and scroll down to around Line 595. Comment out
the `pthread_mutex_lock(&cs_);` statements on Lines 595 and 603; hit `Ctrl`+`S` to save. The section should now look like this:
```Cpp
void CriticalSection::enter() {
#if defined WINDOWS
  EnterCriticalSection(&cs_);
#elif defined LINUX
  //pthread_mutex_lock(&cs_);
#endif
}

void CriticalSection::leave() {
#if defined WINDOWS
  LeaveCriticalSection(&cs_);
#elif defined LINUX
  //pthread_mutex_unlock(&cs_);
#endif
}
```

### `WITH_DETAIL_OID`
While Linux support for the AERA Visualizer is still a work in progress, you can save AERA runs for analysis on another
computer. To work with the AERA Visualizer, we must enable `WITH_DETAIL_OID` as follows. In the Explorer tab, open the folder
`submodules/CoreLibrary/CoreLibrary`. Double-click `base.h`. Uncomment the define for `WITH_DETAIL_OID` and hit `Ctrl`+`S` to save.
The line should now look like this:
```
#define WITH_DETAIL_OID // Enable get_detail_oid() in every object.
```

### Compile
To compile AERA, bring the command palette back up with `Ctrl`+`Shift`+`P` and run `Cmake: Clean rebuild`. A partial build can
also be started using the `Build` button in the lower-left corner but this may lead to errors if you're not careful. In either
case, don't worry about the compiler warnings.


Run
===

## Windows
To run, in Visual Studio on the Debug menu, select "Run Without Debugging". When first installed, the default
is to run the program `main.replicode` which by default loads the program `hello.world.1.replicode`. 
The output window should show some text including:

    0s:50ms:0us: hello world 1

To exit, close the output windows.

## Linux
In Visual Studio Code, go to the "Run and Debug" tab. Press the green play button in the top left and observe the
output in the Terminal view at the bottom of the window. Note that it might take a while to download debug tools
the first time.


# settings.xml

The `settings.xml` file specifies the program to run, AERA meta parameters, and other options for running AERA.
To open `settings.xml`, in the Visual Studio Solution Explorer or the Visual Studio Code Explore tab, open the section
for `AERA`. Double click `settings.xml`.

This has many parameters which are documented at the bottom of the file. Following are some highlights.

## settings.xml source_file_name

The `source_file_name` parameter specifies the seed program to run. For example, a program to use with the AERA Visualizer
is "../AERA/replicode_v1.2/ball.external.replicode".

## settings.xml keep_invalidated_objects

To work with the AERA Visualizer, in the Debug section under Objects, set `keep_invalidated_objects` to "yes". (This is needed
so that the decompiled objects file contains all the objects that were created during the run, including temporary objects.)
