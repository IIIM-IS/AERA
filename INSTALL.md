Replicode
=========

These are instructions to build and run Replicode.

Prerequisites
=============

* Required: On Windows, Visual Studio
* Required: Git
* Required: The code repository from https://github.com/IIIM-IS/replicode

Following are the detailed steps for each platform to install the prerequisites.

## Windows
To install Visual Studio, download and install Visual Studio Community Edition from
https://visualstudio.microsoft.com/vs/community .
In the installer, under "Desktop development with C++", check "Windows 10 SDK (10.0.17134.0)" and
  "MSVC v141 - VS 2017 C++ build tools".

To install Git, download and install GitHub for Desktop from https://desktop.github.com .

To get the code repository, launch GitHub for Desktop and sign in to GitHub. In the File menu, 
click "Clone a Repository". Click the URL tab and enter `https://github.com/IIIM-IS/replicode` . 
It should be a recursive clone (which is the default).

Build
=====
Launch Visual Studio and open the project `Replicode.sln` from the cloned repository. E.g.:
  `C:\Users\Alice\Documents\GitHub\replicode\Replicode.sln`

## `WITH_DEBUG_OID`

To work with the AERA Visualizer, we must enable `WITH_DEBUG_OID` as follows. In the Solution Explorer,
open the section for `CoreLibrary`. Double-click `base.h` . Uncomment the define for `WITH_DEBUG_OID`, so that
the line is:

    #define WITH_DEBUG_OID // Enable get_debug_oid() in every object.

## Compile

On the Build menu, click Build Solution. (Don't worry about all the compiler warnings.)

