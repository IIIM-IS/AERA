AERA
====

These are instructions to build and run AERA.

Prerequisites
=============

* Required: On Windows, Visual Studio
* Required: Git
* Required: The AERA code repository from https://github.com/IIIM-IS/AERA

Following are the detailed steps for each platform to install the prerequisites.

## Windows
To install Visual Studio 2019, download and install Visual Studio Community Edition from
https://visualstudio.microsoft.com/vs/community .
In the installer, under "Desktop development with C++", check "Windows 10 SDK (10.0.17134.0)" and
  "MSVC v141 - VS 2017 C++ build tools".

To install Git, download and install GitHub for Desktop from https://desktop.github.com .

To get the AERA code repository, launch GitHub for Desktop and sign in to GitHub. In the File menu, 
click "Clone a Repository". Click the URL tab and enter `https://github.com/IIIM-IS/AERA` . 
It should be a recursive clone (which is the default).

Build
=====
Launch Visual Studio and open the project `AERA.sln` from the cloned repository. E.g.:
`C:\Users\Alice\Documents\GitHub\replicode\AERA.sln` .
If a dialog box appears asking to retarget the Windows version, click cancel. 

## `WITH_DETAIL_OID`

To work with the AERA Visualizer, we must enable `WITH_DETAIL_OID` as follows. In the Visual Studio Solution Explorer,
open the section for `CoreLibrary`. Double-click `base.h` . Uncomment the define for `WITH_DETAIL_OID`, so that
the line is:

    #define WITH_DETAIL_OID // Enable get_detail_oid() in every object.

## Compile

In the Solution Configurations drop-down, make sure you select Release (unless you plan to debug AERA).
On the Build menu, click Build Solution. (Don't worry about all the compiler warnings.)

Run
===

To run, in Visual Studio on the Debug menu, select "Run Without Debugging". When first installed, the default
is to run the program `main.replicode` which by default loads the program `hello.world.1.replicode`. 
The output window should show some text including:

    0s:50ms:0us: hello world 1

To exit, close the output windows.

# settings.xml

The `settings.xml` file specifies the program to run, AERA meta parameters, and other options for running AERA.
To open `settings.xml`, in the Visual Studio Solution Explorer, open the section for `AERA`. Double click `settings.xml`

This has many parameters which are documented at the bottom of the file. Following are some highlights.

## settings.xml source_file_name

The `source_file_name` parameter specifies the seed program to run. For example, a program to use with the AERA Visualizer
is "../AERA/replicode_v1.2/ball.external.replicode".

## settings.xml keep_invalidated_objects

To work with the AERA Visualizer, in the Debug section under Objects, set `keep_invalidated_objects` to "yes". (This is needed
so that the decompiled objects file contains all the objects that were created during the run, including temporary objects.)
