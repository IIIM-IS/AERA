# Replicode

Replicode is the programing language of AERA (Auto-catalytic Endogenous Reflective Architecture)

This README file documents general setup instructions.

## General Prerequisites

- Visual Studio 2017 (Community works fine)

### Side note

Save yourself a lot of trouble, and keep the code in `C:/IIIM/AERA/replicode`. 
There are quite a few places that use absolute paths, spare yourself the headache of hunting them all down.

## Building

Open the solution file at `Integration_OpenAI/Integration_OpenAI.sln` in Visual Studio 2017 and select the `Debug Win32` or `Release Win32` configuration.

## Running

Set the *Test* project as the Start-Up project and add the following to the project's *Command Arguments* (found in Properties > Debugging):

`settings.xml decompiled.txt`

To change the settings, edit `Test/settings.xml` then re-run.

## notes
Only the `Test/v1.2` examples really function due to syntax changes.

## code structure
The main program execution for the example happens in Test/Test.cpp
- `main()` // loads the `settings.xml` and the seed image, starts up `r_exec/mem.cpp` and then waits for run_time and ends by saving the results to disk to `decompiled.txt`
- `r_exec/mem.cpp` starts up the `runReductionCore` and the `runTimeCore` threads which do the magic
- The `r_exec/pattern_extractor.cpp` and `r_exec/g_monitor.cpp` push new reduction and time jobs to the job queues
- The `r_exec/*_controller.cpp` are the controllers for mdl, pgm and more
- The commands (`cmd`) activated in the replicode scripts would be activated through `r_exec/mem.cpp::eject(Code *command)`, see https://github.com/IIIM-IS/Integration_OpenAI for actual device commands