# Replicode

Replicode is the programing language of AERA (Auto-catalytic Endogenous Reflective Architecture)

This README file documents general setup instructions.

## General Prerequisites

- Visual Studio 2017 (Community works fine)

## Preparation

Check the code out from the IIIM github:

`git clone --recursive https://github.com/IIIM-IS/replicode`

(We use `--recursive` to include the CoreLibrary submodule.)

## Building

Open the solution file at `replicode/Replicode.sln` in Visual Studio 2017 and select the `Debug Win32` or `Release Win32` configuration.

## Running

Set the *Test* project as the Start-Up project and add the following to the project's *Command Arguments* (found in Properties > Debugging):

`settings.xml decompiled.txt`

To change the settings, edit `Test/settings.xml` then re-run.

## code structure
The main program execution for the example happens in Test/Test.cpp
- `main()` // loads the `settings.xml` and the seed image, starts up `r_exec/mem.cpp` and then waits for run_time and ends by saving the results to disk to `decompiled.txt`
- `r_exec/mem.cpp` starts up the `runReductionCore` and the `runTimeCore` threads which do the magic
- The `r_exec/pattern_extractor.cpp` and `r_exec/g_monitor.cpp` push new reduction and time jobs to the job queues
- The `r_exec/*_controller.cpp` are the controllers for mdl, pgm and more
- The commands (`cmd`) activated in the replicode scripts would be activated through `r_exec/mem.cpp::eject(Code *command)`, see https://github.com/IIIM-IS/Integration_OpenAI for actual device commands

## Gotchas

This is a list of unintuitive issues with using Replicode, to help new users.

* Only the `Test/v1.2` examples really function due to syntax changes.
* The term "set" is used even when it means an ordered array with repeated elements.
* The term "hlp" just means "mdl" or "cst", which use similar structures for "high-level pattern".
* Every object definition ends with a propagation of saliency threshold (psln_thr), usually 1.
  For brevity, the documentation often omits it. And there is a macro in std.replicode which allows it to be
  omitted from the definitions in there. But it must be included when you write code.
* "icmd" means "internal command", not "instantiated command" (like "imdl" for "instantiated model").
* imdl set of exposed values doesn't have LHS fact timestamps. See: HLPBindingMap::build_f_ihlp:1037
* If a model's guard assigns a variable, the value passed in with the same template variable is ignored.
* The timestamps of facts defined at the top level automatically have the start time added (and subtracted in `decompiled_objects.txt`).
* min_sim_time_horizon is really "time allowance for the simulated predictions".
* Need CSTController::inject_goal:461 new View resilience 9999 to keep goal commands until used.
* OIDs are not in order of creation. (Only order of binding to a view.)
* Program "guards" which check boolean constraints are different than model "guards" which assign values to variables.
* `pong.2.simplified.replicode`: Prediction is made after the actual fact is injected. (But the prediction doesn't look at it.)
* For external cmd, need a program to intercept (fact (goal (fact (cmd ...)))) and eject the cmd.
  Such a goal abduced from a mdl is put in primary, so the program must be instantiated in primary.
  [Maybe not in primary. Need to understand how simulated goals are committed, and to which group.]
* Prediction failures may come from false assumptions. In `settings.xml`, try `enable_assumptions="no"` .
* Objects with a short resilience aren't in `decompiled_objects.txt`. In `settings.xml`, try `keep_invalidated_objects=yes` .
