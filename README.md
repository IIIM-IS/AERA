# AERA

This project implements AERA (Auto-catalytic Endogenous Reflective Architecture) using the Replicode
programming language.

For an overview, please visit https://openaera.org . See our FAQ which answers many questions about AERA https://openaera.org/faq .

This README file documents general information about the AERA code.

See the file [INSTALL.md](https://github.com/IIIM-IS/AERA/blob/master/INSTALL.md) for build and install instructions.

See the file [CHANGELOG.md](https://github.com/IIIM-IS/AERA/blob/master/CHANGELOG.md) for version numbers and changes.

For code documentation, see https://iiim-is.github.io/AERA .

## Code structure
The main program execution for the example happens in AERA/main.cpp
- `main()` // loads the `settings.xml` and the seed image, starts up `r_exec/mem.cpp` and then waits for run_time and ends by saving the results to disk to `decompiled.txt`
- `r_exec/mem.cpp` starts up the `runReductionCore` and the `runTimeCore` threads which do the magic
- The `r_exec/pattern_extractor.cpp` and `r_exec/g_monitor.cpp` push new reduction and time jobs to the job queues
- The `r_exec/*_controller.cpp` are the controllers for mdl, pgm and more
- The commands (`cmd`) activated in the replicode scripts would be activated through `r_exec/mem.cpp::eject(Code *command)`, see https://github.com/IIIM-IS/Integration_OpenAI for actual device commands

## Gotchas
This is a list of unintuitive issues with using AERA, to help new users.

* Only the `AERA/replicode_v1.2` examples really function due to syntax changes.
* The term "set" is used even when it means an ordered array with repeated elements.
* The term "hlp" just means "mdl" or "cst", which use similar structures for "high-level pattern".
* Every object definition ends with a propagation of saliency threshold (psln_thr), usually 1.
  For brevity, the documentation often omits it. And there is a macro in std.replicode which allows it to be
  omitted from the definitions in there. But it must be included when you write code.
* "icmd" means "internal command", not "instantiated command" (like "imdl" for "instantiated model").
* imdl set of exposed values doesn't have LHS fact timestamps. See: HLPBindingMap::build_f_ihlp:1037
* If a model's guard assigns a variable, the value passed in with the same template variable is ignored.
* The timestamps of facts defined at the top level of a file automatically have the start time added for run time 
  (and subtracted after run time for `decompiled_objects.txt`).
* min_sim_time_horizon is really "time allowance for the simulated predictions".
* OIDs are not in order of creation. (Only order of binding to a view.)
* Program "guards" which check boolean constraints are different than model "guards" which assign values to variables.
* Even though a drive goal is to be accomplished in the next frame, make its time interval start in the current frame
  so that its potential sub-goals will match current facts (and not be injected as new goals).
* A program accumulates matched patterns and will run even if one of the facts is "old". You must use boolean guards to control this.
* `pong.2.simplified.replicode`: Prediction is made after the actual fact is injected. (But the prediction doesn't look at it,
  and the prediction check uses the output of auto focus which correctly doesn't appear until the next frame.)
* For external cmd, need a program to intercept (fact (goal (fact (cmd ...)))) and eject the cmd.
  Such a goal abduced from a mdl is put in primary, so the program must be instantiated in primary.
* Objects with a short resilience aren't in `decompiled_objects.txt`. In `settings.xml`, try `keep_invalidated_objects=yes` .
* If not debugging the code, then in Visual Studio run in the "Release" configuration so that it runs faster.
* Math operations on two floating point numbers produce a floating point number, as expected. And addition
  and subtraction of two time values produces a time value, as expected. But the result of operations with
  mixed types is not obvious. Below 'nb' is a floating-point number and 'ts' is a 64-bit timestamp in microseconds
  and 'us' is a 64-bit duration in microseconds.
  - (+ us ts) or (+ ts us) or (+ nb ts) or (+ ts nb) -> ts (interpret nb as microseconds duration)
  - (+ nb us) or (+ us nb) or (+ us us) -> us (interpret nb as microseconds duration)
  - (+ ts ts) not allowed
  - (- ts ts) -> us
  - (- ts nb) or (- ts us) -> ts (interpret nb as microseconds duration)
  - (- us nb) -> us
  - (- nb ts) or (- us ts) or (- nb us) not allowed
  - (* nb us) -> nb
  - (* us nb) -> us ... different than (* nb us)!
  - (* us us) -> nb ... counter-intuitive
  - (* nb ts) or (* ts nb) or (* us ts) or (* ts us) or (* ts ts) not allowed
  - (/ nb us) -> nb
  - (/ us nb) -> us
  - (/ us us) -> nb
  - (/ nb ts) or (/ ts nb) or (/ us ts) or (/ ts us) or (/ ts ts) not allowed

### Implementation status
Demo = demonstrated functionality. Vis = explanation in Visualizer. Comm = code commented. Doc = details in software design document

| Demo | Vis | Comm | Doc | Functionality
|:----:|:---:|:----:|:---:| -------------
|      |     |      |     | **_Facts / Success_**
| ✓    |     |      |     | Learn models in change targeted pattern extractor (CTPX)
|      |     |      |     | Learn models in goal targeted pattern extractor (GTPX) due to unexpected achievement of goal fact
| ✓    | ✓   |      |     | Non-simulated icst from facts/icsts
| ✓    | ✓   |      |     | Non-simulated prediction of imdl from weak requirement model
| ✓    |     |      |     | Rating of weak requirement models
| ✓    | ✓   |      |     | Testing and re-activation of models and weak requirement models in the secondary group
| ✓    |     |      |     | Use of deleted models to prevent re-creating them
| ✓    | ✓   |      |     | Non-simulated prediction of fact from model
| ✓    |     |      |     | Non-simulated cmd from non-simulated goal RHS
|      |     |      |     | Auto-focus based on active predictions and goals (not pass-through)
| ✓    |     |      |     | Views/groups and saliency propagation
| ✓    |     |      |     | Simulated prediction of icst from facts/icsts
| ✓    |     |      |     | Simulated prediction of imdl from weak requirement model
| ✓    |     |      |     | Simulated prediction of fact from model
| ✓    |     |      |     | Simulated goal from drive
| ✓    |     |      |     | Simulated goal cmd from model
| ✓    |     |      |     | Simulated goal requirement imdl from model
| ✓    |     |      |     | Simulated goal icst from requirement model
| ✓    |     |      |     | Simulated goal facts/icsts from icst
| ✓    |     |      |     | Commiting to optional solutions
|      |     |      |     | **_Anti-facts / Failure_**
| ✓    |     |      |     | Learn strong requirement models (predicted anti-imdl) in prediction targeted pattern extractor (PTPX) due to prediction failure
| ✓    |     |      |     | Learn models in goal targeted pattern extractor (GTPX) due to unexpected achievement of goal anti-fact
| ✓    |     |      |     | Non-simulated prediction of anti-fact from model
| ✓    |     |      |     | Non-simulated prediction of anti-imdl from strong requirement model (to prevent model prediction)
| ✓    |     |      |     | Rating of strong requirement models
|      |     |      |     | Testing and re-activation of strong requirement models in the secondary group
|      |     |      |     | Goal actions to test conflicting weak and strong requirements
|      |     |      |     | Simulated anti-goal from opposite match of drive (and starting of new simulation from anti-goal)
| ✓    |     |      |     | Simulated prediction of anti-imdl from strong requirement model (to prevent model prediction)
|      |     |      |     | Simulated prediction of anti-fact from model (how does this make the simulation proceed?)
|      |     |      |     | Conflict resolution of conflicting predictions (fact and anti-fact)
|      |     |      |     | Simulated anti-goal of cmd from model with anti-matched RHS
|      |     |      |     | Simulated anti-goal requirement imdl from model with anti-matched RHS
|      |     |      |     | Simulated anti-goal icst from requirement model with anti-matched RHS
|      |     |      |     | Simulated anti-goal facts/icsts from anti-fact icst
| ✓    |     |      |     | Commiting to mandatory solutions
