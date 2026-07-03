# AERA Test Documentation


This report documents the test cases created for each intelligence requirement. For every testable requirement, a table lists the associated test cases, with a name, a brief description, and any remarks (e.g., found bugs or inconsistencies in the documentation).

Each test consists of a paired `.replicode` and `.txt` file. The `.replicode` file serves as the seed for the AERA run, and the `.txt` file contains the expected output, which is validated against the decompiled results of that run.

Each `.replicode` file also contains a header documenting the agent's available actions, how it differs from its base task, and any deviations from the `settings.xml` parameter defaults.

## 1. Embodiment

An embodiment (i.e., sensing and acting) is required to find out what the current status of the world is by generating models of it, with respect to one's goals.
This is also a requirement for **4 - Knowledge of causes**.

### 1.1 Sensing
i.e., perceiving and making measurements of the physical world through sensors.  
In AERA, inputs coming from the environment (technically, data produced by the I/O devices) come in the form of facts for which the time interval is reduced to a single point in time (the time the input was produced) and for which the confidence value is one (since the fact represents an observation in the environment).   
This section includes tests for Goals and Predictions, which are both pointers to Facts.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| facts | tests basic fact syntax | (R1) The parameters `after` and `before` use timestamps, not microseconds. |
| antifacts | tests basic anti fact syntax | (R1) The parameters `after` and `before` use timestamps, not microseconds. |
| goals | tests basic goal syntax | (R1) The structure of the goal class has changed. |
| predictions | tests basic prediction syntax | (R1) The structure of the prediction class has changed. |
| ball-external | tests if inputs coming from the (mocked) environment are handled correctly |  (R1) Since we are in a unit test environment, we use the existing mocked I/O device "ball". |

### 1.2 Acting
i.e., acting upon the physical world to achieve goals.  
AERA acts upon the physical world by calling commands on devices.   
There are two types of devices: internal (the executive, e.g.: inj, eje, mod, set, ...) and external (using custom commands).

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| inj | tests the icmd _inj | (R1) In decompiled_objects, the name of the injected object does not match what is provided.<br>(R2) If the object is injected to group "stdin", the object has no view in the decompilation file. |
| eje | tests the icmd _eje | (R1) This has never been used in existing Replicode files, and I wasn't able to reproduce all that the icmd should be able to do. |
| mod | tests the icmd _mod | - |
| set | tests the icmd _set | (R1) The documentation states that -1 represents an infinite resilience, but this was changed to the keyword "forever". |
| new-class | tests the icmd _new_class | (R1) This has never been used in existing Replicode files, and I wasn't able to reproduce all that the icmd should be able to do. |
| del-class | tests the icmd _del_class | (R1) This has never been used in existing Replicode files, and I wasn't able to reproduce all that the icmd should be able to do. |
| ldc | tests the icmd _ldc | (R1) This has never been used in existing Replicode files, and I wasn't able to reproduce all that the icmd should be able to do. |
| swp | tests the icmd _swp | (R1) This has never been used in existing Replicode files, and I wasn't able to reproduce all that the icmd should be able to do.<br>(R2) Documentation states that only values in {0, 1} should be allowed as a parameter, but this is not the case. |
| prb | tests the icmd _prb | (R1) This only ever been used in combination with "print"; which works as expected. |
| stop | tests the icmd _stop | - |
| cmd | tests basic command syntax | (R1) The parameter device:did (as specified in the documentation) does not exist anymore. |
| bathtub | tests the use of external commands to achieve a simple goal | - |

## 2. Constructivism
Required so that a system can autonomously amass the vast majority of its knowledge (except for a small seed), in a way that infuses meaning into information structures. "Autonomous" means that this can be done without interference / help from others / the need to “call home”.

**Note:** This is so fundamental to AERA that it is achieved per design.
All other tests created as part of this work capture the following properties to some extent.
Therefore, Requirement 2 contains no tests.

### 2.1 Generate knowledge autonomously

AERA acquires the vast majority of its knowledge on its own (except for a small seed).

### 2.2 Manage knowledge autonomously 

AERA manages its own growth on its own.

### 2.3 Use knowledge autonomously
AERA may change its own architecture over time, due to experience and learning. 

## 3. Cumulative learning
Required to unify new information and knowledge that is already acquired, in a coherent, efficient and effective manner. Unifies several separate research tracks.  
**Note:** This section could be expanded to include more detailed tests on the Activation, Reaction and Attention Control mechanisms.

### 3.1 Multitask learning
The ability to learn more than one task, either at once or in sequence.  
AERA achieves this through reactive programs and models, which are triggered whenever there is a left-side match and related saliency/activation values are high enough.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| programs | tests basic program syntax | - |
| models | tests basic model syntax | - |
| mediation-mod | tests the mediation of `mod` commands | -|
| mediation-set | tests the mediation of `set` commands | (R1) Setting and averaging works, but illegal values are rounded to 0 or 1. |
| bathtub-hgs-sequence | runs the bathtub and hand-grab-sphere tasks in sequence | - |
| bathtub-hgs-parallel | runs the bathtub and hand-grab-sphere tasks in parallel | (R1) Enabling model M7 causes AERA to run indefinitely. |

### 3.2 Online learning
The ability to learn continuously, uninterrupted, and in real-time from experience as it comes, and without specifically iterating over it many times.  
This section also covers few-shot learning, which means to learn something from very few examples or very little data.  
AERA achieves this by using new knowledge whenever it comes along.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-new-knowledge | tests if the system can use new knowledge whenever it comes along | - |


### 3.3 Lifelong learning
The ability of an AI system to keep learning and integrating knowledge throughout its operational lifetime: learning is “always on”.  
AERA can use new knowledge (or react to changes in saliency/activation) even while/after achieving a more complex goal like hand-grab-sphere.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-act-change | tests the system's reaction to a control value change | - |
| ball-external-new-models | tests model creation from facts observed by an external device during a task | - |


### 3.4 Robust knowledge acquisition
The antithesis of which is brittle learning, where new knowledge results in catastrophic perturbations of prior knowledge (and behavior).  
AERA achieves this through control values (saliency, activation, resilience, model success rate) that control which knowledge is more reliable than others.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-success-rate-increase | tests model success rate update after correct predictions | - |
| bathtub-success-rate-decrease | tests model success rate update after failed predictions | - |
| bathtub-success-rate-reliable | tests model success rate update on reliable models | - |


### 3.5 Transfer learning
The ability to build new knowledge on top of old in a way that the old knowledge facilitates learning the new. While interference/forgetting should not occur, knowledge should still be defeasible.  
In AERA, models exist separately from tasks, so any learned model can be used for any task. 

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| sink | tests the models of the bathtub task on a different but similar task | - |

## 4. Knowledge of causes

Required to guide action. Without knowledge about causal relations it is impossible to get anything systematically done.  
The only known method for creating causal models is combined forward-backward chaining.

### 4.1 Forward chaining
Required to produce predictions. 
Uses matching to produce what might happen next, after a particular state is reached, starting with existing data, until an endpoint (typically a goal) is reached.  
AERA achieves this through concatenated deduction (see section 5.3).

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-forward-chaining | tests the first part of planning: predictions | - |


### 4.2 Backward chaining

Required to produce plans. 
Starts with a given goal or state to be achieved, and proceeds through matching to produce what could possibly have been the state just prior to that particular state.  
AERA achieves this through concatenated abduction (see section 5.4).

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-backward-chaining | tests the second part of planning: sub-goals | - |

## 5. Non-axiomatic reasoning

Required to operate on knowledge in worlds where the axioms are unknown, not guaranteed, and/or fallible. 

### 5.1 Defeasible reasoning

Reasoning where the system-produced output may be found to be incorrect, at any time, given additional data, reconsideration of background assumptions or discovery of logic errors.  
Models that have repeatedly made poor predictions are removed from AERA. Activation values are used to monitor the system, to first deactivate and then entirely remove models/programs.


| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-model-defeated | tests the system's behavior after a model's prediction is defeated  | - |
| bathtub-sim-defeated | checks output after a promoted simulation prediction is defeated  | (R1) This task is incomplete, but it suffices for the purpose here.  |

### 5.2 Pattern matching
Pattern matching is the main method that reasoning systems use in their operation.  
It is central in AERA, where all existing object classes can be matched.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| ptn | tests basic ptn syntax  | (R1) Patterns cannot exist on their own and are therefore tested as part of other objects. |
| bathtub-pattern-matching | tests various instances of pattern matching | - |

### 5.3 Deduction
Figuring out the implication of facts (or predicting what may come). Producing implications from premises.  
In Replicode, deduction is the production of predictions given an input fact (actual or predicted).

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-deduction | tests created predictions and their confidence values | - |


### 5.4 Abduction
Figuring out how things came to be the way they are (or how particular outcomes could be made to come about, or how particular outcomes could be prevented).  
In Replicode, abduction refers to the production of sub-goals, given an input goal.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-abduction | tests created sub-goals and their confidence values | - |

### 5.5 Induction
Figuring out the general case. Making general rules from a (small) set of examples.   
Replicode offers three ways of acquiring models/states, by means of three different
components, called Targeted Pattern Extractors (TPX): Goal TPX, Prediction TPX, Change TPX.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| hgs-GTPX | tests the GTPX mechanism | - |
| ball-PTPX | tests the PTPX mechanism | (R1) The PTPX creates several duplicate composite states and models. |
| ball-CTPX | tests the CTPX mechanism | (R1) The induced model should eventually get added to the blacklist, but I could not verify this behaviour. <br> (R2) v7\*100ms seems to perform (* ts nb), which according to the list of Gotchas shouldn't be allowed.  |

### 5.6 Analogy
Figuring out how things are similar or different. Making inferences about how something X may be (or is) through a comparison to something else Y, where X and Y share some observed properties.

**Note:** The analogy feature, developed [here](https://github.com/IIIM-IS/AERA/commit/2173fc4a3871a4497494248bf9d923bad3e87073), is not merged to the main branch, and thus not tested here.


## 6. Generality (or Domain Independence)

Required as all requirements in this list must be independent of the subject of learning. Any system X that exceeds system Y on one or more of the points below is considered more general than system Y.

### 6.1 Flexibility
Breadth of task-environments: If a system X can operate in more diverse task-environments than system Y, system X is more flexible than system Y. 

**Note:** Similar to Requirement 2, this is achieved per design, as architectural principles do not make any assumption on the target domain.  
Testing this doesn't make sense, as there is no way this is not the case, unless AERA's design is fundamentally changed.
Therefore, Requirement 6.1 contains no tests.

### 6.2 Solution diversity
Breadth of solutions: If a system X can reliably generate a larger variation of acceptable solutions to problems than system Y, system X is more powerful than system Y. 

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| solution-diversity | tests if AERA can find multiple available plans to the goal state | (R1) This task leads to a "prediction explosion" at 0s:500ms:0us, which can be inspected in the Visualizer after loading for a while. |

### 6.3 Constraint diversity
Breadth of constraints on solutions: If a system X can reliably produce acceptable solutions under a higher number of solution constraints than system Y, system X is more powerful than system Y. 

**Note:** This is achieved per design, as any Replicode object can be a solution constraint for AERA to plan around. 
Testing this doesn't make sense, as there is no way this is not the case, unless AERA's design is fundamentally changed.
Therefore, Requirement 6.3 contains no tests.


### 6.4 Goal diversity
Breadth of goals: If a system X can meet a wider range of goals than system Y, system X is more powerful than system Y. Multiple goals should also be achieved concurrently.

**Note:** This is achieved per design, as any Replicode object can be a goal for AERA to plan towards. 
Testing this doesn't make sense, as there is no way this is not the case, unless AERA's design is fundamentally changed.
Therefore, Requirement 6.4 contains no tests.
But we refer to Requirement 3.1 - Multitask learning - and its tests for sequential and parallel goal achievement.

## 7 Transversal functions

The system must have pan-architectural characteristics that enable it to operate consistently as a whole, to be highly adaptive (yet robust) in its own operation across the board, including metacognitive abilities.

### 7.1 Transversal handling of time

Time is a semantic property, and the system must be able to understand – and be able to learn to understand – time as a real-world phenomenon in relation to its own skills and architectural operation.
In AERA, time is relevant to every object; completely ingrained in the system. At one point, it was changed from exact moments to a range.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-timestamps | checks that the bathtub task also succeeds with different timestamps.  | - |

### 7.2 Transversal handling of resources
Resource management (i.e., attention) involves management of knowledge, time, energy, and computing power.  
If AERA starts taking too long to process reductions, three measures can be taken (e.g., to raise group activation/saliency thresholds).

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-real-time | checks the bathtub task in real-time mode | (R1) If AERA does not complete planning in time, it does not commit to the deduction path it partially explored. |
| bathtub-perf | checks perf object created in real-time mode | - |

### 7.3 Preemptibility
The system has to be able to suspend its current operation in favor of more urgent tasks, and resume the former when possible, if it is still desired.  
Attention Control in AERA is implemented via the "auto-focus", which is recomputed every cognitive cycle. It is responsible for directing the system's attention with regards to its internal input.    
**Note:** Testing the specifics of the auto-focus mechanism would have to test C++ code directly, which is not our goal here.
Further, a blackbox test of the mechanism is blocked by the parallel task problem from section 3.1.
Therefore, the test below is an incomplete attempt to run the bathtub and hgs tasks in real-time mode, in parallel, with varying importance.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-hgs-preemptibility | checks that when a more important goal is injected while another is being pursued, the former is pursued. | (R1) AERA is unable to complete this task, as it cannot find a plan in real time.  |

## 8 Reflection

Required for a system to learn, over time, about its own fallacies, shortcomings, and lack of knowledge.

### 8.1 Meta-cognition
The ability of a system to generate its own programs using fixed design-time principles while deciding the specific details at runtime based on its experience.    
This is so central to AERA one could argue it achieves this per design. But we can still write tests for it. Notably, some important parts are Notifications (to evaluate itself) and using any reductions and knowledge about knowledge as input (to reason about itself)  
Except for the final test, this section consists of basic tests for different notifications.  

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| mk-rdx | checks the notification mk.rdx | - |
| mk-new | checks the notification mk.new | - |
| mk-low-sln | checks the notification mk.low_sln | - |
| mk-high-sln | checks the notification mk.high_sln | - |
| mk-low-act | checks the notification mk.low_act | - |
| mk-high-act | checks the notification mk.high_act | - |
| mk-low-res | checks the notification mk.low_res | - |
| mk-sln-chg | checks the notification mk.sln_chg | (R1) Expected results of this test are commented out due to inconsistent creation of mk.sln_chg objects |
| mk-act-chg | checks the notification mk.act_chg | (R1) Expected results of this test are commented out due to inconsistent creation of mk.act_chg objects |
| anything-as-input | checks that different objects (e.g.,  models, programs, act/sln/res notifications) can be used as input for models. | (R1) CSTs and ICSTs occasionally cause read-access violations. <br> (R2) Visualizer fails to parse icmd in models, even though this seems to work fine in the decompiled output.|

### 8.2 Self-programming

Self-programming is the creation of models through induction, therefore here we only refer to the tests in 5.5.
Furthermore, we refer to the basic syntax tests of models and programs in 3.1.  
**Note:** Therefore, this section contains no tests.

## 9 Robustness

When the system achieves consistently a set of goals within some constraints, it shall continue to do so despite novelty occurring in the environment, provided this novelty changes the initial goals and constraints only marginally and provided the system can possibly remain in its (fixed) operational boundaries.

### 9.1 Graceful degradation

Knowledge representation in AERA is based around causal relations, which are essential for mapping out “how the world works”. Because AERA's knowledge processing is organized around goals, with increased knowledge AERA will get closer and closer to “perfect operation” (i.e. meeting its top-level drives/goals, for which each instance was created). Furthermore, AERA can do reflection, so it gets better at evaluating its own performance over time, meaning it makes (causal) models of its own failure modes, increasing its chances of graceful degradation.

We therefore refer here to section 4 - causal relations, and section 8 - reflection. 
However, we include a test that checks AERA's output while failing to achieve 3 sequential goals.

| Test name | Description | Remarks |
| :--- | :--- | :--- |
| bathtub-hgs-ball-failure | checks the system's output after failing to achieve several goals. | |

### 9.2 Scalability
AERA identifies stable paths of subgoals/predictions. These are compacted for more efficiency. The original models are still saved, in case it is later necessary to reinject them after the compact structure proves instable.

**Note:** The compaction mechanism has only been described in theory, and is not implemented:   
"We have also left the Compaction mechanism for future work, and this mechanism has not been implemented in the prototype."    
Therefore, Requirement 9.2 contains no tests.

### 9.3 Predictability

The ability of an outsider to predict the behavior of a controller based on some information.

**Note:** This is too broad to write specific tests. But the purpose of all other tests is precisely this, to verify the behavior of AERA based on some information.
Therefore, Requirement 9.3 contains no tests.

### 9.4 Reliability

The ability of a machine to always return the same - or similar - answer to the same input. Requires predictability.

**Note:** This is too broad to write specific tests. But the purpose of all other tests is precisely this, to verify that AERA behaves - to a point - deterministically in my test cases.
Therefore, Requirement 9.4 contains no tests.

### 9.5 Trustworthiness

The ability of a machine's owner to trust that the machine will do what it is supposed to do. Requires predictability and reliability at multiple levels of operation.

**Note:** This is too broad to write specific tests. See 9.3 - Predictability and 9.4 - Reliability.
Therefore, Requirement 9.5 contains no tests.
