AERA Diagnostic Mode v0.2 (2022-07-01)
--------------------------------------

A release optimized to run in "diagnostic time" which is now the default in settings.xml .
This is a maintentance release from Diagnostic Mode v0.1 .

Bug fixes
* Replicode syntax: Allow ':' in a string. https://github.com/IIIM-IS/AERA/pull/207
* Fix bug abducing a goal imdl from a cst. https://github.com/IIIM-IS/AERA/pull/226
* Fix bug in assume_lhs so that we can enable assumptions. https://github.com/IIIM-IS/AERA/pull/219
* Allow switching to SIM_MANDATORY not at the sim root https://github.com/IIIM-IS/AERA/pull/214
* Other bug fixes. See https://github.com/IIIM-IS/AERA/pulls?q=is%3Apr+is%3Aclosed+label%3Abug

Changes
* BREAKING. Change the Replicode syntax to add durations, distinct from time stamps https://github.com/IIIM-IS/AERA/issues/138
* Move definition of std_grp to std.replicode https://github.com/IIIM-IS/AERA/pull/212
* Remove unused support for enable_assumptions in settings.xml https://github.com/IIIM-IS/AERA/pull/221
* Other enhancements. Please see https://github.com/IIIM-IS/AERA/blob/master/doc/meeting-notes.txt

AERA Diagnostic Mode v0.1 (2021-11-19)
--------------------------------------

A release optimized to run in "diagnostic time" which is now the default in settings.xml .

* Added example hand-grab-sphere.replicode where all models are coded in the seed.
* Added example hand-grab-sphere-learn.replicode which learns the grab and release models, as well
  as the anti-requirement model where grab fails if the sphere and cube are at the same position.
* Other bug fixes and enhancements. Please see https://github.com/IIIM-IS/AERA/blob/master/doc/meeting-notes.txt

replicode-syntax-v1.2 (2020-06-19)
----------------------------------

* Rename the folder Test to AERA.
  Note: To see changes before this renaming, browse at the earlier commit: 
  https://github.com/IIIM-IS/replicode/tree/6154f1f2d0eac5543d760928b89bff4fd30adad3
* Rename the file Test.cpp to main.cpp .
* Rename the Test project to AERA.
* Rename folders V1.1 and V1.2 to replicode_v1.1 and replicode_v1.2, to distinguish the
  Replicode syntax version from the AERA system release version.
