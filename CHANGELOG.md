Interim change since AERA Diagnostic Mode v0.1
----------------------------------------------

Changes
* Change the Replicode syntax to add durations, distinct from time stamps https://github.com/IIIM-IS/AERA/issues/138

Bug fixes
* Replicode syntax: Allow ':' in a string. https://github.com/IIIM-IS/AERA/pull/207

AERA Diagnostic Mode v0.1 (2021-11-19)
-------------------------------------

A release optimized to run in "diagnostic time" which is now the default in settings.xml .

* Added example hand-grab-sphere.replicode where all models are coded in the seed.
* Added example hand-grab-sphere-learn.replicode which learns the grab and release models, as well
  as the anti-requirement model where grab fails if the sphere and cube are at the same position.
* Make other bug fixes and enhancements. Please see https://github.com/IIIM-IS/AERA/blob/master/doc/meeting-notes.txt

replicode-syntax-v1.2 (2020-06-19)
----------------------------------

* Rename the folder Test to AERA.
  Note: To see changes before this renaming, browse at the earlier commit: 
  https://github.com/IIIM-IS/replicode/tree/6154f1f2d0eac5543d760928b89bff4fd30adad3
* Rename the file Test.cpp to main.cpp .
* Rename the Test project to AERA.
* Rename folders V1.1 and V1.2 to replicode_v1.1 and replicode_v1.2, to distinguish the
  Replicode syntax version from the AERA system release version.
