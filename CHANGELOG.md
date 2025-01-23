AERA Diagnostic Mode v0.3 (2025-01-23)
--------------------------------------

A release optimized to run in "diagnostic time" which is now the default in settings.xml .
This is a maintentance release from Diagnostic Mode v0.2 .

Bug fixes
* Use input_code_index when recursing structures. https://github.com/IIIM-IS/AERA/pull/232
* Recursive copy of sub-structures. https://github.com/IIIM-IS/AERA/pull/257
* BindingMap: Fix bug in abstract_f_ihlp template args. https://github.com/IIIM-IS/AERA/pull/262
* In is_evidence, fix is evidence for anti fact target. https://github.com/IIIM-IS/AERA/pull/277
* Don't use CTPX for anti-facts. https://github.com/IIIM-IS/AERA/pull/278
* Bug when intersecting two binding maps. https://github.com/IIIM-IS/AERA/pull/288
* Don't treat trailing space as comment continuation. https://github.com/IIIM-IS/AERA/pull/292
* Other bug fixes. See https://github.com/IIIM-IS/AERA/pulls?q=is%3Apr+is%3Aclosed+label%3Abug

Changes
* Replicode syntax: Treat [] as an empty set if it is not at the end of a line. https://github.com/IIIM-IS/AERA/pull/229
* Support model shared template variables. https://github.com/IIIM-IS/AERA/pull/240
* Added support of sending entities to AERA. https://github.com/IIIM-IS/AERA/pull/247
* PTPX: Accept a mk.val if it shares attribute/value with an accepted mk.val. https://github.com/IIIM-IS/AERA/pull/263
* General guard builder for custom classes. https://github.com/IIIM-IS/AERA/pull/266
* Multi value tcp io device. https://github.com/IIIM-IS/AERA/pull/267
* Use microseconds for Timestamp period. https://github.com/IIIM-IS/AERA/pull/270
* Add the identity operator (id X). https://github.com/IIIM-IS/AERA/pull/280
* success object: Add optional obj_mk_rdx. https://github.com/IIIM-IS/AERA/pull/282
* Other enhancements. Please see https://github.com/IIIM-IS/AERA/blob/master/doc/meeting-notes.txt

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
