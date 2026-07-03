# AERA Test Framework

The `test` project provides an extensible test framework for AERA. Tests consist of Replicode and TXT input-output pairs.  
Replicode files provide the seed with which AERA is run, while TXT files provide the lines which are expected to show up in AERA's decompiled output.

For detailed test documentation, see `report.md`.

## Setup

### Prerequisites
* **Google Mock (gmock) v1.11.0**: This is managed via NuGet.  
To install it, right-click the Solution in Visual Studio and select **Restore NuGet Packages**.  
*Note: The specific dependency configuration can be found in `packages.config`.*

### File & Folder Structure
* **Viewing the Folder Tree:** By default, Visual Studio maps files using virtual filters, which will cause all test files to appear in one flat root list. To see the actual, organized directory structure on disk, click the **Show All Files** icon at the top of the Solution Explorer window.
* **The `tests` Directory:** This folder contains all the Replicode/TXT pairs, organized into four subfolders:
  * `existing/`: Legacy tests for Replicode versions 1.1 and 1.2, copied over directly from the main AERA project.
  * `free-tests/`: Experimental test drafts.
  * `intelligence-requirements/`: The core of the test suite. Contains tests mapping directly to distinct intelligence requirements. Only these tests are configured to be discovered in `test_all.cpp`.
  * `technical-report/`: Tests validating the 2013 technical report on Replicode.  
	*Note: The document `replicode_report_annotated.pdf` contains this original report alongside annotations regarding its correctness.*

### Building & Running Tests
* **Manual Build Required:** By default, the `test` project is excluded from the global solution build to avoid forcing dependencies on other workflows. To compile it, right-click the `test` project explicitly and select **Build**.
* **Target Configuration:** The test framework is configured to build using **Release** + **Win32**. If you need to use a different configuration, you must manually update the file paths defined at the top of `test_all.cpp`.
* **Discovering & Running Tests:** Once the build is complete, all tests will be automatically discovered inside the Visual Studio *Test Explorer* window, where they can be run or debugged. 
* **Running tests from command line:** Alternatively, you can execute the tests directly from the command line by running `Release/test.exe` (from the repository root directory, not within the `test` module folder). You can also apply filters to run specific tests, for example: `./Release/test.exe --gtest_filter=3_CUMULATIVE_LEARNING*`

---

## Test Execution Lifecycle
When a test is executed by the framework, it follows these automated steps:

1. **Configuration Generation:** The runner takes the `settings_template.xml` file and populates it with variables specific to the active test.  
   *Note: First, the top of the test's individual Replicode file is checked for customized parameters. If none are specified, it defaults to the configuration variables defined in the `test_all.cpp` script.*
2. **AERA Execution:** AERA is launched and run using the populated settings XML file and the target Replicode seed.
3. **Output Evaluation:** Once the run completes, the framework parses AERA's decompiled output logs and matches them against the expected values inside the corresponding `.txt` file.  
4. **Pass/Fail Criteria:** If **all** lines listed in the `.txt` file are successfully discovered anywhere within the decompiled output, the test passes. Missing lines cause a failure.