Replicode
=========

These are instructions to build and run Replicode.

* Install Visual Studio Community Edition from https://visualstudio.microsoft.com/vs/community .
  In the installer, under "Desktop development with C++", check "Windows 10 SDK (10.0.17134.0)" and
  "MSVC v141 - VS 2017 C++ build tools".
* Install GitHub for Desktop from https://desktop.github.com
  (Or use your favorite git client.)
* Launch GitHub for Desktop and sign in to GitHub. In the File menu, click "Clone a Repository".
  Click the URL tab and enter `https://github.com/IIIM-IS/replicode` . It should be a recursive 
  clone (which is the default).
* Launch Visual Studio and open the project `Replicode.sln` from the cloned repository. E.g.:
  `C:\Users\Alice\Documents\GitHub\replicode\Replicode.sln`
* To work with the visualizer, we must enable `WITH_DEBUG_OID` as follows. In the Solution Explorer,
  open the section for `CoreLibrary`. Double-click `base.h` . Uncomment the define for `WITH_DEBUG_OID`, so
  the line is:

    #define WITH_DEBUG_OID // Enable get_debug_oid() in every object.

* On the Build menu, click Build Solution. (Don't worry about all the compiler warnings.)

