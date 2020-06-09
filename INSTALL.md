Replicode
=========

These are instructions to build and run Replicode.

* Install Visual Studio Community Edition from https://visualstudio.microsoft.com/vs/community 
* Install GitHub for Desktop from https://desktop.github.com
  (Or use your favorite git client.)
* Launch GitHub for Desktop and sign in to GitHub.
* Click "Clone a Repository from the Internet", click URL and enter 
  `https://github.com/IIIM-IS/replicode` . It should be a recursive clone (which is the default).
* Launch Visual Studio and open the project `Replicode.sln` from the cloned repository. E.g.:
  `C:\Users\Jeff\Documents\GitHub\replicode\Replicode.snl`
* On the Build menu, click Build Solution.
* If the build fails with the error "The Windows SDK version 10.0.17134.0 was not found", then
  close Visual Studio, launch the Visual Studio Installer and install SDK version 10.0.17134.0 .
  Launch Visual Studio and repeat the steps to build the Replicode solution.

  

