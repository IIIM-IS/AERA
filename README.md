Replicode Branch june2011
=========================

This branch is the Replicode code as of June 2011, updated to compile in Visual Studio 2017.
This is the latest code before version V1.2 .

Install Windows on macOS in VirtualBox
======================================

These are instructions to set up a Windows virtual machine on macOS. If you already have a
Windows virtual machine (or are running directly on Windows), skip to "Install on Windows" below.

* Open the web page https://www.microsoft.com/es-es/software-download/windows10ISO .
* Under "Select Edition", select the latest Windows 10 edition and click Confirm.
* Under "Select the product language", select English and click Confirm.
* Click 64-bit download. This is a big file and will take several minutes to download.
* (You can do this step while Windows is downloading.) Download and install VirtualBox for macOS from https://download.virtualbox.org/virtualbox/6.0.4/VirtualBox-6.0.4-128413-OSX.dmg .
* (After Windows is downloaded) Launch VirtualBox.
* In VirtualBox, on the Machine menu, click New. Change the Name to "Windows 10". Make sure the Version says "Windows 10 (64-bit)".  Click Continue and accept all the defaults.
* In the main VirtualBox screen, click Windows 10 and click Settings. Click Storage. At the bottom of the screen, hover over the + icon that says "Adds new storage attachment" and click. Click "Add optical drive". Click "Choose disk". Click the Add button. In the file pick dialog box, select the Downloads folder and select Win10_1809Oct_English_x64.iso . Then click Empty and hover over the X icon that says "Removes selects storage attachment" and click. Click OK.
* In the main VirtualBox screen, click Windows 10 and click Settings. Click System. In Boot Order, uncheck Floppy. Click OK.
* In the main VirtualBox screen, click Windows 10 and click Start. Follow the prompts to install Windows. In the "Activate Windows" screen, click "I don't have a product key". Click "Windows 10 Pro". In "Which type of installation", click "Custom". Follow the prompts to install Windows.

Install on Windows
==================

* If you are running Windows in VirtualBox, launch VirtualBox. In the main screen, click Windows 10 and click Start.
* Launch the Microsoft Edge browser and go to https://github.com/IIIM-IS/replicode . Sign in. Change the branch to june2011. Now you can copy/paste these instructions from the README. 
* To install Git command line, use the Microsoft Edge browser to download and install from https://git-scm.com/download/win .
* Click the Windows Start icon. In the search window, type Command Prompt. In the command prompt screen, enter:

    cd \
    mkdir work
    cd work
    git clone https://github.com/IIIM-IS/CoreLibrary
    cd CoreLibrary
    git checkout april2011
    cd ..
    git clone https://github.com/IIIM-IS/replicode
    cd replicode
    git checkout june2011
    cd ..

* Download and run the Visual Studio Community 2017 installer from https://visualstudio.microsoft.com/downloads .
* Click "Desktop development with C++". In details, click "Windows 8.1 SDK and UCRT SDK". Click Install.
* In Visual Studio, open the solution c:\work\replicode\Replicode.sln . Right-click on the Test project and click Properties. In Debugging, set "Command Arguments" to settings.xml .
* In the Build menu, click "Build Solution".
* In settings.xml , use source_file_name to set the file to run. The default is to run the following annotated example which creates goals to move a hand to grasp an object: https://github.com/IIIM-IS/replicode/blob/june2011/Test/test.1.2011-04-13.v1_0.replicode
* In the Debug menu, click "Start Without Debugging".



