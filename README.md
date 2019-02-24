This branch is the Replicode code as of June 2011, updated to compile in Visual Studio 2017.
This is the latest code before version V1.2 .

Install Git command line. For example, from https://git-scm.com/download/win .
In a command terminal, enter:

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

Download and run the Visual Studio Community 2017 installer from https://visualstudio.microsoft.com/downloads .
Click "Desktop development with C++". In details, click "Windows 8.1 SDK and UCRT SDK". Click Install.

In Visual Studio, open the solution c:\work\replicode\Replicode.sln . Right-click on the Test project and click Properties. In Debugging, set "Command Arguments" to settings.xml

In the Build menu, click "Build Solution".

In settings.xml , use source_file_name to set the file to run. The default is to run the following annotated example which creates goals to move a hand to grasp an object:
https://github.com/IIIM-IS/replicode/blob/june2011/Test/test.1.2011-04-13.v1_0.replicode

In the Debug menu, click "Start Without Debugging".



