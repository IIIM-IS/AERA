These are instructions to install the June 2011 version of Replicode. This is the latest code before version V1.2 .

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

Download and run the Visual Studio 2017 Community installer from https://visualstudio.microsoft.com/downloads .
Click "Desktop development with C++". In details, click "Windows 8.1 SDK and UCRT SDK". Click Install.

In Visual Studio, open the solution c:\work\replicode\Replicode.sln .

