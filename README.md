# FRC_2023-24
2023-24 Season of FRC for the LIGHTSABERS Robots Team

# LIGHTSABERS Getting Started with FRC Robot Code

[![Gradle](https://github.com/wpilibsuite/allwpilib/actions/workflows/gradle.yml/badge.svg?branch=main)](https://github.com/wpilibsuite/allwpilib/actions/workflows/gradle.yml)
[![Java Documentation](https://img.shields.io/badge/documentation-java-orange)](https://github.wpilib.org/allwpilib/docs/development/java/)

Welcome to the LIGHTSABERS Robot Code Repository for the FRC 2023-24 Season. This Repositiory Contains the steps for installing the Coding Enviroment and loading in last year's FRC Robot Code.

- [Preliminaries with Git](#preliminaries-with-git)
- [Robot Home](#robot-home)
- [Zero to Robot](#zero-to-robot)
- [GitHub Revisited](#GitHub-Revisited)
- [Programming Basics](#programming-basics)
- [Creating a New Robot Project](#creating-a-new-robot-project)
- [3rd Party Libraries](#3rd-party-libraries)
- [Copy Last Years Robot Code](#last-years-robot-code)
- [Git Update](#git-update)
- [Git Again](#git-again)


# Preliminaries with Git

First of all, I'm assuming that you are reading this README.md file from the "main" branch on the LIGHTSABERS Web-based GetHub site (i.e. the remote Github repositiory).  The "main" branch is the branch that holds the "competition" code.  This code should always compile and be deployable to the Robot on a moments notice.  So, do not development new code in the "main" branch.  The first thing to do is to create a new code development branch by clicking on the "branch" icon at the top of this repository.
![image](https://user-images.githubusercontent.com/54441806/205994006-44c873f8-bac8-4605-8b4d-bd86b5a21ced.png)
This will bring up a "branch" page, and on the right side of this page is a green button called "New Branch".  Click on "New Branch" button, then type "{username}Dev" in the Branch Name.  You can choose any {username} that is relevant to you, I usually choose my first initial and the first 4 letters of my last name (e.g. cvarn).  After selecting your development branch name, make sure that the "source" is set to the "main" branch, and then click "Create Branch".  
![image](https://user-images.githubusercontent.com/54441806/205996651-fa7224c6-71c3-4abe-b60e-2d3e37ea0f4c.png)
The resulting "Overview" page will then show that there is an Active branch called "{yourname}Dev".  You can click on the "{yourname}Dev" branch and you will see that it has the same files as were in the "main" branch.
![image](https://user-images.githubusercontent.com/54441806/205998157-9fa5980c-fed6-4cf2-befc-6f007a0b72eb.png)
  
At this point, I will assume that you are reading this README.md file from the "{yourname}Dev" branch.  You are now ready to setup your computer to receive your local repository of the FRC_2023-24 Code.  To avoid having this repository proliferate on your computer, it will be stored in common directory.  I recommend the Public User's workspace.  Create a new storage folder on your Windows computer to hold all of your Git projects (ex. GitHub).
  
"{Drive_Letter}:\Users\Public\GitHub"

There are a couple of options for cloning the Online Repository locally on your computer:
1.  (Recommended)  Use the GitHub Desktop.
2.  Download to a Zip file.
   
There is a green button called "Code" on this webpage above the top right-hand corner of this README.  Right click on this button and the various options for "cloneing" this repository are offered.

![image](https://user-images.githubusercontent.com/54441806/206072601-60adb60e-19bf-4245-85d4-27e3497e33dd.png)
---------------------------------------------------------------------------------------------------------------
If you are using the GetHub Desktop, you will need to select the storage location you created above and make sure that the URL is pointing to the FRC_2023-24 Online Repository before clicking Clone. You can also down load the Repository as a Zip file.  This will detach the Repository from this Online Git account.  Any updates that you make are private and cannot be pushed back to this account, which is where the "competition" code resides.   

When GitHub Desktop opens, then it will default to looking at the "main" branch.  Switch it to the "{yourname}Dev" branch.

![image](https://user-images.githubusercontent.com/54441806/206074279-9abeab05-935d-456a-8e2c-fa2f417320b7.png)

This repository now exists locally on your computer at:

"{Drive_Letter}:\Users\Public\GitHub\FRC_2023-24"


# Robot Home

The FRC_2023-24 folder on your computer (local) has this README file, a license file, a folder called "2023_Robot_Code" and another folder called "2024_Robot_Code".  The 2023_Robot_Code folder is provided as an example and a reference for last years Robot code.  All Robot code projects for this year will created in the folder called "2024_Robot_Code".  Each of these folders represents a WPILib Java project.  

# Zero to Robot

Starting with Step 2 of the Zero to Robot instructions (https://docs.wpilib.org/en/stable/docs/zero-to-robot/step-2/wpilib-setup.html) install the WPILib Visual Studio Code libaries.  Since we are programming Java and we are not building the WPILib libraries, it is not necessary to perform the steps for the "Additional C++ Installation for Simulation".  When the instructions on this web page have been completed, the following will be installed:
  
Visual Studio Code - The supported IDE for 2019 and later robot code development. The offline installer sets up a separate copy of VS Code for WPILib development, even if you already have VS Code on your machine. This is done because some of the settings that make the WPILib setup work may break existing workflows if you use VS Code for other projects.

C++ Compiler - The toolchains for building C++ code for the roboRIO

Gradle - The specific version of Gradle used for building/deploying C++ or Java robot code

Java JDK/JRE - A specific version of the Java JDK/JRE that is used to build Java robot code and to run any of the Java based Tools (Dashboards, etc.). This exists side by side with any existing JDK installs and does not overwrite the JAVA_HOME variable

WPILib Tools - SmartDashboard, Shuffleboard, RobotBuilder, Outline Viewer, Pathweaver, Glass, SysID

WPILib Dependencies - OpenCV, etc.

VS Code Extensions - WPILib extensions for robot code development in VS Code

Continue to the next page in the instructions and select the "Visual Studio Code Basics" hyperlink.

# GitHub Revisited

Start WPILib VS Code application and open the folder called "{Drive_Letter}:\Users\Public\GitHub\FRC_2023-24\2024_Robot_Code".  Before doing anything else select the "Source Control" icon on the left-hand wall of the WPILib Application.  This will change the left window pane to show GitHub Source control status and commands.  At the top of this pane, below "Source Control Repositories", you will see your current "local" repository (FRC_2023-24) and the current local "branch" ("main") for that repository.  Click on the branch and, at the bottom of the popup window, you will see your remote (on-line) development branch, which is called "Origin/{username}Dev".  Select your remote development branch and the local branch will then change to be your development branch.  At this point any changes you make to the project are being made to your "development" branch code.  Your changes will not change the "main" branch competition code until you have completed your developement, tested it, and have approved it as ready for competition.

While you can make code changes, test your builds, deploy to, and even run the robot from the code in your 2024_Robot_Code folder; you should save often.  Since you code is also managed byt software source control, you should periodically commit your changes to source control (Git) using the "Commit" button on the Source Control pane.  When you "Commit" the changes, you need to provide a comment above it to be stored as a record of what was was changed in this "Commit" operation.  So you should "Commit" often enough to keep the Comment of what was changed small.

You can then open one of the java source files in the project and examine its code.  This code probably will not build using the WPILib on your computer because you probably have not added 3rd party libaries required to run the devices that this version of the 2024_Robot_Code is using.  Read the FRC documation on installing [3rd party libraries](#3rd-party-libraries) found at:  https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html#rd-party-libraries.

Any files that you add or modify in this folder are registered with GitHub.  The 2024_Robot_Code in the Local Repository is now different from the Remote Repository on the Web.  For example: In the directory above, if you open the README.md file in a text editor (ex. wordpad), change the spelling of a word, and then save the file; GitHub Desktop will change to look like this:

![image](https://user-images.githubusercontent.com/54441806/206082745-758bfe6e-c9ea-4745-9dae-d1e5575a19e8.png)

Even though the file has changed, it is not stored in Git until a "Commit" is completed and has a relevant change comment. After the change has been "Committed" to the Local Repository, it can be backed-up, or sync'ed, to the Remote Repository on the Web by clicking on the "push" button a the top of the GitHub Window.  This whole example shows that source control can be handled by GitHub Desktop if necessary, but the Commit and Push (sync) operations are normally done in the WPILib VS Code Source Control Pane.   

![image](https://user-images.githubusercontent.com/54441806/206085043-25b620a6-297c-477d-adf6-412e7a08d93c.png)

Now you can create or open Robot project files in the local repository using the Visual Studio Code editor and when you get all done you can commit them to Git storage and back them up (or sync them) to the Git remote storage on the Web. 


# Programming Basics

The "Visual Studio Code Basics" hyperlink redirects to a webpage called "Visual Studio Code Basics and WPILib Extension" (https://docs.wpilib.org/en/stable/docs/software/vscode-overview/vscode-basics.html#visual-studio-code-basics-and-the-wpilib-extension), which is part of the "Programming Basics" Tutorial.

Read this page to get familiar with how to call the WPILib command from the command set list.  Click the "Next" button a the bottom of the page to go to next page where a description of each of the WPILib commands is found.  Click the "Next" button to to to the next page, which starts with a discussion on Robot Program and how a Robot Base Class is selected.  Pay particular attention to the "Command Robot Template". 


# Creating a New Robot Project
  
Eventually, a section called "Creating a New WPILib Project" is found toward the middle of the the Robot Program Page (https://docs.wpilib.org/en/stable/docs/software/vscode-overview/creating-robot-program.html).  Unless you are going to start from scratch and delete the 2024_Robot_Code folder, you should not need to create a "new" Project.  But if creating a new project from scratch is your intention (The 2024_Robot_Code had to come from somewhere.  Right?), then perform the tasks stated in Visual Studio Code documetation above. 

While doing these tasks, a "New Project Creator Window" will pop up.  Select the following on the first row:

Project Type:  Template
Language:      java,
Base:          Command Robot

Other item on the creator page are filled in as:
Base Folder:  {Drive Letter}:/Users/Public/GitHub
Use the "Select a new project folder" button to graphically select the Base Folder

Project Name: {New_Project_Name},
Team Number:  3660.

Then click on the "Generate Project" button.

Visual Studio will now have a project created in the FRC_2023-24 Folder called "2023_Robot_Code".

After creating the project in Visual Studio Code, continue reading to the webpage instructions to end of the page and click the "Next" button to continue to "libraries".

# 3rd Party Libraries

At this point in the process, you should be on the 3rd Party Libraries Page of the Web instructions(https://docs.wpilib.org/en/stable/docs/software/vscode-overview/3rd-party-libraries.html). This is an important step for you if you are getting the 2024_Robot_Code from GitHub.  Since you subsequently installed WPILib VS Code, the necessary libaries for running all of the devices used in the 2024_Robot_Code have not been installed.  You must install the necessary libraries before the code will compile (build).  Read down to "Managing VS Code Libraries".  In VS Code, use the "Manage Vendor Libraries > Manage Current Libraries" to show you that:

WPILib-New-Commands

library is already part of your project.  Note: If you libraries are project specific.  If you closed the project, then it must be reopened to show the libraries currently in use.

Pay attention to the "command line" option for installing vendor libraries.  Three-quarters of the way down this page is a list of common vendors and their web pages.

Add the Rev library and the Phenoix Library to your project by:

1. Open a terminal in VS Code from the "View > Terminal" tab.

2. Navigate the directory "<Drive Letter>:\Users\Public\Documents\GitHub\FRC_2022-2023\Robot_Code\Last_Year"

3. Give the following commands at the terminal command prompt:

> ./gradlew vendordep --url=https://software-metadata.revrobotics.com/REVLib.json

>./gradlew vendordep --url=https://maven.ctr-electronics.com/release/com/ctre/phoenix/Phoenix-frc2022-latest.json

Note: these website addresses were valid for the 2022 season.

In VS Code, use the "Manage Vendor Libraries > Manage Current Libraries" to show you 3 libraries are part of the project:

CTRE-Pheonix
RevLib
WPILib-New-Commands

# Last Years Robot Code

Last year's robot code is in the 2023_Robot_Code folder.  If you open the folder for 2023_Robot_Code folder in WPILib VS Code, you can examine that project.  That project will have many errors due to the fact that the 3rd party libraries have changed this year.  If you want it to build, code changes are needed to make it compatable with the 2024 3rd party libraries. 

# Git Update

Remember to select your developement branch for Source Control and to save and "Commit" often.  Push the change to the remote repository when finished by clicking on the "Push Origin" button on the top (right side) of the GitHub Desktop window or by sync'ing in the WPILib Source Control pane.

After you have tested your development code and you feel that the code is good enough for competition, the next step is to check for any changes that have been made on the "main" branch while you were devloping your code.  To do this, make a "pull" request in GitHub Web or with GitHub Desktop.  The request is to pull the changes from the "main" branch into your {username}Dev Branch.  If there are no changes identified in the pull request, the you make a "pull" request to pull the changes in your {username}Dev branch into the "main" branch.  After reviewing the changes that will be made to the "main" branch (they should be only your changes made during your development), accept the pull request and have it committed to the "main" branch.

Before "pull"ing new changes into the "main" branch, you must brief the Drive Team on the changes and how they affect the operation of the Robot.

If the "pull" request to pull the changes from the "main" branch into your {username}Dev branch shows that changes have been made in the "main" branch that you did not consider when developing your change, you must accept the changes and modify your development code to work with the "main" branch changes.  Afterwards, you can make the "pull" request to pull your {username}Dev changes into the "main" branch and there should be no conflict.

# Git Again

If you decide that you want to develop another feature of the Robot, you must prepare your "{username}Dev" branch to be compatible with the competition code.  This is sometimes called "Re-basing" your development branch.  If changes have been made to the competition robot code since the last time you delivered (pulled) your development code to the "main" branch, then your development branch must be re-based.  The process is simple -- just make a pull request from the "main" branch to your "{username}Dev" branch and accept all of the changes.  Afterwards, if the code in your "{username}Dev" branch does not build, it is because new devices have been added to the competion robot from other development activities and they have libraries that have not been installed on your computer.  Find the missing [3rd Party libraries](#3rd-party-libraries) and install them.  The code will then build and is ready for you to begin your new developement activity.  
