# General overview
This code provides a way to capture top view images using two side view images. 
Images are captured using 2 Rapsberry Pi based camera systems and sended to computer over the network.
The computer then generates a top view image from these received images.
The code consists of two parts. First, the code on the rapsberry Pi's should be set up as described below.
Then, the system can be tested and is ready for operation.

Note that this code is far from perfect. It was constructed for prototype purposes without focus on user friendly operation.
Probably, you will need some help with setting up the system. For these questions, you can reach me at y.delaere@student.utwente.nl.
A prerequisite for this to work is that you will need some experience with python programming, and some general knowledge about IT.

# Setting up the RPI cameras
First, make sure the raspberry pi's can be reached by the computer using a SSH connection. As an SSH client I used 'putty'. The two systems were attached to the PC by means of a network switch. 
Unfortunately, I am not sure what the password is of both RPI's anymore. Please try the following passwords: 1. buitenbar 2. Apollo2017 3. Apollo 4. picam1 5. picam2
If these passwords do not work, please reflash the SD cards using the Raspberry Pi Imager which can be downloaded from here: https://www.raspberrypi.org/downloads/. As an OS, select the Raspberry Pi OS.
When the OS had to be installed again, please enable SSH as described here: https://www.raspberrypi.org/documentation/remote-access/ssh/.

From now, I assume that you have SSH acces to the RPIs. Furthermore, I assume that the hostnames of the RPIs are picam1 and picam2 respectively. Hostnames can be changed as described here: https://www.tomshardware.com/how-to/raspberry-pi-change-hostname
Then, construct a private key for both devices as described here: https://devops.ionos.com/tutorials/use-ssh-keys-with-putty-on-windows/. Rename them to picam1key and picam2key respectively and place them into the PC folder. Overwrite the current ones.

The next step is to place the files inside the RPI folder of this repository on the repository. For file transfer between computer and RPI is used FileZilla: https://filezilla-project.org/
Before the files can be placed onto the RPI, some changes should be made to the files:
In all files inside the RPI folder (rpiSide, rpiSide_background, rpiSide_calibrate, rpiSide_operation), the following things should be changed:

ip (line 107) should be set to the local IP address of the computer.
framePort (line 108) should be 9600 for picam1, 9700 for picam2
msgPort (line 109) should be 9700 for picam1, 9701 for picam2
The files should be placed inside the folder 'home/Python' of both rpi's.

Now, the system can be tested, by running the python script: PC/pcSide.py.
If evertything is correct, you can see the low quality livefeed from both cameras in your browser from http://picam1:8000/stream.mjpg and http://picam2:8000/stream.mjpg respectively.
A high quality image can be captured by sending an 'x' character to the RPI's. This is done by typing an 'x' and then pressing the 'return' or 'enter' key on your keyboard. The system can be shut down by pressing ctrl + C in the terminal.

# How to run the system
First, a checkerboard calibration pattern should be printed. A useful website for making a calibration pattern is here: https://calib.io/pages/camera-calibration-pattern-generator
a checkerboard calibration pattern of 38x37 squares should be printed with 15mm squares. In the middlesquare of the pattern, a red circle should be drawn. I did this simply with a red permanent marker.
In my own experiments, the calibration pattern was printed on a sheet of A0 paper. However, it is better to print this on a more rigid material. I suggest to print it on a material as foam.
For example, the website https://www.posterxxl.nl/, provides services to print on a different material.

The calibration pattern should then be placed on the bead apex table, or a test table for testing purposes.
The cameras should be placed such that they can just see the row of squares with the red marker point. 

Now, the following python scripts should be run:
1. calibrate.py
	This script captures the calibration pattern and finds transformations for every square as described in the paper.
	Now remove the calibration pattern from the scene
2. background.py
	This script captures an image of the background and then fixes the camera settings, such as exposure time etc.
	When an image of the background has been captured, the program can be terminated using ctrl + c.
3. operation.py
	This script captures images of beads when they are observed by the system and stores them into the folder 'triggeredImages'.
	Can be terminated using ctrl+c
	
	
# Classifying the beads
The collected data from my time in Hungary can be downloaded from here: https://drive.google.com/file/d/1OJdgkXkxBFHEjM9S5dF6IftIiKFK3VtA/view?usp=sharing
This data is available untill 1-11-2020 and will then be removed from the server. 
Classification is performed using matlab. The different steps of classification are described in the paper.
The datset discussed in the paper is '20200731_6mm_light'. 

The following files are relevant:

1. analyzeImage.m
	Extracts the maximum width of every bead and stores them into a data file
2. classifyBeads.m
	Classifies the beads based on the data file. The folder Classification\ExtractedWidths contains the extracted width information of the dataset
3. showAnalysis.m
	Shows the different steps of extracting the width profile, by specifying a ID of an image and a label.
	
	









	
	
	
	
	

	
	

	
