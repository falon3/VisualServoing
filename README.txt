# VisualServoing
Robotics LabAssign3 
Falon Scheers and Tristan Meleshko

This program implements uncalibrated visual servoing (UVS) for control of a 
planar robot arm to reach a desire position, clamp a target item, 
then move to another destination location selected and release the object there.

To run and operate: 
    - open the VisualMain.jar java project in Eclipse and upload it onto the EV3 brick
    - run VisualMain.java
    - navigate to the directory that contains tracker.py
    - in terminal enter > python tracker.py
    - select with mouse the identifyer yellow object on the robot arm and the object you wish the arm to navigate to
    - press any button on the EV3 brick to start
    - once at destination object location manually press any button on the EV3 brick to stop the claw grabber 
      fingers at the desired position secure around the object.
    - select the next destination point for the object and press any button on the EV3 brick to begin moving there.

references:
* Use of the OpenCV tracker code provided on the course webpage (put into TrackerReader.java)
* Sample python code for the Camshift Visual Tracker(https://fr.wikipedia.org/wiki/Camshift) used to start and run the Visual tracker
* http://www.instructables.com/id/Simplest-EV3-Robot-ClawGripper/
