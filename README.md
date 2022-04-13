# Autonomous-Navigation-Robot - An A-Level Computer science project
### September 2020 - May 2021


The robot was designed as a test platform for an autonomous navigation system for factory / warehouse floors. It uses a single point LIDAR sensor on a servo to
detect it's surroundings and 4 NEMA-17 stepper motors as drive, all controlled by the Raspberry Pi.
It is powered by a large powerbank (for 5V power) and a 4s (14.4V-16.8V) lithium ion cell from an old drill to power the steppers.

<img src = https://github.com/robosam2003/Autonomous-Navigation-Robot/blob/master/resources/Photos%20and%20Videos/IMG_20201222_212848~2.jpg width=900>

It was mostly 3D printed (the base was sign-board (aluminuim-polycarbonate composite), and had rubber (from an old inner tube) added to the wheels for grip.
It could be controlled by a third party playstation controller as well, and was really fun to drive around (*small drift whoop*):

<img src = https://github.com/robosam2003/Autonomous-Navigation-Robot/blob/master/resources/Gifs/drifting.gif width = 400>

The wheels were extremely over engineered for the task at hand...

<img src = https://github.com/robosam2003/Autonomous-Navigation-Robot/blob/master/resources/CAD/overEngineeredWheel.png width = 400>


A GUI was displayed on the 7" touchscreen to allow the user to add coordinates to a queue, either directly, or from a list of pre-saved positions.
<img src = https://github.com/robosam2003/Autonomous-Navigation-Robot/blob/master/resources/Photos%20and%20Videos/IMG_20210112_172352.jpg width = 700>

It used a modifed version of the [A* search algorithm](https://en.wikipedia.org/wiki/A*_search_algorithm) to find the quickest path to the
goal coordinates, all of which was shown on the screen:
<img src=https://github.com/robosam2003/Autonomous-Navigation-Robot/blob/master/resources/Photos%20and%20Videos/IMG_20210104_155820.jpg width = 700>

The final result was quite slow on account of the single point LIDAR having to scan with a servo (a 360 degree LIDAR would be better) but 
it worked as a proof of concept, and I am happy with the results:
<img src = https://github.com/robosam2003/Autonomous-Navigation-Robot/blob/master/resources/Gifs/timelapseNAVIRobot.gif width = 1000>
