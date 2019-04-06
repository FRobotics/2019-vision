===================
Building on desktop
===================

--------
Building
--------

Java 11 is required to build.  Set your path and/or JAVA_HOME environment
variable appropriately.

1) Run "./gradlew build"

---------
Deploying
---------

On the rPi web dashboard:

1) Make the rPi writable by selecting the "Writable" tab
2) In the rPi web dashboard Application tab, select the "Uploaded Java jar"
   option for Application
3) Click "Browse..." and select the "java-multiCameraServer-all.jar" file in
   your desktop project directory in the build/libs subdirectory
4) Click Save

The application will be automatically started.  Console output can be seen by
enabling console output in the Vision Status tab.

=======================
Building locally on rPi
=======================

1) Run "./gradlew build"
2) Run "./install.sh" (replaces /home/pi/runCamera)
3) Run "./runInteractive" in /home/pi or "sudo svc -t /service/camera" to
   restart service.


=======================
TEAM 4150
=======================

- The threshold filter should be tuned for each location and type of camera
    to adjust for brightness.  (GRIPVision.JAVA)

- Results are published to network table variables.

- In the code search for "debug" to comment or uncomment debug code

- There are some "TODO" items, which would be nice but not necessary.

- For HSV -- hue is what color.  saturation is how pure.  value is light/dark range.

