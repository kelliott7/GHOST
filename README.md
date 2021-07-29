# GHOST - Gyroscopic Handheld Orientation Simulation Tool
*A tabletop Three Axis Motion Simulator that takes input from the orientation of an Android device via Bluetooth communication. The center platform of the TAMS is moved by pitch, yaw, and roll motors to match the angular position of the phone.* 

*This simulator was designed and developed by Katie Elliott, Evan Baker, and Hannah Boulware of Hill Technical Solutions, with the lead of Brandon Dias and Richard Barrow, for demonstration at the Space and Missile Defense Symposium.*


## GHOST App
#### To download the GHOST app
1. Open this link: https://gallery.appinventor.mit.edu/?galleryid=b9d5481b-f452-4536-bab6-d944d49cb560
2. At the top of the screen click "Build" then "App ( provide QR code for .apk )"
3. When the QR code appears, scan it with the Android device you wish to download the app to
4. Change settings on Android device to allow app downloads as they pop up
#### GHOST app controls
- Connect Bluetooth
  - Shows list of available Bluetooth devices
  - Select 98:D3:51:FE:30:70 HC-05
  - Button text changes to "Connected" when successfully connected
- Start
  - Starts displaying angles and sending data over Bluetooth
- Stop
  - Stops displaying angles and sending data over Bluetooth

## ghostMain.ino Arduino Sketch
#### Running the sketch
- Upload to Arduino Uno board by clicking the arrow
- Sketch will immediately start running
- Start the sketch over by clicking the rest button on the board itself
#### Functions
- setup()
  - Initializes serial monitor and Bluetooth communication
  - Connects stepper motors to the correct pins
  - Enables CNC shield
- loop()
  - Sets speed and acceleration of each motor
  - Checks for bytes coming in over Bluetooth
  - Writes a full set of pitch, yaw, and roll angles to a string then stores values as floats
  - Checks for bad data coming in by comparing to previous data
  - Converts YPR angles to DCM
  - Converts DCM to PYR angles
  - Set the target position of each motor
  - Moves each motor to target position
  - Stores pitch, yaw, and roll angles to be compared to next data
- EulerYPR_to_DCM()
  - Converts angles coming into the phone to a directional cosine matrix
- DCM_to_EulerPYR()
  - Converts directional cosine matrix to the angles the motors need to move to

## Wiring
#### Motors
- Pitch motor plugs in to X driver on CNC shield
- Yaw motor plugs in to Y driver on CNC shield
- Roll motor pkugs in to Z driver on CNC shield
#### Bluetooth
- Connect RXD on Bluetooth to Hold on CNC shield
- Connect TXD on Bluetooth to Abort on CNC shield
- Connect GND on Bluetooth to GND on CNC shield
- Connect VCC on Bluetooth to 5V on CNC shield
#### Power supply 
- Plug in Arduino board using USB and wall plug
- Connect red lead to positive wire
- Connect black lead to negative wire
- Plug in power supply and flip switch on back
- Press output button to supply power to CNC shield
