# GHOST - Gyroscopic Handheld Orientation Simulation Tool

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
  - Initializes serial monitor and bluetooth communication
  - Connects stepper motors to the correct pins
  - Enables CNC shield
- loop()
  - 
