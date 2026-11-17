Here is the order to run things, from a cold start to the full task. Everything is run from the problem11c folder:


cd "C:\Users\sdouz\Desktop\DTU\Semester 1\Robotics\Projects\problem11c"
Use plain python (your system Python has numpy, OpenCV and pyserial; the .venvrobotics one lacks OpenCV).

Step 0 — Try it without the robot

python line_follower.py --sim
This runs the whole sequence on a virtual robot and paper, so you can see what the windows look like. Press g in the map window when the planned path is shown. Press q or Esc at any time to stop.

Step 1 — Connect and set the port
Switch the robot on, plug in the USB cable, open the Dynamixel Wizard, and scan with Protocol 1.0 at 1 000 000 bps (Annex C). Note the port, e.g. COM8. Close the Wizard afterwards — it holds the port.
Put the port and camera index in config.py: PORT = "COM8", CAMERA_INDEX = 1 (0 is usually the laptop's own webcam).
Step 2 — Check the vision (robot not moving)

python line_follower.py --camera-only
Hold the camera over the sheet. The paper should get a green outline and the black line and dot should turn red. If not, adjust in config.py:

paper not found → lower PAPER_MIN_VALUE or raise PAPER_MAX_SATURATION;
line not found → raise BLACK_RATIO or BLACK_MAX_GRAY.
Step 3 — Check the joint directions

python line_follower.py --check-joints
Torque switches off, so hold the arm before pressing Enter. Move it by hand:

straight up must read q = 0 0 0 0 and tip = (0, 0, 286);
bend each joint and see that the printed angle changes in the direction of the DH model (joint 2 forward = negative, like your Problem 3 values). If one joint has the wrong sign, flip it in JOINT_DIRECTION; small constant errors go into JOINT_OFFSET_DEG.
Stop with Ctrl+C.

Step 4 — Check the camera model and paper height

python line_follower.py --check-camera
The arm goes to the home pose looking down. On the image you see a 20 mm grid projected onto the table and a magenta arrow that must point towards the robot base. If the arrow points elsewhere, change CAMERA_IMAGE_RIGHT_IN_F5 / CAMERA_IMAGE_DOWN_IN_F5 in config.py (swap or negate until the arrow and the grid labels make sense).

Then click a mark you drew on the paper and press t: the stylus moves to 3 mm above that point. If it lands off:

consistently too far / too close → PAPER_Z (paper height) or L4 (real stylus length);
off sideways → camera axes or CAMERA_OFFSET_F4.
Press b to return, q to quit. Repeat until the stylus hits your marks within a few mm.

(Optional, for better accuracy: python calibrate_camera.py with a printed checkerboard; the result is picked up automatically.)

Step 5 — Run the task
Place the A4 sheet in front of the robot with the line about 80–180 mm from the base (closer or farther is out of reach), the dot at the start. Then:


python line_follower.py
It searches for the sheet, maps it, checks the paper height, shows the planned path in the map window, and waits for you to press g before touching the paper. Keep a hand near the power switch the first time.

Useful options:

--paper-z 5 — paper height if you know it (otherwise it's estimated);
--auto — skip the confirmation (for the video);
--release — torque off at the end so the arm goes limp;
--port COM3 --camera 0 — override the config.
For the report / video
Record the screen (camera and map windows) and the robot together; the terminal output gives the measured paper size, the estimated height, the detected line length and the final tip position, which you can quote.


I saved the pattern as checkerboard_9x6.png in the problem11c folder, and OpenCV finds all 54 corners in it, as calibrate_camera.py expects.

If you calibrate (optional, about 3 minutes):

Show the pattern:

Phone or tablet (no printer needed): open the image full-screen at full brightness, and avoid reflections from lamps.
Or print it on any printer and tape it flat on a book or a piece of cardboard. It must not bend.
The size doesn't matter: the field of view comes out the same, so no ruler is needed.

Run:


python calibrate_camera.py
Add --camera 1 if 0 isn't the robot's camera.

Hold the pattern in front of the robot's camera, about 20–40 cm away. When coloured lines appear on the corners, press SPACE. Take about 15 pictures, each slightly different:

tilted left, right, up and down;
nearer and farther;
in the middle and in each corner of the image; the corners matter most.
Press c. It prints the measured field of view and saves camera_calibration.npz, which the main program loads automatically.

After calibrating, repeat the stylus mark test (python line_follower.py --check-camera, click a mark, press t), because the camera model changed slightly.

My recommendation is still to try the full task first: 