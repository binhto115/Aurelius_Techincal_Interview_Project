### HOW TO RUN:

    There are two ways to run:
    (1) Create/upload your own plan in Mission Planner
    (2) Customize your plan in mission.json file

            if (1):
                a. Go to 'Simulation' tab, choose model 'Quad', frame 'Multirotor'
                b. Make sure it's TCP at 115200, and choose your port
                b. Create/upload your plan then write it
                c. Run python mavlinkScript.py tcp:LOCAL_HOST_IP:ArduPilot_PORT
                For example, in my case: python mavlinkScript.py tcp:127.0.0.1:5762

            if (2):
                a. Go to 'Simulation' tab, choose model 'Quad', frame 'Multirotor'
                b. Make sure it's TCP at 115200, and choose your port
                c. Customize the mission.json template
                    i. "connection": estalish connection from your localhost machine to the drone
                    ii. "home": sets the coordinates for new home, or leave the same coordiates for the same one
                    iii. "mission": customizes your latitude, longitude, altitude, and/or speed.

                    ***There can only be 3 types: takeoff, waypoint, and rtl. takeoff goes first and rtl goes last. You can have as many waytpoints as needed. Leave the "speed" parameter blank for default speed***

                d. run python mavlinkScript.py mission.json
                e. Observe the magic!!!

    [Bonus] - Can be used for both ways
        Open a second terminal and run any of these example custimizable commands to guide the drone to a specific coordinates. Should only run one command at a time for better experience.

            Add-content cmd.txt "goto 37.6216249 -122.3765159 15"
            Add-Content cmd.txt "resume"
            Add-Content cmd.txt "rtl"
            Add-Content cmd.txt "joy"

            ***Notes:
                goto:   allows you to guide your drone to a specific location while it's flying
                resume: switches GUIDED to AUTO to resume initially planned mission
                rtl:    allows the drone to return to launch
                joy:    allows you to control the drone manually

        How to control the drone:
            1. Run (2)
            2. Open a second terminal and enter: Add-Content cmd.txt "joy"
            3. Press to allow the drone to move, press again to stop it from moving
            4. Turn the joystick left and right to turn the drone

### Implementation:

    For this project, I wanted a way to control the simulated drone without using the plan and data tabs from Mission Planner. I wrote a python script that takes a .json file and a commnand text file as inputs to control the drone, the terminal as a display for the drone's real-time telemetry, mission.json as a customizization page for routes and patterns for the drone, and a cmd.txt as a guide for drone detour.

    Manual drone control with a joystick:
        (1) Make sure the COM port (e.g. COM5) and baudrate (115200) are the same for both (already set to 115200)
        (2) Arduino Uno board with a joystick sensor
        (3) Wiring connection:
            Joystick pins:           Arduino Uno pins:
                5v            =>          5V
                GND           =>          GND
                VRX           =>          A0
                VRY     (unused)
                SW            =>          D2

        *** Test the Arduino board with the joystick with Serial monitor first before the whole program to make sure it prints "500, 0"

### Limitations:

    - Limits to quad multirotor
    - Limits to simple four commands: takeoff, waypoints, return-to-launch, and joy.
    - Two modes: AUTO and GUIDED
    - Joystick can only allow you to rotate and turn on/off the drone

### Additional improvements I would make with more time:

    - Well-made responsive UI for RT controls and vehicle information
    - A functional option to switch vehicles and models
    - Add a throttle control to the drone
    - Obstacle avoidance/detection feature

### Additional Information (Optional reading):

    Architechture:
        Mission Planner           ==> ArduCopter.exe SITL Flight Controler via MAVLink TCP 5760
        Python program pymavlink  ==> ArduCopter.exe SITL Flight Controller via MAVLink TCP 5760
        (My pc -> ArduCopter.exe -> Simulated drone)

    TCP connection Debugging

        device = "tcp:127.0.0.1:5762"
        Protocol: tcp
        localhost: 127.0.0.1
        Port: 5762
        Use netstat to find and check for available ports

        Commandline I ran:
        python -m pip install pymavlink to install pymavlink
        python -m pip install MAVProxy
        python -m pip install prompt-toolkit
        python -m pip install wxPython
        pip install pyserial

        PS C:\Users\binht> tasklist | findstr ArduCopter
        ArduCopter.exe 35112 Console 1 14,408 K
        PS C:\Users\binht> netstat -ano | findstr "35112"
        TCP 0.0.0.0:5760 0.0.0.0:0 LISTENING 35112
        TCP 0.0.0.0:5762 0.0.0.0:0 LISTENING 35112
        TCP 0.0.0.0:5763 0.0.0.0:0 LISTENING 35112
        TCP 127.0.0.1:5760 127.0.0.1:52989 ESTABLISHED 35112
        UDP 0.0.0.0:5501 _:_ 35112

        PS C:\Users\binht> netstat -ano | findstr "35112"
        TCP 0.0.0.0:5760 0.0.0.0:0 LISTENING 35112
        TCP 0.0.0.0:5762 0.0.0.0:0 LISTENING 35112
        TCP 0.0.0.0:5763 0.0.0.0:0 LISTENING 35112
        TCP 127.0.0.1:5760 127.0.0.1:52989 ESTABLISHED 35112
        UDP 0.0.0.0:5501 _:_ 35112
