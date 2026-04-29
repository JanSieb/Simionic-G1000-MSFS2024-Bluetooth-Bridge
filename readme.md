# Simionic SHB1000 Garmin G1000 MSFS2024 Bluetooth Bridge

Bridges Simionic G1000 hardware bezels to Microsoft Flight Simulator via Bluetooth LE, enabling direct button and encoder input without an iPad.

## Quick Start

### Prerequisites

- **Operating System: Windows 10 or 11 (64-Bit)**\
as the application exclusively targets x64 architecture.\
Obviously, if you download the code and compile it yourself, you can migrate as you please.
- **Bluetooth Adapter**\
Your PC must be able to do Bluetooth.
- **FSUIPC WASM Module**\
Allows manipulation of steering variables directly in MSFS. Download free at FSUIPC.com
- **Microsoft Flight Simulator (MSFS):**\
MSFS (2020 or 2024) must be installed and running.
- **Simionic Bezel Hardware:**\
The corresponding external hardware device (like the Simionic SHB1000) powered on and within Bluetooth range.

### Install
1. **System Preparation**
    1. Enable Bluetooth: Ensure the PC’s Bluetooth receiver is turned on.
    1. Install FSUIPC WASM Module: Your app uses the FSUIPC WAPI to talk to MSFS.
        * Download the FSUIPC WASM module (found on the FSUIPC.com download page).
        * Extract the fsuipc-lvar-module folder into the MSFS Community folder.
1. **Download and Extract this App**
    1. Go to the Releases section on the right side (need to be logged-in). 
    1. Download the standalone .zip file containing the Release build.
    1. Extract the .zip file to a permanent location on the PC\
    (e.g., C:\MSFS_Tools\BLE_Bridge\).
    **Important:** Ensure the .exe, .config, and .map files are all together in the exact same folder.
1. **First Time Setup & Connection**
    1. **Power on the Simionic Bezel**: Turn on your SHB1000 hardware.
       * Ensure it is not actively connected to an iPad or an other Bluetooth device (the bezel can only handle one connection).
   1. **Run this Bridge App**: Double-click *SHB1000_MSFS2024_BLE_Bridge.exe*
       * Note: Windows Defender SmartScreen might show a "Windows protected your PC" popup because the .exe is new and unsigned. Click More info -> **Run anyway**.
       * A black command shell window will open, indicating it is scanning for the Simionic SHB1000 device.
1. **Connect to the Simulator**
    1. Launch Microsoft Flight Simulator.
    1. Once the simulator reaches the main menu, *SHB1000_MSFS2024_BLE_Bridge.exe* will automatically connect to SimConnect and the WASM interface.
    1. Load into a flight with a Garmin G1000 in the cockpit (e.g. a Cesna 172 Skyhawk).\
       Test the connection by turning an encoder (e.g. the ALT) or pressing a button (e.g. the center CDI) on the Simionic bezel; the corresponding action will execute in the virtual cockpit.

## Usage / Slim-Doc

1. **Recognizing your Bezel**
    * **First Time**\
      When you run the app for the first time, it will scan for devices with *"SHB1000"* in their bluetooth device name.\
      If a device gets found, it will prompt you how you would like to connect the device:
      "**(P)FD, (M)FD, R(A)dio, S(K)IP?**"\
      In case there are several devices within range, you will get prompted for each one, so you can make one your PFD and the second one your MFD.\
      To facilitate identifying the physical device you are prompted for the relevant device will blink its backlight a couple of times.
    * **Next Time**\
      Next time the scan process will be quicker as the app has saved the MAC address(es) of you bezel(s) and what role (PFD, MFD) you connected it as.\
      It will use the last known role as the default so you only need to hit **[Enter]** at the prompt.
1. **Brightness of Backlight**
    * To adjust the brightness level of the bezel backlight, simply\
      Press **[1]** for **Low** ... **[4]** for **High**, and **[0]** or **[^]** (left of the 1) for **OFF**.
1. **Reconnect**
    * The app every so often checks if it is still connected to a) the simulator and b) the bezel.\
      Should either connection have gone lost in thin air, it automatically tries to reconnect.\
      Usually, you won't even notice the brief disconnect.\
      If you do, howeveer, you can also trigger the reconnect by hitting the **[R]** key on the app console.\
      **Note**: This needs to happen on the app console, NOT in the sim cockpit.\
      So you need to e.g. **[Alt]+[Tab]** out of the sim over the the bridg app real quick, hit **[R]** and tab back into the cockpit.
1. **Garmin Profiles**
    * **Determin Model**\
      The app will upon every start-up determin, which model airplane you are already sitting in and try to load the corresponding Garmin command map.\
      Should you not yet be in a cockpit, you can always trigger this manually by hitting the **[?]** key on the app console window. The app will then tell you the currently active model variant aircraft.\
      **Note**: From MSFS and the bridg app point of view, a passenger version Cesna 172 Skyhawk is different from a skydive, or cargo.
    * **Command Maps**\
      Depending of the plane version, different command maps are used to account for the Garmin model contained in the sim.\
      Currently, there are two command maps enclosed in the release.
      1. Standard Garmin G1000 (e.g. in C172 Skyhawks)
      2. G1000NXi with separate GMC 710 (e.g. in C208 Grand Caravan)
    * **Reload Command Map**\
      If for some reason you feel the Garmin on your desk does not properly steer the Garmin in the sim, you might want to reload the Garmin Command Map once more.\
      Hit **[M]** on the app consol to trigger a reload of the configured map. This will, incidentally, also trigger a recheck which aircraft you are in. So if, for example, you flew the last mission in a Cesna 172 Skyhawk and are now in the cockpit of a C208 Grand Caravan, simply **[Alt]+[Tab]** over to the bridg app, hit the M key and **[Alt]+[Tab]** back into the cockpit.
1. **Swap Roles**
    * If you have two SHB bezels, one to use as your PFD, the other as MFD and you just happened to place them on your desk in the other order (left/right) you can simply swap their roles by hitting the **[S]** key.\
    The PFD then becomes MFD and vice versa.\
    You can also apply this trick if you only have one bezel.\
    Hitting the **[S]** key will allow you to steer the cockpit's MFD using the same bezel. Don't forget to swap back to bring it back into PFD mode...\
    Needless to say, this only swaps the button input, but NOT what you see on a screen you might have plugged into the bezel...
1. **Quit and Autostart**
    * **Ending the Bridge**\
      When you are done flying for the day and like to quit the bridge app as well, simply hit **[Q]** on its consol. The app like then gracefully shut down.\
      Should this for some reason not work, you obviously can always kill the process in the task manager. Simply look for the EXE's name under Tasks and (right click) end the task.
    * **Autostart**\
      You can also make the bridge app start automatically when you start MSFS.\
      How exactly this is done would exceed the scope of this documentation. But it's not rocket science. Simply ask Gemini in "AI Mode" "How can I make my G1000 bridge app start automatically when I start MSFS2024?". They will tell you exactly how to do it. And if you like, you can do the same for ending it. Ask Gemini how to do it and tell them there is a funktion that gracefully shuts down the app when hitting **[Q]**.
1. **For PROs - Working on the Config Files**
   This app comes preconfigured and deliberately takes care of many things as a "learning" from your actions. But it is also open for tweaking its behavior by directly amending the config files.\
   If you feel so enclined, here is what you can do (might want to copy/paste the files you used so far as a backup somewhere in case you want to roll-back your changes):
    * **Base Configuration**\
      The base configuration of the bridge app is done in the file "**SHB1000.config**". This is the only filename that is hard coded into the app and therefore needs to have specifically this name.\
      There are 4 sections in this JSON format file:
      1. "**general**"
          It defines the highest level behavior of the app.\
          How long until the scan for bluetooth devices times out.\
          What is the file called, that containes the log of the last connected devices for easier connection the next time around.\
          Which command map should be the default one to use.      
      1. "**devices**"
      It defines configuration(s) for different on desk sim devices.\
      Currently, it only holds the config for the SHB1000 Garmin devices from Simionic. But you can also add e.g. their radio device or who knows what else might work.
          Section name ("SHB1000") is also (part of the) bluetooth name the app will scan for and try to connect if configured in this file.\
          The UUID of the bluetooth service data is exchanged on.\
          The UUID of the bluetooth characteristic data is exchanged in.\
          The UUID of the bluetooth characteristic one can set the brightness of the backlight.\
          The default brightness of the backlight.\
          How quickly signals from a rotary encoder need to come after another (e.g. on the HDG knob) in milliseconds to qualify for a "fast" turn.\
          How many increments this shall equal on the sim knob.\
          How quickly signals need to come (in milliseconds) to qualify for a "very fast" turn.\
          How many increments THIS shall equal on the sim knob.\
          How long a pause needs to be (in milliseconds) so an input is interpreted as a new turn signal.
      1. "**maps**"
      This section is a list of configurations for different Garmin device command maps.\
      Currently, there are two maps supplied with the project. For a standard Garmin G1000 as it is e.g. built into the Cesna C172 Skyhawks of MSFS2024, and the newer version G1000 NXi with a cenrtal GMC 710 panel as it can be found in the sim of the Cessna 208 Grand Caravan.\
          The user friendly name of the command map as it will be displayed on the screen.\
          The physical file name of the command map on the hard disk.\
          The logical ID for internal handling in the app.
       1. "**aircraft_history**"
      This section logs which type and variant aircraft has been flown while running the bridge app and which command map has been used.\
      If a model/variant is flownd for the first time but should use a different map than the default one, this currently still needs to be reconfigured manually here.\
          The section name is the aircraft model and variant as it comes automatically via the bridge from MSFS.\
          The logical map ID of the command map to be used in this model aircraft.\
          The timestamp when an aircraft of this type was last used with the bridge app.   
    * **Known Devices**\
      "**SHB1000_known_devices.config**" holds information regarding which physical bluetooth sim devices have been used by the bridge app.\
      The shall make the scan and connect easier as the app can look for known devices you own rather than needing to perform a general scan all the time.\
      There will be one entry for each device ever connected to your bridge app.\
      The file comes preconfigured with 3 datasets for the three most intuitive roles: One each for PFD, MFD, and RADIO (as Simionic also offers the later).
      1. "**devices**"
          There is an attribute holding the MAC address of the device you connected via the bridge.\
          What type the device was (usually I guess it will be an SHB1000 G1000 bezel - but could also be the radio - or who knows what else).\
          What instrument it was used for (PFD, MFD etc.).\
          Whether it was positioned on the left or right side. Important if you have two bezels and use one as PFD, the other as MFD and you sometimes fly as captain and some times as first officer.\
          The timestamp when the device was last connected.\
          Whether you would like the bridge app to automatically reconnect it in case it ever goes missing (true I guess in most cases).


## Background

The Simionic G1000 bezels normally work in two modes:
- **iPad mode**: Bezel sends inputs to iPad via BLE, iPad displays G1000 and relays to MSFS.\
  VR users experience noticeable latency and not all functionality is supported by the Simionic app.
- **LCD mode**: Special Simionic screen with USB serial.\
  Extra hardware cost and shipping one piece around the world from China.

This project enables a **third option**: The bezels sends Bluetooth BLE input signals directly to this bridge app, which updates MSFS via SimConnect/HVars with minimal latency.\
Of course you can still stick an iPad with the *DeskSpace* display export app installed, or a regular 10'' portable monitor into the bezel for maximum emergence.

## Kudos
Kudos to [GregWoods](https://github.com/GregWoods) for the original idea and project.\
You find it linked above as the root of this fork.
