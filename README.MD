# DU-ELEVATOR

A Generic Elevator script for Dual universe

# Guilded Server (better than Discord)

You can join me on Guilded for help or suggestions or requests by following that link : https://guilded.jericho.dev

# Discord Server

You can join me on Discord for help or suggestions or requests by following that link : https://discord.gg/qkdjyqDZQZ

# Edit the code

[![img](https://du-lua.dev/img/open_in_editor_button.png)](https://du-lua.dev/#/editor/github/Jericho1060/du-elevator)

# Instalation

### Important notice

- the script was only tested on a remote control

### Links

You must link the following elements to the remote control (order doesn't matter, the script will detect them automatically):
- Link the core to the remote control
- Link the fuel tank to the remote control (only if you want to see the fuel gauge)

### Setup

- you need at least a vertical engine pointing the bottom
- you need at least one engine on each horizontal direction (forward, right, left, backward)
- you need at least 3 adjustors on each angles of the construct : (see picture below)
    - one pointing up (this one is used for autoroll and autopitch)
    - one pointing on one side (this one is used for autoyaw)
    - one pointing on the orther side of the angle (this one is used for autoyaw)
    - ![Adustors Positioning](./images/adjustors.png)

You may need to adjust the power depending on the weight of your construct and on the total weight you want to lift.

### Installation of the script

- copy the content of the config.json file and paste it on the remote control (right click on the remote control -> advanced -> paste lua configuration from clipboard)

### Lua parameters

- `DEBUG` : if set to true, the script will print debug messages in the chat
- `BaseAltitude` : the default altitude of the elevator when landed on the floor
- `StrafeSpeedFactor` : the speed multiplier when strafing. adjust it depending on how many engines you have on the sides

### Usage

- open the "lua" chat channel and type `goto:<altitude>` to go to the specified altitude with `<altitude>` the altitude in meters, if the altitude is missing or invalid, it will send you the `BaseAltitude`

# Coming Soon

- Bookmark system: you will be able to name and store your favorite floors and go to them with a key press
- support for emitter to be able to send a signal to a specific channel to close a door to lock the elevator in position
- dedicated HUD with more informations and with some customizations possible (AR View of the position of the elevator, custom fuel gauge, etc...)

# Support or donation

if you like it, [<img src="https://github.com/Jericho1060/DU-Industry-HUD/blob/main/ressources/images/ko-fi.png?raw=true" width="150">](https://ko-fi.com/jericho1060)