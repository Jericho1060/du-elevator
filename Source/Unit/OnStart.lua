--[[
    DU-ELEVATOR by Jericho
]]

__DEBUG = false --export: Debug mode, will print more information in the console

--[[
    Version Management
]]

local version = "V 1.0.0"
local log_split = "================================================="
--printing version in lua chat
system.print(log_split)local a=""local b=math.ceil((50-#version-2)/2)for c=1,b,1 do a=a..'='end;a=a.." "..version.." "for c=1,b,1 do a=a..'='end;system.print(a)system.print(log_split)

--[[
    Detecting elements linked
]]
core = nil
fuelTanks = {}

for slot_name, slot in pairs(unit) do
    if
        type(slot) == "table"
        and type(slot.export) == "table"
        and slot.getClass
    then
        local class = slot.getClass():lower()
        if class:find("coreunit") then
            core = slot
        elseif class:find("fuelcontainer") then
            table.insert(fuelTanks, slot)
        end
    end
end

--[[
    stopping the script if required elements are not linked
]]
if core == nil then
    system.print('Core unit is not linked, exiting the script')
    unit.exit()
end

--[[
    Default Unit Start Mechanics
]]
pitchInput = 0
pitchInputFromDevice = 0
rollInput = 0
yawInput = 0
verticalStrafeInput = 0
lateralStrafeInput = 0
brakeInput = 1 --braking by default
goingBack = false
goingForward = false
shiftPressed = false
jumpDelta = 0
baseAcceleration = 0.8

Nav = Navigator.new(system, core, unit)
Nav.axisCommandManager:setupCustomTargetSpeedRanges(axisCommandId.longitudinal, {100, 500, 1000, 2000,3000, 5000})
Nav.axisCommandManager:setTargetGroundAltitude(0)
unit.deactivateGroundEngineAltitudeStabilization()

-- Parenting widget
parentingPanelId = system.createWidgetPanel("Docking")
parentingWidgetId = system.createWidget(parentingPanelId,"parenting")
system.addDataToWidget(unit.getWidgetDataId(),parentingWidgetId)

-- Fuel widget generation (Updated code By Jericho)
--TODO: replace by custom HUD
if (#fuelTanks > 0) then
    fuelTankPanelId = system.createWidgetPanel("Fuel Tanks")
    fuelTankWidgetId = system.createWidget(fuelTankPanelId,"fuel_container")
    for _,tank in pairs(fuelTanks) do
        system.addDataToWidget(tank.getWidgetDataId(),fuelTankWidgetId)
    end
end

--[[
    Storing base position and orientation of the elevator
]]
ConstructInitPos = construct.getWorldPosition()
--replace that pos by a hand written value to be sure the construct will always realign the same position
--ConstructInitPos = {-1231538.185042, 1201297.6879598, -2617220.2766464}
if __DEBUG then system.print('ConstructInitPos: ' .. json.encode(ConstructInitPos)) end
BaseForward = vec3(construct.getWorldForward())
if __DEBUG then system.print('BaseForward: ' .. json.encode(BaseForward)) end
BaseAltitute = 285 --export: the start altitude of the elevator (lower position altitude)

--TODO: to replace with a computing in flush from the current acceleration and the friction acceleration (construct.getWorldAirFrictionAcceleration)
MaxSpeed = construct.getFrictionBurnSpeed() -- for security to avoid burning if going too fast
if __DEBUG then system.print('MaxSpeed: ' .. MaxSpeed) end

--init a value to store the target altitude
TargetAltitude = core.getAltitude() --by default to the start altitude to avoid falling down if in the air

--[[
    Kinematics functions by Jaylebreak
    Source available at https://gitlab.com/JayleBreak/dualuniverse/-/blob/master/DUflightfiles/autoconf/custom/kinematics.lua
]] 
function computeAccelerationTime(initial, acceleration, final) return (final - initial)/acceleration end
function computeDistanceAndTime(initial, final, mass, thrust, t50, brakeThrust)
    t50 = t50 or 0
    brakeThrust = brakeThrust or 0
    local speedUp = initial < final
    local a0 = thrust / (speedUp and mass or -mass)
    local b0 = -brakeThrust/mass
    local totA = a0+b0
    if initial == final then
        return 0, 0
    elseif speedUp and totA <= 0 or not speedUp and totA >= 0 then
        return -1, -1
    end
    local distanceToMax, timeToMax = 0, 0
    if a0 ~= 0 and t50 > 0 then
        local c1  = math.pi/t50/2
        local v = function(t)
            return a0*(t/2 - t50*math.sin(c1*t)/math.pi) + b0*t + initial
        end
        local speedchk = speedUp and function(s) return s >= final end or function(s) return s <= final end
        timeToMax  = 2*t50
        if speedchk(v(timeToMax)) then
            local lasttime = 0
            while math.abs(timeToMax - lasttime) > 0.25 do
                local t = (timeToMax + lasttime)/2
                if speedchk(v(t)) then
                    timeToMax = t 
                else
                    lasttime = t
                end
            end
        end
        local K = 2*a0*t50^2/math.pi^2
        distanceToMax = K*(math.cos(c1*timeToMax) - 1) + (a0+2*b0)*timeToMax^2/4 + initial*timeToMax
        if timeToMax < 2*t50 then
            return distanceToMax, timeToMax
        end
        initial = v(timeToMax)
    end
    local a = a0+b0
    local t = computeAccelerationTime(initial, a, final)
    local d = initial*t + a*t*t/2
    return distanceToMax+d, timeToMax+t
end

--[[
    Commpute angle betwwen vectors by Jericho
]]
function signedAngleBetween(vec1, vec2, planeNormal)
    local normVec1 = vec1:project_on_plane(planeNormal):normalize()
    local normVec2 = vec2:normalize()
    local v1v2dot = normVec1:dot(normVec2)
    local angle = math.acos(utils.clamp(v1v2dot,-1,1))
    local crossProduct = vec1:cross(vec2)
    if crossProduct:dot(planeNormal) < 0 then
        return -angle
    end
    return angle
end

--[[
    DU-LUA-Framework by Jericho
    Permit to code easier by grouping most of the code in a single event Unit > Start
    Unminified Source available here: https://github.com/Jericho1060/du-lua-framework
]]--
local a=system.print;local b=error;local c=pcall;local d=assert;local e=coroutine;local f=e.create;local g=e.status;local h=e.resume;local i="dead"local j="suspended"function runFunction(k,l,...)local m,n=c(k,...)if not m then b(l..n)end end;local o={__index={cos={update={},flush={}},fns={update={},flush={},action={start={},stop={},loop={}},inputText=nil,start=nil,stop=nil},ACTIONS={FORWARD="forward",BACKWARD="backward",YAW_LEFT="yawleft",YAW_RIGHT="yawright",STRAFE_LEFT="strafeleft",STRAFE_RIGHT="straferight",LEFT="left",RIGHT="right",UP="up",DOWN="down",GROUND_ALTITUDE_UP="groundaltitudeup",GROUND_ALTITUDE_DOWN="groundaltitudedown",LEFT_ALT="lalt",LEFT_SHIFT="lshift",GEAR="gear",LIGHT="light",BRAKE="brake",OPTION_1="option1",OPTION_2="option2",OPTION_3="option3",OPTION_4="option4",OPTION_5="option5",OPTION_6="option6",OPTION_7="option7",OPTION_8="option8",OPTION_9="option9",OPTION_10="option10",OPTION_11="option11",OPTION_12="option12",OPTION_13="option13",OPTION_14="option14",OPTION_15="option15",OPTION_16="option16",OPTION_17="option17",OPTION_18="option18",OPTION_19="option19",OPTION_20="option20",OPTION_21="option21",OPTION_22="option22",OPTION_23="option23",OPTION_24="option24",OPTION_25="option25",OPTION_26="option26",OPTION_27="option27",OPTION_28="option28",OPTION_29="option29",LEFT_MOUSE="leftmouse",STOP_ENGINES="stopengines",SPEED_UP="speedup",SPEED_DOWN="speeddown",ANTIGRAVITY="antigravity",BOOSTER="booster"},main={update=f(function()end),flush=f(function()end)},update=function(self)local p=g(self.main.update)if p==i then self.main.update=f(function()self:runUpdate()end)elseif p==j then d(h(self.main.update))end end,flush=function(self)local p=g(self.main.flush)if p==i then self.main.flush=f(function()self:runFlush()end)elseif p==j then d(h(self.main.flush))end end,action=function(self,q,r)if self.fns.action[q][r]then runFunction(self.fns.action[q][r],"System Action "..q.." Error: ")end end,actionStart=function(self,r)self:action('start',r)end,actionStop=function(self,r)self:action('stop',r)end,actionLoop=function(self,r)self:action('loop',r)end,inputText=function(self,s)if self.fns.inputText then runFunction(self.fns.inputText,"System Input Text Error: ",s)end end,runUpdate=function(self)for t,u in pairs(self.cos.update)do local m=g(u)if m==i then self.cos.update[t]=f(self.fns.update[t])elseif m==j then d(h(u))end end end,runFlush=function(self)for t,u in pairs(self.cos.flush)do local m=g(u)if m==i then self.cos.flush[t]=f(self.fns.flush[t])elseif m==j then d(h(u))end end end,onUpdate=function(self,v)for t,w in pairs(v)do self.fns.update[t]=w;self.cos.update[t]=f(w)end end,onFlush=function(self,v)for t,w in pairs(v)do self.fns.flush[t]=w;self.cos.flush[t]=f(w)end end,onAction=function(self,q,v)for t,w in pairs(v)do self.fns.action[q][t]=w end end,onActionStart=function(self,v)self:onAction("start",v)end,onActionStop=function(self,v)self:onAction("stop",v)end,onActionLoop=function(self,v)self:onAction("loop",v)end,onInputText=function(self,k)self.fns.inputText=k end}}local x={__index={timers={},stopFn=function()end,timer=function(self,y)if self.timers[y]then runFunction(self.timers[y],"Unit Timer "..y.." Error: ")end end,setTimer=function(self,y,z,k)self.timers[y]=k;unit.setTimer(y,z)end,stopTimer=function(self,y)unit.stopTimer(y)self.timers[y]=nil end,onStop=function(self,k)self.stopFn=k end,stop=function(self)if self.stopFn then runFunction(self.stopFn,"Unit Stop Error: ")end end}}local A={__index={parentChangedFn=function(B,C)end,onParentChange=function(self,k)self.parentChangedFn=k end,parentChanged=function(self,B,C)if self.parentChangedFn then runFunction(self.parentChangedFn,"Player Parent Changed Error: ",B,C)end end}}local D={__index={dockedFn=function(E)end,onDocked=function(self,k)self.dockedFn=k end,docked=function(self,E)if self.dockedFn then runFunction(self.dockedFn,"Construct Docked Error: ",E)end end,undockedFn=function(E)end,onUndocked=function(self,k)self.undockedFn=k end,undocked=function(self,E)if self.undockedFn then runFunction(self.undockedFn,"Construct Undocked Error: ",E)end end,playerBoardedFn=function(E)end,onPlayerBoarded=function(self,k)self.playerBoardedFn=k end,playerBoarded=function(self,E)if self.playerBoardedFn then runFunction(self.playerBoardedFn,"Construct Player Boarded Error: ",E)end end,VRStationEnteredFn=function(E)end,onVRStationEntered=function(self,k)self.VRStationEnteredFn=k end,VRStationEntered=function(self,E)if self.VRStationEnteredFn then runFunction(self.VRStationEnteredFn,"Construct VR Station Entered Error: ",E)end end,constructDockedFn=function(E)end,onConstructDocked=function(self,k)self.constructDockedFn=k end,constructDocked=function(self,E)if self.constructDockedFn then runFunction(self.constructDockedFn,"Construct Construct Docked Error: ",E)end end,PvPTimerFn=function(F)end,onPvPTimer=function(self,k)self.PvPTimerFn=k end,PvPTimer=function(self,F)if self.PvPTimerFn then runFunction(self.PvPTimerFn,"Construct PvP Timer Error: ",F)end end}}DU_Framework={__index={system=setmetatable({},o),unit=setmetatable({},x),player=setmetatable({},A),construct=setmetatable({},D)}}

Script = {}
setmetatable(Script, DU_Framework)

--[[
    String Helpers For Lua By Jericho
]]
String = {
    __index = {
        split = function(self, delimiter)
            local result = {}
            for match in (self..delimiter):gmatch("(.-)"..delimiter) do
                table.insert(result, match)
            end
            return result
        end
    }
}
string = setmetatable(string, String)
string.__index = string

local systemOnUpdate = {
    function ()
        Nav:update()
    end
}

local systemOnFlush = {
    function()
        local yawSpeedFactor = 1.5
        local yawAccelerationFactor = 3
        local lateralAntiDriftFactor = 1
        local lateralStrafeFactor = 5
        local brakeSpeedFactor = 1
        local brakeFlatFactor = 4 
        local autoBrakeSpeed = 15
        -- validate params
        brakeSpeedFactor = math.max(brakeSpeedFactor, 0.01)
        brakeFlatFactor = math.max(brakeFlatFactor, 0.01)
        yawSpeedFactor = math.max(yawSpeedFactor, 0.01)
        yawAccelerationFactor =  math.max(yawAccelerationFactor, 0.01)
        --init all the PIDs as global if first flush
        if (rollPID == nil) then rollPID = pid.new(0.2, 0, 10) end
        if (pitchPID == nil) then pitchPID = pid.new(0.2, 0, 10) end
        if (yawPID == nil) then yawPID = pid.new(0.2, 0, 10) end
        if (lateralPID == nil) then lateralPID = pid.new(0.2, 0, 10) end
        if (longitudinalPID == nil) then longitudinalPID = pid.new(0.2, 0, 10) end
        if (distancePID == nil) then distancePID = pid.new(0.2, 0, 10) end

        -- final inputs
        if unit.isMouseDirectControlActivated() then
            -- in direct control, we tweak the pitch to behave inbetween virtual joystick and direct control
            -- this helps a lot for ground construct control
            pitchInputFromDevice = utils.clamp(pitchInputFromDevice + system.getControlDeviceForwardInput() * system.getActionUpdateDeltaTime(), -1.0, 1.0)
        else
            pitchInputFromDevice = system.getControlDeviceForwardInput()
        end
        local finalPitchInput = pitchInput + pitchInputFromDevice
        local finalRollInput = rollInput + system.getControlDeviceYawInput()
        local finalYawInput = yawInput - system.getControlDeviceLeftRightInput()
        local combinedRollYawInput = utils.clamp(finalRollInput - finalYawInput, -1.0, 1.0);
        local finalVerticalStrafeInput = verticalStrafeInput
        local finalLateralStrafeInput = lateralStrafeInput;
        local finalBrakeInput = brakeInput

        -- Axis
        local worldVertical = vec3(core.getWorldVertical())
        local constructUp = vec3(construct.getWorldOrientationUp())
        local constructForward = vec3(construct.getWorldOrientationForward())
        local constructRight = vec3(construct.getWorldOrientationRight())
        local constructVelocity = vec3(construct.getWorldVelocity())
        local constructVelocityDir = vec3(construct.getWorldVelocity()):normalize()
        local constructAngularVelocity = vec3(construct.getWorldAngularVelocity())
        local constructYawVelocity = constructAngularVelocity:dot(constructUp)
        local constructWorldPosition = vec3(construct.getWorldPosition())
        local constructTargetPosition = vec3(ConstructInitPos)

        -- Engine commands
        local keepCollinearity = 0 -- for easier reading
        local dontKeepCollinearity = 1 -- for easier reading
        local tolerancePercentToSkipOtherPriorities = 1 -- if we are within this tolerance (in%), we don't go to the next priorities

        -- keeping the start position alignement
        local StrafeSpeedFactor = 50 --export: useg to increase the force of the alignement, decrease the value if the alignement is too strong or increase it if it's too slow
        local positionDifference = constructTargetPosition - constructWorldPosition
        local lateralOffset = positionDifference:project_on(constructRight):len() * utils.sign(positionDifference:dot(constructRight))
        local longitudinalOffset = positionDifference:project_on(constructForward):len() * utils.sign(positionDifference:dot(constructForward))
        lateralPID:inject(lateralOffset)
        Nav.axisCommandManager:setThrottleCommand(axisCommandId.lateral, lateralPID:get() * StrafeSpeedFactor)
        longitudinalPID:inject(longitudinalOffset)
        Nav.axisCommandManager:setThrottleCommand(axisCommandId.longitudinal, longitudinalPID:get() * StrafeSpeedFactor)

        -- Rotation
        local currentRollDeg = getRoll(worldVertical, constructForward, constructRight)
        local currentPitchDeg = -math.asin(constructForward:dot(worldVertical)) * constants.rad2deg
        local targetRollDeg = 0
        local targetPitchDeg = 0
        local targetYawDeg = signedAngleBetween(BaseForward,constructForward,constructUp)*180/math.pi
        rollPID:inject(targetRollDeg - currentRollDeg)
        pitchPID:inject(targetPitchDeg - currentPitchDeg)
        yawPID:inject(-targetYawDeg*yawSpeedFactor)

        local constructYawTargetVelocity = -combinedRollYawInput * yawSpeedFactor
        local constructYawTargetAcceleration = yawAccelerationFactor * (constructYawTargetVelocity - constructYawVelocity)
        local constructTargetAngularVelocity = rollPID:get() * constructForward + pitchPID:get() * constructRight + constructYawTargetAcceleration * constructUp

        Nav:setEngineTorqueCommand('torque', constructTargetAngularVelocity, keepCollinearity, 'airfoil', '', '', tolerancePercentToSkipOtherPriorities)

        -- moving up or down from TargetAltitude
        local verticalSpeed = constructVelocity:project_on(worldVertical):len()
        local verticalSpeedSigned = verticalSpeed * -utils.sign(constructVelocity:dot(worldVertical))
        local brakeDistance = 0
        local maxBrake = construct.getMaxBrake()
        if maxBrake ~= nil then
            brakeDistance, _ = computeDistanceAndTime(verticalSpeed, 0, construct.getInertialMass(), 0, 0, maxBrake - (core.getGravityIntensity() * construct.getInertialMass()) * utils.sign(verticalSpeedSigned))
        end
        local coreAltitude = core.getAltitude()
        local distance = TargetAltitude - coreAltitude
        local targetDistance = utils.sign(distance) * (math.abs(distance)-brakeDistance)
        distancePID:inject(targetDistance)
        if distancePID:get() > 0.5 or verticalSpeedSigned < -MaxSpeed then --using a 0.5 meter deadband for stabilizing
            Nav.axisCommandManager:setThrottleCommand(axisCommandId.vertical, 1)
        elseif distancePID:get() < -0.5 or verticalSpeedSigned > MaxSpeed then
            Nav.axisCommandManager:setThrottleCommand(axisCommandId.vertical, -1)
        else
            Nav.axisCommandManager:resetCommand(axisCommandId.vertical)
        end

        --Brakes
        if
            (math.abs(distance) < verticalSpeed)
            or verticalSpeed > MaxSpeed
            or (brakeDistance > math.abs(distance))
            or (finalBrakeInput == 0 and autoBrakeSpeed > 0 and Nav.axisCommandManager.throttle == 0 and constructVelocity:len() < autoBrakeSpeed)
        then
            finalBrakeInput = 1
        end
        local brakeAcceleration = -finalBrakeInput * (brakeSpeedFactor * constructVelocity + brakeFlatFactor * constructVelocityDir)
        
        Nav:setEngineForceCommand('brake', brakeAcceleration)

        -- AutoNavigation regroups all the axis command by 'TargetSpeed'
        local autoNavigationEngineTags = ''
        local autoNavigationAcceleration = vec3()
        local autoNavigationUseBrake = false

        -- Longitudinal Translation
        local longitudinalEngineTags = 'thrust analog longitudinal'
        local longitudinalCommandType = Nav.axisCommandManager:getAxisCommandType(axisCommandId.longitudinal)
        if (longitudinalCommandType == axisCommandType.byThrottle) then
            local longitudinalAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromThrottle(longitudinalEngineTags,axisCommandId.longitudinal)
            Nav:setEngineForceCommand(longitudinalEngineTags, longitudinalAcceleration, keepCollinearity)
        elseif  (longitudinalCommandType == axisCommandType.byTargetSpeed) then
            local longitudinalAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromTargetSpeed(axisCommandId.longitudinal)
            autoNavigationEngineTags = autoNavigationEngineTags .. ' , ' .. longitudinalEngineTags
            autoNavigationAcceleration = autoNavigationAcceleration + longitudinalAcceleration
            if (Nav.axisCommandManager:getTargetSpeed(axisCommandId.longitudinal) == 0 or -- we want to stop
                Nav.axisCommandManager:getCurrentToTargetDeltaSpeed(axisCommandId.longitudinal) < - Nav.axisCommandManager:getTargetSpeedCurrentStep(axisCommandId.longitudinal) * 0.5) -- if the longitudinal velocity would need some braking
            then
                autoNavigationUseBrake = true
            end
        end

        -- Lateral Translation
        local lateralStrafeEngineTags = 'thrust analog lateral'
        local lateralCommandType = Nav.axisCommandManager:getAxisCommandType(axisCommandId.lateral)
        if (lateralCommandType == axisCommandType.byThrottle) then
            local lateralStrafeAcceleration =  Nav.axisCommandManager:composeAxisAccelerationFromThrottle(lateralStrafeEngineTags,axisCommandId.lateral)
            Nav:setEngineForceCommand(lateralStrafeEngineTags, lateralStrafeAcceleration, keepCollinearity)
        elseif  (lateralCommandType == axisCommandType.byTargetSpeed) then
            local lateralAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromTargetSpeed(axisCommandId.lateral)
            autoNavigationEngineTags = autoNavigationEngineTags .. ' , ' .. lateralStrafeEngineTags
            autoNavigationAcceleration = autoNavigationAcceleration + lateralAcceleration
        end

        -- Vertical Translation
        local verticalStrafeEngineTags = 'thrust analog vertical'
        local verticalCommandType = Nav.axisCommandManager:getAxisCommandType(axisCommandId.vertical)
        if (verticalCommandType == axisCommandType.byThrottle) then
            local verticalStrafeAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromThrottle(verticalStrafeEngineTags,axisCommandId.vertical)
            Nav:setEngineForceCommand(verticalStrafeEngineTags, verticalStrafeAcceleration, keepCollinearity, 'airfoil', 'ground', '', tolerancePercentToSkipOtherPriorities)
        elseif  (verticalCommandType == axisCommandType.byTargetSpeed) then
            local verticalAcceleration = Nav.axisCommandManager:composeAxisAccelerationFromTargetSpeed(axisCommandId.vertical)
            autoNavigationEngineTags = autoNavigationEngineTags .. ' , ' .. verticalStrafeEngineTags
            autoNavigationAcceleration = autoNavigationAcceleration + verticalAcceleration
        end

        -- Auto Navigation (Cruise Control)
        if (autoNavigationAcceleration:len() > constants.epsilon) then
            if (brakeInput ~= 0 or autoNavigationUseBrake or math.abs(constructVelocityDir:dot(constructForward)) < 0.95)  -- if the velocity is not properly aligned with the forward
            then
                autoNavigationEngineTags = autoNavigationEngineTags .. ', brake'
            end
            Nav:setEngineForceCommand(autoNavigationEngineTags, autoNavigationAcceleration, dontKeepCollinearity, '', '', '', tolerancePercentToSkipOtherPriorities)
        end

        -- Rockets
        Nav:setBoosterCommand('rocket_engine')
    end
}


Script.system:onUpdate(systemOnUpdate)
Script.system:onFlush(systemOnFlush)

--[[
    Chat Commands
]]

Script.system:onInputText(function (text)
    if text:lower():find('goto:') then
        TargetAltitude = tonumber(text:split(':')[2]) or BaseAltitute
        brakeInput = 0
        system.print('Target Altitude set to ' .. TargetAltitude .. 'm')
    else
        system.print('Unknown command')
    end
end)