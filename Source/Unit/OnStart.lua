--[[
    DU-ELEVATOR by Jericho
]]

--[[
    Bookmarks: you can store several altitudes here to navigate easily
]]
local Bookmarks = {
    { name = 'Start Point', altitude = 294 },
    { name = 'Hovering', altitude = 300 },
    { name = 'Floor 1', altitude = 350 },
    { name = 'Floor 2', altitude = 500 },
    { name = 'Space 1', altitude = 5000 },
    { name = 'Space 2', altitude = 6000 },
    { name = 'Space 3', altitude = 10000 },
    { name = 'Space 4', altitude = 20000 },
    { name = 'Space 5', altitude = 50000 },
    { name = 'Space 6', altitude = 200000 },
}

--[[
    LUA PARAMETERS
]]
__DEBUG = false --export: Debug mode, will print more information in the console
ShowParentingWidget = false --export: Show the parenting widget
ShowControlUnitWidget = false --export: show the default widget of the construct
DisplayAtlasData = true --export: show the Atlas widget with closest planet informations

--[[
    HUD Rendering
]]
function RENDER_HUD(ElevatorData)
    --[[
        That function is used for rendering the HUD,
        You can update it or replace all its content to customise it.
        You can also remove its content if you don't want a HUD.
        It's not affecting in any way how the elevator is working and it's just a display
        
        ElevatorData is an object containing values that are refresh all the time.
        ElevatorData.isBreaking - boolean - if the brakes a are enabled or not.
        ElevatorData.verticalSpeed - number - the absolute vertical speed of the elevator in m/s.
        ElevatorData.verticalSpeedSigned - number - the real vertical speed of the elevator in m/s. if <0 the elevator is falling.
        ElevatorData.lateralSpeed - numver - the absolute lateral speed of the elevator in m/s.
        ElevatorData.longitudinalSpeed - number - the absolute Longitudinal speed of the elevator in m/s.
        ElevatorData.coreAltitude - number - the altitude from sea level returned by the core in meters. (warning: this altitude in space when far from planets is 0)
        ElevatorData.altitude - number - computed altitude from the distance between the construct and the sea level of the closest planet. (not 0 when in space)
        ElevatorData.atmosphereDistance - number - computed distance from the construct to the atmosphere of the closest planet
        ElevatorData.atmosphereAltitude - number - computed altitude of the atmosphere of the closest planet.
        ElevatorData.atmoMaxSpeed - number - the maximum speed of the elevator in m/s when in atmosphere. (anti burn security)
        ElevatorData.currentMaxSpeed - number - the maximum speed of the elevator in m/s. (50km/h in space)
        ElevatorData.planetData - table - Atlas Data of the closest planet. See atlas in game file for the structure (Game Directory\data\lua\atlas.lua)
    ]]
    --locale for Translation
    local locale = system.getLocale()
    localeIndex = 1 --default to english
    if locale == 'fr-FR' then
        localeIndex = 2 --french index in atlas
    elseif locale == 'de-DE' then
        localeIndex = 3 --german index in atlas
    end
    --direction if the elevator from the speed
    local direction = 'Stabilizing'
    if ElevatorData.verticalSpeedSigned > 0 then
        direction = 'Up'
    elseif ElevatorData.verticalSpeedSigned < 0 then
        direction = 'Down'
    end
    --braking status text for hud
    local brakeStatus = 'Off'
    if ElevatorData.isBreaking then
        brakeStatus = 'On'
    end
    --html rendering
    local html = [[
        <style>
            * {font-size:1vh !important;}
            .hud {
                position: absolute;
                left: 5vh;
                top: 5vh;
                right: 5vh;
                display: flex;
                flex-direction: column;
                justify-content: left;
                align-items: left;
            }
            .widget_container {
                border: 2px solid orange;
                border-radius:.5vh;
                background-color: rgba(0, 0, 0, .5);
                display: flex;
                flex-direction: column;
                padding:.5vh;
                margin-top:1vh;
            }
            .widget_container div {
                display: flex;
                flex-direction: row;
                justify-content: space-between;
            }
            .widget_container div div {
                margin:.25vh;
            }
            .widget_container div div:first-child {
                text-transform: uppercase;
                font-weight: bold;
            }
            .selected {
                color: teal;
            }
            .movingto {
                color: green;
            }
            .atmo .gauge {
                background-color: #22d3ee;
            }
            .atmo .gauge_label {
                color: #155e75;
            }
            .space .gauge {
                background-color: #fbbf24;
            }
            .space .gauge_label {
                color: #92400e;
            }
            .rocket .gauge {
                background-color: #a78bfa;
            }
            .rocket .gauge_label {
                color: #5b21b6;
            }
            .gauge_container {
                display:block;
                width:100%;
                min-width:10vw;
                position: relative;
                border: 1px solid black;
                background-color: rgba(0, 0, 0, .75);
                height: 1.5vh;
            }
            .gauge {
                position: absolute;
                z-index:1;
                top:-.25vh;
                left:-.25vh;
                height:100%;
                margin:0;
                background-color: cyan;
            }
            .gauge_label {
                position:relative;
                display:block;
                z-index:10;
                margin:0;
                padding:0;
                width:100%;
                text-align:center;
            }
        </style>
        <div class="hud">
            <div class="widget_container">
                <div>
                    <div>Base Altitude</div><div>]] .. format_number(utils.round(BaseAltitute)) .. [[m</div>
                </div>
                <div>
                    <div>Target Altitude</div><div>]] .. format_number(utils.round(TargetAltitude)) .. [[m</div>
                </div>
                <div>
                    <div>Current Altitude</div><div>]] .. format_number(utils.round(ElevatorData.altitude)) .. [[m</div>
                </div>
                <div>
                    <div>Atmosphere Distance</div><div>]] .. format_number(utils.round(ElevatorData.atmosphereDistance)) .. [[m</div>
                </div>
                <div>
                    <div>Maximum Speed</div><div>]] .. format_number(math.abs(utils.round(ElevatorData.currentMaxSpeed*3.6))) .. [[km/h</div>
                </div>
                <div>
                    <div>Vertical Speed</div><div>]] .. format_number(math.abs(utils.round(ElevatorData.verticalSpeed*3.6))) .. [[km/h</div>
                </div>
                <div>
                    <div>Lateral Speed</div><div>]] .. format_number(math.abs(utils.round(ElevatorData.lateralSpeed*3.6))) .. [[km/h</div>
                </div>
                <div>
                    <div>Longitudinal Speed</div><div>]] .. format_number(math.abs(utils.round(ElevatorData.longitudinalSpeed*3.6))) .. [[km/h</div>
                </div>
                <div>
                    <div>Direction</div><div>]] .. ElevatorData.direction .. [[</div>
                </div>
                <div>
                    <div>Brake</div><div>]] .. brakeStatus .. [[</div>
                </div>
            </div>
    ]]
    if #fuelTanks > 0 then
        html = html .. '<div class="widget_container">'
        for _, tank in pairs(fuelTanks) do
            html = html .. '<div><div>' .. tank:getName() .. '</div><div>' .. tank:getTimeLeftString() .. '</div></div><div class="gauge_container ' .. tank:getFuelType() .. '"><div class="gauge" style="width:' .. utils.round(tank:getPercentFilled()) .. '%;"></div><div class="gauge_label"><div style="margin-left:auto;margin-right:auto;padding:0;margin-top:-.25vh;">' .. format_number(utils.round(tank:getPercentFilled())) .. '%</div></div></div>'
        end
        html = html .. '</div>'
    end
    if #Bookmarks > 0 then
        html = html .. '<div class="widget_container"><div><div style="text-align:center !important;border-bottom:1px solid black;width:100%;">Bookmarks</div></div>'
        for index, bookmark in ipairs(Bookmarks) do
            local class=''
            if selectedBookmarkIndex == index then
                class = 'selected'
            end
            local displayName = bookmark.name
            if selectedBookmarkIndex > 0 and bookmark.altitude == TargetAltitude then
                displayName = ' >> ' .. bookmark.name
                class = 'movingto'
            end
            html = html .. '<div class="' .. class .. '"><div>' .. displayName .. '</div><div>' .. format_number(bookmark.altitude) .. 'm</div></div>'
        end
        html = html .. '</div>'
    end
    if DisplayAtlasData and (ElevatorData.planetData ~= nil) then
        html = html .. '<div class="widget_container">'
        html = html .. '<div><div style="text-align:center;border-bottom:1px solid black;width:100%;">Atlas Data</div></div>'
        --html = html .. '<div><img src="/'..ElevatorData.planetData.iconPath..'" style="width:10vh;height:10vh;"></div>' --image is blinking in hud, you can display it here but by default i'm removing it
        html = html .. '<div><div>Planet Name</div><div>'..ElevatorData.planetData.name[localeIndex]..'</div></div>'
        html = html .. '<div><div>Planet Type</div><div>'..ElevatorData.planetData.type[localeIndex]..'</div></div>'
        html = html .. '<div><div>Planet gravity</div><div>'..ElevatorData.planetData.gravity..'m/s²</div></div>'
        html = html .. '<div><div>Planet Surface Average Altitude</div><div>'..ElevatorData.planetData.surfaceAverageAltitude..'m</div></div>'
        html = html .. '<div><div>Planet Surface Min Altitude</div><div>'..ElevatorData.planetData.surfaceMinAltitude..'m</div></div>'
        html = html .. '<div><div>Planet Surface Max Altitude</div><div>'..ElevatorData.planetData.surfaceMaxAltitude..'m</div></div>'
        html = html .. '<div><div>Planet Max Altitude For Satic Core</div><div>'..ElevatorData.planetData.maxStaticAltitude..'m</div></div>'
        html = html .. '</div>'
    end
    html = html .. '</div>'
    system.setScreen(html) --render the HUD
end

--[[
    Version Management
]]
local version = "V 1.3.3"
local log_split = "================================================="
--printing version in lua chat
system.print(log_split)local a=""local b=math.ceil((50-#version-2)/2)for c=1,b,1 do a=a..'='end;a=a.." "..version.." "for c=1,b,1 do a=a..'='end;system.print(a)system.print(log_split)
--[[
    Lua Serializer by Jericho
    Version: 1.0.0
    Source available at: https://github.com/Jericho1060/DualUniverse/blob/master/Serializer/Serializer.lua
]]
Serializer={__index={stringify=function(self,a)if type(a)=='number'then return self:stringifyNumber(a)elseif type(a)=='string'then return self:stringifyString(a)elseif type(a)=='table'then return self:stringifyTable(a)elseif type(a)=='boolean'then return self:stringifyBoolean(a)end;return nil end,parse=function(self,b)return load("return "..b)()end,stringifyTableKey=function(self,c)if type(c)=='number'then return'['..self:stringifyNumber(c)..']'end;return c end,stringifyNumber=function(self,d)return tostring(d)end,stringifyString=function(self,b)return string.format('%q',b):gsub('\\\n','\n'):gsub('\\\r','\r'):gsub('\\\t','\t')end,stringifyTable=function(self,e)local b='{'local f=1;local g=self:tableLength(e)for h,i in pairs(e)do b=b..self:stringifyTableKey(h)..'='..self:stringify(i)if f<g then b=b..','end;f=f+1 end;return b..'}'end,stringifyBoolean=function(self,j)return tostring(j)end,tableLength=function(self,e)local k=0;for l in pairs(e)do k=k+1 end;return k end}}
Serializer = setmetatable({}, Serializer)
--[[
    Fuel Tanks Metatable By Jericho
    Version: 1.0.0
    Unminified Source available at: https://github.com/Jericho1060/DualUniverse/blob/master/ElementsMetatable/FuelTank.lua
]]
FuelTank={__index={name='',percentage=0,timeLeft=0,lastRefresh=0,fuelType='rocket',refresh=function(self)local a=system.getUtcTime()if lastRefresh~=a then local b=self.getClass():lower()if b:find('atmo')then self.fuelType="atmo"elseif b:find('space')then self.fuelType="space"end;local c=self.getWidgetData()local d=json.decode(c)self.name=d.name;self.percentage=d.percentage;self.timeLeft=tonumber(d.timeLeft)self.lastRefresh=a end end,getName=function(self)self:refresh()return self.name end,getPercentFilled=function(self)self:refresh()return self.percentage end,getSecondsLeft=function(self)self:refresh()return self.timeLeft end,getFuelType=function(self)self:refresh()return self.fuelType end,getTimeLeftString=function(self)local e=self:getSecondsLeft()if e==nil or e<=0 then return"-"end;days=string.format("%2.f",math.floor(e/(3600*24)))hours=string.format("%2.f",math.floor(e/3600-days*24))mins=string.format("%2.f",math.floor(e/60-hours*60-days*24*60))secs=string.format("%2.f",math.floor(e-hours*3600-days*24*60*60-mins*60))str=""if tonumber(days)>0 then str=str..days.."d "end;if tonumber(hours)>0 then str=str..hours.."h "end;if tonumber(mins)>0 then str=str..mins.."m "end;if tonumber(secs)>0 then str=str..secs.."s"end;return str end,dump=function(self)self:refresh()system.print('{name="'..self.name..'",percentage='..self.percentage..',timeLeft='..self.timeLeft..',fuelType="'..self.fuelType..'"}')end}}
--[[
    Detecting elements linked
]]
core = nil
fuelTanks = {}
databank = nil
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
            fuelTanks[#fuelTanks + 1] = setmetatable(slot, FuelTank)
        elseif class:find("databank") then
            databank = slot
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
if databank == nil then
    system.print('Databank is not linked')
end

--[[
    Atlas
]]
atlas = require"atlas"

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
if ShowParentingWidget then
    parentingPanelId = system.createWidgetPanel("Docking")
    parentingWidgetId = system.createWidget(parentingPanelId,"parenting")
    system.addDataToWidget(unit.getWidgetDataId(),parentingWidgetId)
end
if not ShowControlUnitWidget then
    unit.hideWidget()
end

--[[
    Storing base position and orientation of the elevator
]]
function storeIniPosAndForward(force)
    if force == nil then force = false end
    ConstructInitPos = construct.getWorldPosition()
    --replace that pos by a hand written value to be sure the construct will always realign the same position
    --ConstructInitPos = {-1231461.6525868,1201334.4594833,-2617227.3718653}
    if __DEBUG then system.print('ConstructInitPos: ' .. json.encode(ConstructInitPos)) end
    BaseForward = construct.getWorldForward()
    --replace that pos by a hand written value to be sure the construct will always realign the same forward orientation
    --BaseForward = {-0.085554607212543,0.2445342447567,-0.9658600687807}
    if __DEBUG then system.print('BaseForward: ' .. json.encode(BaseForward)) end
    if databank ~= nil then
        if databank.hasKey('ConstructInitPos') and not force then
            ConstructInitPos = Serializer:parse(databank.getStringValue('ConstructInitPos'))
            system.print("ConstructInitPos loaded from databank")
        else
            local toStore = Serializer:stringify(ConstructInitPos)
            databank.setStringValue('ConstructInitPos', toStore)
            system.print("ConstructInitPos stored in databank")
            system.print(toStore)
        end
        if databank.hasKey('BaseForward') and not force then
            BaseForward = Serializer:parse(databank.getStringValue('BaseForward'))
            system.print("BaseForward loaded from databank")
        else
            local toStore = Serializer:stringify(BaseForward)
            databank.setStringValue('BaseForward', toStore)
            system.print("BaseForward stored in databank")
            system.print(toStore)
        end
    else
        system.print("No Databank detected")
    end
    BaseForward = vec3(BaseForward)
end

--[[
    Storing Planet ID in Databank to be sure we always have the origin planet store even where too far from planet (1su or more)
]]
function storeReferencePlanet(force)
    if force == nil then force = false end
    local PlanetID = nil
    if databank.hasKey('PlanetID') and not force then
        PlanetID = databank.getIntValue('PlanetID')
    else
        PlanetID = core.getCurrentPlanetId()
        if PlanetID ~= nil then
            databank.setIntValue('PlanetID', PlanetID)
            system.print("Planet ID stored in databank: " .. PlanetID)
        end
    end
    if PlanetID == nil then
        system.print("WARNING: no planet detected, you may be too far away in spance, please, restart the elevator from the planet surface")
    else
        ElevatorData.planetData = atlas[0][PlanetID]
    end
    if __DEBUG then system.print('Planet ID: ' .. PlanetID) end
end

--[[
    Compute planet atmosphere altitude
]]
function computePlanetAtmoAltitude(planetData)
    return planetData.atmosphereRadius - planetData.radius
end

--[[
    --compute distance from a planet sea level
]]
function computePlanetSeaDistance(planetData, constructWorldPosition)
    return (vec3(planetData.center) - vec3(constructWorldPosition)):len() - planetData.radius
end

ConstructInitPos = nil
BaseForward = nil
storeIniPosAndForward()
if ConstructInitPos == nil or BaseForward == nil then
    system.print("An Error occurs during Init. Try removing the databank, remove its dynamic properties and install it again ...")
end
BaseAltitute = Bookmarks[1].altitude --getting the base altitude from the 1st bookmark
--init a value to store the target altitude
TargetAltitude = core.getAltitude() --by default to the start altitude to avoid falling down if in the air

MaxSpeed = construct.getFrictionBurnSpeed() -- for security to avoid burning if going too fast
MaxSpeedOnPlanet = MaxSpeed
if __DEBUG then system.print('MaxSpeed: ' .. (MaxSpeed*3.6) .. 'km/h') end

ElevatorData = {
    isBreaking = brakeInput == 1,
    verticalSpeed = 0,
    verticalSpeedSigned = 0,
    lateralSpeed = 0,
    longitudinalSpeed = 0,
    coreAltitude = TargetAltitude,
    altitude = TargetAltitude,
    planetData = nil,
    direction = 'Stabilizing',
    atmosphereDistance = 0,
    atmosphereAltitude = 0,
    atmoMaxSpeed = MaxSpeedOnPlanet,
    currentMaxSpeed = MaxSpeed,
}

storeReferencePlanet(false)

--computing the distance from the planet sea level as target altitude
if ElevatorData.planetData ~= nil then
    TargetAltitude = computePlanetSeaDistance(ElevatorData.planetData, construct.getWorldPosition())
    ElevatorData.atmosphereAltitude = computePlanetAtmoAltitude(ElevatorData.planetData)
end

--base selected bookmark is 0 -> none selected as index 0 doesn't exist in lua
selectedBookmarkIndex = 0

--[[
    init the HUD
]]
system.showScreen(true)

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
    Compute angle between vectors by Jericho
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
	formatting numbers by adding a space between thousands by Jericho
]]
function format_number(a)local b=a;while true do b,k=string.gsub(b,"^(-?%d+)(%d%d%d)",'%1 %2')if k==0 then break end end;local c=string.sub(b,-2)if c=='.0'then b=string.sub(b,1,b:len()-2)end;return b end
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

local systemOnFlush = {
    function()
        local yawSpeedFactor = 0.1 --export: the auto yaw speed multiplier
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
        local worldRight = vec3(core.getWorldRight())
        local worldForward = vec3(core.getWorldForward())
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

        --fake braking value
        local canBrake = false

        -- keeping the start position alignement
        local StrafeSpeedFactor = 1 --export: useg to increase the force of the alignement, decrease the value if the alignement is too strong or increase it if it's too slow
        local positionDifference = constructTargetPosition - constructWorldPosition
        local lateralOffset = positionDifference:project_on(constructRight):len() * utils.sign(positionDifference:dot(constructRight))
        ElevatorData.lateralSpeed = constructVelocity:project_on(worldRight):len()
        local lateralDistance = math.abs(lateralOffset)
        local longitudinalOffset = positionDifference:project_on(constructForward):len() * utils.sign(positionDifference:dot(constructForward))
        ElevatorData.longitudinalSpeed = constructVelocity:project_on(worldForward):len()
        local longitudinalDistance = math.abs(longitudinalOffset)
        if (lateralDistance < .25) and (longitudinalDistance < .25) then canBrake = true end
        if ((ElevatorData.lateralSpeed*3.6) > lateralDistance) or ((ElevatorData.longitudinalSpeed*3.6) > longitudinalDistance) then canBrake = true end
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

        local constructTargetAngularVelocity = rollPID:get() * constructForward + pitchPID:get() * constructRight + yawPID:get() * constructUp

        Nav:setEngineTorqueCommand('torque', constructTargetAngularVelocity, keepCollinearity, 'airfoil', '', '', tolerancePercentToSkipOtherPriorities)

        -- moving up or down from TargetAltitude
        ElevatorData.verticalSpeed = constructVelocity:project_on(worldVertical):len()
        ElevatorData.verticalSpeedSigned = ElevatorData.verticalSpeed * -utils.sign(constructVelocity:dot(worldVertical))
        local brakeDistance = 0
        local brakeDistanceToAtmosphere = 0
        local maxBrake = construct.getMaxBrake()
        ElevatorData.coreAltitude = core.getAltitude()
        ElevatorData.atmosphereAltitude = computePlanetAtmoAltitude(ElevatorData.planetData)
        ElevatorData.altitude = computePlanetSeaDistance(ElevatorData.planetData, constructWorldPosition)
        ElevatorData.atmosphereDistance = ElevatorData.altitude - ElevatorData.atmosphereAltitude
        local isInAtmosphere = ElevatorData.atmosphereDistance <= 0
        local targetIsInAtmosphere = TargetAltitude <= ElevatorData.atmosphereAltitude
        if maxBrake ~= nil then
            local g = core.getGravityIntensity()
            if ElevatorData.verticalSpeedSigned < 0 then
                g = ElevatorData.planetData.gravity --when falling, apply the max gravity to the braking computing to increase braking
            end
            local inertialMass = construct.getInertialMass()
            local speedSign = utils.sign(ElevatorData.verticalSpeedSigned)
            brakeDistanceToAtmosphere, _ = computeDistanceAndTime(ElevatorData.verticalSpeed, ElevatorData.atmoMaxSpeed, inertialMass, 0, 0, maxBrake - (g * inertialMass * speedSign))
            brakeDistance, _ = computeDistanceAndTime(ElevatorData.verticalSpeed, 0, inertialMass, 0, 0, maxBrake - (g * inertialMass * speedSign))
        end
        if isInAtmosphere or (targetIsInAtmosphere and (brakeDistanceToAtmosphere > ElevatorData.atmosphereDistance)) then
            ElevatorData.currentMaxSpeed = ElevatorData.atmoMaxSpeed
        else
            ElevatorData.currentMaxSpeed = 50000/3.6 -- 50km/h in m/s = 13888.88888888889....
        end
        local distance = TargetAltitude - ElevatorData.altitude
        local targetDistance = utils.sign(distance) * (math.abs(distance)-brakeDistance)
        distancePID:inject(targetDistance)
        if distancePID:get() > 0.25 or ElevatorData.verticalSpeedSigned < -ElevatorData.currentMaxSpeed then --using a 0.25 meter deadband for stabilizing
            Nav.axisCommandManager:setThrottleCommand(axisCommandId.vertical, 1)
            ElevatorData.direction = 'up'
            if ElevatorData.verticalSpeedSigned < 0 then
                finalBrakeInput = 1
            end
        elseif distancePID:get() < -0.25 or ElevatorData.verticalSpeedSigned > ElevatorData.currentMaxSpeed then
            Nav.axisCommandManager:setThrottleCommand(axisCommandId.vertical, -1)
            ElevatorData.direction = 'down'
            if ElevatorData.verticalSpeedSigned > 0 then
                finalBrakeInput = 1
            end
        else
            Nav.axisCommandManager:resetCommand(axisCommandId.vertical)
            ElevatorData.direction = 'stabilizing'
        end

        --Brakes
        if (math.abs(distance) < (ElevatorData.verticalSpeed*3.6) and canBrake) --speed control
            or (ElevatorData.verticalSpeed >= ElevatorData.currentMaxSpeed) --anti burn speed
            or ((math.abs(distancePID:get()) < .25) and canBrake) --altitude reached with 0.25m deadband
            or ((brakeDistance > math.abs(distance)) and canBrake) --braking distance
            or (finalBrakeInput == 0 and autoBrakeSpeed > 0 and Nav.axisCommandManager.throttle == 0 and constructVelocity:len() < autoBrakeSpeed)
        then
            finalBrakeInput = 1
            --use engines to help braking
            if ElevatorData.verticalSpeedSigned < 0 then
                Nav.axisCommandManager:setThrottleCommand(axisCommandId.vertical, 1)
            elseif ElevatorData.verticalSpeedSigned > 0 then
                Nav.axisCommandManager:setThrottleCommand(axisCommandId.vertical, -1)
            end
        end
        ElevatorData.isBreaking = (finalBrakeInput == 1)
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

local systemOnUpdate = {
    function ()
        Nav:update()
        RENDER_HUD(ElevatorData)
    end
}

Script.system:onUpdate(systemOnUpdate)
Script.system:onFlush(systemOnFlush)

--[[
    Actions
]]
local systemActionsStart = {}

systemActionsStart[Script.system.ACTIONS.DOWN] =  function ()
    if selectedBookmarkIndex < #Bookmarks then
        selectedBookmarkIndex = selectedBookmarkIndex + 1
    else
        selectedBookmarkIndex = 1
    end
    if __DEBUG then system.print('Selected Bookmark Index: ' .. selectedBookmarkIndex) end
end
systemActionsStart[Script.system.ACTIONS.UP] =  function ()
    if selectedBookmarkIndex > 1 then
        selectedBookmarkIndex = selectedBookmarkIndex - 1
    else
        selectedBookmarkIndex = #Bookmarks
    end
    if __DEBUG then system.print('Selected Bookmark Index: ' .. selectedBookmarkIndex) end
end
systemActionsStart[Script.system.ACTIONS.STRAFE_RIGHT] =  function ()
    brakeInput = 0
    if selectedBookmarkIndex > 0 then
        TargetAltitude = Bookmarks[selectedBookmarkIndex].altitude
        if __DEBUG then system.print('Target Altitude: ' .. TargetAltitude) end
    else
        system.print('No bookmark selected')
    end
end
systemActionsStart[Script.system.ACTIONS.OPTION_1] = function()
    storeIniPosAndForward(true)
end
systemActionsStart[Script.system.ACTIONS.OPTION_2] = function()
    storeReferencePlanet(true)
end
Script.system:onActionStart(systemActionsStart) --loading all "actionStart" functions

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
