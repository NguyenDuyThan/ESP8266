--start-----
---------------Channel ESP8266----------------
--duythan.n92@gmail.com
--chrome
channel_ID = 164239 --146004
write_API_key = "2WZ9DLMB3WFOGS0U" --"WZNVOYF704JPZGVY"
read_API_key = "PJTS6M51VK4IC3JD" --"ZAMBM7K2ZUD1MW6W"
---------------END channel-----------------
debug = false 
debugData = false
entry_id = 0
----------------check status---------------
require("newProfileWifiSetting")
if (debug) then
    print("SSID setup:"..SSID)
    print("Pass setup:"..pass)
end
package.loaded["newProfileWifiSetting"]=nil
 
require("statusOnNewStart")
if (debug) then
    if(resetMode) then
        print("resetmode True")
    else
        print("resetmode False")
    end
end
package.loaded["statusOnNewStart"]=nil
  
require("configWifiSetting")
tmr.alarm(6, 1000, 1, function()
        if wifi.ap.getip() == nil then
        else
            package.loaded["configWifiSetting"]=nil
            tmr.stop(6)
        end
end)

require("controlOutputDevice")
--------------------------------------------------
--------------------define inrf-------------------
--------------------------------------------------
onStatus = 0
local inRfPin1 = 7 --gpio13
local inRfPin2 = 6 --gpio
local inRfPin3 = 1 --gpio5
local inRfPin4 = 2 --gpio4
--buzzer = 11 
whitePin = 5 --gpio14
--yellow = 5
pwmDutyWhite = 10
--pwmdutyyellow = 500
pwmClock=1000

gpio.mode(inRfPin1, gpio.INT)
gpio.mode(inRfPin2, gpio.INT)
gpio.mode(inRfPin3, gpio.INT)
gpio.mode(inRfPin4, gpio.INT)
--gpio.mode(buzzer, gpio.OUTPUT)
pwm.setup(whitePin, pwmClock, pwmDutyWhite)
--pwm.setup(yellow, pwmclock, 500)
pwm.start(whitePin)
--pwm.start(yellow)
----------------------------------------------------
------------------------read------------------------
----------------------------------------------------
local function readDataFromServer()
if (true) then
    fields = {}
    createdTimes = {}
    readTS = require("readTS")
    readTS.setChannelID(channel_ID)
    readTS.setKey(read_API_key)
    readTS.readData(debug, 1, nil) -- args: filename containing writekey, readkey on two separate lines; show debug msg = true; number of results to return; optional callback file to run after completed request
    readTS = nil
    package.loaded["readTS"]=nil
    --tmr.delay(1000000)
    --print(fields[4])
    --print(createdTimes[2])
    read = false
end
end
-------------------------------------------------
---------------------write-----------------------
-------------------------------------------------
local function writeDataToServer(field1, field2, field3, field4, field5, field6, field7, field8)
if (true) then
    sendToTS = require('sendToTS')
    sendToTS.setKey(write_API_key)
    valSet = sendToTS.setValue(1,field1)
    valSet = sendToTS.setValue(2,field2)
    valSet = sendToTS.setValue(3,field3)
    valSet = sendToTS.setValue(4,field4)
    valSet = sendToTS.setValue(5,field5)
    valSet = sendToTS.setValue(6,field6)
    valSet = sendToTS.setValue(7,field7)
    valSet = sendToTS.setValue(8,field8) -- field number, data.  sendToTS returns a boolean, true if set successfully
    sendToTS.sendData(debug, nil) -- show debug msgs T/F, callback file to run when done
    sendToTS = nil
    package.loaded["sendToTS"]=nil -- these last two lines help free up memory
    write = false
end
end
-------------------------------------------------------
--------------------update data to server--------------
-------------------------------------------------------

--[[print(fields[5])
inbrightValueNew        =1--"in"
debrightValueNew        =0--"de"
onValueNew              =1--"on"
offValueNew             =0--"off"
timeInternetSetValueNew =15--"tmr"
timerSetValueNew        =10--"tmrin"
keyGetValueNew          =1234567--"key"
nothingValueNew         =00000--"no"
writeDataToServer(inbrightValueNew, debrightValueNew, onValueNew, offValueNew, timeInternetSetValueNew, timerSetValueNew, keyGetValueNew, nothingValueNew)

--]]----------------------------------------------
---------------------ir control-----------------
------------------------------------------------
do
    --gpio.mode(inrf1, gpio.INT)
    local function inrf1cb(level)
        inBrightFunCtr()
    end
    gpio.trig(inRfPin1, "down", inrf1cb)

    local function inrf2cb(level)
        deBrightFunCtr()
    end
    gpio.trig(inRfPin2, "down", inrf2cb)

    local function inrf3cb(level)
        onFunCtr()
    end
    gpio.trig(inRfPin3, "down", inrf3cb)

    local function inrf4cb(level)
        offFunCtr()
    end
    gpio.trig(inRfPin4, "down", inrf4cb)
end
-------------------------------------------
------------get data from server-----------
-------------------------------------------
tmr.alarm(2, 3000, tmr.ALARM_AUTO, function()
    if(wifi.sta.getip()==nil) then
        if(debug) then
            print("chua connect wifi")
        end
    else
        readDataFromServer()
        tmr.alarm(4, 1000, tmr.ALARM_AUTO, function()
            if(createdTimes[1] ==nil) then
                if(debug) then
                    print("dang lay du lieu tren server")
                end
            else
                if(createdTimes[1] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    timeUpdateCammandControl = createdTimes[1]
                    if(debugData) then
                        print(timeUpdateCammandControl)
                        print(entry_id)
                    end        
                end
                if(fields[1] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    inbrightValueGet    = fields[1]
                        if(debugData) then
                            print(inbrightValueGet)
                        end
                end
                if(fields[2] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    debrightValueGet    = fields[2]
                        if(debugData) then
                            print(debrightValueGet)
                        end
                end
                if(fields[3] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    onValueGet          = fields[3]
                        if(debugData) then
                            print(onValueGet)
                        end
                end
                if(fields[4] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    offValueGet         = fields[4]
                        if(debugData) then
                            print(offValueGet)
                        end 
                end
                if(fields[5] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    timerSetValue       = fields[5]
                        if(debugData) then
                            print(timerSetValue)
                        end
                end
                if(fields[6] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    timeInternetSetValue= fields[6]
                        if(debugData) then
                            print(timeInternetSetValue)
                        end
                end
                if(fields[7] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    keyGetValue         = fields[7]
                        if(debugData) then
                            print(keyGetValue)
                        end
                end
                if(fields[8] == nil) then
                    if(debug) then
                        print("not get data")
                    end
                else
                    nothingValue        = fields[8]
                        if(debugData) then
                            print(nothingValue)
                        end
                end  
                --tmr.stop(2)
                if(debug) then
                    print("get data ok")
                end
                debug = false
                if(entry_id_old == entry_id)then
                    if(debug)then
                        print("not change entry_id")
                    end
                else
---------------internetnet control--------------------
                    if(debug)then
                        print("new entry_id")
                        print(entry_id)
                    end
                    entry_id_old = entry_id
                    if(inbrightValueGet == "1")then
                        inBrightFunCtr()
                    end
                    if(debrightValueGet=="1")then
                        deBrightFunCtr()
                    end
                    if(onValueGet=="1")then
                        onFunCtr()
                    end
                    if(offValueGet=="1")then
                        offFunCtr()
                    end
-----------------------------------------------------                    
                end
            end
        end) 
    end    
end)
------------------------------------------------------
-------------------reset device-----------------------
------------------------------------------------------
local function resetfc()
            file.remove("statusOnNewStart.lc");
            file.open("statusOnNewStart.lc","w+");
            file.writeline("resetMode = false")
            file.writeline("restoreDefault = false")
            file.close();
            --dofile("analyzedistinguish.lua")
            node.restart();
end  
local function ctnwffc() 
                --stringssidtemp="thanzxcvbnmduy"
                stringssidtemp2=stringssidtemp:gsub("zxcvbnm", " ")
                stringssid = string.format("%q", stringssidtemp2)
                stringpass = string.format("%q", stringpasstemp) 
                --print(stringssid)
                file.remove("nwf.lua");
                file.open("nwf.lua","w+");
                file.writeline("SSID = "..stringssid)
                file.writeline("pass = "..stringpass)
                file.close();
                file.remove("status.lua");
                file.open("status.lua","w+");
                file.writeline("resetMode = true")
                file.writeline("restoreDefault = true")
                file.close();
                --dofile("analyzedistinguish.lua")
                node.restart();
end
if (restoreDefault)then
    file.remove("status.lua");
    file.open("status.lua","w+");
    file.writeline("resetMode =".. resetmode)
    file.writeline("restoreDefault = false")
    file.close();    
    tmr.softwd(5)
end
-------------------------------------------------------------
---------
--print(gpio.read(mixer1))
--analogWriteFreq(2000)
