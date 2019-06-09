--file.open("SSID.txt", "r")
--print(file.readline(2))
--file.close()
--file.open("SSID.txt", "r")
-- skip the first 5 bytes of the file
---file.seek("set", 30)
--print(file.readline())
--file.close()
--file.open("status.lua", "r")
--resetmode=tonumber(file.readline())
--print(resetmode)
--file.close()
--SSID = "Dien Quang Lamp"
--pass = "dqc09152030"
--file.open("SSID.lua", "r")
--ssidtemp=file.read()
--n=string.len(ssidtemp)
--SSID=ssidtemp:sub(1,n-1)
--print(n)
--print(SSID) 
--file.close()
--file.open("pass.lua", "r")
--passtemp=file.read()
--m=string.len(passtemp)
--pass=passtemp:sub(1,m-1)
--print(pass)
--file.close()
dofile("nwf.lua");
dofile("status.lua");
dofile("cfgwf.lua");
dofile("fcct.lua");                                                                         
dofile("server.lua");
dofile("analyzedistinguish.lua")
 
if (resetmode == 0)then
    wifi.setmode(wifi.SOFTAP)
    wifi.ap.setip(cfg)
    wifi.ap.config(cfg1)
elseif(resetmode == 1)then
    wifi.setmode(wifi.STATION)
    wifi.sta.config(SSID,pass)
    wifi.sta.setip(cfgstadq)
    ip = wifi.sta.getip()
elseif(resetmode == 2)then
    wifi.setmode(wifi.STATION)
    wifi.sta.config(SSID,pass)
    wifi.sta.setip(cfgsta)
    ip = wifi.sta.getip()
end

wifi.sleeptype(wifi.NONE_SLEEP)
--print(wifi.sta.getip())
onstatic = 1
--buzzer = 11 
mixer1 = 1--2
mixer2 = 2--12
mixer3 = 7
mixer4 = 12--1 
white = 6
yellow = 5
pwmdutywhite = 500
pwmdutyyellow = 500
pwmclock=1000
--gpio.mode(buzzer, gpio.OUTPUT)
gpio.mode(mixer1, gpio.OUTPUT)
gpio.mode(mixer2, gpio.OUTPUT)
gpio.mode(mixer3, gpio.OUTPUT)
gpio.mode(mixer4, gpio.OUTPUT)
gpio.write(mixer1, gpio.HIGH)
gpio.write(mixer2, gpio.HIGH)
--gpio.write(mixer3, gpio.LOW)
gpio.write(mixer4, gpio.HIGH)
pwm.setup(white, pwmclock, 500)
pwm.setup(yellow, pwmclock, 500)
pwm.start(white)
pwm.start(yellow)
print("ok")
function resetfc()
            file.remove("status.lua");
            file.open("status.lua","w+");
            file.writeline("resetmode = 0")
            file.writeline("resettime = 1")
            file.close();
            --dofile("analyzedistinguish.lua")
            node.restart();
end  
function ctnwffc()
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
                file.writeline("resetmode = 1")
                file.writeline("resettime = 1")
                file.close();
                --dofile("analyzedistinguish.lua")
                node.restart();
end
if (resettime ==1)then
    file.remove("status.lua");
    file.open("status.lua","w+");
    file.writeline("resetmode =".. resetmode)
    file.writeline("resettime = 0")
    file.close();    
    tmr.softwd(5)
end
--print(gpio.read(mixer1))
--analogWriteFreq(2000)
