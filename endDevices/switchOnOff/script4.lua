--init.lua
print("Setting up WIFI...")
wifi.setmode(wifi.STATION)
--modify according your wireless router settings
wifi.sta.config("SSID","PASSWORD")
wifi.sta.connect()
tmr.alarm(1, 1000, 1, function() 
if wifi.sta.getip()== nil then 
print("IP unavaiable, Waiting...") 
else 
tmr.stop(1)
print("Config done, IP is "..wifi.sta.getip())
dofile("ds1820.lua")
end 
end)
ws = websocket.createClient()
ws:connect('ws://echo.websocket.org')

local a, b = http.request("https://api.thingspeak.com/update?key=LLIIEPGM1YJZSRIT&field1=30", nil, function(code, data)
    if (code < 0) then
      print("HTTP request failed")
    else
      print(code, data)
    end
  end)