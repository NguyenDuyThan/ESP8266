cfg =
{
    ip="192.168.1.10",
    netmask="255.255.255.0",
    gateway="192.168.1.1"
}
--wifi.ap.setip(cfg)
cfg1={}
 cfg1.ssid="NDT_LED_OPTRAN"
 cfg1.pwd="12345678"
 --wifi.ap.config(cfg1)
if (resetmode)then
    wifi.setmode(wifi.SOFTAP)
    wifi.ap.setip(cfg)
    wifi.ap.config(cfg1)
    wifi.setphymode(wifi.PHYMODE_N)
    tmr.alarm(0, 1000, 1, function()
        if wifi.ap.getip() == nil then
            if(debug) then
                print("Connecting to AP...\n")
            end
        else
            if(debug) then
                print("access point ip: "..wifi.ap.getip())
            end
            tmr.stop(0)
        end
    end)
else
    wifi.setmode(wifi.STATION)
    wifi.sta.config(SSID,pass,0)
    wifi.sta.connect()
    tmr.alarm(0, 1000, 1, function()        
        if wifi.sta.getip() == nil then
            if(debug) then         
                print("Connecting to AP...\n")
            end
        else
            if (debug) then
                ssid, password, bssid_set, bssid=wifi.sta.getconfig()
                ip, nm, gw=wifi.sta.getip()
                print("wifi status :" .. wifi.sta.status())
                print("IP Info: \nIP Address: ",ip)
                print("Netmask: ",nm)
                print("Gateway Addr: ",gw,'\n')
                print("wf chanel: ".. wifi.getchannel())
                print("wf mode: "..wifi.getmode())
                print("wf phymode: "..wifi.getphymode())
                print("\nCurrent Station configuration:\nSSID : "..ssid
                .."\nPassword  : "..password
                .."\nBSSID_set  : "..bssid_set
                .."\nBSSID: "..bssid.."\n")              
            end
            package.loaded["configWifi"]=nil
            tmr.stop(0)
            tmr.start(1)
            tmr.start(2)
        end
    end)
end

wifi.sleeptype(wifi.NONE_SLEEP)
if(debug) then
    print("end cfgwf")    
end



