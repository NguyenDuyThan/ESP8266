
function ledWhiteFunCtr(heso)
    if(pwmDutyWhite <=100 and pwmDutyWhite >=20)then
        pwmDutyWhite = pwmDutyWhite + heso
    end
    if (pwmDutyWhite > 100 ) then   
        pwmDutyWhite = 100
    elseif (pwmDutyWhite <20) then
        pwmDutyWhite = 20
    end
    pwm.setduty(whitePin, pwmDutyWhite)
end
--function ledyellowfc(heso)
--    if(pwmdutyyellow <=1023 and pwmdutyyellow >=0)then
--        pwmdutyyellow = pwmdutyyellow + heso
--    end
--    if (pwmdutyyellow > 1023) then
--        pwmdutyyellow = 1023    
--    elseif (pwmdutyyellow < 0) then
--        pwmdutyyellow = 0
--    end
--    pwm.setduty(yellow, pwmdutyyellow)
--end
function inBrightFunCtr()
                        if (onStatus)then
                            --fcbuzzer();
                            ledWhiteFunCtr(10)
                            --ledyellowfc(100)
                            if(debug) then
                                print("increase bright")
                            end
                        end
end
function deBrightFunCtr()
                        if (onStatus)then
                            --fcbuzzer();
                            ledWhiteFunCtr(-10);
                            --ledyellowfc(-100);
                            if(debug) then
                                print("decrease bright")
                            end
                        end
end
function inWarmFunCtr ()
                        if(onStatus)then
                            --fcbuzzer();
                            --ledwhitefc(-100);
                            --ledyellowfc(100);
                            pwmDutyWhite = 30
                            pwm.setduty(whitePin, pwmDutyWhite)
                            if(debug) then
                                print("increase warmwhite")
                            end
                        end
end
function inCoolFunCtr()
                        if(onStatus)then
                            --fcbuzzer();
                            --ledwhitefc(100);
                            --ledyellowfc(-100);
                            pwmDutyWhite = 100
                            pwm.setduty(whitePin, pwmDutyWhite)
                            if(debug) then
                                print("increase coolwhite")
                            end
                        end
end                        
function optionFunCtr()
                        if(onStatus)then
                            --fcbuzzer();
                            --gpio.write(mixer1, gpio.HIGH);
                            --gpio.write(mixer2, gpio.HIGH);
                            --gpio.write(mixer3, gpio.LOW);
                            --gpio.write(mixer4, gpio.HIGH);
                            --pwm.setduty(white, 880);
                            pwmDutyWhite = 60
                            --pwmdutyyellow = 600
                            --ledwhite(0)
                            --pwm.setduty(yellow, pwmdutyyellow);
                            --ledyellow(0);
                            pwm.setduty(whitePin, pwmDutyWhite)
                            --onstatic==1
                            if(debug) then
                                print("option")
                            end
                        end
end
function nightLightFunCtr()                                        
                        if(onStatus)then
--                            fcbuzzer();
                            --gpio.write(mixer1, gpio.LOW)
                            --gpio.write(mixer2, gpio.LOW)
                            --gpio.write(mixer3, gpio.HIGH)
                            --gpio.write(mixer4, gpio.LOW)
                            --ledwhite(900);
                            ledWhiteFunCtr = 30
                            --pwmdutyyellow = 100
                            pwm.setduty(whitePin, pwmDutyWhite)
                            --pwm.setduty(yellow, pwmdutyyellow)
                            if(debug) then
                                print("night light")
                            end
                        end
end 
function onFunCtr()
                    onStatus = true; 
                    --print("ok") 
                    --fcbuzzer();  
                    --gpio.write(mixer1, gpio.HIGH)
                    --gpio.write(mixer2, gpio.HIGH)
                    --gpio.write(mixer4, gpio.HIGH)
                    ledWhiteFunCtr(0)
                    --pwm.setduty(white, 800)
                    --ledyellowfc(0)
                    --pwm.setduty(yellow, 800)
end
function offFunCtr()
                    --fcbuzzer();
                    --print("ok")
                    onStatus = false
                    --gpio.write(mixer1, gpio.LOW)
                    --gpio.write(mixer2, gpio.LOW)
                    --gpio.write(mixer4, gpio.LOW)
                    pwm.setduty(whitePin, 0)
                    --pwm.setduty(yellow, 0)
                    --gpio.write(mixer3, gpio.LOW)
end                          
         
