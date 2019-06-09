function ledwhitefc(heso)
    if(pwmdutywhite <=1023 and pwmdutywhite >=0)then
        pwmdutywhite = pwmdutywhite + heso
    end
    if (pwmdutywhite > 1023 ) then   
        pwmdutywhite = 1023
    elseif (pwmdutywhite <0) then
        pwmdutywhite = 0
    end
    pwm.setduty(white, pwmdutywhite)
end

function ledyellowfc(heso)
    if(pwmdutyyellow <=1023 and pwmdutyyellow >=0)then
        pwmdutyyellow = pwmdutyyellow + heso
    end
    if (pwmdutyyellow > 1023) then
        pwmdutyyellow = 1023    
    elseif (pwmdutyyellow < 0) then
        pwmdutyyellow = 0
    end
    pwm.setduty(yellow, pwmdutyyellow)
end
function inbrightfc()
                        if (onstatic==1)then
                            --fcbuzzer();
                            ledwhitefc(100)
                            ledyellowfc(100)
                            print("ok")
                        end
end
function debrightfc()
                        if (onstatic==1)then
                            --fcbuzzer();
                            ledwhitefc(-100);
                            ledyellowfc(-100);
                            print("ok")
                        end
end
function inwarmfc ()
                        if(onstatic==1)then
                            --fcbuzzer();
                            ledwhitefc(-100);
                            ledyellowfc(100);
                            print("ok")
                        end
end
function incoolfc()
                        if(onstatic==1)then
                            --fcbuzzer();
                            ledwhitefc(100);
                            ledyellowfc(-100);
                            print("ok")
                        end
end                        
function optionfc()
                        if(onstatic==1)then
                            --fcbuzzer();
                            gpio.write(mixer1, gpio.HIGH);
                            gpio.write(mixer2, gpio.HIGH);
                            gpio.write(mixer3, gpio.LOW);
                            gpio.write(mixer4, gpio.HIGH);
                            --pwm.setduty(white, 880);
                            pwmdutywhite = 600
                            pwmdutyyellow = 600
                            --ledwhite(0)
                            pwm.setduty(yellow, pwmdutyyellow);
                            --ledyellow(0);
                            pwm.setduty(white, pwmdutywhite)
                            --onstatic==1
                            print("ok")
                        end
end
function nightlightfc()                                        
                        if(onstatic==1)then
--                            fcbuzzer();
                            gpio.write(mixer1, gpio.LOW)
                            gpio.write(mixer2, gpio.LOW)
                            gpio.write(mixer3, gpio.HIGH)
                            gpio.write(mixer4, gpio.LOW)
                            --ledwhite(900);
                            pwmdutywhite = 100
                            pwmdutyyellow = 100
                            pwm.setduty(white, pwmdutywhite)
                            pwm.setduty(yellow, pwmdutyyellow)
                            print("ok")
                        end
end 
function onfc()
                    onstatic = 1; 
                    --print("ok") 
                    --fcbuzzer();  
                    gpio.write(mixer1, gpio.HIGH)
                    gpio.write(mixer2, gpio.HIGH)
                    gpio.write(mixer4, gpio.HIGH)
                    ledwhitefc(0)
                    --pwm.setduty(white, 800)
                    ledyellowfc(0)
                    --pwm.setduty(yellow, 800)
end
function offfc()
                    --fcbuzzer();
                    --print("ok")
                    onstatic = 0
                    gpio.write(mixer1, gpio.LOW)
                    gpio.write(mixer2, gpio.LOW)
                    gpio.write(mixer4, gpio.LOW)
                    pwm.setduty(white, 0)
                    pwm.setduty(yellow, 0)
                    gpio.write(mixer3, gpio.LOW)
end                          
         
