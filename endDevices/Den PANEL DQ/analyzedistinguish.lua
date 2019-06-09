function analyzedistinguishfc()       
        if(signdistinguish==nil)then
            signdistinguish=signdistinguish
        elseif(signdistinguish == "BUTTON")then
                commandcontrol=stringrecive:sub(7,15);         
                if(commandcontrol == nil)then
                    commandcontrol=commandcontrl;
                elseif(commandcontrol == "INBRIGHT")then
                    inbrightfc()
                    --print("ok")
                elseif(commandcontrol == "DEBRIGHT")then
                    debrightfc()
                    --print("ok")
                elseif(commandcontrol == "ININWARM")then
                    inwarmfc()
                    --print("ok")
                elseif(commandcontrol == "ININCOOL")then
                    incoolfc()
                    --print("ok")
                elseif(commandcontrol == "ONOPTION")then
                    optionfc()
                    --print("ok")
                elseif(commandcontrol == "NIGLIGHT")then
                    nightlightfc()
                    --print("ok")
                elseif(commandcontrol == "INONONON")then
                    onfc()
                elseif(commandcontrol == "INOFFOFF")then
                    offfc()
                end
        elseif(signdistinguish == "RESSAP")then
            resetfc()
        elseif(signdistinguish == "CHASTA")then
            --stringrecive="sfdg123sta123@conkhi123bt596@1st246@fghrh"
            stringssidtemp=stringrecive:match("sta123(.*)bt596")
            stringpasstemp=stringrecive:match("bt596(.*)ps246")
            --stringstatus=stringrecive:match("ps246(.*)st537")
            print(stringrecive)
            if (stringssidtemp ==nil)then
                print("loi roi")
            elseif ( stringpasstemp == nil)then
                print("loi roi")
            --elseif(stringstatus==nil)then
                --print("loi roi")
            else  
                ctnwffc()
            end            
        end
end        
 