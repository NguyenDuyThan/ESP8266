
tmr.register(0, 5000, tmr.ALARM_SINGLE, function() print("hey there") end)
if not tmr.start(0) then print("uh oh") end
tmr.start(0)
tmr.state(0)