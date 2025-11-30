AVE!
Thanks for attention!

Here is a boiler room controller what I made. Tested and works stable.
I use if for controlling a mixing valve and a circulating pump. 
I have underfloor heating.
Heat comes from a puffer what is heated up by wood boiler.
Sensors are PT100, in series with 100 ohm fix resistor. This system gets around 1 volts (5V is too much for PT100). So you have to use the VREF of the arduino.  
It measures AMBIENT temp and room temp. 
You can adjust the heating curve (5 points), and you can compensate it with the internal wall/room temp.
You still need to have a room thermostat what gaves out contact it can be more precise.
It can control 3 point controlled valve motor, and the circulating pump.
You need to bulid 100nF capacitors next to the AnalogInput ports, and 100nF-100OHM snubber next to the 230VAC (valve motor and pump) relays agains EMC.
I dont use the Differential part from the PID, that is why kd= 0.0. The resolution for PT100 is too small, it can make mad the PID :).

Of course you have to set up your pid paramters, a valve times, and maybe your error temperatures as well.
And you have to fine tune your heating curve too. (actually the curve is just constant temp at the moment in the code, that is the last step during commissioning).


Text me for feedback , would be nice to see how it works in other places.
VALE ! :) Zoltán.
