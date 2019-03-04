# Test Sigfox coverage

This repository contains code to test Sigfox coverage. 
It uses the Pytrack expansion board and the Sipy/Lopy module from Pycom.
It needs an antenna connected to Pytrack, and an SD into the Pytrack to save data files.

There are some time variables to adjust (in seconds):
time_searching_GPS = 30
time_searching_Sigfox = 60
time_to_deep_sleep = 300
