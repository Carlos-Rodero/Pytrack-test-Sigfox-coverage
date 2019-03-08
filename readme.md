# Test Sigfox coverage

[Sigfox](https://www.sigfox.com/en) is a global cellular connectivity solution for the Internet of Things.

This repository contains code to test Sigfox coverage. 
It uses the Pytrack expansion board and the Sipy/Lopy module from Pycom.
It needs an external Sigfox antenna connected to Pytrack, and an SD into the Pytrack to save data files.

There are some time variables to adjust (in seconds):
- time_searching_GPS
- time_searching_Sigfox
- time_to_deep_sleep
