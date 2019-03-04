from network import WLAN, Bluetooth

# Disable wifi
wlan = WLAN()
wlan.deinit()

# Disable bluetooth
bluetooth = Bluetooth()
bluetooth.deinit()
