# ble_app_moustache

## **Summary**

Every advertiser contains two characteristics :
- State of the LED characteristic
  - write only 
  - central writes RGB value into it
- State of the Button characteristic
  - notify only
  - peripheral notifies advertiser that the button is pressed 

Depending on previous and current values of the characteristics of each peripheral device
central device decides which peripheral device to turn on.
