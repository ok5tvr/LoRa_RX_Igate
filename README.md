# LoRa_RX_Igate Update 3.6.2023
Version 2.1.1 is available on GitHub. This version fixes the issue with IP address assignment using the DHCP server. In the settings, the option "IP_manual <true>" corresponds to a static IP address, while "IP_manual <false>" corresponds to automatic IP address assignment.

# LoRa_RX_Igate Update 2.6.2023
With the update to version 2.1.0, the configuration from the config.txt file is fully functional. You can modify the igate settings by changing the configuration in the txt file. The desired values must be enclosed in "<>". The end of the file is marked with "!".

# LoRa_RX_Igate Update 1.6.2023
This update enables OTA updates through a web interface. The address is the IP address of the device followed by "/update". Additionally, it is possible to configure a static or DHCP-assigned address in the settings. The address will be displayed on the home screen. The configuration is done on line 44 of the code ("bool pouzitPevnouIP = true;") where false corresponds to DHCP. Preparation is underway to extract the settings from the base code using SPIFFS.h. Therefore, before the initial launch via PlatformIO, it is necessary to upload the "config.txt" file using PlatformIO --> Upload Filesystem Image.
 <b>Please perform the configuration in the source code.</b>

# LoRa_RX_Igate Update 28.5.2023
In the update, code for sending telemetry to the APRS network has been added. An option for decoding compressed location from an APRS packet has been included. Basic icons can be decoded from APRS packets, and they are displayed on the igate website.
# LoRa_RX_Igate Update 22.5.2023
Simple LoRa Igate. The Igate operates using the LoRa.h library. Upon receiving a packet, it undergoes editing, checking, and is subsequently sent to an APRS server. The Igate includes safeguards against communication failure via Wi-Fi and connection drops between the Igate and the server. The Igate has a web interface with a fixed IP address that can be set by modifying it in Platformio. The website displays the five most recent stations with their respective distance and azimuth. Additionally, the RSSI and S/N parameters are shown. On the OLED display, five stations with their RSSI and SN are displayed
# LoRa_RX_Igate
Simple LoRa Igate. The Igate operates using the LoRa.h library. Upon receiving a packet, it undergoes editing, verification, and is subsequently sent to an APRS server. The Igate is equipped with safeguards against communication failure via WiFi and loss of connection between the Igate and the server.
