## Meter Reader

This is the code for Rock Meter Reader Model for DH307 project(TinyML for digital diagnostics)

ESP32s3 was used for this project.

### Project Description:
Detecting the temperature and humidity from the temperature meter and sending alert to a remote server when the temperature is above a specific level.

Connections:
Connect the SDA,SCA,GND & VCC of the OLED Display with SDA(GPIO5),SCA(GPIO6), GND & 3v3 of ESP32s3. Also connect GPIO2 of ESP32s3 with the button and GND of ESP32s3 with the button.


Setup Instructions Install ESP IDF: https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/windows-setup.html

After you install ESP IDF in your device, so the following

Open ESP-IDF powershell
Write the following commands 
i. idf.py set-target esp32s3 
ii. idf.py build 
iii. idf.py flash monitor
