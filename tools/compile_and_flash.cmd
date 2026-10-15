@echo off
"C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe" compile -u -p COM14 --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,CDCOnBoot=cdc" --no-color "C:\Users\dell\Desktop\rover\s3_remote"
