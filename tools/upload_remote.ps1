$cli = "C:\Program Files\Arduino IDE\resources\app\lib\backend\resources\arduino-cli.exe"
& $cli upload -p COM14 --fqbn "esp32:esp32:esp32s3:PSRAM=opi,FlashMode=qio,FlashSize=16M,CDCOnBoot=cdc" "C:\Users\dell\Desktop\rover\s3_remote"
