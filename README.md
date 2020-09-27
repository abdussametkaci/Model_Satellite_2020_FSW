# Model_Uydu_2020_Ucus_Yazilimi
Türsat 2020 Model Uydu Yarışması'nda kullanacağımız uçuş yazılımı. 
Bu uçuş yazılımı sadece sensör ve modüllerden gelen dataları işleyip bizim ikinci modülümüz olan ESP32-Cam modülüne verileri USART haberleşme ile göndermektedir.
## Kullanılan Cihazlar
1) Arduino Nano             -> mikrokontrolcü
2) BMP280                   -> sıcaklık ve basınç sensörü
3) Adafruit Ultimate GPS    -> GPS
4) Adafruit BNO055          -> ivme ölçer, jiroskopp ve manyometre
