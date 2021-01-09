
sơ đồ nối chân RASPBERRY PI

DIO0 ---> pin 35
MISO ---> pin 21
MOSI ---> pin 19
SCK ---> pin 23
NSS ---> pin 24
RST ---> pin 22


g++ -Wall -o LoRaSender LoRaSender.cpp LoRa.cpp Print.cpp WString.cpp itoa.cpp -lwiringPi


sơ đồ nối chân ARDUINO
DIO0 ---> D2
MISO ---> D12
MOSI ---> D11
SCK ---> D13
NSS ---> D10
RST ---> D9
