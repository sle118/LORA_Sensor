river water level monitoring via long range wireless sensor (LORA) and send update every 15 minute

i made this project for monitor the river near my house that suject to some flooding
sensor is desin to be small and fit inside PVC "T" and have long battery live
 great versatility and hackability

1-)based arond SAMD21G17 CPU and arduino commpatible
2-)long range LORA FCC certified radio module (up to 10KM)
3-)over 60 day on a singel 18650 3.6 litium cell (optional solar charger)
4-)mesure up to 10meter whit 1cm accuracy or 5meter whit 1mm accuracy
5-)magetic reed switch for detection of ice covert melt
6-)barometric presure and temperature sensor
7-)provision for water level and temperature detection via sumerged sensor
8-)provision for turbidity sensor
9-)RS-485 for add custom sensor
10-)user programable GPIO

all this in 51.68 mm * 19.94 mm

main water level detection is via Maxbotic ultrasonic sensor or china clone
submerged water level is via LPS28DFW   1meter water = 9.78kPa or 1.42PSI
ice melt is via a simple rock on the ice the tie to a magnet via a fish line
when ice melt rock shink and pull the magmet so it trig the magmetic reed switch
a provision is made on the PCB for put sensor power OFF when magnet is present for keep battery live over the winter

solar charger is in developmemt and come soon
same for turbidity sensor , by use of low-cost dish-washer sensor

receiver side is adafruit Adafruit Feather 32u4 RFM95 LoRa Radio- 868 or 915 MHz
combined to Adafruit Feather HUZZAH with ESP8266 - Loose Headers
both board is stacked whit GND and USB pin connected togeter
TX pin of Feather 32u4 go to RX pin of Feather HUZZAH

server side is plain web service hosting whit a FTP server  ,create a dedicated FTP accont on a WWW accesible folder  and copy the .php file to it

live exemple :
http://www.alphatronique.com/lora_sensor/mesure.php    <- real time sensor value

http://www.alphatronique.com/lora_sensor/donnes.txt    <- sensor data history


have copmlet assembled and tested PCB aviable please contack me via www.alphatronique.com
