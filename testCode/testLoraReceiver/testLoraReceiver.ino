#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>

#define PRINT_DEBUG
#define RELAY_PIN (3)
#define LED_DEBUG (4)
#define BLINK_LED_FREQ (500)

String receved_packet = "";
u32 prev_blink_led = 0;
u8 packet_count = 0;

void LoRa_Init(void)
{
  if (!LoRa.begin(433E6))
  {
    Serial.println("Starting LoRa failed!");
    while (1)
    {
      Serial.println("Error: Starting LoRa failed!");
      delay(500);
    }
  }
  Serial.println("Start LoRa success");
}

String LoRa_ReceivePacket()
{
  String receive_buffer = "";
  u16 receive_data = 0;
  u16 packetSize = LoRa.parsePacket();
  if (packetSize > 0)
  {
    /* Check LoRa data available */
    while (LoRa.available() > 0)
    {
      // receive_buffer += Serial.print((char)LoRa.read());
      receive_buffer += (char)LoRa.read();
    }
#if (defined(PRINT_DEBUG) && defined(DEBUG_LORA))
    Serial.print("Received packet: ");
    Serial.println(receive_buffer);
#endif
    return receive_buffer;
    // print RSSI of packet
    // Serial.print("' with RSSI ");
    // Serial.println(LoRa.packetRssi());
  }
  else
  {
    /* No data arrived */
    return "";
  }
}

void setup()
{
  Serial.println("-------------Test LoRa Receiver-------------");
  Serial.begin(9600);
  while (!Serial)
  {
  }

  LoRa_Init();

  pinMode(RELAY_PIN, OUTPUT);
  pinMode(LED_DEBUG, OUTPUT);

  prev_blink_led = millis();
}

void loop()
{
  u32 now = millis();
  u32 data = 0;
  // try to parse packet
  int packetSize = LoRa.parsePacket();
  if (packetSize)
  {
    packet_count++;
    receved_packet = "";
    // received a packet
    Serial.print("Received packet '");

    // read packet
    while (LoRa.available())
    {
      receved_packet += ((char)LoRa.read());
    }
    data = receved_packet.toInt();
    Serial.print(data, HEX);

    // print RSSI of packet
    Serial.print("' with RSSI ");
    Serial.println(LoRa.packetRssi());
    if(packet_count == 2){
      packet_count = 0;
      Serial.println("\r\n");
    }
  }

  if (now - prev_blink_led >= BLINK_LED_FREQ)
  {
    digitalWrite(LED_DEBUG, !digitalRead(LED_DEBUG));
    prev_blink_led = now;
  }
}
