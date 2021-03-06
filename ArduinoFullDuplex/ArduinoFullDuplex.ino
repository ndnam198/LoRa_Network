#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <stdarg.h>
#include <string.h>

/* --------------------------------- Setting -------------------------------- */
/**
 * @brief NODE_ID value is available from 0x0000 to 0x3FFF, change this value to be uniquely
 * among all working nodes 
 */
#define NODE_ID (0x3BCB)
#define NODE_POSITION (e_TranDuyHung)
#define NODE_RELAY_STATE (e_relayOff)

#define SEND_TO_GW_FREQ (30000ul)
#define LIGHT_SENSOR_THRESHOLD (900ul)
#define NUMBER_OF_RELAY_CHECK (200ul)
// #define PRINT_DEBUG
#define DEBUG_LORA (0)
#define DEBUG_NODE (1)
#define ENABLE_DEBUG (TRUE)
#define UNUSED (0xFF)

/* ----------------------------- Pin definition ----------------------------- */
#define RELAY_PIN (3)
#define LIGHT_SENSOR_PIN (A0)
#define LED_DEBUG (4)

/* --------------------------- Constant definition -------------------------- */


/* ----------------------------- Function Macro ----------------------------- */
#define relayControl(state) (digitalWrite(RELAY_PIN, state))
#define IS_NODE_ID(node_id) ((node_id >= 0x0000) && (node_id <= 0xFFFF))

#define TRUE (1)
#define FALSE (0)

#define print_debug(log_level, str, ...)                     \                                                                                             
    (ENABLE_DEBUG == TRUE) ?                                 \
        ((log_level == DEBUG_LORA) ? (p(str, __VA_ARGS__)) : \
        ((log_level == DEBUG_NODE) ? (p(str, __VA_ARGS__)) : \
        (p("%s",""))))                                       \
        : (p("%s",""))

/* --------------------------- Define Node struct --------------------------- */
typedef struct nodeInfo
{
  const u16 _ID;
  u8 _position;
  u8 _relayState;
} nodeInfo_t;

typedef enum PositionEnumerations
{
    e_TranDuyHung = 0x00,
    e_LeTrongTan = 0x01,
    e_GiangVo = 0x02,
    e_GiaiPhong = 0x03,
    e_YenLang = 0x04,
    e_DuongLang = 0x05,
    e_DaiLa = 0x06,
    e_DaiCoViet = 0x07,
    e_TranDaiNghia = 0x08,
    e_TaQuangBuu = 0x09,
    e_BachMai = 0x0A,
} ePosition;

typedef enum BulbStatusEnumerations
{
    e_relayOn = 0x01,
    e_relayOff = 0x00,
    e_relayFailedOn = 0x10,
    e_relayFailedOff = 0x11,
} eBulbStatus;

/* ---------------------------- Global variables ---------------------------- */
/* Store node parameters in Flash */
const nodeInfo_t this_node PROGMEM = {
    ._ID = NODE_ID,
    ._position = NODE_POSITION,
    ._relayState = NODE_RELAY_STATE,
};

/* Store pointer to nodeStruct in Flash  */
nodeInfo_t *const node_mem_ptr PROGMEM = &this_node;
nodeInfo_t *this_node_ptr = NULL;

u32 send_info_to_gw_freq = 0;
static u32 relay_check_count = 0;
/* ------------------------- Function Implementation ------------------------ */

/**
 * @brief printf 
 * 
 * @param fmt: 
 * @param ... 
 */
void p(char *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, 128, fmt, args);
    va_end(args);
    Serial.print(buf);
}

/**
 * @brief Init LoRa device
 * 
 */
void LoRa_Init(void)
{
  if (!LoRa.begin(433E6))
  {
    print_debug(DEBUG_LORA, "%s", "Start LoRa failed\r\n");
    while (1)
    {
    }
  }
  print_debug(DEBUG_LORA, "%s", "Start LoRa success\r\n");
}

/**
 * @brief Get node's ID value from a 32-bit message
 * 
 * @param node_msg 
 * @return u16 
 */
u16 Node_GetIdfromMsg(u32 node_msg)
{
  u16 id = 0;
  id = node_msg >> 16;
  return id;
}

/**
 * @brief Get node's Position value from a 32-bit message
 * 
 * @param node_msg 
 * @return u8 
 */
u8 Node_GetPositionFromMsg(u32 node_msg)
{
  return (u8)((node_msg & 0x0000FF00) >> 8);
}

/**
 * @brief Get node's relay state from a 32-bit message
 * 
 * @param node_msg 
 * @return u8 
 */
u8 Node_GetRelayStatusFromMsg(u32 node_msg)
{
  return (u8)(node_msg & 0x000000FF);
}

void LoRa_rxMode()
{
//  LoRa.disableInvertIQ(); // normal mode
  LoRa.receive();         // set receive mode
}

void LoRa_txMode()
{
  LoRa.idle();           // set standby mode
//  LoRa.enableInvertIQ(); // active invert I and Q signals
}

/**
 * @brief Create a string message in accordance with a specific node status: ID + POSITION + RELAY_STATE
 * 
 * @param node_ptr node's data to be encoded and sent subsequently
 * @return String 
 */
String Node_GenerateInfoMsg(nodeInfo_t* node_ptr){
  String msg_to_sent = "";
  // msg_to_sent = ((u32)(NODE_ID) << 16) | ((u32)(node_ptr->_position) << 8) | (node_ptr->_relayState);
  msg_to_sent = String(NODE_ID) + "." + String(node_ptr->_position) + "." + String(node_ptr->_relayState);
  return msg_to_sent;
}

/**
 * @brief Send out a string message to GW device
 * 
 * @param message msg to send
 */
void LoRa_sendMessage(String message)
{
  LoRa_txMode();        // set tx mode
  LoRa.beginPacket();   // start packet
  LoRa.print(message);  // add payload
  LoRa.endPacket(true); // finish packet and send it
  print_debug(DEBUG_LORA, "Sent to GW: %s\r\n", message.c_str());
}

/**
 * @brief Parse received message by delimiter "."
 * 
 * @param msg message to be parsed
 * @return u32 a 32-bit value that exactly presents a node's information
 */
u32 Node_ParseMsg(char *msg)
{
  char *str = NULL;
  u32 temp = 0;
  u32 return_value = 0;
  u32 id = 0;
  u32 pos = 0;
  u32 relay = 0;
  u8 count = 0;
  
  while ((str = strtok_r(msg, ".", &msg)) != NULL)
  {
    if (count == 0)
    {
      // id = (int)strtol(str, NULL, 16);
      /* atoi: array to interger */
      id = atoi(str);
      count++;
    }
    else if (count == 1)
    {
      // pos = (int)strtol(str, NULL, 16);
      pos = atoi(str);
      count++;
    }
    else
    {
      // relay = (int)strtol(str, NULL, 16);
      relay = atoi(str);
    }
  }
  return_value = (id << 16) | (pos << 8) | (relay);
  return return_value;
}

/**
 * @brief Stuffs must be done whenever LoRa device signals a data arrival, work as same as receive's interrupt
 * 
 * @param packetSize don't care
 */
void onReceive(int packetSize)
{
  String message = "";
  char *temp_msg_ptr = NULL;
  u32 temp = 0;
  u16 temp_id = 0;
  u8 temp_pos = 0;
  u8 temp_state = 0;

  while (LoRa.available())
  {
    /* Append character to destined String */
    message += (char)LoRa.read();
  }

  /* Cast from C++ String type to C char* type */
  temp_msg_ptr = message.c_str();

  /* Parse received msg data */
  temp = Node_ParseMsg(temp_msg_ptr);
  temp_id = Node_GetIdfromMsg(temp);
  temp_pos = Node_GetPositionFromMsg(temp);
  temp_state = Node_GetRelayStatusFromMsg(temp);

  /* Get current relay status */
  if (temp_id == NODE_ID)
  {
    print_debug(DEBUG_LORA, "%s\r\n", "+++ Matched +++");
    print_debug(DEBUG_LORA, "Received: %s\r\n", temp_msg_ptr);
    print_debug(DEBUG_LORA, "msg's id %x\r\n", temp_id);
    print_debug(DEBUG_LORA, "msg's pos: %x\r\n", temp_pos);
    print_debug(DEBUG_LORA, "msg's state: %x\r\n\r\n", temp_state);
    switch (temp_state)
    {
    case e_relayOn:
      /* If Node is currently failed, ignore message  */
      if (this_node_ptr->_relayState != e_relayFailed)
      {
        relayControl(HIGH);
        relay_check_count = NUMBER_OF_RELAY_CHECK;
      }
      break;
    case e_relayOff:
      /* If Node is currently failed, ignore message  */
      if (this_node_ptr->_relayState != e_relayFailed)
      {
        relayControl(LOW);
        relay_check_count = NUMBER_OF_RELAY_CHECK;
      }
      break;
    default:
      break;
    }
    this_node_ptr->_position = temp_pos;
    this_node_ptr->_relayState = temp_state;
    /* Update new info to GW immediately */
    LoRa_sendMessage(Node_GenerateInfoMsg(this_node_ptr));
  }
  else{
    print_debug(DEBUG_LORA,"%s\r\n\r\n", "---Not Matched---");
  }
  message = "";
}

/**
 * @brief Stuffs to do whenever LoRa device completed sending out data to GW, work as same as send's interrupt
 * 
 */
void onTxDone()
{
  print_debug(DEBUG_LORA, "%s", "Node Info sent\r\n");
  LoRa_rxMode();
}

bool runEvery(unsigned long interval)
{
  static unsigned long previousMillis = 0;
  unsigned long currentMillis = millis();
  if (currentMillis - previousMillis >= interval)
  {
    previousMillis = currentMillis;
    return true;
  }
  return false;
}

/**
 * @brief Return node to its previous condition before any kind of MCU_RESET
 * 
 */
void Node_RetrieveOldState()
{
  u8 prev_relay_state = 0;
  this_node_ptr = pgm_read_ptr_near(node_mem_ptr);
  prev_relay_state = this_node_ptr->_relayState;
  switch (prev_relay_state)
  {
  case e_relayOn:
    relayControl(HIGH);
    relay_check_count = NUMBER_OF_RELAY_CHECK;
    break;
  case e_relayOff:
    relayControl(LOW);
    relay_check_count = NUMBER_OF_RELAY_CHECK;
    break;
  case e_relayFailed:
    relayControl(LOW);
    break;
  default:
    break;
  }
  print_debug(DEBUG_NODE, "Prev Relay state: %ul\r\n", prev_relay_state);
}

void setup()
{
  Serial.begin(9600);
  print_debug(DEBUG_LORA, "%s","Node's Log(Arduino)\r\n");
  if(!IS_NODE_ID(NODE_ID)){
    print_debug(DEBUG_NODE, "Node ID Invalid: %x\r\n", NODE_ID);
    while(1)
      ;
  }
  LoRa_Init();

  pinMode(LED_DEBUG, OUTPUT);
  pinMode(RELAY_PIN, OUTPUT);

  LoRa.onReceive(onReceive);
  LoRa.onTxDone(onTxDone);
  LoRa_rxMode();

  Node_RetrieveOldState();

  /* Enable watchdog timer - Avoid halt*/
  wdt_enable(WDTO_8S);
  send_info_to_gw_freq = millis();
}

void loop()
{
  u32 light_sensor_value = 0;
  u32 now = millis();
  if (runEvery(1000))
  {
    /* Blink Led */
    digitalWrite(LED_DEBUG, !digitalRead(LED_DEBUG));
  }

  /* Send this node information to GW every SEND_TO_GW_FREQ */
  if (now - send_info_to_gw_freq > SEND_TO_GW_FREQ)
  {
    /* Update new info to GW periodicly if not in relay check phase */
    if (relay_check_count == 0)
    {
      LoRa_sendMessage(Node_GenerateInfoMsg(this_node_ptr));
    }
    send_info_to_gw_freq = millis();
  }

  /* Bulb ON: light_value < 900, else > 900 */
  if (relay_check_count != 0)
  {
    relay_check_count--;
    /* Read light sensor */
    light_sensor_value = analogRead(LIGHT_SENSOR_PIN);

    if (this_node_ptr->_relayState == e_relayOn)
    {
      /* Work properly */
      if (light_sensor_value < LIGHT_SENSOR_THRESHOLD)
      {
        /* check done, led is on */
        this_node_ptr->_relayState = e_relayOn;
        relay_check_count = 0;
      }
      /* Failed to turn Bulb On */
      else
      {
        this_node_ptr->_relayState = e_relayFailed;
      }
    }

    else if (this_node_ptr->_relayState == e_relayOff)
    {
      if (light_sensor_value >= LIGHT_SENSOR_THRESHOLD)
      {
        /* check done, led is off */
        this_node_ptr->_relayState = e_relayOff;
        relay_check_count = 0;
      }
      /* Failed to turn Bulb Off */
      else
      {
        this_node_ptr->_relayState = e_relayFailed;
      }
    }

    if (relay_check_count == 0)
    {
      print_debug(DEBUG_NODE, "Relay check done: State = %d\r\n", this_node_ptr->_relayState);
    }
  }

  wdt_reset();
}
