#include <SPI.h>
#include <LoRa.h>
#include <stdint.h>
#include <avr/wdt.h>
#include <stdarg.h>

/* --------------------------------- Setting -------------------------------- */
// #define PRINT_DEBUG
#define DEBUG_LORA (0)
#define DEBUG_NODE (1)
#define ENABLE_DEBUG (TRUE)
#define UNUSED (0xFF)

/**
 * @brief NODE_ID value is available from 0x0000 to 0x3FFF, change this value to be uniquely
 * among all working nodes 
 * 
 */
#define NODE_ID (0x3BCA)
#define NODE_POSITION (e_TranDuyHung)
#define NODE_RELAY_STATE (e_BulbOff)

/* ----------------------------- Pin definition ----------------------------- */
#define RELAY_PIN (3)
#define LIGHT_SENSOR_PIN (A0)
#define LED_DEBUG (4)
/* ----------------------------- Function Macro ----------------------------- */
#define relayControl(state) (digitalWrite(RELAY_PIN, state))
#define IS_NODE_ID(node_id) ((node_id >= 0x0000) && (node_id <= 0x3FFF))

#define TRUE (1)
#define FALSE (0)

#define print_debug(log_level, str, ...)                     \                                                                                             
    (ENABLE_DEBUG == TRUE) ?                                 \
        ((log_level == DEBUG_LORA) ? (p(str, __VA_ARGS__)) : \
        ((log_level == DEBUG_NODE) ? (p(str, __VA_ARGS__)) : \
        (p("%s",""))))                                       \
        : (p("%s",""))

/* --------------------------- Constant definition -------------------------- */
#define NODE_ID_INIT ((NODE_ID & (NODE_ID_MSG_IDENTIFIER_1)) | NODE_ID_MSG_IDENTIFIER_2)
#define NODE_STATUS_INIT ((((u16)NODE_POSITION << 8) | (u16)NODE_RELAY_STATE) | (NODE_STATUS_MSG_IDENTIFIER))
#define LORA_FREQ_BAND (433E6)

/* --------------------------- Bitmath definition --------------------------- */
#define NODE_ID_MSG_IDENTIFIER_1 (0x3FFF)   /* (00)11 1111 1111 1111  */
#define NODE_ID_MSG_IDENTIFIER_2 (0x8000)   /* (10)00 0000 0000 0000  */
#define NODE_STATUS_MSG_IDENTIFIER (0xC000) /* (11)00 0000 0000 0000  */

#define FRAME_TYPE_FILTER_MASK (0xC000)   /* 1100 0000 0000 0000   */
#define POSITION_FILTER_MASK (0x3F00)     /* 00(11 1111) 0000 0000 */
#define RELAY_STATE_FILTER_MASK (0x0003)  /* 0000 0000 0000 00(11) */
#define RELAY_STATE_CONTROL_MASK (0x0001) /* 0000 0000 0000 000(1) */
#define RELAY_STATE_CONTROL_OFFSET (0x00)

/* --------------------------- Define Node struct --------------------------- */
typedef struct nodeInfo
{
    const u16 nodeID;
    u16 nodeStatus;
} nodeInfo_t;

typedef enum PositionEnumerations
{
    e_TranDuyHung = 0x00,
    e_LeTrongTan = 0x01,
    e_GiangVo = 0x02,
    e_GiaiPhong = 0x03,
} ePosition;

typedef enum BulbStatusEnumerations
{
    e_BulbOn = 0x01,
    e_BulbOff = 0x00,
    e_BulbFail = 0x10,
} eBulbStatus;

typedef enum FrameTypeEnumerations
{
    /* (10)11 1111 1111 1111  */
    /* (11)00 0000 0000 0000 */
    e_NodeIdFrame = 0b1000000000000000,
    e_NodeStatusFrame = 0b1100000000000000,
} eFrameType;
/* ---------------------------- Global variables ---------------------------- */
/* Store node parameters in Flash */
const nodeInfo_t this_node PROGMEM = {
    .nodeID = NODE_ID_INIT,
    .nodeStatus = NODE_STATUS_INIT,
};

/* Store pointer to nodeStruct in Flash  */
nodeInfo_t *const node_ptr PROGMEM = &this_node;
nodeInfo_t *this_node_ptr = NULL;

/* Error count variables */
u32 receice_failed_1_count = 0;
u32 receice_failed_2_count = 0;

u32 prev_blink_led_task = 0;
u32 node_send_info_task = 0;
/* ------------------------- Function Implementation ------------------------ */
void p(char *fmt, ...)
{
    char buf[128]; // resulting string limited to 128 chars
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, 128, fmt, args);
    va_end(args);
    Serial.print(buf);
}

u16 Node_GetNodeID(u16 nodeIdMsg)
{
    nodeIdMsg &= NODE_ID_MSG_IDENTIFIER_1;
    return nodeIdMsg;
}

u8 Node_GetNodePosition(u16 nodeStatusMsg)
{
    u8 pos = 0;
    pos = (nodeStatusMsg & POSITION_FILTER_MASK) >> 8;
    return pos;
}

u8 Node_GetNodeRelayStatus(u16 nodeStatusMsg)
{
    u8 relay_state = 0;
    relay_state = nodeStatusMsg & RELAY_STATE_FILTER_MASK;
    return relay_state;
}

void LoRa_Init(void)
{
    if (!LoRa.begin(433E6))
    {
        print_debug(DEBUG_LORA, "%s", "Starting LoRa failed!\r\n");
        while (1)
        {
            print_debug(DEBUG_LORA, "%s", "Error: Starting LoRa failed!\r\n");
            delay(500);
        }
    }
    print_debug(DEBUG_LORA, "%s", "Start LoRa success\r\n");
}

template <class dataType>
void LoRa_SendPacket(dataType data)
{
    LoRa.beginPacket();
    LoRa.print(data);
    LoRa.endPacket();
}

int LoRa_ReceivePacket()
{
    String receive_buffer = "";
    u16 receive_data = 0;
    u16 packetSize = LoRa.parsePacket();
    if (packetSize > 0)
    {
        /* Check LoRa data available */
        while (LoRa.available() > 0)
        {
            receive_buffer += (char)LoRa.read();
        }
        receive_data = receive_buffer.toInt();
        print_debug(DEBUG_LORA, "Received packet: %ul\r\n", receive_data);
        return receive_data;
    }
    else
    {
        /* No data arrived */
        return -1;
    }
}

int LoRa_ReceiveFrame()
{
    int packet_1 = 0;
    int packet_2 = -1;
    u16 frame_type = 0;

    packet_1 = LoRa_ReceivePacket();
    u32 return_data = 0;
    if (packet_1 == -1)
    {
        return -1;
    }
    else
    {
        frame_type = packet_1 & FRAME_TYPE_FILTER_MASK;
        /* Check if received packet is nodeID packet  */
        if (frame_type == e_NodeIdFrame)
        {
            while (packet_2 == -1)
            {
                packet_2 = LoRa_ReceivePacket();
            }

            /* If packet_2 is false */
            if ((packet_2 & FRAME_TYPE_FILTER_MASK) != e_NodeStatusFrame)
            {
                receice_failed_2_count++;
                print_debug(DEBUG_LORA, "[Error]: data_sequence_failed_2: %ul\r\n", receice_failed_2_count);
                return -1;
            }
            else /* Receice status packet */
            {
                return_data = ((u32)packet_1 << 16) | (packet_2);
                return return_data;
            }
        }
        else
        {
            receice_failed_1_count++;
            print_debug(DEBUG_LORA, "[Error]: data_sequence_failed_1: %ul\r\n", receice_failed_1_count);
            return -1;
        }
    }
}

u16 Node_ParseNodeIdFromMsg(u16 nodeIDmsg)
{
    nodeIDmsg &= NODE_ID_MSG_IDENTIFIER_1;
    return nodeIDmsg;
}

void Node_RetrieveOldState()
{
    u8 prev_relay_state = 0;
    this_node_ptr = pgm_read_ptr_near(node_ptr);
    prev_relay_state = this_node_ptr->nodeStatus & RELAY_STATE_CONTROL_MASK;
    relayControl(prev_relay_state);
    print_debug(DEBUG_NODE, "Prev Relay state: %ul\r\n", prev_relay_state);
}

u8 Node_Update(nodeInfo_t *this_node, u32 receive_data)
{
    u16 packet_node_id = 0;
    u8 packet_position = 0;
    u8 packet_relay_state = 0;
    u16 packet_1 = 0;
    u16 packet_2 = 0;
    u8 this_node_relay_state = 0;

    packet_1 = receive_data >> 16;
    packet_2 = receive_data & 0xFFFF;

    packet_node_id = packet_1 & NODE_ID_MSG_IDENTIFIER_1;
    packet_position = (packet_2 & POSITION_FILTER_MASK) >> 8;
    packet_relay_state = (packet_2 & RELAY_STATE_FILTER_MASK);

    print_debug(DEBUG_LORA, "%s", "\r\n---Packet info begin---\r\n");
    print_debug(DEBUG_LORA, "NodeIdMsg: %x\r\nNodeStatusMsg: %x\r\n", packet_1, packet_2);
    print_debug(DEBUG_LORA, "%s", "---Packet info end---\r\n");

    /* Packet discard due to NodeID not matched */
    if (packet_node_id != NODE_ID)
    {
        print_debug(DEBUG_NODE, "Target NodeID: %x\r\nThis Node's ID: %x\r\n", packet_node_id, NODE_ID);
        print_debug(DEBUG_NODE, "%s", "___NodeID not matched: Message discarded___\r\n\r\n");
        return -1;
    }

    /* NodeID matched */
    else
    {
        print_debug(DEBUG_NODE, "%s" ,"+++ Node matched begin +++\r\n");
        print_debug(DEBUG_NODE, "NodeID: %x\r\n", packet_node_id);
        this_node_relay_state = this_node->nodeStatus & RELAY_STATE_FILTER_MASK;

        /* Check relay state needs to be updated */
        if (this_node_relay_state != packet_relay_state)
        {
            print_debug(DEBUG_NODE, "Relay state changed: %ul\r\n", packet_relay_state);
            relayControl(packet_relay_state);
        }

        print_debug(DEBUG_NODE, "%s", "+++ Node matched end +++\r\n");
        /* Update Node Struct */
        this_node->nodeStatus = packet_2;
        return 0;
    }
}

void Node_SendToGW(void){
    u32 msg_to_send = 0;
    u16 node_pos = 0;
    u8 node_state = 0;
    node_pos = Node_GetNodePosition(this_node_ptr->nodeStatus);
    node_state = Node_GetNodeRelayStatus(this_node_ptr->nodeStatus);
    msg_to_send = (NODE_ID << 16) | (node_pos << 8) | (node_state);
    print_debug(DEBUG_LORA, "Send Node state to GW: %x\r\n", msg_to_send);
    LoRa_SendPacket<u32>(msg_to_send);
}

void LoRa_rxMode()
{
  LoRa.disableInvertIQ(); // normal mode
  LoRa.receive();         // set receive mode
}

void LoRa_txMode()
{
  LoRa.idle();           // set standby mode
  LoRa.enableInvertIQ(); // active invert I and Q signals
}

void onReceive(int packetSize)
{
  String message = "";

  while (LoRa.available())
  {
    message += (char)LoRa.read();
  }

  Serial.print("Node Receive: ");
  Serial.println(message);
}

void onTxDone()
{
  Serial.println("TxDone");
  LoRa_rxMode();
}
/* -------------------------------------------------------------------------- */
/*                                    SETUP                                   */
/* -------------------------------------------------------------------------- */
void setup()
{
    Serial.begin(9600);
    while (!Serial)
    {
    }

    Serial.println("/* ------------------------------ LoRa Receiver ----------------------------- */");
    Serial.print("This node's ID:");
    Serial.print(Node_ParseNodeIdFromMsg(this_node.nodeID));
    Serial.print(" - ");
    Serial.println(Node_ParseNodeIdFromMsg(this_node.nodeID), HEX);

    LoRa_Init();
    LoRa.onReceive(onReceive);
    LoRa.onTxDone(onTxDone);
    LoRa_rxMode();
    /* Check NODE_ID validity */
    if (!IS_NODE_ID(NODE_ID))
    {
        Serial.println("NodeID not satisfied: NODE_ID value must be in range from 0x0000 to 0x3FFF");
        while (1)
        {
            Serial.println("App halts");
            delay(500);
        }
    }

    Serial.println("Pointer to node value: ");
    Serial.println((u32)pgm_read_ptr_near(node_ptr));

    pinMode(LED_DEBUG, OUTPUT);
    pinMode(RELAY_PIN, OUTPUT);
    digitalWrite(RELAY_PIN, LOW);
    digitalWrite(LED_DEBUG, LOW);

    /* Get node'state before being powered off */
    Node_RetrieveOldState();
    wdt_enable(WDTO_8S);
    prev_blink_led_task = node_send_info_task = millis();
}

/* -------------------------------------------------------------------------- */
/*                                  SUPERLOOP                                 */
/* -------------------------------------------------------------------------- */
void loop()
{
    /* 100ms */
    u32 now = millis();
    u32 temp = LoRa_ReceiveFrame();
    if (temp != -1)
    {
        Node_Update(pgm_read_ptr_near(node_ptr), temp);
    }

    if (now - node_send_info_task >= 5000)
    {
        Node_SendToGW();
        node_send_info_task = millis();
    }

    if (now - prev_blink_led_task >= 2000)
    {
        digitalWrite(LED_DEBUG, !digitalRead(LED_DEBUG));
        prev_blink_led_task = millis();
    }

    wdt_reset();
}