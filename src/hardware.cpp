#include "hardware.h"

HP203B hp;
Mpu6500 mpu(SPI1, IMU_CS);
ODriveCAN odrv(wrap_can_intf(CAN), ODRV_NODE_ID);
Heartbeat_msg_t lastHeartbeat;

static inline void canCallback(int packet_size) {
  if (packet_size > 8) {
    return;  // not supported
  }
  CanMsg msg = {.id = (unsigned int)CAN.packetId(),
                .len = (uint8_t)packet_size};
  CAN.readBytes(msg.buffer, packet_size);
  onReceive(msg, odrv);
}

float motorvel;
float motorpos;
void odriveFeedback(Get_Encoder_Estimates_msg_t& msg, void* user_data) {
  motorpos = msg.Pos_Estimate;
  motorvel = msg.Vel_Estimate;
}

void odriveHeartbeat(Heartbeat_msg_t& msg, void* user_data) {
  lastHeartbeat = msg;
}

void ledWrite(float r, float g, float b) {
  analogWrite(LEDR, (1.0f - r) * 255);
  analogWrite(LEDG, (1.0f - g) * 255);
  analogWrite(LEDB, (1.0f - b) * 255);
}

void setupHardware() {
  pinMode(LEDR, OUTPUT);
  pinMode(LEDG, OUTPUT);
  pinMode(LEDB, OUTPUT);
  ledWrite(0.04, 0.04, 0.04);

  Serial.begin(115200);

  Wire.setSCL(BARO_SCL);
  Wire.setSDA(BARO_SDA);
  Wire.setClock(400000);  // Set I2C clock to 400kHz
  Wire.begin();

  SPI.setMISO(CAN_MISO);
  SPI.setMOSI(CAN_MOSI);
  SPI.setSCK(CAN_SCK);
  SPI.begin(false);

  SPI1.setSCK(IMU_SCK);
  SPI1.setMOSI(IMU_MOSI);
  SPI1.setMISO(IMU_MISO);
  SPI1.begin(false);

  // HP203B
  hp.getAddr_HP203B(HP203B_ADDRESS_UPDATED);
  hp.setOSR(OSR_256);
  if (!hp.begin()) {
    while (1) {
      ledWrite(0.1, 0.0, 0.0);
      Serial.println("HP203B not found!");
      delay(1000);
    }
  }

  // MPU6500
  while (!mpu.begin()) {
    ledWrite(0.1, 0.0, 0.0);
    Serial.println("MPU6500 not found!");
    delay(1000);
  }
  mpu.setDLPF(Mpu6x00::DLPF_92HZ);  // 92 Hz bandwidth for smooth data
  mpu.setODR(500);                  // 500 Hz sample rate

  // CAN
  odrv.onFeedback(odriveFeedback, NULL);
  odrv.onStatus(odriveHeartbeat, NULL);

  CAN.setPins(CAN_CS, CAN_INT);
  CAN.setClockFrequency(MCP2515_CLK_HZ);

  if (!CAN.begin(CAN_BAUDRATE)) {
    while (1) {
      ledWrite(0.1, 0.0, 0.0);
      Serial.println("CAN not found!");
      delay(1000);
    }
  }

  CAN.onReceive(canCallback);

  // Enable closed-loop
  while (lastHeartbeat.Axis_State !=
         ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    odrv.clearErrors();
    delay(1);
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);
    // Pump events for 150ms. This delay is needed for two reasons;
    // 1. If there is an error condition, such as missing DC power, the ODrive
    // might
    //    briefly attempt to enter CLOSED_LOOP_CONTROL state, so we can't rely
    //    on the first heartbeat response, so we want to receive at least two
    //    heartbeats (100ms default interval).
    // 2. If the bus is congested, the setState command won't get through
    //    immediately but can be delayed.
    for (int i = 0; i < 15; ++i) {
      delay(10);
      pumpEvents(CAN);
    }
  }
}