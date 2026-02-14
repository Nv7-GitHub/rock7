#include "hardware.h"

HP203B hp;
Mpu6500 mpu(SPI1, IMU_CS, Mpu6x00::GYRO_2000DPS, Mpu6x00::ACCEL_16G);
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
  mpu.setDLPF(Mpu6x00::DLPF_184HZ);  // 184 Hz bandwidth for smooth data
  mpu.setODR(500);                   // 500 Hz sample rate

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
}

void EnableOdrv() {
  // Enable closed-loop
  if (lastHeartbeat.Axis_State ==
      ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    return;
  }
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

  // If in error, read out error (print human-readable names)
  if (lastHeartbeat.Axis_State == ODriveAxisState::AXIS_STATE_IDLE &&
      lastHeartbeat.Axis_Error != 0) {
    Serial.print("ODrive error: ");
    uint32_t e = lastHeartbeat.Axis_Error;
    bool any = false;
    if (e & ODRIVE_ERROR_INITIALIZING) {
      if (any) Serial.print(", ");
      Serial.print("INITIALIZING");
      any = true;
    }
    if (e & ODRIVE_ERROR_SYSTEM_LEVEL) {
      if (any) Serial.print(", ");
      Serial.print("SYSTEM_LEVEL");
      any = true;
    }
    if (e & ODRIVE_ERROR_TIMING_ERROR) {
      if (any) Serial.print(", ");
      Serial.print("TIMING_ERROR");
      any = true;
    }
    if (e & ODRIVE_ERROR_MISSING_ESTIMATE) {
      if (any) Serial.print(", ");
      Serial.print("MISSING_ESTIMATE");
      any = true;
    }
    if (e & ODRIVE_ERROR_BAD_CONFIG) {
      if (any) Serial.print(", ");
      Serial.print("BAD_CONFIG");
      any = true;
    }
    if (e & ODRIVE_ERROR_DRV_FAULT) {
      if (any) Serial.print(", ");
      Serial.print("DRV_FAULT");
      any = true;
    }
    if (e & ODRIVE_ERROR_MISSING_INPUT) {
      if (any) Serial.print(", ");
      Serial.print("MISSING_INPUT");
      any = true;
    }
    if (e & ODRIVE_ERROR_DC_BUS_OVER_VOLTAGE) {
      if (any) Serial.print(", ");
      Serial.print("DC_BUS_OVER_VOLTAGE");
      any = true;
    }
    if (e & ODRIVE_ERROR_DC_BUS_UNDER_VOLTAGE) {
      if (any) Serial.print(", ");
      Serial.print("DC_BUS_UNDER_VOLTAGE");
      any = true;
    }
    if (e & ODRIVE_ERROR_DC_BUS_OVER_CURRENT) {
      if (any) Serial.print(", ");
      Serial.print("DC_BUS_OVER_CURRENT");
      any = true;
    }
    if (e & ODRIVE_ERROR_DC_BUS_OVER_REGEN_CURRENT) {
      if (any) Serial.print(", ");
      Serial.print("DC_BUS_OVER_REGEN_CURRENT");
      any = true;
    }
    if (e & ODRIVE_ERROR_CURRENT_LIMIT_VIOLATION) {
      if (any) Serial.print(", ");
      Serial.print("CURRENT_LIMIT_VIOLATION");
      any = true;
    }
    if (e & ODRIVE_ERROR_MOTOR_OVER_TEMP) {
      if (any) Serial.print(", ");
      Serial.print("MOTOR_OVER_TEMP");
      any = true;
    }
    if (e & ODRIVE_ERROR_INVERTER_OVER_TEMP) {
      if (any) Serial.print(", ");
      Serial.print("INVERTER_OVER_TEMP");
      any = true;
    }
    if (e & ODRIVE_ERROR_VELOCITY_LIMIT_VIOLATION) {
      if (any) Serial.print(", ");
      Serial.print("VELOCITY_LIMIT_VIOLATION");
      any = true;
    }
    if (e & ODRIVE_ERROR_POSITION_LIMIT_VIOLATION) {
      if (any) Serial.print(", ");
      Serial.print("POSITION_LIMIT_VIOLATION");
      any = true;
    }
    if (e & ODRIVE_ERROR_WATCHDOG_TIMER_EXPIRED) {
      if (any) Serial.print(", ");
      Serial.print("WATCHDOG_TIMER_EXPIRED");
      any = true;
    }
    if (e & ODRIVE_ERROR_ESTOP_REQUESTED) {
      if (any) Serial.print(", ");
      Serial.print("ESTOP_REQUESTED");
      any = true;
    }
    if (e & ODRIVE_ERROR_SPINOUT_DETECTED) {
      if (any) Serial.print(", ");
      Serial.print("SPINOUT_DETECTED");
      any = true;
    }
    if (e & ODRIVE_ERROR_BRAKE_RESISTOR_DISARMED) {
      if (any) Serial.print(", ");
      Serial.print("BRAKE_RESISTOR_DISARMED");
      any = true;
    }
    if (e & ODRIVE_ERROR_THERMISTOR_DISCONNECTED) {
      if (any) Serial.print(", ");
      Serial.print("THERMISTOR_DISCONNECTED");
      any = true;
    }
    if (e & ODRIVE_ERROR_CALIBRATION_ERROR) {
      if (any) Serial.print(", ");
      Serial.print("CALIBRATION_ERROR");
      any = true;
    }
    if (!any) {
      // Unknown bits set, fall back to hex
      Serial.print("0x");
      Serial.println(e, HEX);
    } else {
      Serial.println();
    }
  }
}

uint32_t axisError = 0;

void odrvPosition(float pos) {
  odrv.setPosition(pos);
  if (lastHeartbeat.Axis_State !=
      ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL) {
    if (lastHeartbeat.Axis_State == ODriveAxisState::AXIS_STATE_IDLE &&
        lastHeartbeat.Axis_Error != 0) {
      axisError = lastHeartbeat.Axis_Error;
    }
    odrv.setState(ODriveAxisState::AXIS_STATE_CLOSED_LOOP_CONTROL);

    return;
  }
}