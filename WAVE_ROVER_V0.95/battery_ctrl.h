#define INA219_ADDRESS 0x42
INA219_WE ina219 = INA219_WE(INA219_ADDRESS);

float shuntVoltage_mV = 0.0f;
float loadVoltage_V   = 0.0f;
float busVoltage_V    = 0.0f;
float current_mA      = 0.0f;
float power_mW        = 0.0f;
bool  ina219_overflow = false;

void ina219_init() {
    if (!ina219.init()) {
        Serial.println("INA219 not connected!");
    }

    // ADC resolution / samples
    ina219.setADCMode(INA219_BIT_MODE_9);

    // Programmable gain (shunt voltage range)
    ina219.setPGain(INA219_PG_320);

    // Bus voltage range (16 V full-scale)
   ina219.setBusRange(INA219_BRNG_16);

    // Shunt resistor value in ohms
    ina219.setShuntSizeInOhms(0.01f);
}

void inaDataUpdate() {
    shuntVoltage_mV = ina219.getShuntVoltage_mV();
    busVoltage_V    = ina219.getBusVoltage_V();
    current_mA      = ina219.getCurrent_mA();

    /* Your library still exposes this as getBusPower() */
    power_mW        = ina219.getBusPower();

    loadVoltage_V   = busVoltage_V + (shuntVoltage_mV / 1000.0f);

    ina219_overflow = ina219.getOverflow();
}
