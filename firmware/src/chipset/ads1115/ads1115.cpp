#include "ads1115.h"

#include <unordered_map>

#include "ads1115_registers.h"

ADS1115::ADS1115(ADCServoType id) {
    switch (id) {
        case ADCServoType::ADC_SERVO_1: {
            i2c_address = 0x49;
            break;
        }
        case ADCServoType::ADC_SERVO_2: {
            i2c_address = 0x48;
            break;
        }
    }
}

ADS1115& ADS1115::get_instance(ADCServoType type) {
    static std::unordered_map<ADCServoType, ADS1115 *> instances;
    auto it = instances.find(type);
    if (it == instances.end()) {
        instances[type] = new ADS1115(type);
    }
    return *instances[type];
}

uint32_t ADS1115::init(void) {
    uint32_t status = 0;

    if (i2c_handle.init() == 0) {
        if (config() == 0) {
            is_initialized = true;
        } else {
            status = 1;
        }
    } else {
        status = 2;
    }

    return status;
}

uint32_t ADS1115::config(void) {
    uint32_t status = 0;
	uint16_t register_data = 0;
    uint8_t data[2] = {0};

    register_data = ADS1115_COMP_QUE;						// 11  : Disable comparator and set ALERT/RDY pin to high-impedance (default)
    register_data |= (ADS1115_DR_2 | ADS1115_DR_1 | ADS1115_DR_0);	// 111 : 860 SPS (default)
    register_data &= ~ADS1115_MODE;							// 1   : continiuos
    register_data &= ~ADS1115_PGA;							// 000 : FSR = ±6.144 V (187.5μV)

    register_data |= ADS1115_MUX_2;						    // 100 : AINP = AIN0 and AINN = GND
    register_data &= ~ADS1115_OS;							// 0   : no effect

    data[0] = register_data >> 8;							// msb
    data[1] = register_data & 0xFF;							// lsb

    if (i2c_handle.write_bytes(i2c_address, ADS1115_REG_CONFIG, data, 2, i2c_timeout) != 0) {
        status = 1;
    }

    return status;
}

uint32_t ADS1115::read_value(uint16_t &value) {
    uint32_t status = 0;
	uint8_t data[2] = {0};

    if (is_initialized == true) {
        if (i2c_handle.read_bytes(i2c_address, ADS1115_REG_CONVERSION, data, 2, i2c_timeout) == 0) {
            if (data[0] == 0 && data[1] == 0) {
                ADS1115::config(); // TODO test
                status = 3;
            } else {
                value = (uint16_t)((data[0] << 8) | data[1]);
            }
        } else {
            status = 2;
        }
    } else {
        status = 1;
    }

	return status;
}
