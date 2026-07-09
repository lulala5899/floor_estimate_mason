#include "HP206C.h"

#include "config.h"

namespace {
constexpr uint8_t kFallbackPrimaryAddr = 0x76;
constexpr uint8_t kFallbackSecondaryAddr = 0x77;
}

bool HP206C::configureSensor() {
    // Balanced settings for stable altitude estimation at low latency.
    if (bmp_.setTemperatureOversampling(BMP3_OVERSAMPLING_8X) == false) {
        return false;
    }
    if (bmp_.setPressureOversampling(BMP3_OVERSAMPLING_4X) == false) {
        return false;
    }
    if (bmp_.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3) == false) {
        return false;
    }
    if (bmp_.setOutputDataRate(BMP3_ODR_50_HZ) == false) {
        return false;
    }
    return true;
}

bool HP206C::begin() {
#ifdef BARO_I2C_ADDR_PRIMARY
    const uint8_t primary = BARO_I2C_ADDR_PRIMARY;
#else
    const uint8_t primary = kFallbackPrimaryAddr;
#endif
#ifdef BARO_I2C_ADDR_SECONDARY
    const uint8_t secondary = BARO_I2C_ADDR_SECONDARY;
#else
    const uint8_t secondary = kFallbackSecondaryAddr;
#endif

    const uint8_t try_addrs[] = {primary, secondary};
    for (uint8_t addr : try_addrs) {
        if (bmp_.begin_I2C(addr, &Wire)) {
            i2c_address_ = addr;
            return configureSensor();
        }
    }

    i2c_address_ = 0;
    return false;
}

bool HP206C::readAll(float* temp_C, float* pressure_hPa) {
    if (temp_C == nullptr || pressure_hPa == nullptr) {
        return false;
    }
    if (bmp_.performReading() == false) {
        return false;
    }

    *temp_C = bmp_.temperature;
    *pressure_hPa = bmp_.pressure / 100.0f;
    return true;
}
