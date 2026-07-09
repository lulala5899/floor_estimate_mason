#ifndef CONFIG_H
#define CONFIG_H

// Wi-Fi credentials are defined in src/config.cpp.
// Fill in local values before flashing, but do not commit real SSIDs,
// passwords, identities, or network-specific IPs.
extern const char *SSID_IOT;
extern const char *SSID_IOT_PASSWORD;

// for WPA2-Enterprise (Eduroam)
extern const char* EDUROAM_SSID;
extern const char* EDUROAM_IDENTITY;
extern const char* EDUROAM_PASSWORD;

// I2C pins for barometer. Override with build_flags if needed.
#ifndef BARO_I2C_SDA_PIN
#define BARO_I2C_SDA_PIN SDA
#endif

#ifndef BARO_I2C_SCL_PIN
#define BARO_I2C_SCL_PIN SCL
#endif

// BMP390 supports 0x76 (SDO low) and 0x77 (SDO high).
#ifndef BARO_I2C_ADDR_PRIMARY
#define BARO_I2C_ADDR_PRIMARY 0x76
#endif

#ifndef BARO_I2C_ADDR_SECONDARY
#define BARO_I2C_ADDR_SECONDARY 0x77
#endif

#endif
