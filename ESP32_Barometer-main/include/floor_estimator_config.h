#ifndef FLOOR_ESTIMATOR_CONFIG_H
#define FLOOR_ESTIMATOR_CONFIG_H

#include <cstddef>

// ---------- Network ----------
#ifndef NODE_HTTP_PORT
#define NODE_HTTP_PORT 8080
#endif

#ifndef BASE_HOST_IP
// Configure this before using direct base-host registration.
// Do not commit a personal LAN or Tailscale IP here.
#define BASE_HOST_IP ""
#endif

#ifndef BASE_HOST_PORT
#define BASE_HOST_PORT 80
#endif

#ifndef BASE_REGISTER_PATH
#define BASE_REGISTER_PATH "/clientip"
#endif

constexpr unsigned long kRegisterRetryMs = 3000;
constexpr unsigned long kWifiReconnectIntervalMs = 5000;
constexpr unsigned long kStatusLogIntervalMs = 5000;

// ---------- Sampling ----------
// Paper setup reports mobile barometer ~6Hz and base barometer ~3Hz.
constexpr unsigned long kMobileSampleIntervalMs = 160;
constexpr unsigned long kSampleStaleMs = 5000;

// ---------- ISA Pressure->Height (Eq. 1 in arXiv:2601.02184v1) ----------
// h(P,T) = (T/L) * (1 - (P/P0)^kappa), kappa = R*L/g
constexpr float kIsaLapseRate = 0.0065f;        // K/m
constexpr float kIsaGasConstant = 287.05f;      // J/(kg*K)
constexpr float kIsaGravity = 9.80665f;         // m/s^2
constexpr float kSeaLevelPressureHpa = 1010.0f; // P0 (can be replaced by weather API upstream)

// ---------- Sensor validation ----------
constexpr float kMinPressureHpa = 900.0f;
constexpr float kMaxPressureHpa = 1100.0f;
constexpr float kMinTempC = -30.0f;
constexpr float kMaxTempC = 60.0f;

// ---------- Relative calibration offsets (Eq. 10) ----------
// Paper uses calibrated values: p_tilde = p_raw - beta_p, T_tilde = T_raw - beta_T.
// Set these beta constants from your offline collocation calibration.
constexpr float kMobilePressureBetaHpa = 0.0f;
constexpr float kMobileTempBetaC = 0.0f;
constexpr float kBasePressureBetaHpa = 0.0f;
constexpr float kBaseTempBetaC = 0.0f;

// ---------- Floor indexing (Eq. 13) ----------
// Replace with measured checkpoint/floor heights of your building, relative to base floor.
// Example below is only a placeholder for quick bring-up.
constexpr float kFloorHeightsM[] = {
    0.0f,
    3.0f,
    6.0f,
    9.0f,
    12.0f,
};

constexpr size_t kFloorCount = sizeof(kFloorHeightsM) / sizeof(kFloorHeightsM[0]);

#endif // FLOOR_ESTIMATOR_CONFIG_H
