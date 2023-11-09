
#ifndef CONFIG_H
#define CONFIG_H

/*
 * Device unique 16-bit identifier.
 * It is used as a "minor" in iBeacon data and 2 low bytes of BLE MAC.
 */
#define DEVICE_ID_16BITS    (uint16_t)1
#define DEVICE_ID_LSB       (uint8_t)(DEVICE_ID_16BITS & 0xFF)
#define DEVICE_ID_MSB       (uint8_t)((DEVICE_ID_16BITS >> 8) & 0xFF)

/*
 * Device BLE MAC
 */
#define BLE_MAC_ADDRESS {DEVICE_ID_LSB, DEVICE_ID_MSB, 0x40, 0x10, 0xF0, 0xAC}

/*
 * Value representing the received strength signal indication (RSSI)
 * value at a distance of 1 meter in dB
 */
#define DEVICE_CALIBRATED_RSSI_1M    -70

/*
 * BLE advertising tx power
 */
#define BLE_RF_TX_POWER    RF_PHY_TX_POWER_N15DBM

/*
 * BLE advertising interval
 */
#define BLE_ADV_INTERVAL_MS    300

#endif /* CONFIG_H */
