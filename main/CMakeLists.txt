idf_component_register(
  SRCS
    "main.c"
    "fingerprint.c"
    "imk_crypto.c"
    "imk_proto.c"
    "imk_service.c"
    "imk_keystore.c"
    "battery.c"
  INCLUDE_DIRS "."
  REQUIRES driver esp_adc nvs_flash esp_timer esp_driver_usb_serial_jtag mbedtls bt
)
