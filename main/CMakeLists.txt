# Rejestracja komponentu głównego
idf_component_register(SRCS "EspRez_Api.c"
                       INCLUDE_DIRS "."
                       PRIV_REQUIRES driver esp_driver_gpio)
