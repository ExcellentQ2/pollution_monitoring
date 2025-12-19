idf_component_register(SRCS "c02.c"
                            "scd30/scd30_i2c.c"
                            "scd30/sensirion_common.c"
                            "scd30/sensirion_i2c.c"
                            "scd30/sensirion_i2c_hal.c"  # <-- Убедитесь, что этот наш файл тоже здесь!
                       INCLUDE_DIRS "." "scd30"    # <-- Добавляем папку scd30 в пути поиска заголовков
                       REQUIRES driver)