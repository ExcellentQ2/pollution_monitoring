idf_component_register(SRCS "sensirion_i2c_hal.c" 
                            "scd30_i2c.c"          # <-- Изменилось имя файла!
                            "sensirion_i2c.c" 
                            "sensirion_common.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver)