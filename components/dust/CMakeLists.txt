idf_component_register(SRCS "dust.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver driver) # Добавьте driver, если используете GPIO или AD 