idf_component_register(SRCS "main.c" "scd30_task.c" "sps30_task.c"
                       INCLUDE_DIRS "."
                       REQUIRES driver json)