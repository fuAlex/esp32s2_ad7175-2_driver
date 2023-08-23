set(srcs "main.c"
    "AD7175.c"
    "Communication.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS ".")
