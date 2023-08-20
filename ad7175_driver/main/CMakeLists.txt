set(srcs "pretty_effect.c"
    "spi_master_example_main.c"
    "decode_image.c"
    "AD7175.c"
    "Communication.c"
    )

idf_component_register(SRCS ${srcs}
                    INCLUDE_DIRS "."
                    EMBED_FILES image.jpg)
