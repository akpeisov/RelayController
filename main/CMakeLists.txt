idf_component_register(SRCS "main.c"
                            "core.c"
                            "hardware.c"
                       INCLUDE_DIRS "."
                       EMBED_FILES ../certs/jwt.pem)
