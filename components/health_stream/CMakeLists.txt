set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES app_update health_hal health_ota health_comm)

set(COMPONENT_SRCS  ./health_stream.c)

register_component()

