set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES app_update)

set(COMPONENT_SRCS  ./health_ota.c)

register_component()

