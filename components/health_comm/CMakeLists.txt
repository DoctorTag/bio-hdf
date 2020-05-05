set(COMPONENT_ADD_INCLUDEDIRS ./include)

set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES health_hal bt)


set(COMPONENT_SRCS  ./health_ble.c
                    ./health_comm.c
                    ./health_comm_protocol.c
                    ./health_tcp.c
		    ./local_uart.c)

register_component()

