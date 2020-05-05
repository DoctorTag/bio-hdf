set(COMPONENT_ADD_INCLUDEDIRS ./hw
				./hw/board)

set(COMPONENT_REQUIRES )
set(COMPONENT_PRIV_REQUIRES health_comm main)


set(COMPONENT_SRCS  ./hw/health_hw.c
                    ./hw/health_hw_i2c.c
                    ./hw/health_hw_spi.c
                    ./hw/health_hw_uart.c)

register_component()

