if(NOT DEFINED $ENV{IDF_PATH})
set(ENV{IDF_PATH} "$ENV{HDF_PATH}/esp-idf/")
endif()
set(EXTRA_COMPONENT_DIRS $ENV{HDF_PATH}/components)
