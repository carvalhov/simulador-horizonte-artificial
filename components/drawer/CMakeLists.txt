idf_component_register(SRCS "drawer.c"
                    REQUIRES esp_lcd_gc9a01
                    REQUIRES unity driver
                    REQUIRES gyroscope
                    INCLUDE_DIRS "include")

include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})