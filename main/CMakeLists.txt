# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.

idf_component_register(
    SRCS main.c ssd1306.c fonts.c xi2c.c button.c sensors.c bt_functions.c     # list the source files of this component
    INCLUDE_DIRS        # optional, add here public include directories
    PRIV_INCLUDE_DIRS  
    REQUIRES          
    PRIV_REQUIRES       # optional, list the private requirements
)
