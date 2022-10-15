# The following five lines of boilerplate have to be in your project's
# CMakeLists in this exact order for cmake to work correctly
cmake_minimum_required(VERSION 3.5)

# Karl's additions to make it build in netbeans...
set(ENV{IDF_TARGET} esp32c3)
#set(ENV{IDF_PYTHON_ENV_PATH} /home/karlp/tools/espressif/python_env/idf4.4_py3.10_env)
set(ENV{IDF_PATH}  /home/karlp/src/esp/esp-idf)


set(EXTRA_COMPONENT_DIRS $ENV{IDF_PATH}/examples/common_components/led_strip)

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(blink)
