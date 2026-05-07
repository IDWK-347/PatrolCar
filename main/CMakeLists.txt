file(GLOB SRCS *.cpp *.hpp)

idf_component_register(
    SRCS ${SRCS}
    INCLUDE_DIRS "."
)
