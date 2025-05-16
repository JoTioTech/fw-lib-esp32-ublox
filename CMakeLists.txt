file(GLOB ublox_source
	"ublox/*.cpp"
	)

idf_component_register(SRCS ${ublox_source}
	INCLUDE_DIRS ublox
	PRIV_REQUIRES
	esp_timer
	)

target_compile_options(${COMPONENT_LIB} PRIVATE -std=gnu++17 -Wno-reorder)

