idf_component_register(
	SRCS
		"main.c" "dhrystone.c" "whetstone.c"
	PRIV_REQUIRES
		esp_lcd
		fatfs
		nvs_flash
		badge-bsp
	INCLUDE_DIRS
		"."
)
