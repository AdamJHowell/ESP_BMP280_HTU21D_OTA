cmake_minimum_required( VERSION 3.23 )
project( ESP_BMP280_HTU21D_OTA )

set( CMAKE_CXX_STANDARD 14 )

include_directories( .
		C:/Code/Arduino/libraries/libraries/ )

add_executable( ESP_BMP280_HTU21D_OTA
		ESP_BMP280_HTU21D_OTA.ino
		ESP_BMP280_HTU21D_OTA.h
		privateInfo.h )

set_target_properties( ESP_BMP280_HTU21D_OTA PROPERTIES LINKER_LANGUAGE CXX )
