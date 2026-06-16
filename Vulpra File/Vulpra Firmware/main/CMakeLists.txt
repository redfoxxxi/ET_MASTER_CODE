file(GLOB_RECURSE app_sources ${CMAKE_SOURCE_DIR}/main/*.*)
idf_component_register(SRCS ${app_sources} 
                    INCLUDE_DIRS "."
                    "." 
                    "./tcpip" 
                    "./drivers"
                    "./html"
                    "C:/Espressif/frameworks/esp-idf-v5.1.6/components/freertos/FreeRTOS-Kernel/include/freertos"
                    "C:/Espressif/frameworks/esp-idf-v5.1.6/components/hal/include/hal"
                    "C:/Espressif/frameworks/esp-idf-v5.1.6/components/esp_adc/include/esp_adc"
                    "C:/Espressif/frameworks/esp-idf-v5.1.6/components/esp_http_server/include"

                    EMBED_FILES 
                    "html/FreeEtarget.html"
                    "html/FreeEtargetIcon.png"
                    "html/help.html"
                    "html/menu.html"
                    "tcpip/ca_cert.pem"
                    )

