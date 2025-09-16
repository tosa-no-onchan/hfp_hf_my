idf_component_register(SRCS "app_hf_msg_set.c"
                            "i2s_acc_std.c"
                            "bt_app_core.c"
                            "bt_app_hf.c"
                            "gpio_pcm_config.c"
                            "bt_app_pbac.c"
                            "main.c"
                    PRIV_REQUIRES bt nvs_flash esp_driver_gpio console esp_ringbuf esp_driver_i2s
                    INCLUDE_DIRS ".")

#set(COMPONENT_ADD_PRIV_REQUIRES  "esp_driver_i2s" "esp_driver_gpio")
