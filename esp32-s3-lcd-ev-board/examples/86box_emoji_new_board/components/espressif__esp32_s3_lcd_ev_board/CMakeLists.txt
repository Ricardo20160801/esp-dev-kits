set(SUB_BOARD_SRC "")

set(SUB_BOARD_SRC "sub_board/sub_board_480x480.c")

idf_component_register(
    SRCS "esp32_s3_lcd_ev_board.c" ${SUB_BOARD_SRC}
    INCLUDE_DIRS "include"
    PRIV_INCLUDE_DIRS "priv_include"
    REQUIRES driver
    PRIV_REQUIRES esp_timer esp_lcd
)
