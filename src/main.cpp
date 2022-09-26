// define this for less flicker:
#define DOUBLE_BUFFER
#include <Arduino.h>
#include <lvgl.h>
#include "lcd_controller.h"
#include "FT6236.h"
#define get_pos ft6236_pos
#define I2C_SCL 39
#define I2C_SDA 38
#define LCD_BLK 45
#define LCD_H_RES            320
#define LCD_V_RES            480
// change this to control display RAM use:
#define LVGL_LCD_BUF_SIZE            (LCD_H_RES * 32)

static lv_disp_draw_buf_t disp_buf;  // contains internal graphic buffer(s) called draw buffer(s)
static lv_disp_drv_t disp_drv;       // contains callback functions
static lv_color_t *lv_disp_buf;
#ifdef DOUBLE_BUFFER
static lv_color_t *lv_disp_buf2;
#endif
static bool is_initialized_lvgl = false;

/*Read the touchpad*/
void touch_read( lv_indev_drv_t * indev_driver, lv_indev_data_t * data )
{
    int pos[2];
    if(get_pos(pos)) {
        data->state = LV_INDEV_STATE_PR;
        data->point.x = pos[0];
        data->point.y = pos[1];   
    } else {
        data->state = LV_INDEV_STATE_REL;
    }
}
static bool lvgl_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t *edata, void *user_ctx) {
    if(is_initialized_lvgl) {
        lv_disp_flush_ready((lv_disp_drv_t *)user_ctx);
        return true;
    }
    return false;
}
static void lvgl_flush(lv_disp_drv_t *drv, const lv_area_t *area, lv_color_t *color_map) {
    lcd_flush(area->x1,area->y1,area->x2,area->y2,color_map);
}

void setup() {
    Serial.begin(115200);
    Serial.printf("PSRAM size is %dMB\n",(int)(ESP.getPsramSize()/1024.0/1024.0));
    Wire.begin(I2C_SDA, I2C_SCL);
   lcd_color_trans_done_register_cb(lvgl_flush_ready,&disp_drv);
   lcd_init(LVGL_LCD_BUF_SIZE*sizeof(lv_color_t));
   
    /* Lighten the screen with gradient */
    ledcSetup(0, 10000, 8);
    ledcAttachPin(LCD_BLK, 0);
    for (uint8_t i = 0; i < 0xFF; i++) {
        ledcWrite(0, i);
        delay(2);
    }
    lv_init();
#ifdef DOUBLE_BUFFER    
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t)/2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if(lv_disp_buf==NULL) {
        Serial.println("Out of memory");
        while(1);
    }
    lv_disp_buf2 = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t)/2, MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if(lv_disp_buf2==NULL) {
        Serial.println("Out of memory");
        while(1);
    }
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf,lv_disp_buf2,LVGL_LCD_BUF_SIZE/2);
#else
    lv_disp_buf = (lv_color_t *)heap_caps_malloc(LVGL_LCD_BUF_SIZE * sizeof(lv_color_t), MALLOC_CAP_DMA | MALLOC_CAP_INTERNAL);
    if(lv_disp_buf==NULL) {
        Serial.println("Out of memory");
        while(1);
    }
    lv_disp_draw_buf_init(&disp_buf, lv_disp_buf,NULL,LVGL_LCD_BUF_SIZE);
#endif
    /*Initialize the display*/
    lv_disp_drv_init(&disp_drv);
    /*Change the following line to your display resolution*/
    disp_drv.hor_res = LCD_H_RES;
    disp_drv.ver_res = LCD_V_RES;
    disp_drv.flush_cb = lvgl_flush;
    disp_drv.draw_buf = &disp_buf;
    disp_drv.user_data = NULL;
    lv_disp_drv_register(&disp_drv);
    static lv_indev_drv_t indev_drv;
    lv_indev_drv_init( &indev_drv );
    indev_drv.type = LV_INDEV_TYPE_POINTER;
    indev_drv.read_cb = touch_read;
    lv_indev_drv_register( &indev_drv );
    is_initialized_lvgl = true;
    Serial.printf("SRAM heap size is %dKB\n",(int)(ESP.getHeapSize()/1024.0));
    Serial.printf("SRAM heap free is %dKB\n",(int)(ESP.getFreeHeap()/1024.0));
    Serial.printf("Largest block is %dKB\n",(int)(ESP.getMaxAllocHeap()/1024.0));
}
void loop() {
    lv_timer_handler();
    delay(5);
}