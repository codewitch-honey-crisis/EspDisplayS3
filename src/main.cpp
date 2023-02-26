#include <Arduino.h>
#include <Wire.h>

#include <Telegrama.hpp>
#include <ft6236.hpp>
#include <gfx.hpp>
#include <uix.hpp>
#include "lcd_controller.h"
#define I2C_SCL 39
#define I2C_SDA 38
#define LCD_BLK 45
#define LCD_H_RES 320
#define LCD_V_RES 480
#define LCD_LINES 16
using namespace arduino;
using namespace gfx;
using namespace uix;
constexpr static const open_font& text_font = Telegrama;
constexpr static const uint8_t screen_rotation = 3;
constexpr static const size_t buffer_size = 32*1024;
using touch_t = ft6236<LCD_H_RES, LCD_V_RES>;
touch_t touch;
;
using screen_t = screen<LCD_V_RES,LCD_H_RES,rgb_pixel<16>>;
using button_t = push_button<typename screen_t::pixel_type>;
using label_t = label<typename screen_t::pixel_type>;
using color_t = color<typename screen_t::pixel_type>;
using color32_t = color<rgba_pixel<32>>;
uint8_t render_buffer1[buffer_size];
uint8_t render_buffer2[buffer_size];
screen_t main_screen(sizeof(render_buffer1),render_buffer1,render_buffer2);
button_t test_button(main_screen);
label_t test_label(main_screen);
static bool uix_flush_ready(esp_lcd_panel_io_handle_t panel_io, esp_lcd_panel_io_event_data_t* edata, void* user_ctx) {
    main_screen.set_flush_complete();
    return true;
}
static void uix_flush(point16 location,typename screen_t::bitmap_type& bmp, void* state) {
    lcd_flush(location.x,location.y,location.x+bmp.dimensions().width-1,location.y+bmp.dimensions().height-1,bmp.begin());
}
static void uix_touch(point16* out_locations, size_t* in_out_locations_size, void* state) {
    if(in_out_locations_size<=0) {
        *in_out_locations_size=0;
        return;
    }
    if(touch.update()) {
        if(touch.xy(&out_locations[0].x,&out_locations[0].y)) {
            if(*in_out_locations_size>1) {
                *in_out_locations_size = 1;
                if(touch.xy2(&out_locations[1].x,&out_locations[1].y)) {
                    *in_out_locations_size = 2;
                }
            } else {
                *in_out_locations_size = 1;
            }
        } else {
            *in_out_locations_size = 0;
        }
    }
}
void setup() {
    Serial.begin(115200);
    Serial.printf("PSRAM size is %dMB\n", (int)(ESP.getPsramSize() / 1024.0 / 1024.0));
    Wire.begin(I2C_SDA, I2C_SCL);
    lcd_color_trans_done_register_cb(uix_flush_ready, nullptr);
    lcd_init(sizeof(render_buffer1));
    /* Lighten the screen with gradient */
    ledcSetup(0, 10000, 8);
    ledcAttachPin(LCD_BLK, 0);
    for (uint8_t i = 0; i < 0xFF; i++) {
        ledcWrite(0, i);
        delay(2);
    }
    touch.initialize();
    touch.rotation(screen_rotation);
    Serial.printf("SRAM heap size is %dKB\n", (int)(ESP.getHeapSize() / 1024.0));
    Serial.printf("SRAM heap free is %dKB\n", (int)(ESP.getFreeHeap() / 1024.0));
    Serial.printf("Largest block is %dKB\n", (int)(ESP.getMaxAllocHeap() / 1024.0));
    main_screen.background_color(color_t::white);

    test_label.bounds(srect16(spoint16(10,10),ssize16(200,60)));
    test_label.text_color(color32_t::blue);
    test_label.text_open_font(&text_font);
    test_label.text_line_height(50);
    test_label.text_justify(uix_justify::center);
    test_label.round_ratio(NAN);
    test_label.padding({8,8});
    test_label.text("Hello");    
    main_screen.register_control(test_label);

    test_button.bounds(srect16(spoint16(25,25),ssize16(200,100)));
    auto bg = color32_t::light_blue;
    bg.channelr<channel_name::A>(.5);
    test_button.background_color(bg,true);
    test_button.border_color(color32_t::black);
    test_button.text_color(color32_t::black);
    test_button.text_open_font(&text_font);
    test_button.text_line_height(25);
    test_button.text_justify(uix_justify::bottom_right);
    test_button.round_ratio(NAN);
    test_button.padding({8,8});
    test_button.text("Released");
    test_button.on_pressed_changed_callback([](bool pressed,void* state) {test_button.text(pressed?"Pressed":"Released");});
    main_screen.register_control(test_button);

    main_screen.on_flush_callback(uix_flush,nullptr);
    main_screen.on_touch_callback(uix_touch,nullptr);
    
}
void loop() {
    main_screen.update();
    uint16_t x, y;
    touch.update();
    if (touch.xy(&x, &y)) {
        Serial.printf("x: %d, y: %d\n", (int)x, (int)y);
        
    }
}