#include "main.h"
#include "graphics.h"
#include "../coordinates/coordinates.h"

/**
 * Draws a rectangle on Vex Brain
 * 
 * @param coordinates the coordinates (top-left) of the rectangle
 * @param width the width of the rectangle
 * @param height the height of the rectangle
 * @param color the color of the rectangle
 * @returns the rectangle shape object
 */
lv_obj_t* Graphics::draw_rectangle(Coordinates coordinates, int width, int height, lv_color_t color) {
    // method copied from https://www.vexforum.com/t/lvgl-how-to-draw-a-rectangle-using-lvgl/50977/6
    lv_obj_t* rectangle = lv_obj_create(lv_scr_act(), NULL);
    // create object style
    lv_style_t* rectangle_style = (lv_style_t*)malloc(sizeof(lv_style_t));
    lv_style_copy(rectangle_style, &lv_style_plain_color);
    rectangle_style->body.empty = 1;
    rectangle_style->body.border.color = color;
    rectangle_style->body.border.width = 1;
    rectangle_style->body.border.part = LV_BORDER_FULL;
    // apply object style
    lv_obj_set_style(rectangle, rectangle_style);
    lv_obj_set_pos(rectangle, coordinates.get_x(), coordinates.get_y());
    lv_obj_set_size(rectangle, width, height);
    return rectangle;
}