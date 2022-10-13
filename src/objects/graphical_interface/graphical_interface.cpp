#include "graphical_interface.h"
#include "../../datas/constants.h"
#include "main.h"

#define WIDTH  Constants::GraphicalInterface::SCREEN_WIDTH
#define HEIGHT Constants::GraphicalInterface::SCREEN_HEIGHT

GraphicalInterface::GraphicalInterface() {
    this->interface_menu();
}

void GraphicalInterface::interface_menu() {
    // header
    lv_obj_t* header_container = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_t* header_selector  = lv_btn_create(header_container, NULL);
    lv_obj_t* header_utilities = lv_btn_create(header_container, NULL);
    this->object_scale(header_container, 480, 50, 0, 0);
    this->object_scale(header_selector , 100, 50, 0, 0);
    this->object_scale(header_utilities, 100, 50, 100, 0);
    // footer
    lv_obj_t* footer_container = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_t* footer_return    = lv_btn_create(footer_container, NULL);
    lv_obj_t* footer_menu      = lv_btn_create(footer_container, NULL);
    this->object_scale(footer_container, 480, 50, 0, (HEIGHT - 50));
    this->object_scale(footer_return   , 100, 50, 0, 0);
    this->object_scale(footer_menu     , 100, 50, (WIDTH - 100), 0);
}

void GraphicalInterface::object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates) {
    lv_obj_set_size(object, width, height);
	lv_obj_align(object, NULL, LV_ALIGN_IN_TOP_LEFT, x_coordinates, y_coordinates);
}

lv_style_t GraphicalInterface::object_style(lv_obj_t* object, GraphicalInterface::InterfaceComponent type) {
    static lv_style_t object_style;
	lv_style_copy(&object_style, &lv_style_btn_rel);
    switch (type) {
        case GraphicalInterface::InterfaceComponent::MENU_CONTAINER:
            object_style.body.main_color = LV_COLOR_MAKE(255, 255, 255);
	        object_style.body.grad_color = LV_COLOR_MAKE(255, 255, 255);
            object_style.body.border.width = 1; // border line width
	        object_style.body.radius = 0;
            break;
    }
    return object_style;
}