#pragma once
#include "main.h"

class GraphicalInterface {

private:
    enum InterfaceComponent {
        MENU_CONTAINER,
        MENU_BUTTON
    };
    void interface_menu();
    void object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates);
    lv_style_t GraphicalInterface::object_style(lv_obj_t* object, GraphicalInterface::InterfaceComponent type);

public:
    GraphicalInterface();

};