#pragma once
#include "main.h"

class GraphicalInterface {

private:
    enum InterfaceComponent {
        MENU_CONTAINER,
        MENU_BUTTON,
        MENU_LABEL
    };
    enum InterfaceAction {
        MENU_SELECTOR,
        MENU_UTILITIES,
        MENU_RETURN,
        MENU_MENU
    };
    struct InterfaceStyle {
        lv_obj_t* object_pointer;
        GraphicalInterface::InterfaceComponent object_type;
    };

    void interface_menu();
    void object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates);
    void object_style(GraphicalInterface::InterfaceStyle objects[], int objects_size);

    lv_obj_t* button_initialize(lv_obj_t* button_parent, std::string button_text, GraphicalInterface::InterfaceAction button_action);
    lv_obj_t* label_initialize(lv_obj_t* label_parent, std::string label_text);

public:
    GraphicalInterface();

};