#pragma once
#include <iostream>
#include "main.h"

class GraphicalInterface {
public:
    enum InterfaceType {
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
    struct InterfaceComponent {
        lv_obj_t*                         object_pointer;
        lv_obj_t*                         object_parent;
        GraphicalInterface::InterfaceType object_type;
    };

    GraphicalInterface();
    static void interface_hide_type(GraphicalInterface::InterfaceType object_type, bool object_hidden);

private:
    static std::vector<InterfaceComponent> interface_components;

    void interface_menu();
    void object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates);
    void object_style(GraphicalInterface::InterfaceComponent objects[], int objects_size);

    static std::vector<GraphicalInterface::InterfaceComponent> get_children_by_object(lv_obj_t* object_parent);
    static std::vector<GraphicalInterface::InterfaceComponent> get_children_by_type(GraphicalInterface::InterfaceType object_type);

    lv_obj_t* container_initialize(lv_obj_t* container_parent);
    lv_obj_t* button_initialize(lv_obj_t* button_parent, std::string button_text, GraphicalInterface::InterfaceAction button_action);
    lv_obj_t* label_initialize(lv_obj_t* label_parent, std::string label_text);
};