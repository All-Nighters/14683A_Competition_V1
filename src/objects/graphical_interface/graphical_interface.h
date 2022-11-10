#pragma once
#include <iostream>
#include "main.h"

class GraphicalInterface {
public:
    enum InterfaceType {
        MENU_CONTAINER,
        MENU_BUTTON,
        MENU_LABEL,
        SELECTOR_CONTAINER,
        SELECTOR_SIDEBAR_CONTAINER,
        SELECTOR_SIDEBAR_BUTTON,
        SELECTOR_BODY_CONTAINER,
        SELECTOR_BODY_LABEL
    };
    enum InterfaceAction {
        MENU_SELECTOR,
        MENU_UTILITIES,
        MENU_RETURN,
        MENU_MENU,
        SELECTOR_AUTONOMOUS,
        SELECTOR_SKILL
    };
    enum InterfaceStatus {
        HOME,
        SELECTOR,
        UTILITIES
    };
    struct InterfaceComponent {
        lv_obj_t*                         object_pointer;
        lv_obj_t*                         object_parent;
        GraphicalInterface::InterfaceType object_type;
    };

    static InterfaceStatus interface_status;

    GraphicalInterface();
    static void interface_hide_type(GraphicalInterface::InterfaceType object_type, bool object_hidden);
    static void interface_window();

private:
    static std::vector<InterfaceComponent> interface_components;
    static std::vector<lv_style_t>         interface_style;

    void interface_menu();
    void object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates);
    void object_style(GraphicalInterface::InterfaceComponent objects[], int objects_size);

    static std::vector<GraphicalInterface::InterfaceComponent> get_children_by_object(lv_obj_t* object_parent);
    static std::vector<GraphicalInterface::InterfaceComponent> get_children_by_type(GraphicalInterface::InterfaceType object_type);

    lv_obj_t* container_initialize(lv_obj_t* container_parent, GraphicalInterface::InterfaceType container_type);
    lv_obj_t* button_initialize(lv_obj_t* button_parent, std::string button_text, GraphicalInterface::InterfaceType button_type, GraphicalInterface::InterfaceAction button_action);
    lv_obj_t* label_initialize(lv_obj_t* label_parent, std::string label_text, GraphicalInterface::InterfaceType label_type);
};