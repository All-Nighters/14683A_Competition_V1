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
    lv_obj_t* header_selector  = this->button_initialize(header_container, "Selector");
    lv_obj_t* header_utilities = this->button_initialize(header_container, "Utilities");
    lv_obj_t* header_label     = this->label_initialize(header_container, "  14683A");
    this->object_scale(header_container, 480, 50, 0, 0);
    this->object_scale(header_selector , 100, 50, 0, 0);
    this->object_scale(header_utilities, 100, 50, 100, 0);
    this->object_scale(header_label, 100, 16, (WIDTH - 100), (50 / 3));
    // footer
    lv_obj_t* footer_container = lv_cont_create(lv_scr_act(), NULL);
    lv_obj_t* footer_return    = this->button_initialize(footer_container, "Return");
    lv_obj_t* footer_menu      = this->button_initialize(footer_container, "Menu");
    this->object_scale(footer_container, 480, 50, 0, (HEIGHT - 50));
    this->object_scale(footer_return   , 100, 50, 0, 0);
    this->object_scale(footer_menu     , 100, 50, (WIDTH - 100), 0);
    // style
    GraphicalInterface::InterfaceStyle header_renderer[7] = {
        {header_container, GraphicalInterface::InterfaceComponent::MENU_CONTAINER},
        {header_selector,  GraphicalInterface::InterfaceComponent::MENU_BUTTON},
        {header_utilities, GraphicalInterface::InterfaceComponent::MENU_BUTTON},
        {header_label,     GraphicalInterface::InterfaceComponent::MENU_LABEL},
        {footer_container, GraphicalInterface::InterfaceComponent::MENU_CONTAINER},
        {footer_return,    GraphicalInterface::InterfaceComponent::MENU_BUTTON},
        {footer_menu,      GraphicalInterface::InterfaceComponent::MENU_BUTTON},
    };
    this->object_style(header_renderer, sizeof(header_renderer) / sizeof(header_renderer[0]));
}

void GraphicalInterface::object_scale(lv_obj_t* object, lv_coord_t width, lv_coord_t height, lv_coord_t x_coordinates, lv_coord_t y_coordinates) {
    lv_obj_set_size(object, width, height);
	lv_obj_align(object, NULL, LV_ALIGN_IN_TOP_LEFT, x_coordinates, y_coordinates);
}

void GraphicalInterface::object_style(GraphicalInterface::InterfaceStyle objects[], int objects_size) {
    std::vector<lv_style_t> object_style;
    int object_style_offset = 0;
    for (int object_index = 0; object_index < objects_size; object_index++) {
        GraphicalInterface::InterfaceStyle object_loop = objects[object_index];
        int object_style_size = 0;
        // allocate new styles
        switch (object_loop.object_type) {
            case GraphicalInterface::InterfaceComponent::MENU_CONTAINER:
                object_style_size = 1;
                break;
            case GraphicalInterface::InterfaceComponent::MENU_BUTTON:
                object_style_size = 2;
                break;
            case GraphicalInterface::InterfaceComponent::MENU_LABEL:
                object_style_size = 1;
                break;
        }
        for (int style_index = 0; style_index < object_style_size; style_index++) {
            object_style.push_back(lv_style_t());
            lv_style_copy(&object_style[object_style_offset + style_index], &lv_style_pretty);
        }
        // styles by type
        switch (object_loop.object_type) {
            case GraphicalInterface::InterfaceComponent::MENU_CONTAINER:
                object_style[object_style_offset + 0].body.main_color = Constants::GraphicalInterface::CONTAINER_BACKGROUND;
                object_style[object_style_offset + 0].body.grad_color = Constants::GraphicalInterface::CONTAINER_BACKGROUND;
                object_style[object_style_offset + 0].body.border.width = 1; // border line width
                object_style[object_style_offset + 0].body.radius = 0;
                lv_obj_set_style(object_loop.object_pointer, &object_style[object_style_offset + 0]);
                break;
            case GraphicalInterface::InterfaceComponent::MENU_BUTTON:
                // 0=RELEASE 1=PRESS
                object_style[object_style_offset + 0].body.main_color = Constants::GraphicalInterface::BUTTON_BACKGROUND_RELEASED;
                object_style[object_style_offset + 0].body.grad_color = Constants::GraphicalInterface::BUTTON_BACKGROUND_RELEASED;
                object_style[object_style_offset + 0].text.color      = Constants::GraphicalInterface::BUTTON_FOREGROUND_RELEASED;
                object_style[object_style_offset + 0].body.border.width = 1; // border line width
                object_style[object_style_offset + 0].body.radius = 2;
                object_style[object_style_offset + 1].body.main_color = Constants::GraphicalInterface::BUTTON_BACKGROUND_PRESSED;
                object_style[object_style_offset + 1].body.grad_color = Constants::GraphicalInterface::BUTTON_BACKGROUND_PRESSED;
                object_style[object_style_offset + 1].text.color      = Constants::GraphicalInterface::BUTTON_FOREGROUND_PRESSED;
                object_style[object_style_offset + 1].body.border.width = 1; // border line width
                object_style[object_style_offset + 1].body.radius = 2;
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_REL, &object_style[object_style_offset + 0]);
                lv_btn_set_style(object_loop.object_pointer, LV_BTN_STYLE_PR,  &object_style[object_style_offset + 1]);
                break;
            case GraphicalInterface::InterfaceComponent::MENU_LABEL:
                object_style[object_style_offset + 0].text.color      = Constants::GraphicalInterface::LABEL_FOREGROUND;
                lv_obj_set_style(object_loop.object_pointer, &object_style[object_style_offset + 0]);
                break;
        }
        // shift styles offset
        object_style_offset += object_style_size;
    }
}

lv_obj_t* GraphicalInterface::button_initialize(lv_obj_t* button_parent, std::string button_text) {
    lv_obj_t* button_object  = lv_btn_create(button_parent, NULL);
    lv_obj_t* button_label = this->label_initialize(button_object, button_text);
    return button_object;
}

lv_obj_t* GraphicalInterface::label_initialize(lv_obj_t* label_parent, std::string label_text) {
    lv_obj_t* label_object = lv_label_create(label_parent, NULL);
	lv_label_set_text(label_object, label_text.c_str());
    return label_object;
}