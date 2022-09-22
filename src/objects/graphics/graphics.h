#pragma once
#include <string>
#include "main.h"
#include "../coordinates/coordinates.h"

class Graphics {

public:
	static lv_obj_t* draw_rectangle(Coordinates coordinates, int width, int height, lv_color_t color);
	static lv_obj_t* draw_text(Coordinates coordinates, std::string text);

};