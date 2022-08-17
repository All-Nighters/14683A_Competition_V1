#pragma once
#include "../coordinates/coordinates.h"

class Graphics {

public:
	static lv_obj_t* draw_rectangle(Coordinates coordinates, int width, int height, lv_color_t color);

};