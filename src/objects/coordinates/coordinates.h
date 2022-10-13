#pragma once
#include <vector>

class Coordinates {

private:
	float coordinates[3];

public:
	Coordinates(float x_coordinates, float y_coordinates, float direction);
	float get_distance(Coordinates target_coordinates);
	float get_acute(Coordinates target_coordinates);
	float get_x();
	float get_y();
	float get_direction();
	Coordinates get_offset(float x_offset, float y_offset, float direction_offset);
	Coordinates get_resize(float resize_ratio);

	static float get_distance_sum(std::vector<Coordinates> chained_coordinates);

	enum CoordinateType {
		X_COORDINATE,
		Y_COORDINATE,
		DIRECTION
	};
};