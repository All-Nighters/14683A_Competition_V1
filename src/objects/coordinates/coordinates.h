#pragma once
#include <vector>

class Coordinates {

private:
	float coordinates[3];

public:
	Coordinates(float x_coordinates, float y_coordinates, float direction);
	float get_distance(Coordinates target_coordinates);
	float get_x();
	float get_y();
	float get_direction();

	static float get_distance_sum(std::vector<Coordinates> chained_coordinates);

	enum CoordinateType {
		X_COORDINATE,
		Y_COORDINATE,
		DIRECTION
	};
};