#pragma once

class Coordinates {

private:
	float coordinates[];

public:
	Coordinates(float x_coordinates, float y_coordinates, float direction);
	float get_distance(Coordinates target_coordinates);
	float get_x();
	float get_y();
	float get_direction();

};