#pragma once
#include <vector>
#include "../coordinates/coordinates.h"

class CatmullRom {

private:
	std::vector<Coordinates> coordinates;

	float get_coordinate(int coordinate_offset, Coordinates::CoordinateType coordinate_type, float spline_progress);
	float get_degree_influence(int influence_degree, float spline_progress);

public:
	CatmullRom(std::vector<Coordinates> coordinates);
	Coordinates get_coordinates(int coordinate_offset, float spline_progress);

};