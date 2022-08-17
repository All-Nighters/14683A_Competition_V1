#pragma once
#include <vector>
#include "../coordinates/coordinates.h"
#include "../catmull_rom/catmull_rom.h"

struct PathFilterCache {
	Coordinates coordinates;
	float chained_distance;
};

class PathFilter {

private:
	CatmullRom spline_object;
	std::vector<PathFilterCache> distance_cache;
	float deviation;

	Coordinates request_coordinates(int coordinates_offset, float spline_progress);

public:
	PathFilter(CatmullRom spline_object, std::vector<Coordinates> anchor_coordinates, float deviation);
	//Coordinates get_next(float chained_progress, float sight_range);
	float get_chained_distance(int coordinates_offset, float spline_progress);
};