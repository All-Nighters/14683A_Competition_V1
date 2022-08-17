#include <vector>
#include "path_filter.h"
#include "../catmull_rom/catmull_rom.h"
#include "../coordinates/coordinates.h"

// removed callback function Coordinates(*callback_function)()

PathFilter::PathFilter(CatmullRom spline_object, std::vector<Coordinates> anchor_coordinates, float deviation) {
	this->spline_object = spline_object;
	this->deviation = deviation;
}

// can't skip anchor points
// binary search for finding precision
// use cache
/*Coordinates PathFilter::get_next(float chained_progress, float sight_range) {
	
}*/

float PathFilter::get_chained_distance(int coordinates_offset, float spline_progress) {
	if (coordinates_offset <= 1 && spline_progress <= 0) {
		return 0.0f;
	}
	// TODO: add distance cache heres
	std::vector<Coordinates> chained_coordinates;
	for (int coordinate_index = 1; coordinate_index < (coordinates_offset + 1); coordinate_index++) {
		// request points on graph, until reaches progress
		float spline_progress_maximum = coordinate_index < coordinates_offset ? 1.0f : spline_progress;
		for (float progress_index = 0; progress_index < spline_progress_maximum; progress_index += this->deviation) {
			chained_coordinates.push_back(this->request_coordinates(coordinate_index, progress_index));
		}
	}
	// get the distance between every coordinates
	return Coordinates::get_distance_sum(chained_coordinates);
}

Coordinates PathFilter::request_coordinates(int coordinates_offset, float spline_progress) {
	return this->spline_object.get_coordinates(coordinates_offset, spline_progress);
}