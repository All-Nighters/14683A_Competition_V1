#include <vector>
#include "path_filter.h"
#include "../catmull_rom/catmull_rom.h"
#include "../coordinates/coordinates.h"

// removed callback function Coordinates(*callback_function)()

PathFilter::PathFilter(CatmullRom spline_object, std::vector<Coordinates> anchor_coordinates, float deviation) {
	this->spline_object = spline_object;
	this->deviation = deviation;
	this->distance_cache.push_back(0.0f); // 0
	this->distance_cache.push_back(0.0f); // 1
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
	// load distance cache
	int cached_coordinate = std::min((int) this->distance_cache.size() - 1, coordinates_offset);
	float cached_distance = this->distance_cache[cached_coordinate];
	// calculate the remaining distance
	std::vector<Coordinates> chained_coordinates;
	float chained_distance = cached_distance;
	for (int coordinate_index = cached_coordinate; coordinate_index < (coordinates_offset + 1); coordinate_index++) {
		// request points on graph, until reaches progress
		float spline_progress_maximum = coordinate_index < coordinates_offset ? 1.0f : spline_progress;
		for (float progress_index = 0; progress_index < spline_progress_maximum; progress_index += this->deviation) {
			Coordinates progress_coordinates = this->request_coordinates(coordinate_index, progress_index);
			chained_coordinates.push_back(progress_coordinates);
			// check cache availability
			if (progress_index != 0 || this->distance_cache.size() != coordinate_index) {
				// already cached or somehow couldn't cache
				continue;
			}
			// cache anchor point distance
			chained_distance += Coordinates::get_distance_sum(chained_coordinates);
			this->distance_cache.push_back(chained_distance);
			// clear calculated progress coordinates
			chained_coordinates.clear();
			chained_coordinates.push_back(progress_coordinates);
		}
	}
	// get the distance between every coordinates
	return chained_distance + Coordinates::get_distance_sum(chained_coordinates);
}

Coordinates PathFilter::request_coordinates(int coordinates_offset, float spline_progress) {
	return this->spline_object.get_coordinates(coordinates_offset, spline_progress);
}