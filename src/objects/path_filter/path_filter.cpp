#include <vector>
#include <cmath>
#include "path_filter.h"
#include "../catmull_rom/catmull_rom.h"
#include "../coordinates/coordinates.h"

#include "../graphics/graphics.h"
#include "main.h"
#include <sstream>

/**
 * Initialize a PathFilter object
 * 
 * @param spline_object the spline generator object
 * @param deviation the maximum acceptable deviation
 */
PathFilter::PathFilter(CatmullRom spline_object, float deviation) {
	this->spline_object = spline_object;
	this->deviation = deviation;
	this->distance_cache.push_back(0.0f); // 0
	this->distance_cache.push_back(0.0f); // 1
}

// can't skip anchor points
// binary search for finding precision
// use cache
Coordinates PathFilter::get_next(float chained_distance, float sight_range) {
	return this->get_chained_coordinates(chained_distance + sight_range);
}

/**
 * Calculates the chained distance from the beginning of the path
 * 
 * @param coordinates_offset the offset of the coordinates
 * @param spline_progress the progress point in the spline
 * @returns the chained distance from the beginning of the path
 */
float PathFilter::get_chained_distance(int coordinates_offset, float spline_progress) { // 4 - 0
	int cached_coordinate = std::min((int) this->distance_cache.size() - 1, coordinates_offset); // 1/4 -> 1
	float sum_distance = this->distance_cache[cached_coordinate]; // [1]->0

	std::stringstream ss;
	ss << "cached to " << this->distance_cache.size() - 1 << ", requested to " << coordinates_offset << ", start at " << cached_coordinate << " sum=" << sum_distance;
	printf("%s\n", ss.str().c_str());
	
	for (int coordinate_index = cached_coordinate + 1; coordinate_index <= coordinates_offset + 1; coordinate_index++) { // 2~5
		std::vector<Coordinates> spline_coordinates;
		float coordinate_progress_maximum = (coordinate_index == (coordinates_offset + 1) ? spline_progress : 1.0f); // 2!=4 3~=4 4==4 -> 1, 1, 0
		for (float progress_index = 0; progress_index < coordinate_progress_maximum; progress_index += this->deviation) {
			spline_coordinates.push_back(this->request_coordinates(coordinate_index - 2, progress_index)); // HERE
			//Graphics::draw_rectangle(spline_coordinates[spline_coordinates.size() - 1].get_resize(4.0f).get_offset(0.0f, 5.0f, 0.0f), 5, 5, LV_COLOR_PURPLE);
		}
		spline_coordinates.push_back(this->request_coordinates(coordinate_index - 2, 1.0f)); // HERE
		float spline_length = Coordinates::get_distance_sum(spline_coordinates);

		float before = sum_distance;

		printf("Spline Length: %f\n", spline_length);
		sum_distance += spline_length;

		std::stringstream ss2;
		ss2 << "before=" << before << " after=" << sum_distance;
		printf("%s\n", ss2.str().c_str());

		if (this->distance_cache.size() == coordinate_index) {
			this->distance_cache.push_back(sum_distance);

			std::stringstream ss2;
			ss2 << "expanded cache to " << coordinate_index << ", value: " << sum_distance << " (increase by " << spline_length << ")";

			if (spline_length > 100) {
				for (int index = 0; index < spline_coordinates.size(); index++) {
					Coordinates loop = spline_coordinates[index];
					ss2 << "\n" << loop.get_x() << "-" << loop.get_y();
				}
			}

			printf("%s\n", ss2.str().c_str());


		}
	}
	return sum_distance;
}

/**
 * Obtains the coordinate of a point, with the chained distance from the beginning of the path
 * 
 * @param chained_distance the chained distance from the beginning of the path
 * @returns the coordinate of a point with chained distance from the beginning of the path
 */
Coordinates PathFilter::get_chained_coordinates(float chained_distance) {
	int base_coordinate = this->get_base_coordinate(chained_distance);
	float base_distance = this->distance_cache[base_coordinate];
	if (chained_distance == base_distance) {
		// exactly on anchor point
		return this->request_coordinates(base_coordinate, 0.0f);
	}
	// find point by binary search
	float progress_minimum = 0.0f, progress_maximum = 1.0f;
	while (true) {
		float progress_average = (progress_minimum + progress_maximum) / 2.0f;
		float progress_distance = this->get_chained_distance(base_coordinate, progress_average);
		float progress_deviation = std::abs(chained_distance - progress_distance);
		if (progress_deviation <= this->deviation || (progress_maximum - progress_minimum) <= this->deviation) {
			// close enough or no other choice, return it
			return this->request_coordinates(base_coordinate - 1, progress_average);
		} else if (progress_distance < chained_distance) {
			progress_minimum = progress_average;
		}
		else {
			progress_maximum = progress_average;
		}
	}
}

/**
 * Obtain the base coordinate of a point, with the chained distance from the beginning of the path
 * 
 * @param chained_distance the chained distance from the beginning of the path
 * @returns the base coordinate of a point, with the chained distance from the beginning of the path
 */
int PathFilter::get_base_coordinate(float chained_distance) {
	int cached_last_coordinate = this->distance_cache.size() - 1;
	float cached_last_distance = this->distance_cache[cached_last_coordinate];
	int maximum_coordinate = (this->spline_object.get_anchors().size() - 2);
	if (cached_last_distance < chained_distance) {
		// request chained distance out of bounds
		if (maximum_coordinate <= cached_last_coordinate) {
			// no more cache available
			return 0;
		}
		// generate cache
		this->get_chained_distance(maximum_coordinate, 0.0f);
	}
	// find maximum base coordinate
	for (int coordinate_index = 2; coordinate_index <= maximum_coordinate; coordinate_index++) {
		if (this->distance_cache[coordinate_index] < chained_distance) {
			continue;
		}
		return coordinate_index - 1;
	}
	return 0;
}

/**
 * Request the coordinate of a point on the spline
 * 
 * @param coordinates_offset the offset of the coordinates
 * @param spline_progress the progress point in the spline
 * @returns the coordinate of a point with chained distance from the beginning of the path
 */
Coordinates PathFilter::request_coordinates(int coordinates_offset, float spline_progress) {
	return this->spline_object.get_coordinates(coordinates_offset, spline_progress);
}