#include <cmath>
#include "coordinates.h"

/**
 * Initialize a coordinates object, with given x-coordinate, y-coordinate, and direction
 * 
 * @param x_coordinates the x-coordinate of the location
 * @param y_coordinates the y-coordinate of the location
 * @param direction the direction (in degrees) of the location
 */
Coordinates::Coordinates(float x_coordinates, float y_coordinates, float direction) {
	float new_coordinates[] = {x_coordinates, y_coordinates, direction};
	this->coordinates = new_coordinates;
}

/**
 * Calculates the distance between two coordinates object
 *
 * @param target_coordinates the target coordinates
 * @returns the distance between two coordinates object
 */
float Coordinates::get_distance(Coordinates target_coordinates) {
	float x_difference = target_coordinates.get_x() - this->get_x();
	float y_difference = target_coordinates.get_y() - this->get_y();
	return std::sqrt(std::pow(x_difference, 2) + std::pow(y_difference, 2));
}

/**
 * Access the x-coordinate of the location
 * 
 * @returns the x-coordinate of the location
 */
float Coordinates::get_x() {
	return this->coordinates[0];
}

/**
 * Access the y-coordinate of the location
 * 
 * @returns the y-coordinate of the location
 */
float Coordinates::get_y() {
	return this->coordinates[1];
}

/*
 * Access the direction (in degrees) of the location
 * 
 * @returns the direction (in degrees) of the location
 */
float Coordinates::get_direction() {
	return this->coordinates[2];
}