#ifndef __RAYTRACING_H
#define __RAYTRACING_H

#include "objects.h"
#include <stdint.h>

void raytracing(uint8_t *pixels, color background_color,
                rectangular_node rectangulars, sphere_node spheres,
                light_node lights, const viewpoint *view,
                int width, int height);
#endif
