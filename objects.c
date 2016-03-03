#include <stdlib.h>
#include <stdio.h>

#include "primitives.h"
#include "objects.h"

#define FUNC_BEGIN(name) \
    void append_##name (const name *X, name##_node *list) { \
        name##_node newNode = malloc(sizeof(struct __##name##_node));

#define FUNC_END(name) \
        newNode->next = NULL; \
        if (!*list) { *list = newNode; } \
        else { \
            name##_node tmp; \
            for (tmp = *list; tmp->next; tmp = tmp->next) ; \
            tmp->next = newNode; \
        } \
    } \
    void delete_##name##_list(name##_node *list) { \
	name##_node pos = *list; \
        while (pos) { \
            name##_node tmp = pos->next; \
            free(pos); pos = tmp; \
        } \
        *list = NULL; \
    }

// *INDENT-OFF*
FUNC_BEGIN(light)
    COPY_POINT3(newNode->element.position, X->position);
    COPY_COLOR(newNode->element.light_color, X->light_color);
    newNode->element.intensity = X->intensity;
FUNC_END(light)

FUNC_BEGIN(rectangular)
    COPY_OBJECT_FILL(newNode->element.rectangular_fill, X->rectangular_fill);
    for (int i = 0; i < 4; i++) {
        COPY_POINT3(newNode->element.vertices[i], X->vertices[i]);
        COPY_POINT3(newNode->element.normal, X->normal);
    }
FUNC_END(rectangular)

FUNC_BEGIN(sphere)
    COPY_OBJECT_FILL(newNode->element.sphere_fill, X->sphere_fill);
    newNode->element.radius = X->radius;
    COPY_POINT3(newNode->element.center, X->center);
    FUNC_END(sphere)

// *INDENT-ON*
