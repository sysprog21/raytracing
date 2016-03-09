#ifndef __RAY_PRIMITIVES_H
#define __RAY_PRIMITIVES_H

typedef double point3[3];
typedef double point4[3];
typedef double color[3];

typedef struct {
    color light_color; /**< scale (0,1) */
    point3 position;
    double intensity;
} light;

typedef struct {
    color fill_color; /**< RGB is in terms of 0.0 to 1.0 */
    double Kd; /**< the diffuse component */
    double Ks; /**< the specular */
    double T;  /**< transmittance (fraction of light passed per unit) */
    double R;  /**< reflectance (effectiveness in reflecting)*/
    double index_of_refraction;
    double phong_power; /**< the Phong cosine power for highlights */
} object_fill;

typedef struct {
    point3 center;
    double radius;
    object_fill sphere_fill;
} sphere;

typedef struct {
    point3 vertices[4];
    point3 normal;
    object_fill rectangular_fill;
} rectangular;

typedef struct {
    point3 vrp;
    point3 vpn;
    point3 vup;
} viewpoint;

typedef struct {
    point3 point;
    point3 normal;
} intersection;

#define COPY_OBJECT_FILL(a, b) { \
    (a).fill_color[0] = (b).fill_color[0]; \
    (a).fill_color[1] = (b).fill_color[1]; \
    (a).fill_color[2] = (b).fill_color[2]; \
    (a).Kd = (b).Kd; \
    (a).Ks = (b).Ks; \
    (a).R  = (b).R;  \
    (a).phong_power = (b).phong_power; \
    (a).T = (b).T; \
    (a).index_of_refraction = (b).index_of_refraction; }

#define COPY_POINT3(a, b) { \
    (a)[0] = (b)[0]; \
    (a)[1] = (b)[1]; \
    (a)[2] = (b)[2]; }

#define COPY_COLOR(a, b) { \
    (a)[0] = (b)[0]; \
    (a)[1] = (b)[1]; \
    (a)[2] = (b)[2]; }

#define SET_COLOR(r, R, G, B) { \
    (r)[0] = (R); \
    (r)[1] = (G); \
    (r)[2] = (B); }

#endif
