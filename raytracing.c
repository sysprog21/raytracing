#include <stdio.h>
#include <stdlib.h>

#include "math-toolkit.h"
#include "primitives.h"
#include "raytracing.h"

#define MAX_REFLECTION_BOUNCES	3
#define MAX_DISTANCE 1000000000000.0
#define MIN_DISTANCE 0.00001

#define SQUARE(x) (x * x)
#define MAX(a, b) (a > b ? a : b)

/* @param t t distance
 * @return 1 means hit, otherwise 0
 */
static int raySphereIntersection(const point3 ray_e,
                                 const point3 ray_d,
                                 const sphere *sph,
                                 point3 surface_normal,
                                 double *t1)
{
    point3 l, p;
    subtract_vector(sph->center, ray_e, l);
	double s = dot_product(l, ray_d);
	double l2 = dot_product(l, l);
	double r2 = sph->radius*sph->radius;

	if(s<0 && l2 > r2)
		return 0;
	float m2 = l2 - s*s;
	if(m2>r2)
		return 0;
	float q = sqrt(r2 - m2);
	if(l2>r2)
		*t1 = s - q;
	else
		*t1 = s + q;
    // p=e+t1*d
    multiply_vector(ray_d, *t1, p);
    add_vector(ray_e, p, p);

    subtract_vector(p, sph->center, surface_normal);
    normalize(surface_normal);

	return 1;
}

/* @return 1 means hit, otherwise 0; */
static int rayRectangularIntersection(const point3 ray_e,
                                      const point3 ray_d,
                                      rectangular *rec,
                                      point3 surface_normal,
                                      double *t1)
{
    point3 e01, e03, p;
    subtract_vector(rec->vertices[1], rec->vertices[0], e01);
    subtract_vector(rec->vertices[3], rec->vertices[0], e03);

    cross_product(ray_d, e03, p);

    double det = dot_product(e01, p);

    // Reject rays orthagonal to the normal vector. I.e. rays parallell to the plane.
    if(det < 1e-4)
        return 0;
    
    double inv_det = 1.0/det;

    point3 s;
    subtract_vector(ray_e, rec->vertices[0], s);

    double alpha = inv_det * dot_product(s, p);

    if(alpha > 1.0)
        return 0;
    if(alpha < 0.0)
        return 0;

    point3 q;
    cross_product(s, e01, q);

    double beta = inv_det * dot_product(ray_d, q);
    if(beta > 1.0)
        return 0;
    if(beta < 0.0)
        return 0;

    *t1 = inv_det * dot_product(e03, q);

    point3 intersect;
    if(alpha + beta > 1.0f) {
        /* for the second triangle */
        point3 e23, e21;
        subtract_vector(rec->vertices[3], rec->vertices[2], e23);
        subtract_vector(rec->vertices[1], rec->vertices[2], e21);
        
        cross_product(ray_d, e21, p);

        det = dot_product(e23, p);

        if(det < 1e-4)
            return 0;

        inv_det = 1.0/det;
        subtract_vector(ray_e, rec->vertices[2], s);
        
        alpha = inv_det * dot_product(s, p);
        if(alpha < 0.0)
            return 0;
        
        cross_product(s, e23, q);
        beta = inv_det * dot_product(ray_d, q);

        if(beta < 0.0)
            return 0;

        if(beta + alpha > 1.0)
            return 0;
 
        *t1 = inv_det * dot_product(e21, q);
    }
    
    if(*t1 < 1e-4)
        return 0;
    
    COPY_POINT3(surface_normal, rec->normal);
    subtract_vector(intersect, ray_e, intersect);
    return 1;
}

static void localColor(color local_color,
                       const color light_color, double diffuse,
                       double specular, const object_fill *fill)
{
    color ambi = { 0.1, 0.1, 0.1 };
    color diff, spec, lightCo,surface;

    /* Local Color = ambient * surface +
     *               light * ( kd * surface * diffuse + ks * specular)
     */

    COPY_COLOR(diff, fill->fill_color);
    multiply_vector(diff, fill->Kd, diff);
    multiply_vector(diff, diffuse, diff);
    COPY_COLOR(lightCo, light_color);
    multiply_vectors(diff, lightCo, diff);

    COPY_COLOR(spec, light_color);
    multiply_vector(spec, fill->Ks, spec);
    multiply_vector(spec, specular, spec);

    COPY_COLOR(surface, fill->fill_color);
    multiply_vectors(ambi,surface, ambi);
    add_vector(diff, ambi, diff);
    add_vector(diff, spec, diff);
    add_vector(local_color, diff, local_color);
}

/* @param d direction of the ray into intersection
 * @param l direction of intersection to light
 * @param n surface normal
 */
static void compute_specular_and_diffuse(double *diffuse, double *specular,
        const point3 d, const point3 l,
        const point3 n, double phong_pow)
{
    point3 d_copy, l_copy, middle, r;

    /* Calculate vector to eye V */
    COPY_POINT3(d_copy, d);
    multiply_vector(d_copy, -1, d_copy);
    normalize(d_copy);

    /* Calculate vector to light L */
    COPY_POINT3(l_copy, l);
    multiply_vector(l_copy, -1, l_copy);
    normalize(l_copy);

    /* Calculate reflection direction R */
    double tmp = dot_product(n, l_copy);
    multiply_vector(n, tmp, middle);
    multiply_vector(middle, 2, middle);
    subtract_vector(middle, l_copy, r);
    normalize(r);

    /* diffuse = max(0, dot_product(n, -l)) */
    *diffuse = MAX(0, dot_product(n, l_copy));

    /* specular = (dot_product(r, -d))^p */
    *specular = pow(MAX(0, dot_product(r, d_copy)), phong_pow);
}

/* @param r direction of reflected ray
 * @param d direction of primary ray into intersection
 * @param n surface normal at intersection
 */
static void reflection(point3 r, const point3 d, const point3 n)
{
    /* r = d - 2(d . n)n */
    multiply_vector(n, -2.0 * dot_product(d, n), r);
    add_vector(r, d, r);
}

static void refraction(point3 t, const point3 d, const point3 n,
                       double from, double to)
{
    /* t = (n_f(d - n(d.n)))/n_t - n*sqrt(1 - (n_f^2(1 - (d.n)^2))/n_t^2) */
    point3 tmp;

    multiply_vector(n, dot_product(d, n), tmp);
    subtract_vector(d, tmp, t);
    multiply_vector(t, from, t);
    multiply_vector(t, 1.0 / to, t);

    double x = sqrt(1 - ((SQUARE(from) *
                          (1 - SQUARE(dot_product(d, n)))) / SQUARE(to)));
    multiply_vector(n, x, tmp);
    subtract_vector(t, tmp, t);
}

/* @param t distance */
static double ray_hit_object(const point3 e, const point3 d,
                             double t0, double t1,
                             point3 normal,
                             const rectangular_node rectangulars,
                             rectangular_node *hit_rectangular,
                             const rectangular_node last_rectangular,
                             const sphere_node spheres,
                             sphere_node *hit_sphere,
                             const sphere_node last_sphere)
{
    /* set these to not hit */
    *hit_rectangular = NULL;
    *hit_sphere = NULL;

    point3 biased_e;
    multiply_vector(d, t0, biased_e);
    add_vector(biased_e, e, biased_e);

    double nearest = t1;
    point3 tmpnormal;

    for (rectangular_node rec = rectangulars; rec; rec = rec->next) {
        if (rec == last_rectangular)
            continue;

        if (rayRectangularIntersection(biased_e, d, &(rec->element), tmpnormal,
                                       &t1) && t1<nearest) {
            /* hit is closest so far */
            *hit_rectangular = rec;
            nearest = t1;
            COPY_POINT3(normal, tmpnormal);
        }
    }

    /* check the spheres */
    for (sphere_node sphere = spheres; sphere; sphere = sphere->next) {
        if (sphere == last_sphere)
            continue;

        if (raySphereIntersection(biased_e, d, &(sphere->element), tmpnormal,
                                  &t1) && t1<nearest) {
            *hit_sphere = sphere;
            *hit_rectangular = NULL;
            nearest = t1;
            COPY_POINT3(normal, tmpnormal);
        }
    }

    return nearest;
}

/* @param d direction of ray
 * @param w basic vectors
 */
static void rayConstruction(point3 d, const point3 u, const point3 v,
                            const point3 w, unsigned int i, unsigned int j,
                            const viewpoint *view, unsigned int width,
                            unsigned int height)
{
    double xmin = -0.0175;
    double ymin = -0.0175;
    double xmax = 0.0175;
    double ymax = 0.0175;
    double focal = 0.05;

    double u_s, v_s, w_s;
    point3 u_tmp, v_tmp, w_tmp, s;

    w_s = focal;
    u_s = xmin + ((xmax - xmin) * (float)i / (width - 1));
    v_s = ymax + ((ymin - ymax) * (float)j / (height - 1));

    /* s = e + u_s * u + v_s * v + w_s * w */
    multiply_vector(u, u_s, u_tmp);
    multiply_vector(v, v_s, v_tmp);
    multiply_vector(w, w_s, w_tmp);
    add_vector(view->vrp, u_tmp, s);
    add_vector(s, v_tmp, s);
    add_vector(s, w_tmp, s);

    /* p(t) = e + td = e + t(s - e) */
    subtract_vector(s, view->vrp, d);
    normalize(d);
}

static void calculateBasisVectors(point3 u, point3 v, point3 w,
                                  const viewpoint *view)
{
    /* w  */
    COPY_POINT3(w, view->vpn);
    normalize(w);

    /* u = (t x w) / (|t x w|) */
    cross_product(w, view->vup, u);
    normalize(u);

    /* v = w x u */
    cross_product(u, w, v);

    normalize(v);
}

/* @brief protect color value overflow */
static void protect_color_overflow(color c)
{
    for (int i = 0; i < 3; i++)
        if (c[i] > 1.0) c[i] = 1.0;
}

static unsigned int ray_color(const point3 e, int t,
                              const point3 d,
                              const rectangular_node rectangulars,
                              const sphere_node spheres,
                              const light_node lights,
                              color object_color, int bounces_left,
                              const rectangular_node last_rectangular,
                              const sphere_node last_sphere)
{
    rectangular_node hit_rec = NULL, light_hit_rec = NULL;
    sphere_node hit_sphere = NULL, light_hit_sphere = NULL;
    double t1 = MAX_DISTANCE, diffuse, specular;
    point3 p, surface_normal, l, _l, ignore_me, r;
    object_fill fill;

    color reflection_part;
    color refraction_part;
    /* might be a reflection ray, so check how many times we've bounced */
    if (bounces_left < 0) {
        SET_COLOR(object_color, 0.0, 0.0, 0.0);
        return 0;
    }

    /* check for intersection with a sphere or a rectangular */
    t1 = ray_hit_object(e, d, t, MAX_DISTANCE, surface_normal, rectangulars,
                        &hit_rec, last_rectangular, spheres, &hit_sphere,
                        last_sphere);
   
    if (!hit_rec && !hit_sphere)
        return 0;

    /* p = e + t * d */
    multiply_vector(d, t1, p);
    add_vector(e, p, p);

    /* pick the fill of the object that was hit */
    fill = hit_rec ?
           hit_rec->element.rectangular_fill :
           hit_sphere->element.sphere_fill;

    /* assume it is a shadow */
    SET_COLOR(object_color, 0.0, 0.0, 0.0);

    for (light_node light = lights; light; light = light->next) {
        /* calculate the intersection vector pointing at the light */
        subtract_vector(p, light->element.position, l);
        multiply_vector(l, -1, _l);
        normalize(_l);

        /* check for intersection with an object. use ignore_me
         * because we don't care about this normal
        */
        ray_hit_object(p, _l, MIN_DISTANCE, length(l), ignore_me,
                       rectangulars, &light_hit_rec, hit_rec,
                       spheres, &light_hit_sphere, hit_sphere);

        /* the light was not block by itself(lit object) */
        if (light_hit_rec || light_hit_sphere)
            continue;


        compute_specular_and_diffuse(&diffuse, &specular, d, l,
                                     surface_normal, fill.phong_power);

        localColor(object_color, light->element.light_color,
                   diffuse, specular, &fill);

        /* totalColor = localColor + Ks*reflection + T*refraction */
        if (fill.Ks > 0) {
            reflection(r, d, surface_normal);

            /* if we hit something, add the color
            * that's a result of that */
            if (ray_color(p, MIN_DISTANCE , r, rectangulars, spheres,
                          light, reflection_part,
                          bounces_left - 1,
                          hit_rec, hit_sphere)) {
                multiply_vector(reflection_part, fill.Ks,
                                reflection_part);
                add_vector(object_color, reflection_part,
                           object_color);
            }
        }

        /* calculate refraction ray */
        if (fill.T > 0.0 && fill.index_of_refraction > 0.0) {
            double idx = 1.0;
            if (last_rectangular)
                idx = last_rectangular->element.rectangular_fill.index_of_refraction;
            else if (last_sphere)
                idx = last_sphere->element.sphere_fill.index_of_refraction;
            refraction(r, d, surface_normal, idx,
                       fill.index_of_refraction);

            multiply_vector(d, -t1, p);
            add_vector(e, p, p);

            if (ray_color(p, MIN_DISTANCE, r, rectangulars, spheres,
                          lights, refraction_part,
                          bounces_left - 1, hit_rec,
                          hit_sphere)) {
                multiply_vector(refraction_part, fill.T,
                                refraction_part);
                add_vector(object_color, refraction_part,
                           object_color);
            }
        }
        protect_color_overflow(object_color);
    }
    return 1;
}

/* @param background_color this is not ambient light */
void raytracing(uint8_t *pixels, color background_color,
                rectangular_node rectangulars, sphere_node spheres,
                light_node lights, const viewpoint *view,
                int width, int height)
{
    point3 u, v, w, d;
    color object_color = { 0.0, 0.0, 0.0 };

    /* calculate u, v, w */
    calculateBasisVectors(u, v, w, view);

    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            rayConstruction(d, u, v, w, i, j, view, width, height);
            if (ray_color(view->vrp, 0.0, d, rectangulars, spheres,
                          lights, object_color,
                          MAX_REFLECTION_BOUNCES, NULL,
                          NULL)) {
                pixels[((i + (j*width)) * 3) + 0] = object_color[0] * 255;
                pixels[((i + (j*width)) * 3) + 1] = object_color[1] * 255;
                pixels[((i + (j*width)) * 3) + 2] = object_color[2] * 255;
            } else {
                pixels[((i + (j*width)) * 3) + 0] = background_color[0] * 255;
                pixels[((i + (j*width)) * 3) + 1] = background_color[1] * 255;
                pixels[((i + (j*width)) * 3) + 2] = background_color[2] * 255;
            }
        }
    }
}
