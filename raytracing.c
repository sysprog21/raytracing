#include <stdio.h>
#include <stdlib.h>

#include "math-toolkit.h"
#include "primitives.h"
#include "raytracing.h"
#include "idx_stack.h"

#define MAX_REFLECTION_BOUNCES	3
#define MAX_DISTANCE 1000000000000.0
#define MIN_DISTANCE 0.00001
#define SAMPLES 4

#define SQUARE(x) (x * x)
#define MAX(a, b) (a > b ? a : b)

/* @param t t distance
 * @return 1 means hit, otherwise 0
 */
static int raySphereIntersection(const point3 ray_e,
                                 const point3 ray_d,
                                 const sphere *sph,
                                 intersection *ip, double *t1)
{
    point3 l;
    subtract_vector(sph->center, ray_e, l);
    double s = dot_product(l, ray_d);
    double l2 = dot_product(l, l);
    double r2 = sph->radius * sph->radius;

    if (s < 0 && l2 > r2)
        return 0;
    float m2 = l2 - s * s;
    if (m2 > r2)
        return 0;
    float q = sqrt(r2 - m2);
    *t1 = (l2 > r2) ? (s - q) : (s + q);
    /* p = e + t1 * d */
    multiply_vector(ray_d, *t1, ip->point);
    add_vector(ray_e, ip->point, ip->point);

    subtract_vector(ip->point, sph->center, ip->normal);
    normalize(ip->normal);
    if (dot_product(ip->normal, ray_d) > 0.0)
        multiply_vector(ip->normal, -1, ip->normal);
    return 1;
}

/* @return 1 means hit, otherwise 0; */
static int rayRectangularIntersection(const point3 ray_e,
                                      const point3 ray_d,
                                      rectangular *rec,
                                      intersection *ip, double *t1)
{
    point3 e01, e03, p;
    subtract_vector(rec->vertices[1], rec->vertices[0], e01);
    subtract_vector(rec->vertices[3], rec->vertices[0], e03);

    cross_product(ray_d, e03, p);

    double det = dot_product(e01, p);

    /* Reject rays orthagonal to the normal vector.
     * I.e. rays parallell to the plane.
     */
    if (det < 1e-4)
        return 0;

    double inv_det = 1.0 / det;

    point3 s;
    subtract_vector(ray_e, rec->vertices[0], s);

    double alpha = inv_det * dot_product(s, p);

    if ((alpha > 1.0) || (alpha < 0.0))
        return 0;

    point3 q;
    cross_product(s, e01, q);

    double beta = inv_det * dot_product(ray_d, q);
    if ((beta > 1.0) || (beta < 0.0))
        return 0;

    *t1 = inv_det * dot_product(e03, q);

    if (alpha + beta > 1.0f) {
        /* for the second triangle */
        point3 e23, e21;
        subtract_vector(rec->vertices[3], rec->vertices[2], e23);
        subtract_vector(rec->vertices[1], rec->vertices[2], e21);

        cross_product(ray_d, e21, p);

        det = dot_product(e23, p);

        if (det < 1e-4)
            return 0;

        inv_det = 1.0 / det;
        subtract_vector(ray_e, rec->vertices[2], s);

        alpha = inv_det * dot_product(s, p);
        if (alpha < 0.0)
            return 0;

        cross_product(s, e23, q);
        beta = inv_det * dot_product(ray_d, q);

        if ((beta < 0.0) || (beta + alpha > 1.0))
            return 0;

        *t1 = inv_det * dot_product(e21, q);
    }

    if (*t1 < 1e-4)
        return 0;

    COPY_POINT3(ip->normal, rec->normal);
    if (dot_product(ip->normal, ray_d)>0.0)
        multiply_vector(ip->normal, -1, ip->normal);
    multiply_vector(ray_d, *t1, ip->point);
    add_vector(ray_e, ip->point, ip->point);

    return 1;
}

static void localColor(color local_color,
                       const color light_color, double diffuse,
                       double specular, const object_fill *fill)
{
    color ambi = { 0.1, 0.1, 0.1 };
    color diff, spec, lightCo, surface;

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
static void compute_specular_diffuse(double *diffuse,
                                     double *specular,
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

/* reference: https://www.opengl.org/sdk/docs/man/html/refract.xhtml */
static void refraction(point3 t, const point3 I, const point3 N,
                       double n1, double n2)
{
    double eta = n1 / n2;
    double dot_NI = dot_product(N,I);
    double k = 1.0 - eta * eta * (1.0 - dot_NI * dot_NI);
    if (k < 0.0 || n2 <= 0.0)
        t[0] = t[1] = t[2] = 0.0;
    else {
        point3 tmp;
        multiply_vector(I, eta, t);
        multiply_vector(N, eta * dot_NI + sqrt(k), tmp);
        subtract_vector(t, tmp, t);
    }
}

/* @param i direction of incoming ray, unit vector
 * @param r direction of refraction ray, unit vector
 * @param normal unit vector
 * @param n1 refraction index
 * @param n2 refraction index
 *
 * reference: http://graphics.stanford.edu/courses/cs148-10-summer/docs/2006--degreve--reflection_refraction.pdf
 */
static double fresnel(const point3 r, const point3 l,
                      const point3 normal, double n1, double n2)
{
    /* TIR */
    if (length(l) < 0.99)
        return 1.0;
    double cos_theta_i = -dot_product(r, normal);
    double cos_theta_t = -dot_product(l, normal);
    double r_vertical_root = (n1 * cos_theta_i - n2 * cos_theta_t) /
                             (n1 * cos_theta_i + n2 * cos_theta_t);
    double r_parallel_root = (n2 * cos_theta_i - n1 * cos_theta_t) /
                             (n2 * cos_theta_i + n1 * cos_theta_t);
    return (r_vertical_root * r_vertical_root +
            r_parallel_root * r_parallel_root) / 2.0;
}

/* @param t distance */
static intersection ray_hit_object(const point3 e, const point3 d,
                                   double t0, double t1,
                                   const rectangular_node rectangulars,
                                   rectangular_node *hit_rectangular,
                                   const sphere_node spheres,
                                   sphere_node *hit_sphere)
{
    /* set these to not hit */
    *hit_rectangular = NULL;
    *hit_sphere = NULL;

    point3 biased_e;
    multiply_vector(d, t0, biased_e);
    add_vector(biased_e, e, biased_e);

    double nearest = t1;
    intersection result, tmpresult;

    for (rectangular_node rec = rectangulars; rec; rec = rec->next) {
        if (rayRectangularIntersection(biased_e, d, &(rec->element),
                                       &tmpresult, &t1) && (t1 < nearest)) {
            /* hit is closest so far */
            *hit_rectangular = rec;
            nearest = t1;
            result = tmpresult;
        }
    }

    /* check the spheres */
    for (sphere_node sphere = spheres; sphere; sphere = sphere->next) {
        if (raySphereIntersection(biased_e, d, &(sphere->element),
                                  &tmpresult, &t1) && (t1 < nearest)) {
            *hit_sphere = sphere;
            *hit_rectangular = NULL;
            nearest = t1;
            result = tmpresult;
        }
    }

    return result;
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
    double xmax =  0.0175;
    double ymax =  0.0175;
    double focal = 0.05;

    point3 u_tmp, v_tmp, w_tmp, s;

    double w_s = focal;
    double u_s = xmin + ((xmax - xmin) * (float) i / (width - 1));
    double v_s = ymax + ((ymin - ymax) * (float) j / (height - 1));

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

static unsigned int ray_color(const point3 e, double t,
                              const point3 d,
                              idx_stack *stk,
                              const rectangular_node rectangulars,
                              const sphere_node spheres,
                              const light_node lights,
                              color object_color, int bounces_left)
{
    rectangular_node hit_rec = NULL, light_hit_rec = NULL;
    sphere_node hit_sphere = NULL, light_hit_sphere = NULL;
    double diffuse, specular;
    point3 l, _l, r, rr;
    object_fill fill;

    color reflection_part;
    color refraction_part;
    /* might be a reflection ray, so check how many times we've bounced */
    if (bounces_left == 0) {
        SET_COLOR(object_color, 0.0, 0.0, 0.0);
        return 0;
    }

    /* check for intersection with a sphere or a rectangular */
    intersection ip= ray_hit_object(e, d, t, MAX_DISTANCE, rectangulars,
                                    &hit_rec, spheres, &hit_sphere);
    if (!hit_rec && !hit_sphere)
        return 0;

    /* pick the fill of the object that was hit */
    fill = hit_rec ?
           hit_rec->element.rectangular_fill :
           hit_sphere->element.sphere_fill;

    void *hit_obj = hit_rec ? (void *) hit_rec : (void *) hit_sphere;

    /* assume it is a shadow */
    SET_COLOR(object_color, 0.0, 0.0, 0.0);

    for (light_node light = lights; light; light = light->next) {
        /* calculate the intersection vector pointing at the light */
        subtract_vector(ip.point, light->element.position, l);
        multiply_vector(l, -1, _l);
        normalize(_l);
        /* check for intersection with an object. use ignore_me
         * because we don't care about this normal
        */
        ray_hit_object(ip.point, _l, MIN_DISTANCE, length(l),
                       rectangulars, &light_hit_rec,
                       spheres, &light_hit_sphere);
        /* the light was not block by itself(lit object) */
        if (light_hit_rec || light_hit_sphere)
            continue;

        compute_specular_diffuse(&diffuse, &specular, d, l,
                                 ip.normal, fill.phong_power);

        localColor(object_color, light->element.light_color,
                   diffuse, specular, &fill);
    }

    reflection(r, d, ip.normal);
    double idx = idx_stack_top(stk).idx, idx_pass = fill.index_of_refraction;
    if (idx_stack_top(stk).obj == hit_obj) {
        idx_stack_pop(stk);
        idx_pass = idx_stack_top(stk).idx;
    } else {
        idx_stack_element e = { .obj = hit_obj,
                                .idx = fill.index_of_refraction
                              };
        idx_stack_push(stk, e);
    }

    refraction(rr, d, ip.normal, idx, idx_pass);
    double R = (fill.T > 0.1) ?
               fresnel(d, rr, ip.normal, idx, idx_pass) :
               1.0;

    /* totalColor = localColor +
                    mix((1-fill.Kd) * fill.R * reflection, T * refraction, R)
     */
    if (fill.R > 0) {
        /* if we hit something, add the color */
        int old_top = stk->top;
        if (ray_color(ip.point, MIN_DISTANCE, r, stk, rectangulars, spheres,
                      lights, reflection_part,
                      bounces_left - 1)) {
            multiply_vector(reflection_part, R * (1.0 - fill.Kd) * fill.R,
                            reflection_part);
            add_vector(object_color, reflection_part,
                       object_color);
        }
        stk->top = old_top;
    }
    /* calculate refraction ray */
    if ((length(rr) > 0.0) && (fill.T > 0.0) &&
            (fill.index_of_refraction > 0.0)) {
        normalize(rr);
        if (ray_color(ip.point, MIN_DISTANCE, rr, stk,rectangulars, spheres,
                      lights, refraction_part,
                      bounces_left - 1)) {
            multiply_vector(refraction_part, (1 - R) * fill.T,
                            refraction_part);
            add_vector(object_color, refraction_part,
                       object_color);
        }
    }

    protect_color_overflow(object_color);
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

    idx_stack stk;

    int factor = sqrt(SAMPLES);
    for (int j = 0; j < height; j++) {
        for (int i = 0; i < width; i++) {
            double r = 0, g = 0, b = 0;
            /* MSAA */
            for (int s = 0; s < SAMPLES; s++) {
                idx_stack_init(&stk);
                rayConstruction(d, u, v, w,
                                i * factor + s / factor,
                                j * factor + s % factor,
                                view,
                                width * factor, height * factor);
                if (ray_color(view->vrp, 0.0, d, &stk, rectangulars, spheres,
                              lights, object_color,
                              MAX_REFLECTION_BOUNCES)) {
                    r += object_color[0];
                    g += object_color[1];
                    b += object_color[2];
                } else {
                    r += background_color[0];
                    g += background_color[1];
                    b += background_color[2];
                }
                pixels[((i + (j * width)) * 3) + 0] = r * 255 / SAMPLES;
                pixels[((i + (j * width)) * 3) + 1] = g * 255 / SAMPLES;
                pixels[((i + (j * width)) * 3) + 2] = b * 255 / SAMPLES;
            }
        }
    }
}
