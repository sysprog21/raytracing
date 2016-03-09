static const
light light1 = {
    .light_color = { 0.7, 0.5, 0.5 },
    .position = { 5, 5, 20 },
    .intensity = 200.0,
};

static const
light light2 = {
    .light_color = { 0.8,  0.8, 0.8 },
    .position = { 10, 10, 20 },
    .intensity = 200.0,
};

static const
sphere sphere1 = {
    .center = { 5, 0, 5 },
    .radius = 3,
    .sphere_fill = {
        .fill_color = { 0.8, 0.8, 0.8 },
        .Kd = 0.8, .Ks = 0.8, .T = 0.0, .R = 0.6,
        .index_of_refraction = 0.0, .phong_power = 30.0
    }
};

static const
sphere sphere2 = {
    .center = { 20, 15, 15 },
    .radius = 3,
    .sphere_fill = {
        .fill_color = { 0.8, 0.6, 0.4 },
        .Kd = 0.0, .Ks = 1.0, .T = 1.0, .R = 1.0,
        .index_of_refraction = 1.5, .phong_power = 30.0
    }
};

static const
sphere sphere3 = {
    .center = { 5, 10, 5 },
    .radius = 3,
    .sphere_fill = {
        .fill_color = { 0.4, 0.8, 0.6 },
        .Kd = 0.8, .Ks = 0.1, .T = 0.0, .R = 0.1,
        .index_of_refraction = 0.0, .phong_power = 30.0
    }
};

static const
viewpoint view = {
    .vrp = { 40.0, 40.0, 40.0 },
    .vpn = { -1.0, -1.0, -1.0 },
    .vup = {  0.0,  0.0,  1.0 }
};

static const
rectangular rectangular1 = {
    .vertices = {
        {  0.0, 0.0,  0.0 },	/* v0 */
        {  0.0, 0.0, 20.0 },	/* v1 */
        { 20.0, 0.0, 20.0 },	/* v3 */
        { 20.0, 0.0,  0.0 },	/* v2 */
    },
    .normal = { 0.0, 1.0, 0.0 },
    .rectangular_fill = {
        .fill_color = { 0.6, 0.6, 0.6 },
        .Kd = 0.8, .Ks = 0.0, .T = 0.0, .R = 0.5,
        .index_of_refraction = 0.0, .phong_power = 5.0,
    },
};

static const
rectangular rectangular2 = {
    .vertices = {
        {  0.0,  0.0, 0.0 },	/* v0 */
        { 20.0,  0.0, 0.0 },	/* v1 */
        { 20.0, 20.0, 0.0 },	/* v3 */
        {  0.0, 20.0, 0.0 },	/* v2 */
    },
    .normal = { 0.0, 0.0, 1.0 },
    .rectangular_fill = {
        .fill_color = { 0.6, 0.1, 0.1 },
        .Kd = 0.8, .Ks = 0.0, .T = 0.0, .R = 0.5,
        .index_of_refraction = 0.0, .phong_power = 5.0,
    },
};

static const
rectangular rectangular3 = {
    .vertices = {
        { 0.0,  0.0,  0.0 },	/* v0 */
        { 0.0, 20.0,  0.0 },	/* v1 */
        { 0.0, 20.0, 20.0 },	/* v3 */
        { 0.0,  0.0, 20.0 },	/* v2 */
    },
    .normal = { 1.0, 0.0, 0.0 },
    .rectangular_fill = {
        .fill_color = { 0.1, 0.1, 0.6 },
        .Kd = 0.8, .Ks = 0.0, .T = 0.0, .R = 0.5,
        .index_of_refraction = 0.0, .phong_power = 5.0,
    },
};
