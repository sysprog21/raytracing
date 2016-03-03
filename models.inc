static const
light light1 = {
    .light_color = {   1.0,  1.0, 1.0 },
    .position =    { -10.0, 10.0, 2.0 },
    .intensity = 200.0,
};

static const
light light2 = {
    .light_color = { 1.0,  0.0, 0.0 },
    .position =    { 0.0, 10.0, 2.0 },
    .intensity = 200.0
};

static const
light light3 = {
    .light_color = { 0.0, 1.0,  0.0 },
    .position =    { 1.0, 2.0, 10.0 },
    .intensity = 200.0
};

static const
viewpoint view = {
    .vrp = { -2.0,  4.0,  4.0 },
    .vpn = {  4.0, -4.0, -4.0 },
    .vup = {  0.0,  4.0,  4.0 }
};

static const
sphere sphere1 = {
    .center = { 1.4, 0.3, 0.4 },
    .radius = 0.3,
    .sphere_fill = {
        .fill_color = { 1.0, 1.0, 1.0 },
        .Kd = 0.75, .Ks = 0.5, .T = 0.2,
        .index_of_refraction = 20.0, .phong_power = 5.0
    }
};

static const
sphere sphere2 = {
    .center = { 0.0, 2.0, 1.0 },
    .radius = 0.2,
    .sphere_fill = {
        .fill_color = { 1.0, 1.0, 1.0 },
        .Kd = 0.75, .Ks = 0.5, .T = 0.5,
        .index_of_refraction = 20.0, .phong_power = 10.0
    }
};

static const
sphere sphere3 = {
    .center = { 0.0, 1.75, 0.1 },
    .radius = 0.1,
    .sphere_fill = {
        .fill_color = { 1.0, 1.0, 1.0 },
        .Kd = 1.0, .Ks = 1.0, .T = 0.0,
        .index_of_refraction = 0.0, .phong_power = 10.0
    }
};

static const
sphere sphere4 = {
    .center ={ 0.2, 0.3, 1.6 },
    .radius = 0.3,
    .sphere_fill = {
        .fill_color = { 1.0, 1.0, 1.0 },
        .Kd = 0.8, .Ks = 0.2, .T = 0.8,
        .index_of_refraction = 10.0, .phong_power = 10.0
    }
};

static const
rectangular rectangular1 = {
    .vertices = {
        { 0.0, 0.0, 0.0 },  /* v0 */
        { 0.0, 0.0, 2.0 },  /* v1 */
        { 2.0, 0.0, 2.0 },  /* v2 */
        { 2.0, 0.0, 0.0 },  /* v3 */
    },
    .normal = { 0.0, 1.0, 0.0 },
    .rectangular_fill = {
        .fill_color = { 0.1, 0.1, 0.1 },
        .Kd = 0.8, .Ks = 0.5, .T = 0.0,
        .index_of_refraction = 0.0, .phong_power = 5.0
    },
};

static const
rectangular rectangular2 = {
    .vertices = {
        { 0.0, 0.0, 0.0 },	/* v0 */
        { 0.0, 2.0, 0.0 },	/* v1 */
        { 2.0, 2.0, 0.0 },	/* v2 */
        { 2.0, 0.0, 0.0 },	/* v3 */
    },
    .normal = { 0.0, 0.0, 1.0 },
    .rectangular_fill = {
        .fill_color = { 0.1, 0.1, 0.1 },
        .Kd = 0.8, .Ks = 0.5, .T = 0.0,
        .index_of_refraction = 0.0, .phong_power = 5.0,
    },
};

static const
rectangular rectangular3 = {
    .vertices = {
        { 2.0, 0.0, 0.0 },	/* v0 */
        { 2.0, 0.0, 2.0 },	/* v1 */
        { 2.0, 2.0, 2.0 },	/* v2 */
        { 2.0, 2.0, 0.0 },	/* v3 */
    },
    .normal = { -1.0, 0.0, 0.0 },
    .rectangular_fill = {
        .fill_color = { 0.1, 0.1, 0.1 },
        .Kd = 0.8, .Ks = 0.5, .T = 0.0,
        .index_of_refraction = 0.0, .phong_power = 5.0,
    },
};
