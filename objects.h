#ifndef __RAY_OBJECTS_H
#define __RAY_OBJECTS_H

#define DECLARE_OBJECT(name) \
    struct __##name##_node; \
    typedef struct __##name##_node *name##_node; \
    struct __##name##_node { \
        name element; \
        name##_node next; \
    }; \
    void append_##name(const name *X, name##_node *list); \
    void delete_##name##_list(name##_node *list);

DECLARE_OBJECT(light)
DECLARE_OBJECT(rectangular)
DECLARE_OBJECT(sphere)

#undef DECLARE_OBJECT

#endif
