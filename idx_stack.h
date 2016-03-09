#ifndef _RAY_IDX_STACK_H
#define _RAY_IDX_STACK_H

#define MAX_STACK_SIZE 16

typedef struct {
    double idx;
    void *obj;
} idx_stack_element;

#define AIR_ELEMENT (idx_stack_element) { .obj = NULL, .idx = 1.0 }

typedef struct {
    idx_stack_element data[MAX_STACK_SIZE];
    int top;
} idx_stack;

static inline void idx_stack_init(idx_stack *stk)
{
    stk->top = 0;
}

static inline void idx_stack_push(idx_stack *stk, idx_stack_element element)
{
    if (stk->top < MAX_STACK_SIZE)
        stk->data[stk->top++] = element;
}

static inline int idx_stack_empty(idx_stack *stk)
{
    return !stk->top;
}

static inline idx_stack_element idx_stack_pop(idx_stack *stk)
{
    if (!idx_stack_empty(stk))
        return stk->data[--stk->top];
    return AIR_ELEMENT;
}

static inline idx_stack_element idx_stack_top(idx_stack *stk)
{
    if (!idx_stack_empty(stk))
        return stk->data[stk->top-1];
    return AIR_ELEMENT;
}

#endif
