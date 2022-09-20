#ifndef __CHANNEL_SPLIT_H__
#define __CHANNEL_SPLIT_H__

#include "types.h"

/* len is the size of data */
static inline int split_data_to_lr(signed short *data, signed short *l, signed short *r,
                                   int len)
{
    for (int i = 0; i < (len >> 1); i++) {
        l[i] = data[i * 2];
        r[i] = data[i * 2 + 1];
    }
    return len >> 1;
}

/* len is the size of l */
static inline int converge_lr_to_data(signed short *l, signed short *r,
                                      signed short *data, int len)
{
    for (int i = 0; i < len; i++) {
        data[i * 2] = l[i];
        data[i * 2 + 1] = r[i];
    }
    return len << 1;
}

#endif
