#ifndef SMPL_ASSERT_H
#define SMPL_ASSERT_H

#include <cstdio>
#include <cstdlib>

#define SMPL_ASSERT_LEVEL_NONE      0   // compile out all assertions
#define SMPL_ASSERT_LEVEL_STRICT    1   // compile in SMPL_ASSERT_STRICT
#define SMPL_ASSERT_LEVEL_NORMAL    2   // compile in SMPL_ASSERT and SMPL_ASSERT_STRICT
#define SMPL_ASSERT_LEVEL_DEBUG     3   // compile in SMPL_ASSERT_DEBUG, SMPL_ASSERT, SMPL_ASSERT_STRICT

#ifndef SMPL_ASSERT_LEVEL
#define SMPL_ASSERT_LEVEL SMPL_ASSERT_LEVEL_NORMAL
#endif

#define SMPL_ASSERT_AUX(cond, msg) \
    do { \
        if (!(cond)) { \
            ::std::fprintf(stderr, "%s::%d: Assertion `%s` failed.\n", __FILE__, __LINE__, #cond); \
            ::std::abort(); \
        } \
    } while (0)

#define SMPL_ASSERT_AUX_EMPTY(cond, msg) do { (void)sizeof((cond)); } while (0)

#if SMPL_ASSERT_LEVEL >= SMPL_ASSERT_LEVEL_DEBUG
#define SMPL_ASSERT_DEBUG(cond, msg) SMPL_ASSERT_AUX(cond, msg)
#else
#define SMPL_ASSERT_DEBUG(cond, msg) SMPL_ASSERT_AUX_EMPTY(cond, msg)
#endif

#if SMPL_ASSERT_LEVEL >= SMPL_ASSERT_LEVEL_NORMAL
#define SMPL_ASSERT(cond, msg) SMPL_ASSERT_AUX(cond, msg)
#else
#define SMPL_ASSERT(cond, msg) SMPL_ASSERT_AUX_EMPTY(cond, msg)
#endif

#if SMPL_ASSERT_LEVEL >= SMPL_ASSERT_LEVEL_STRICT
#define SMPL_ASSERT_STRICT(cond, msg) SMPL_ASSERT_AUX(cond, msg)
#else
#define SMPL_ASSERT_STRICT(cond, msg) SMPL_ASSERT_AUX_EMPTY(cond, msg)
#endif

#endif

