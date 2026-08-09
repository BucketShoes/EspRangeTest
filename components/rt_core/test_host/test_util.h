// Minimal assert-based test scaffolding. Deliberately dependency-free so the core logic
// can be verified with nothing but a C compiler.

#pragma once

#include <stdio.h>
#include <stdlib.h>

static int g_checks;
static int g_failures;

#define CHECK(cond)                                                              \
    do {                                                                         \
        g_checks++;                                                              \
        if (!(cond)) {                                                           \
            g_failures++;                                                        \
            printf("  FAIL %s:%d: %s\n", __FILE__, __LINE__, #cond);             \
        }                                                                        \
    } while (0)

#define CHECK_EQ(actual, expected)                                               \
    do {                                                                         \
        g_checks++;                                                              \
        const long long _a = (long long)(actual);                                \
        const long long _e = (long long)(expected);                              \
        if (_a != _e) {                                                          \
            g_failures++;                                                        \
            printf("  FAIL %s:%d: %s == %s (got %lld, want %lld)\n",             \
                   __FILE__, __LINE__, #actual, #expected, _a, _e);              \
        }                                                                        \
    } while (0)

#define RUN(fn)                                                                  \
    do {                                                                         \
        printf("- %s\n", #fn);                                                   \
        fn();                                                                    \
    } while (0)

#define TEST_MAIN_END()                                                          \
    do {                                                                         \
        printf("%d checks, %d failures\n", g_checks, g_failures);                \
        return g_failures == 0 ? EXIT_SUCCESS : EXIT_FAILURE;                    \
    } while (0)
