#define _POSIX_C_SOURCE 200809L

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <stdarg.h>
#include "../src/internal/alloc.h"

extern void* HEAP_START;

// heap buffer + reset helper
static unsigned char heap_buf[TOTAL_HEAP_SIZE];
static void reset_heap(void) {
    memset(heap_buf, 0xA5, sizeof(heap_buf)); // fill pattern
    HEAP_START = (void*)heap_buf;
    int r = (int)init_heap(); // init allocator
    if (r != 1) { fprintf(stderr, "[ERROR] init_heap failed\n"); exit(1); }
}

typedef void (*test_fn_t)(void);

// test case object
typedef struct { const char* name; test_fn_t fn; } TestCase;
#define TEST_CASE(fn) { #fn, fn }

// global counters
static int total_asserts = 0;
static int total_failures = 0;

// failure object
typedef struct { const char* test_name; const char* msg; } Failure;
#define MAX_FAILURES 512
static Failure failures[MAX_FAILURES];
static int failure_count = 0;

static const char* current_test_name = NULL;
static int failure_pipe_fd = -1;
static int local_asserts = 0;
static int local_failures = 0;

static FILE* out_fp = NULL;

// write to stdout and output file (if open)
static void log_printf(const char* fmt, ...) {
	va_list ap;
	va_start(ap, fmt);
	vfprintf(stdout, fmt, ap);
	va_end(ap);

	if (out_fp) {
		va_list ap2;
		va_start(ap2, fmt);
		vfprintf(out_fp, fmt, ap2);
		va_end(ap2);
		fflush(out_fp);
	}
	fflush(stdout);
}

// assert helper
static void assert_check(int condition, const char* msg) {
    local_asserts++; // count each assertion

    const char* tag = condition ? "[OK]" : "[FAIL]";
    const int tag_col = 80;
    const int indent = 6;  
    int tag_len = (int)strlen(tag);
    int max_msg_len = tag_col - indent - tag_len;
    if (max_msg_len < 0) max_msg_len = 0;

    char trunc_msg[1024];
    int msg_len = snprintf(trunc_msg, sizeof(trunc_msg), "%s", msg);
    if (msg_len > max_msg_len) msg_len = max_msg_len;

    // build aligned line into buffer, then print via log_printf
    char linebuf[2048];
    int pos = 0;
    pos += snprintf(linebuf + pos, sizeof(linebuf) - pos, "    - ");
    // append truncated message
    if (msg_len > 0) {
        strncat(linebuf + pos, trunc_msg, (size_t)msg_len);
        pos = (int)strlen(linebuf);
    }
    int spaces = tag_col - indent - msg_len - tag_len;
    if (spaces < 1) spaces = 1;
    for (int i = 0; i < spaces && pos + 1 < (int)sizeof(linebuf); ++i) {
        linebuf[pos++] = ' ';
    }
    // append tag and null-terminate
    if (pos + tag_len + 2 < (int)sizeof(linebuf)) {
        strcpy(linebuf + pos, tag);
        pos += tag_len;
    }
    linebuf[pos++] = '\n';
    linebuf[pos] = '\0';

    log_printf("%s", linebuf);

    if (!condition) {
        local_failures++;
        if (failure_pipe_fd >= 0) { // pipe to parent
            char buf[1024];
            int n = snprintf(buf, sizeof(buf), "%s|%s\n", current_test_name, msg);
            if (n > 0) write(failure_pipe_fd, buf, (size_t)n);
        }
    }
}

// run test in forked child
static void run_test(const TestCase* tc) {
    int pfd[2];
    // idea is that one failure shouldn't cause rest of tests to not run.
    // so separate child process is made w/ fork(). pipe is used for communication w/
    // processes-- info about failures is written to pipe.
    if (pipe(pfd) != 0) { perror("pipe"); pfd[0] = pfd[1] = -1; }

    pid_t pid = fork(); // child process for isolation
    if (pid < 0) { perror("fork"); return; }

    if (pid == 0) {
        if (pfd[0] >= 0) close(pfd[0]);
        failure_pipe_fd = (pfd[1] >= 0) ? pfd[1] : -1;

        current_test_name = tc->name;
        log_printf("%s\n", tc->name);
        fflush(stdout);

        failure_count = 0;
        local_asserts = 0;
        local_failures = 0;

        tc->fn(); // execute test

        // send counts to parent
        if (failure_pipe_fd >= 0) {
            char cntbuf[128];
            int n = snprintf(cntbuf, sizeof(cntbuf), "__COUNTS__|%d|%d\n", local_asserts, local_failures);
            if (n > 0) write(failure_pipe_fd, cntbuf, (size_t)n);
            close(failure_pipe_fd);
        }
        exit(0);
    }

    if (pfd[1] >= 0) close(pfd[1]);
    int status = 0;
    waitpid(pid, &status, 0); // wait child

    if (pfd[0] >= 0) {
        FILE* rf = fdopen(pfd[0], "r");
        if (rf) {
            char line[1024];
            while (fgets(line, sizeof(line), rf)) {
                char* sep = strchr(line, '|');
                if (!sep) continue;
                *sep = '\0';
                char* tname = line;
                char* msg = sep + 1;
                char* nl = strchr(msg, '\n');
                if (nl) *nl = '\0';

                if (strcmp(tname, "__COUNTS__") == 0) {
                    char* second_sep = strchr(msg, '|');
                    if (second_sep) {
                        *second_sep = '\0';
                        int a = atoi(msg);
                        int f = atoi(second_sep + 1);
                        total_asserts += a;
                        total_failures += f;
                    }
                    continue;
                }

                if (failure_count < MAX_FAILURES) {
                    failures[failure_count].test_name = strdup(tname);
                    failures[failure_count].msg = strdup(msg);
                    failure_count++;
                }
            }
            fclose(rf);
        } else close(pfd[0]);
    }

    if (WIFSIGNALED(status)) { // crash
        int sig = WTERMSIG(status);
        log_printf("    Result: CRASH (signal %d)\n", sig);
        if (failure_count < MAX_FAILURES) {
            failures[failure_count].test_name = tc->name;
            failures[failure_count].msg = "test crashed (signal)";
            failure_count++;
        }
        total_failures += 1;
    }
}

// basic heap init
static void test_init_heap_basic(void) {
    reset_heap();
    assert_check(isFree(HEAP_START), "heap start free");
    void* oob = (void*)((unsigned char*)HEAP_START + TOTAL_HEAP_SIZE + 8);
    assert_check(!isFree(oob), "out-of-range not free");
}

// alloc/free/realloc
static void test_alloc_free_realloc(void) {
    reset_heap();
    void* a = alloc(16);
    assert_check(a != NULL, "alloc 16");
    assert_check(!isFree(a), "allocated marked free");
    uintptr_t a_addr = (uintptr_t)a;
    free(a);
    assert_check(isFree((void*)a_addr), "freed block ok");
    void* b = alloc(16);
    assert_check(b != NULL, "realloc block");
    assert_check((uintptr_t)b == a_addr, "same block reused");
    free(b);
}

// double free
static void test_double_free(void) {
    reset_heap();
    void* a = alloc(32);
    assert_check(a != NULL, "alloc 32");
    uintptr_t a_addr = (uintptr_t)a;
    free(a);
    assert_check(isFree((void*)a_addr), "freed block ok");
    free((void*)a_addr); // second freed on same place
    assert_check(isFree((void*)a_addr), "double free safe");
}

// free NULL / out-of-range
static void test_free_null_and_oob(void) {
    reset_heap();
    free(NULL); // no-op
    uintptr_t heap_base = (uintptr_t)HEAP_START;
    void* oob = (void*)(heap_base + TOTAL_HEAP_SIZE + 16u);
    free(oob); // should not crash
    assert_check(!isFree(oob), "oob still not free");
}

// exhaust smallest pool
static void test_exhaust_small_pool(void) {
    reset_heap();
    const int try_size = 16;
    void* arr[2560];
    int i = 0;
    for (; i < (int)(sizeof(arr)/sizeof(arr[0])); ++i) {
        arr[i] = alloc(try_size); // fill pool
        if (!arr[i]) break; // stop when exhausted
    }
    assert_check(i > 0, "some allocs succeeded");
    assert_check(i < (int)(sizeof(arr)/sizeof(arr[0])), "pool eventually full");
    for (int j = 0; j < i; ++j) free(arr[j]); // cleanup
}

// alloc 0 / too large
static void test_invalid_and_large_allocs(void) {
    reset_heap();
    void* z = alloc(0); // zero alloc
    assert_check(z == NULL || isFree(z), "alloc 0 ok");
    void* too = alloc(TOTAL_HEAP_SIZE + 1024); // too big
    assert_check(too == NULL, "alloc too large");
}

// isFree across pools
static void test_isFree_across_pools(void) {
    reset_heap();
    void* a = alloc(16);
    void* b = alloc(32);
    void* c = alloc(64);
    assert_check(a && b && c, "multi alloc");
    assert_check(!isFree(a) && !isFree(b) && !isFree(c), "blocks in use");
    uintptr_t b_addr = (uintptr_t)b;
    free(b);
    assert_check(isFree((void*)b_addr), "freed ok");
    assert_check(!isFree(a) && !isFree(c), "others unaffected");
    uintptr_t a_addr = (uintptr_t)a;
    uintptr_t c_addr = (uintptr_t)c;
    free(a); free(c);
    assert_check(isFree((void*)a_addr) && isFree((void*)c_addr), "all freed");
}

// stress alloc/free pattern
static void test_stress_pattern(void) {
    reset_heap();
    const int N = 1000;
    void* last = NULL;
    int alloc_count = 0;
    for (int i = 0; i < N; ++i) {
        void* p = alloc(16 + (i % 4) * 8); // varying sizes
        if (!p) break;
        alloc_count++;
        if (last) { free(last); last = NULL; } else { last = p; } // alternating free
    }
    if (last) free(last);
    assert_check(alloc_count > 0, "stress did some allocs");
}

// helper to check pools by index (testing all pool sizes)
// allocate every block in pool idx, check exhaustion, free one and re-alloc
static void test_pool_generic(uint32_t idx) {
    reset_heap();
    uint32_t sz = POOL_BLOCK_SIZES[idx];
    uint32_t cnt = POOL_SIZES[idx];

    // allocate all blocks from the pool
    void* arr[cnt];
    char msg[128];
    for (uint32_t i = 0; i < cnt; ++i) {
        arr[i] = alloc(sz);
        snprintf(msg, sizeof(msg), "alloc(%u) in pool %u (i=%u) should succeed", sz, idx, i);
        assert_check(arr[i] != NULL, msg);
        if (arr[i]) {
            snprintf(msg, sizeof(msg), "allocated block for pool %u not marked free", idx);
            assert_check(!isFree(arr[i]), msg);
        }
    }


    // free one block and ensure a subsequent alloc succeeds
    if (cnt > 0 && arr[0]) {
        free(arr[0]);
        snprintf(msg, sizeof(msg), "freed block in pool %u should be marked free", idx);
        assert_check(isFree(arr[0]), msg);

        void* r = alloc(sz);
        snprintf(msg, sizeof(msg), "alloc(%u) after free in pool %u should succeed", sz, idx);
        assert_check(r != NULL, msg);
        if (r) free(r);
    }

    // cleanup remaining allocations
    for (uint32_t i = 0; i < cnt; ++i) {
        if (arr[i]) free(arr[i]);
    }
}

// test all pool sizes
static void test_pool_16(void)   { test_pool_generic(0); }
static void test_pool_32(void)   { test_pool_generic(1); }
static void test_pool_64(void)   { test_pool_generic(2); }
static void test_pool_128(void)  { test_pool_generic(3); }
static void test_pool_256(void)  { test_pool_generic(4); }
static void test_pool_512(void)  { test_pool_generic(5); }
static void test_pool_1024(void) { test_pool_generic(6); }

int main(void) {
    // open output file
    out_fp = fopen("alloctest_output.txt", "w");
    if (!out_fp) {
        perror("fopen alloctest_output.txt");
        // continue without file output
    }

    log_printf("Running alloc unit tests...\n");

    TestCase tests[] = {
        TEST_CASE(test_init_heap_basic),
        TEST_CASE(test_alloc_free_realloc),
        TEST_CASE(test_double_free),
        TEST_CASE(test_free_null_and_oob),
        TEST_CASE(test_exhaust_small_pool),
        TEST_CASE(test_invalid_and_large_allocs),
        TEST_CASE(test_isFree_across_pools),
        
        TEST_CASE(test_pool_16),
        TEST_CASE(test_pool_32),
        TEST_CASE(test_pool_64),
        TEST_CASE(test_pool_128),
        TEST_CASE(test_pool_256),
        TEST_CASE(test_pool_512),
        TEST_CASE(test_pool_1024),

        TEST_CASE(test_stress_pattern)
    };

    int num_tests = sizeof(tests) / sizeof(tests[0]);
    for (int i = 0; i < num_tests; ++i) {
        run_test(&tests[i]);
    }

    log_printf("\nSummary: %d/%d assertions passed, %d failed.\n",
           total_asserts - total_failures, total_asserts, total_failures);

    if (total_failures > 0) {
        log_printf("Failures:\n");
        for (int i = 0; i < failure_count; ++i) {
            log_printf("  [%s] %s\n", failures[i].test_name, failures[i].msg);
        }
    }

    if (out_fp) {
        fclose(out_fp);
        out_fp = NULL;
    }

    return (total_failures == 0) ? 0 : 1;
}