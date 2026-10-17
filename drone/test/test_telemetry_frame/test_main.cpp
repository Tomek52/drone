// Unit tests for common/stampfly_link/src/telemetry_frame.hpp (pio test -e native).
// The golden byte vectors are shared with tools/test_stampfly_telemetry.py so the
// C++ encoder and the Python decoder cannot drift apart.

#include <unity.h>
#include <stdio.h>
#include <telemetry_frame.hpp>

using namespace stampfly_link;

void setUp() {}
void tearDown() {}

static const uint8_t kGoldenData[] = {0xA5, 0x5A, 0x02, 0x0E, 0xE8, 0x03, 0x00, 0x00, 0x07, 0x00,
                                      0x00, 0x00, 0x80, 0x3F, 0x00, 0x00, 0x20, 0xC0, 0xA3, 0x5D};
static const uint8_t kGoldenSchema[] = {0xA5, 0x5A, 0x01, 0x08, 0x01, 0x02, 0x00, 0x02,
                                        0x61, 0x2C, 0x62, 0x63, 0x61, 0xA4};

void test_fletcher16_known_value() {
    // Classic reference value: Fletcher-16 of "abcde" is 0xC8F0.
    const uint8_t s[] = {'a', 'b', 'c', 'd', 'e'};
    TEST_ASSERT_EQUAL_HEX16(0xC8F0, fletcher16(s, sizeof(s)));
}

void test_encode_data_matches_golden() {
    uint8_t out[kMaxFrameSize];
    const float v[] = {1.0f, -2.5f};
    const size_t n = encode_data(1000, 7, v, 2, out, sizeof(out));
    TEST_ASSERT_EQUAL(sizeof(kGoldenData), n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kGoldenData, out, n);
}

void test_encode_schema_matches_golden() {
    uint8_t out[kMaxFrameSize];
    const char* names[] = {"a", "bc"};
    size_t consumed = 0;
    const size_t n = encode_schema_chunk(names, 2, 0, &consumed, out, sizeof(out));
    TEST_ASSERT_EQUAL(2, consumed);
    TEST_ASSERT_EQUAL(sizeof(kGoldenSchema), n);
    TEST_ASSERT_EQUAL_HEX8_ARRAY(kGoldenSchema, out, n);
}

void test_schema_is_split_into_chunks() {
    // 40 names of 15 characters do not fit into one 240-byte payload.
    static char storage[40][16];
    const char* names[40];
    for (int i = 0; i < 40; i++) {
        snprintf(storage[i], sizeof(storage[i]), "field_name_%04d", i);
        names[i] = storage[i];
    }
    uint8_t out[kMaxFrameSize];
    size_t first = 0, chunks = 0;
    while (first < 40) {
        size_t consumed = 0;
        const size_t n = encode_schema_chunk(names, 40, first, &consumed, out, sizeof(out));
        TEST_ASSERT_NOT_EQUAL(0, n);
        TEST_ASSERT_TRUE(is_valid_frame(out, n));
        TEST_ASSERT_EQUAL(40, out[kHeaderSize + 1]);     // total
        TEST_ASSERT_EQUAL(first, out[kHeaderSize + 2]);  // first index
        TEST_ASSERT_EQUAL(consumed, out[kHeaderSize + 3]);
        first += consumed;
        chunks++;
    }
    TEST_ASSERT_EQUAL(3, chunks);
}

void test_is_valid_frame() {
    TEST_ASSERT_TRUE(is_valid_frame(kGoldenData, sizeof(kGoldenData)));
    uint8_t broken[sizeof(kGoldenData)];
    memcpy(broken, kGoldenData, sizeof(broken));
    broken[10] ^= 0x01;
    TEST_ASSERT_FALSE(is_valid_frame(broken, sizeof(broken)));
    TEST_ASSERT_FALSE(is_valid_frame(kGoldenData, sizeof(kGoldenData) - 1));
}

void test_rejects_too_many_fields() {
    uint8_t out[kMaxFrameSize];
    float v[kMaxDataFields + 1] = {0};
    TEST_ASSERT_EQUAL(0, encode_data(0, 0, v, kMaxDataFields + 1, out, sizeof(out)));
    TEST_ASSERT_NOT_EQUAL(0, encode_data(0, 0, v, kMaxDataFields, out, sizeof(out)));
}

void test_rejects_small_output_buffer() {
    uint8_t out[8];
    const float v[] = {1.0f};
    TEST_ASSERT_EQUAL(0, encode_data(0, 0, v, 1, out, sizeof(out)));
}

int main(int, char**) {
    UNITY_BEGIN();
    RUN_TEST(test_fletcher16_known_value);
    RUN_TEST(test_encode_data_matches_golden);
    RUN_TEST(test_encode_schema_matches_golden);
    RUN_TEST(test_schema_is_split_into_chunks);
    RUN_TEST(test_is_valid_frame);
    RUN_TEST(test_rejects_too_many_fields);
    RUN_TEST(test_rejects_small_output_buffer);
    return UNITY_END();
}
