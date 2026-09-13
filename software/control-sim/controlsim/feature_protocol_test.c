#include <assert.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>

#include "services/acoustic_feature_assembler.h"

static void feature_fill(acoustic_feature_t * feature, uint16_t event_id, uint16_t frame_index)
{
    memset(feature, 0, sizeof(*feature));
    feature->event_id = event_id;
    feature->frame_index = frame_index;
    feature->frame_count = ACOUSTIC_FEATURE_EVENT_FRAME_COUNT;
    feature->n_bins = ACOUSTIC_FEATURE_BIN_COUNT;
    feature->flags = (uint8_t) (frame_index == 0U ? 1U : 0U);

    for (uint16_t offset = 0U; offset < ACOUSTIC_FEATURE_FRAMES_PER_PACKET; offset++) {
        uint16_t const frame = (uint16_t) (frame_index + offset);
        for (uint16_t bin = 0U; bin < ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
            uint16_t const flat_index = (uint16_t) ((offset * ACOUSTIC_FEATURE_BIN_COUNT) + bin);
            feature->mel[flat_index] = (int8_t) ((((uint32_t) frame * ACOUSTIC_FEATURE_BIN_COUNT + bin) % 256U) - 128);
        }
    }
}

static void protocol_round_trip_test(void)
{
    acoustic_feature_t input;
    feature_fill(&input, 0x1234U, 6U);
    input.flags = 0xA5U;

    uint8_t encoded[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE] = {0};
    size_t const encoded_size =
        acoustic_protocol_encode_feature(0x89ABCDEFU, 0x10203040U, &input, encoded, sizeof(encoded));
    assert(encoded_size == ACOUSTIC_PROTOCOL_HEADER_SIZE + ACOUSTIC_FEATURE_PAYLOAD_SIZE + ACOUSTIC_PROTOCOL_CRC_SIZE);
    assert(encoded[3] == ACOUSTIC_MESSAGE_FEATURE);
    assert(encoded[14] == 0x34U);
    assert(encoded[15] == 0x12U);
    assert(encoded[16] == 6U);
    assert(encoded[20] == ACOUSTIC_FEATURE_BIN_COUNT);
    assert(encoded[21] == 0xA5U);

    acoustic_protocol_parser_t parser;
    acoustic_protocol_parser_init(&parser);
    acoustic_frame_t frame = {0};
    acoustic_parse_result_t parse_result = ACOUSTIC_PARSE_MORE;
    for (size_t index = 0U; index < encoded_size; index++) {
        parse_result = acoustic_protocol_parser_push(&parser, encoded[index], &frame);
        assert((index + 1U == encoded_size) || (parse_result == ACOUSTIC_PARSE_MORE));
    }
    assert(parse_result == ACOUSTIC_PARSE_FRAME_READY);
    assert(frame.sequence == 0x89ABCDEFU);
    assert(frame.uptime_ms == 0x10203040U);

    acoustic_feature_t output = {0};
    assert(acoustic_protocol_decode_feature(&frame, &output));
    assert(output.event_id == input.event_id);
    assert(output.frame_index == input.frame_index);
    assert(output.frame_count == input.frame_count);
    assert(output.n_bins == input.n_bins);
    assert(output.flags == input.flags);
    assert(memcmp(output.mel, input.mel, sizeof(input.mel)) == 0);

    assert(acoustic_protocol_encode_feature(0U, 0U, NULL, encoded, sizeof(encoded)) == 0U);
    assert(acoustic_protocol_encode_feature(0U, 0U, &input, encoded, encoded_size - 1U) == 0U);
    frame.type = ACOUSTIC_MESSAGE_HEALTH;
    assert(!acoustic_protocol_decode_feature(&frame, &output));
    frame.type = ACOUSTIC_MESSAGE_FEATURE;
    frame.payload_length--;
    assert(!acoustic_protocol_decode_feature(&frame, &output));
}

static void assembler_complete_test(void)
{
    acoustic_feature_assembler_t assembler;
    acoustic_feature_assembler_init(&assembler);
    acoustic_feature_t feature;

    for (uint16_t frame = 0U; frame < ACOUSTIC_FEATURE_EVENT_FRAME_COUNT;
         frame = (uint16_t) (frame + ACOUSTIC_FEATURE_FRAMES_PER_PACKET)) {
        feature_fill(&feature, 77U, frame);
        acoustic_feature_assembler_result_t const result = acoustic_feature_assembler_push(&assembler, &feature);
        if (frame + ACOUSTIC_FEATURE_FRAMES_PER_PACKET == ACOUSTIC_FEATURE_EVENT_FRAME_COUNT) {
            assert(result == CPU0_ACOUSTIC_FEATURE_COMPLETE);
        } else {
            assert(result == CPU0_ACOUSTIC_FEATURE_MORE);
        }
    }

    assert(!assembler.active);
    assert(assembler.patch.event_id == 77U);
    for (uint16_t frame = 0U; frame < ACOUSTIC_FEATURE_EVENT_FRAME_COUNT; frame++) {
        for (uint16_t bin = 0U; bin < ACOUSTIC_FEATURE_BIN_COUNT; bin++) {
            int8_t const expected =
                (int8_t) ((((uint32_t) frame * ACOUSTIC_FEATURE_BIN_COUNT + bin) % 256U) - 128);
            assert(assembler.patch.frames[frame][bin] == expected);
        }
    }
}

static void assembler_rejection_test(void)
{
    acoustic_feature_assembler_t assembler;
    acoustic_feature_t feature;
    acoustic_feature_assembler_init(&assembler);

    feature_fill(&feature, 1U, 2U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR);

    feature_fill(&feature, 1U, 0U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_MORE);
    feature_fill(&feature, 1U, 4U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR);

    feature_fill(&feature, 1U, 0U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_MORE);
    feature_fill(&feature, 1U, 2U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_MORE);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR);

    feature_fill(&feature, 1U, 0U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_MORE);
    feature_fill(&feature, 2U, 2U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_SEQUENCE_ERROR);

    feature_fill(&feature, 1U, 0U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_MORE);
    feature_fill(&feature, 2U, 0U);
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_RESTARTED);
    assert(assembler.active);
    assert(assembler.patch.event_id == 2U);

    feature.n_bins = ACOUSTIC_FEATURE_BIN_COUNT - 1U;
    assert(acoustic_feature_assembler_push(&assembler, &feature) == CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR);
    assert(!assembler.active);
    assert(acoustic_feature_assembler_push(NULL, &feature) == CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR);
    assert(acoustic_feature_assembler_push(&assembler, NULL) == CPU0_ACOUSTIC_FEATURE_FORMAT_ERROR);
}

int main(void)
{
    protocol_round_trip_test();
    assembler_complete_test();
    assembler_rejection_test();
    return 0;
}
