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

static void observation_round_trip_test(void)
{
    acoustic_observation_t const input = {
        .doa_deg = 359U,
        .raw_doa_deg = 1U,
        .level_dbfs_x100 = -3210,
        .peak_dbfs_x100 = -2800,
        .vad = 1U,
        .doa_confidence = 87U,
        .xvf_status = ACOUSTIC_XVF_STATUS_READY,
        .audio_flags = ACOUSTIC_AUDIO_FLAG_DOA_FALLBACK,
        .xvf_raw_status = 0x5AU,
        .reserved = 0U,
        .audio_frame_count = 0x12345678U,
        .sample_sequence = 0xFEDCBA98U,
    };
    uint8_t encoded[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE] = {0};
    size_t const encoded_size =
        acoustic_protocol_encode_observation(7U, 11U, &input, encoded, sizeof(encoded));
    assert(encoded_size == ACOUSTIC_PROTOCOL_HEADER_SIZE + ACOUSTIC_OBSERVATION_PAYLOAD_SIZE +
                           ACOUSTIC_PROTOCOL_CRC_SIZE);

    acoustic_protocol_parser_t parser;
    acoustic_protocol_parser_init(&parser);
    acoustic_frame_t frame = {0};
    acoustic_parse_result_t result = ACOUSTIC_PARSE_MORE;
    for (size_t index = 0U; index < encoded_size; index++) {
        result = acoustic_protocol_parser_push(&parser, encoded[index], &frame);
    }
    assert(result == ACOUSTIC_PARSE_FRAME_READY);
    acoustic_observation_t output = {0};
    assert(acoustic_protocol_decode_observation(&frame, &output));
    assert(memcmp(&input, &output, sizeof(input)) == 0);
}

static void navigation_diagnostics_round_trip_test(void)
{
    acoustic_nav_diagnostics_t const input = {
        .schema_version = 1U,
        .flags = ACOUSTIC_NAV_FLAG_SOURCE_VALID | ACOUSTIC_NAV_FLAG_TARGET_VALID,
        .doa_confidence = 82U,
        .arrival_state = 2U,
        .observation_sequence = 0x12345678U,
        .raw_doa_deg = 359U,
        .filtered_doa_deg = 1U,
        .rover_x_mm = -1200,
        .rover_y_mm = 340,
        .rover_heading_mrad = -1570,
        .source_x_mm = 4500,
        .source_y_mm = -800,
        .source_range_mm = 3800U,
        .source_bearing_deg = -18,
        .source_confidence = 76U,
        .observation_count = 12U,
        .localization_residual_mm = 45U,
        .crossing_angle_deg = 31U,
        .baseline_mm = 920U,
        .source_position_shift_mm = 55U,
        .arrival_confirm_count = 2U,
        .sensor_rule = 7U,
        .think_state = 18U,
        .reserved = 0U,
        .autonomous_backup_count = 0U,
    };
    uint8_t encoded[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE] = {0};
    size_t const encoded_size = acoustic_protocol_encode_nav_diagnostics(3U, 4U, &input, encoded, sizeof(encoded));
    assert(encoded_size == ACOUSTIC_PROTOCOL_HEADER_SIZE + ACOUSTIC_NAV_DIAGNOSTICS_PAYLOAD_SIZE +
                           ACOUSTIC_PROTOCOL_CRC_SIZE);

    acoustic_protocol_parser_t parser;
    acoustic_protocol_parser_init(&parser);
    acoustic_frame_t frame = {0};
    acoustic_parse_result_t result = ACOUSTIC_PARSE_MORE;
    for (size_t index = 0U; index < encoded_size; index++) {
        result = acoustic_protocol_parser_push(&parser, encoded[index], &frame);
    }
    assert(result == ACOUSTIC_PARSE_FRAME_READY);
    acoustic_nav_diagnostics_t output = {0};
    assert(acoustic_protocol_decode_nav_diagnostics(&frame, &output));
    assert(memcmp(&input, &output, sizeof(input)) == 0);
}

static acoustic_frame_t parse_single_frame(uint8_t const * bytes, size_t length)
{
    acoustic_protocol_parser_t parser;
    acoustic_protocol_parser_init(&parser);
    acoustic_frame_t frame = {0};
    acoustic_parse_result_t result = ACOUSTIC_PARSE_MORE;
    for (size_t index = 0U; index < length; index++) {
        result = acoustic_protocol_parser_push(&parser, bytes[index], &frame);
    }
    assert(result == ACOUSTIC_PARSE_FRAME_READY);
    return frame;
}

static void feature_channel_capability_test(void)
{
    acoustic_hello_t const hello = {
        .firmware_major = 1U,
        .capabilities = ACOUSTIC_CAPABILITY_DOA | ACOUSTIC_CAPABILITY_FEATURE_SLOT1,
        .boot_id = 1234U,
    };
    uint8_t encoded[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE] = {0};
    size_t const used = acoustic_protocol_encode_hello(3U, 4U, &hello, encoded, sizeof(encoded));
    assert(used != 0U);
    acoustic_frame_t const frame = parse_single_frame(encoded, used);
    acoustic_hello_t decoded = {0};
    assert(acoustic_protocol_decode_hello(&frame, &decoded));
    assert((decoded.capabilities & ACOUSTIC_CAPABILITY_FEATURE_SLOT1) != 0U);
    assert(decoded.boot_id == hello.boot_id);
}

static void ai_lab_diagnostic_protocol_test(void)
{
    assert(ACOUSTIC_ROVER_TELEMETRY_PAYLOAD_SIZE == 96U);
    assert(ACOUSTIC_AI_LAB_SNAPSHOT_PAYLOAD_SIZE == 48U);

    acoustic_ai_lab_snapshot_t const snapshot = {
        .schema_version = 2U,
        .flags = ACOUSTIC_AI_LAB_FLAG_SUMMARY_VALID,
        .think_state = 3U,
        .infer_status = 3U,
        .doa_deg = 128U,
        .level_dbfs_x100 = -3200,
        .peak_dbfs_x100 = -2400,
        .vad = 1U,
        .xvf_status = ACOUSTIC_XVF_STATUS_READY,
        .learning_samples = 5U,
        .target_peak_bin = 29U,
        .current_peak_bin = 4U,
        .nearest_sample = 2U,
        .active_frame_count = 80U,
        .cosine_distance_x1000 = 242U,
        .identifier_threshold_x1000 = 55U,
        .feature_generation = 0x12345678U,
        .reserved = {0x05U, 0xFAU, 0x00U},
    };
    uint8_t encoded[ACOUSTIC_PROTOCOL_MAX_FRAME_SIZE] = {0};
    size_t const length = acoustic_protocol_encode_ai_lab_snapshot(1U, 2U, &snapshot, encoded, sizeof(encoded));
    assert(length == ACOUSTIC_PROTOCOL_HEADER_SIZE + ACOUSTIC_AI_LAB_SNAPSHOT_PAYLOAD_SIZE +
                    ACOUSTIC_PROTOCOL_CRC_SIZE);
    assert(encoded[ACOUSTIC_PROTOCOL_HEADER_SIZE + 45U] == 0x05U);
    assert(encoded[ACOUSTIC_PROTOCOL_HEADER_SIZE + 46U] == 0xFAU);
    assert(encoded[ACOUSTIC_PROTOCOL_HEADER_SIZE + 47U] == 0x00U);
    acoustic_frame_t frame = parse_single_frame(encoded, length);
    acoustic_ai_lab_snapshot_t decoded_snapshot = {0};
    assert(acoustic_protocol_decode_ai_lab_snapshot(&frame, &decoded_snapshot));
    assert(decoded_snapshot.schema_version == 2U);
    assert(decoded_snapshot.feature_generation == snapshot.feature_generation);
    assert(memcmp(decoded_snapshot.reserved, snapshot.reserved, sizeof(snapshot.reserved)) == 0);

    uint8_t payload[ACOUSTIC_AI_LAB_SUMMARY_CHUNK_PAYLOAD_SIZE] = {0};
    payload[0] = 0x78U;
    payload[1] = 0x56U;
    payload[2] = 0x34U;
    payload[3] = 0x12U;
    payload[4] = 2U;
    payload[5] = 3U;
    payload[6] = 1U;
    payload[7] = 9U;
    for (size_t index = 0U; index < ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE; index++) {
        payload[8U + index] = (uint8_t) index;
    }
    size_t chunk_length = acoustic_protocol_encode(ACOUSTIC_MESSAGE_AI_LAB_SUMMARY_CHUNK, 3U, 4U,
        payload, sizeof(payload), encoded, sizeof(encoded));
    frame = parse_single_frame(encoded, chunk_length);
    acoustic_ai_lab_summary_chunk_t summary_chunk = {0};
    assert(acoustic_protocol_decode_ai_lab_summary_chunk(&frame, &summary_chunk));
    assert(summary_chunk.feature_generation == snapshot.feature_generation);
    assert(summary_chunk.chunk_index == 2U && summary_chunk.chunk_count == 3U);
    assert(summary_chunk.schema_version == 1U && summary_chunk.cpu_drop_count == 9U);
    assert((uint8_t) summary_chunk.data[63] == 63U);

    memset(payload, 0, sizeof(payload));
    payload[0] = 0xEFU;
    payload[1] = 0xCDU;
    payload[2] = 0xABU;
    payload[3] = 0x89U;
    payload[4] = 4U;
    payload[5] = 1U;
    payload[6] = 3U;
    payload[7] = 5U;
    for (size_t index = 0U; index < ACOUSTIC_AI_LAB_CHUNK_DATA_SIZE; index++) {
        payload[8U + index] = (uint8_t) (255U - index);
    }
    chunk_length = acoustic_protocol_encode(ACOUSTIC_MESSAGE_AI_LAB_PROFILE_CHUNK, 5U, 6U,
        payload, sizeof(payload), encoded, sizeof(encoded));
    frame = parse_single_frame(encoded, chunk_length);
    acoustic_ai_lab_profile_chunk_t profile_chunk = {0};
    assert(acoustic_protocol_decode_ai_lab_profile_chunk(&frame, &profile_chunk));
    assert(profile_chunk.profile_generation == 0x89ABCDEFU);
    assert(profile_chunk.sample_index == 4U && profile_chunk.chunk_index == 1U);
    assert(profile_chunk.chunk_count == 3U && profile_chunk.sample_count == 5U);
    assert((uint8_t) profile_chunk.data[63] == (uint8_t) (255U - 63U));
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
    observation_round_trip_test();
    feature_channel_capability_test();
    navigation_diagnostics_round_trip_test();
    ai_lab_diagnostic_protocol_test();
    assembler_complete_test();
    assembler_rejection_test();
    return 0;
}
