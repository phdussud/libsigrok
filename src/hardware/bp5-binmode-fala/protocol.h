/*
 * This file is part of the libsigrok project.
 *
 * Copyright (C) 2024 Patrick Dussud <phdussud@hotmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef LIBSIGROK_HARDWARE_BP5_BINMODE_FALA_PROTOCOL_H
#define LIBSIGROK_HARDWARE_BP5_BINMODE_FALA_PROTOCOL_H

#include <stdint.h>
#include <glib.h>
#include <libsigrok/libsigrok.h>
#include "libsigrok-internal.h"

#define LOG_PREFIX "bp5-binmode-fala"
#define LOGIC_BUFSIZE 4096
#define MIN_NUM_SAMPLES 4

struct dev_context
{
    uint64_t cur_samplerate;
    uint64_t limit_samples;
    uint64_t limit_msec;
    uint64_t limit_frames;
    uint64_t num_samples;
    uint64_t sent_frame_samples; /* Number of samples that were sent for current frame. */
    uint32_t num_transfers;
    int64_t start_us;
    int64_t spent_us;
    uint64_t step;
    /* Logic */
    int32_t num_logic_channels;
    size_t logic_unitsize;
    uint64_t all_logic_channels_mask;
    uint8_t *raw_sample_buf;
};


typedef struct fala_header
{
    uint32_t n_channels;
    uint32_t trigger_channel_mask;
    uint32_t trigger_mask;
    gboolean edge_trigger;
    uint32_t sample_rate;
    uint32_t sample_count;
    uint32_t before_trigger_sample_count;
} fala_header;

SR_PRIV gboolean parse_header(const char *buf, fala_header *hd);
SR_PRIV int bp5_binmode_fala_receive_data(int fd, int revents, void *cb_data);
#endif
