/*
    pkTriggerCord
    Remote control of Pentax DSLR cameras.
    Copyright (C) 2011-2019 Andras Salamon <andras.salamon@melda.info>

    based on:

    pslr-shoot

    Command line remote control of Pentax DSLR cameras.
    Copyright (C) 2009 Ramiro Barreiro <ramiro_barreiro69@yahoo.es>
    With fragments of code from PK-Remote by Pontus Lidman.
    <https://sourceforge.net/projects/pkremote>

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU Lesser General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU Lesser General Public License for more details.

    You should have received a copy of the GNU General Public License
    and GNU Lesser General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdio.h>
#include <unistd.h>
#include <getopt.h>
#include <fcntl.h>
#include <ctype.h>
#include <time.h>
#include <stdarg.h>
#include <math.h>
#include <sys/time.h>

#include "pslr.h"

#ifdef WIN32
#define FILE_ACCESS O_WRONLY | O_CREAT | O_TRUNC | O_BINARY
#else
#define FILE_ACCESS O_WRONLY | O_CREAT | O_TRUNC
#endif

extern char *optarg;
extern int optind, opterr, optopt;
bool debug = false;
bool warnings = false;

const char *shortopts = "m:q:a:r:d:t:o:i:F:fghvsSw";

pslr_settings settings;
bool bulb_timer_before=false;
bool astrotracer_before=false;
bool need_bulb_new_cleanup=false;
bool need_one_push_bracketing_cleanup=false;

static struct option const longopts[] = {
    {"exposure_mode", required_argument, NULL, 'm'},
    {"resolution", required_argument, NULL, 'r'},
    {"quality", required_argument, NULL, 'q'},
    {"aperture", required_argument, NULL, 'a'},
    {"shutter_speed", required_argument, NULL, 't'},
    {"iso", required_argument, NULL, 'i'},
    {"file_format", required_argument, NULL, 1},
    {"output_file", required_argument, NULL, 'o'},
    {"help", no_argument, NULL, 'h'},
    {"version", no_argument, NULL, 'v'},
    {"status", no_argument, NULL, 's'},
    {"status_hex", no_argument, NULL, 2},
    {"frames", required_argument, NULL, 'F'},
    {"delay", required_argument, NULL, 'd'},
    {"auto_focus", no_argument, NULL, 'f'},
    {"green", no_argument, NULL, 'g'},
    {"warnings", no_argument, NULL, 'w'},
    {"exposure_compensation", required_argument, NULL, 3},
    {"flash_exposure_compensation", required_argument, NULL, 5},
    {"debug", no_argument, NULL, 4},
    {"dust_removal", no_argument, NULL, 6},
    {"color_space", required_argument, NULL, 7},
    {"af_mode", required_argument, NULL, 8},
    {"ae_metering", required_argument, NULL, 9},
    {"flash_mode", required_argument, NULL, 10},
    {"drive_mode", required_argument, NULL, 11},
    {"select_af_point", required_argument, NULL, 12},
    {"jpeg_image_tone", required_argument, NULL, 13},
    {"white_balance_mode", required_argument, NULL, 14},
    {"white_balance_adjustment", required_argument, NULL, 15},
    {"model", required_argument, NULL, 16},
    {"nowarnings", no_argument, NULL, 17},
    {"device", required_argument, NULL, 18},
    {"reconnect", no_argument, NULL, 19},
    {"timeout", required_argument, NULL, 20},
    {"noshutter", no_argument, NULL, 21},
    {"servermode", no_argument, NULL, 22},
    {"servermode_timeout", required_argument, NULL, 23},
    {"pentax_debug_mode", required_argument, NULL,24},
    {"dangerous", no_argument, NULL, 25},
    {"read_datetime", no_argument, NULL, 26},
    {"read_firmware_version", no_argument, NULL, 27},
    {"settings_hex", no_argument, NULL, 28},
    {"dump_memory", required_argument, NULL, 29},
    {"settings", no_argument, NULL, 'S'},
    { NULL, 0, NULL, 0}
};

int save_buffer(pslr_handle_t camhandle, int bufno, int fd, pslr_status *status, user_file_format filefmt, int jpeg_stars) {
    pslr_buffer_type imagetype;
    uint8_t buf[65536];
    uint32_t length;
    uint32_t current;

    if (filefmt == USER_FILE_FORMAT_PEF) {
        imagetype = PSLR_BUF_PEF;
    } else if (filefmt == USER_FILE_FORMAT_DNG) {
        imagetype = PSLR_BUF_DNG;
    } else {
        imagetype = pslr_get_jpeg_buffer_type( camhandle, jpeg_stars );
    }

    DPRINT("get buffer %d type %d res %d\n", bufno, imagetype, status->jpeg_resolution);

    if (pslr_buffer_open(camhandle, bufno, imagetype, status->jpeg_resolution) != PSLR_OK) {
        return 1;
    }

    length = pslr_buffer_get_size(camhandle);
    DPRINT("Buffer length: %d\n", length);
    current = 0;

    while (true) {
        uint32_t bytes;
        bytes = pslr_buffer_read(camhandle, buf, sizeof (buf));
        if (bytes == 0) {
            break;
        }
        ssize_t r = write(fd, buf, bytes);
        if (r == 0) {
            DPRINT("write(buf): Nothing has been written to buf.\n");
        } else if (r == -1) {
            perror("write(buf)");
        } else if (r < bytes) {
            DPRINT("write(buf): only write %zu bytes, should be %d bytes.\n", r, bytes);
        }
        current += bytes;
    }
    pslr_buffer_close(camhandle);
    return 0;
}

void save_memory(pslr_handle_t camhandle, int fd, uint32_t length) {
    uint8_t buf[65536];
    uint32_t current;

    DPRINT("save memory %d\n", length);

    current = 0;

    while (current<length) {
        uint32_t bytes;
        int readsize=length-current>65536 ? 65536 : length-current;
        bytes = pslr_fullmemory_read(camhandle, buf, current, readsize);
        if (bytes == 0) {
            break;
        }
        ssize_t r = write(fd, buf, bytes);
        if (r == 0) {
            DPRINT("write(buf): Nothing has been written to buf.\n");
        } else if (r == -1) {
            perror("write(buf)");
        } else if (r < bytes) {
            DPRINT("write(buf): only write %zu bytes, should be %d bytes.\n", r, bytes);
        }
        current += bytes;
    }
    return;
}


void print_status_info( pslr_handle_t h, pslr_status status ) {
    printf("\n");
    printf( "%s", collect_status_info( h, status ) );
}

void print_settings_info( pslr_handle_t h, pslr_settings settings ) {
    printf("\n");
    printf( "%s", collect_settings_info( h, settings ) );
}
