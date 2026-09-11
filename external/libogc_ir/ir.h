/*
 * Portable extraction of the libogc Wiiuse IR component, modified 2026-09-11.
 * Original declarations are copied verbatim from the pinned libogc sources.
 * This component remains under the GNU GPL with the libogc section 18 linking
 * exception, explicitly extended to this modified version. See
 * license_libogc.txt, libogc_license.txt, NOTICE.txt and UPSTREAM.json beside
 * this header. The surrounding independent project is not relicensed.
 * The original Wiiuse notice below is from upstream/wiiuse_internal.h.
 */
/*
 *	wiiuse
 *
 *	Written By:
 *		Michael Laforest	< para >
 *		Email: < thepara (--AT--) g m a i l [--DOT--] com >
 *
 *	Copyright 2006-2007
 *
 *	This file is part of wiiuse.
 *
 *	This program is free software; you can redistribute it and/or modify
 *	it under the terms of the GNU General Public License as published by
 *	the Free Software Foundation; either version 3 of the License, or
 *	(at your option) any later version.
 *
 *	This program is distributed in the hope that it will be useful,
 *	but WITHOUT ANY WARRANTY; without even the implied warranty of
 *	MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *	GNU General Public License for more details.
 *
 *	You should have received a copy of the GNU General Public License
 *	along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 *	$Header: /lvm/shared/ds/ds/cvs/devkitpro-cvsbackup/libogc/wiiuse/wiiuse_internal.h,v 1.8 2008-12-10 16:16:40 shagkur Exp $
 *
 */

#ifndef SWITCH_PICO_LIBOGC_IR_H
#define SWITCH_PICO_LIBOGC_IR_H

typedef unsigned char ubyte;
typedef float f32;

/* IR correction types */
typedef enum ir_position_t {
	WIIUSE_IR_ABOVE,
	WIIUSE_IR_BELOW
} ir_position_t;

/**
 *	@struct orient_t
 *	@brief Orientation struct.
 *
 *	Yaw, pitch, and roll range from -180 to 180 degrees.
 */
typedef struct orient_t {
	float roll;						/**< roll, this may be smoothed if enabled	*/
	float pitch;					/**< pitch, this may be smoothed if enabled	*/
	float yaw;

	float a_roll;					/**< absolute roll, unsmoothed				*/
	float a_pitch;					/**< absolute pitch, unsmoothed				*/
} orient_t;

/**
 *	@struct ir_dot_t
 *	@brief A single IR source.
 */
typedef struct ir_dot_t {
	ubyte visible;					/**< if the IR source is visible		*/

	short rx;						/**< raw X coordinate (0-1023)			*/
	short ry;						/**< raw Y coordinate (0-767)			*/

	ubyte size;						/**< size of the IR dot (0-15)			*/
} ir_dot_t;


typedef struct fdot_t {
	float x,y;
} fdot_t;

typedef struct sb_t {
	fdot_t dots[2];
	fdot_t acc_dots[2];
	fdot_t rot_dots[2];
	float angle;
	float off_angle;
	float score;
} sb_t;

/**
 *	@enum aspect_t
 *	@brief Screen aspect ratio.
 */
typedef enum aspect_t {
	WIIUSE_ASPECT_4_3,
	WIIUSE_ASPECT_16_9
} aspect_t;


/**
 *	@struct ir_t
 *	@brief IR struct. Hold all data related to the IR tracking.
 */
typedef struct ir_t {
	struct ir_dot_t dot[4];			/**< IR dots							*/
	ubyte num_dots;					/**< number of dots at this time		*/

	int state;						/**< keeps track of the IR state		*/

	int raw_valid;					/**< is the raw position valid? 		*/
	sb_t sensorbar;					/**< sensor bar, detected or guessed	*/
	float ax;						/**< raw X coordinate					*/
	float ay;						/**< raw Y coordinate					*/
	float distance;					/**< pixel width of the sensor bar		*/
	float z;						/**< calculated distance in meters		*/
	float angle;					/**< angle of the wiimote to the sensor bar*/

	int smooth_valid;				/**< is the smoothed position valid? 	*/
	float sx;						/**< smoothed X coordinate				*/
	float sy;						/**< smoothed Y coordinate				*/
	float error_cnt;				/**< error count, for smoothing algorithm*/
	float glitch_cnt;				/**< glitch count, same					*/

	int valid;						/**< is the bounded position valid? 	*/
	float x;						/**< bounded X coordinate				*/
	float y;						/**< bounded Y coordinate				*/
	enum aspect_t aspect;			/**< aspect ratio of the screen			*/
	enum ir_position_t pos;			/**< IR sensor bar position				*/
	unsigned int vres[2];			/**< IR virtual screen resolution		*/
	int offset[2];					/**< IR XY correction offset			*/

} ir_t;

enum {
	IR_STATE_DEAD = 0,
	IR_STATE_GOOD,
	IR_STATE_SINGLE,
	IR_STATE_LOST,
};

/* aspect ratio */
#define WM_ASPECT_16_9_X	660
#define WM_ASPECT_16_9_Y	370
#define WM_ASPECT_4_3_X		560
#define WM_ASPECT_4_3_Y		420

#ifdef __cplusplus
extern "C" {
#endif

float calc_yaw(struct ir_t* ir);
void interpret_ir_data(struct ir_t* ir, struct orient_t *orient);
void find_sensorbar(struct ir_t* ir, struct orient_t *orient);
void apply_ir_smoothing(struct ir_t *ir);

#ifdef __cplusplus
}
#endif

#endif /* SWITCH_PICO_LIBOGC_IR_H */
