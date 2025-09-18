/*
 * Blob detection
 * Copyright 2014-2015 Philipp Zabel
 * Copyright 2019-2023 Jan Schmidt
 * SPDX-License-Identifier:	LGPL-2.0+ or BSL-1.0
 */
/*!
 * @file
 * @brief  Blob thresholding and tracking in camera frames
 * @author Jan Schmidt <jan@centricular.com>
 * @ingroup constellation
 */
#include <assert.h>
#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>

#include "util/u_logging.h"

#include "blobwatch.h"

/* Keep enough history frames that the caller can keep hold
 * of a previous blobservation and pass it to the long term
 * tracker while we still have 2 left to ping-pong between */
#define NUM_FRAMES_HISTORY 5
#define MAX_EXTENTS_PER_LINE 30

/* Set to 1 to do extra array tracking consistency checks */
#define CONSISTENCY_CHECKS 0

#define QUEUE_ENTRIES (NUM_FRAMES_HISTORY + 1)

#define abs(x) ((x) >= 0 ? (x) : -(x))
#define min(x, y) ((x) < (y) ? (x) : (y))
#define max(x, y) ((x) > (y) ? (x) : (y))

struct extent
{
	uint16_t start;
	uint16_t end;
	/* inherited parameters */
	uint16_t top;
	uint16_t left;
	uint16_t right;
	uint32_t area;

	/* Maximum pixel colour detected */
	uint8_t max_pixel;
};

struct extent_line
{
	struct extent extents[MAX_EXTENTS_PER_LINE];
	uint16_t num;
	uint16_t padding[3];
};

typedef struct blobservation_queue blobservation_queue;

struct blobservation_queue
{
	blobservation *data[QUEUE_ENTRIES];
	unsigned int head, tail;
};

#define INIT_QUEUE(q) (q)->head = (q)->tail = 0;

#define PUSH_QUEUE(q, b)                                                                                               \
	do {                                                                                                           \
		unsigned int next = ((q)->tail + 1) % QUEUE_ENTRIES;                                                   \
		assert(next != (q)->head); /* Check there's room */                                                    \
		assert((b) != NULL);                                                                                   \
		(q)->data[(q)->tail] = (b);                                                                            \
		(q)->tail = next;                                                                                      \
	} while (0)

static blobservation *
POP_QUEUE(blobservation_queue *q)
{
	blobservation *b;
	unsigned int next_head = (q->head + 1) % QUEUE_ENTRIES;

	if ((q)->tail == (q)->head) /* Check there's something in the queue */
		return NULL;

	b = q->data[q->head];
	q->head = next_head;

	return b;
}

/*
 * Blob detector internal state
 */
struct blobwatch
{
	uint32_t next_blob_id;
	uint8_t pixel_threshold;         /* Minimum pixel magnitude considered non-black */
	uint8_t blob_required_threshold; /* Minimum pixel magnitude a blob must contain somewhere to be retained */
	int blob_max_wh;

	bool debug;

	blobservation observations[NUM_FRAMES_HISTORY];

	blobservation_queue observation_q;

	blobservation *last_observation;
};

/*
 * Allocates and initializes blobwatch structure.
 *
 * Returns the newly allocated blobwatch structure.
 */
blobwatch *
blobwatch_new(uint8_t pixel_threshold, uint8_t blob_required_threshold)
{
	blobwatch *bw = malloc(sizeof(*bw));
	int i;

	if (!bw)
		return NULL;

	memset(bw, 0, sizeof(*bw));
	bw->next_blob_id = 1;

	/* Minimum pixel magnitude to be included in a blob at all */
	bw->pixel_threshold = pixel_threshold;

	/* Require at least 1 pixel over this threshold in a blob -
	 * allows for collecting fainter blobs, as long as they have a bright
	 * point somewhere, and helps to eliminate generally faint background noise */
	bw->blob_required_threshold = blob_required_threshold;

	/* Don't store blobs that are too big to be LEDs sensibly
	 * (arbitrary 35 pixel cut-off. FIXME: revisit this number) */
	bw->blob_max_wh = 35;

	bw->last_observation = NULL;
	bw->debug = true;

	INIT_QUEUE(&bw->observation_q);
	/* Push all observations into the available queue */
	for (i = 0; i < NUM_FRAMES_HISTORY; i++)
		PUSH_QUEUE(&bw->observation_q, bw->observations + i);

	return bw;
}

void
blobwatch_free(blobwatch *bw)
{
	free(bw);
}

static void
compute_greysum(blobwatch *bw, struct xrt_frame *frame, struct extent *e, int end_y, float *led_x, float *led_y)
{
	const uint16_t width = e->right - e->left + 1;
	const uint16_t height = end_y - e->top + 1;
	uint8_t *pixels;
	uint16_t x, y;
	uint32_t x_pos, y_pos;
	uint32_t greysum_total = 0, greysum_x = 0, greysum_y = 0;

	/* Point to top left pixel of the extent */
	pixels = frame->data + frame->stride * e->top + e->left;

	for (y = 0; y < height; y++) {
		/* Use 1...frame_width/height as coords, otherwise the first column never contributes anything */
		y_pos = e->top + y + 1;
		x_pos = e->left + 1;

		for (x = 0; x < width; x++) {
			uint32_t pix = pixels[x];

			greysum_total += pix;
			greysum_x += x_pos * pix;
			greysum_y += y_pos * pix;
			x_pos++;
		}

		pixels += frame->stride;
	}

	*led_x = (float)(greysum_x) / greysum_total - 1;
	*led_y = (float)(greysum_y) / greysum_total - 1;
}

/*
 * Stores blob information collected in the last extent e into the blob
 * array b at the given index.
 */
static inline void
store_blob(struct extent *e, int index, int end_y, struct blob *b, uint32_t blob_id, float led_x, float led_y)
{
	b += index;
	b->blob_id = blob_id;
	b->x = led_x;
	b->y = led_y;
	b->vx = 0;
	b->vy = 0;

	b->left = e->left;
	b->top = e->top;
	b->width = e->right - e->left + 1;
	b->height = end_y - e->top + 1;
	b->area = e->area;
	b->age = 0;
	b->track_index = -1;
	b->id_age = 0;
	b->prev_led_id = b->led_id = LED_INVALID_ID;
}

static void
extent_to_blobs(blobwatch *bw, blobservation *ob, struct extent *e, int y, struct xrt_frame *frame)
{
	const int max_blobs = MAX_BLOBS_PER_FRAME;
	struct blob *blobs = ob->blobs;

	/* Don't store unless there was at least one "bright enough" pixel in the blob */
	if (e->max_pixel < bw->blob_required_threshold) {
		ob->dropped_dark_blobs++;
		return;
	}

	/* Don't store 1x1 blobs */
	if (e->top == y && e->left == e->right)
		return;

	/* Check width and height against the blob "maximum size" */
	if (y - e->top > bw->blob_max_wh || e->right - e->left > bw->blob_max_wh)
		return;

	/* In the future we could generate multiple blobs from one extent if we detect
	 * it as multiple LEDs */
	while (ob->num_blobs < max_blobs) {
		float led_x, led_y;

		compute_greysum(bw, frame, e, y, &led_x, &led_y);

		store_blob(e, ob->num_blobs++, y, blobs, bw->next_blob_id++, led_x, led_y);
		break;
	}
}

/*
 * Collects contiguous ranges of pixels with values larger than a threshold of
 * THRESHOLD in a given scanline and stores them in extents. Processing stops after
 * num_extents.
 * Extents are marked with the same index as overlapping extents of the previous
 * scanline, and properties of the formed blobs are accumulated.
 *
 * Returns the number of extents found.
 */
static void
process_scanline(uint8_t *line,
                 blobwatch *bw,
                 uint32_t y,
                 struct extent_line *el,
                 struct extent_line *prev_el,
                 struct xrt_frame *frame,
                 blobservation *ob)
{
	struct extent *le_end = prev_el->extents;
	struct extent *le = prev_el->extents;
	struct extent *extent = el->extents;
	int num_extents = MAX_EXTENTS_PER_LINE;
	float center;
	uint32_t x;
	int e = 0;

	if (prev_el)
		le_end += prev_el->num;

	for (x = 0; x < frame->width; x++) {
		int start, end;
		bool is_new_extent = true;
		uint8_t max_pixel = 0;

		/* Loop until pixel value exceeds threshold */
		if (line[x] <= bw->pixel_threshold)
			continue;

		start = x++;

		/* Loop until pixel value falls below threshold */
		while (x < frame->width && line[x] > bw->pixel_threshold) {
			if (line[x] > max_pixel)
				max_pixel = line[x];
			x++;
		}

		end = x - 1;

		center = (start + end) / 2.0;

		extent->start = start;
		extent->end = end;
		extent->area = x - start;
		extent->max_pixel = max_pixel;

		if (prev_el) {
			/*
			 * Previous extents without significant overlap are the
			 * bottom of finished blobs. Store them into an array.
			 */
			while (le < le_end && le->end < center) {
				extent_to_blobs(bw, ob, le, y, frame);
				le++;
			}

			/*
			 * A previous extent with significant overlap is
			 * considered to be part of the same blob.
			 */
			if (le < le_end && le->start <= center && le->end >= center) {
				extent->top = le->top;
				extent->left = min(extent->start, le->left);
				extent->right = max(extent->end, le->right);
				if (le->max_pixel > extent->max_pixel)
					extent->max_pixel = le->max_pixel;
				extent->area += le->area;
				is_new_extent = false;
				le++;
			}
		}

		/*
		 * If this extent is not part of a previous blob, increment the
		 * blob index.
		 */
		if (is_new_extent) {
			extent->top = y;
			extent->left = extent->start;
			extent->right = extent->end;
		}

		if (++e == num_extents)
			break;
		extent++;
	}

	if (prev_el) {
		/*
		 * If there are no more extents on this line, all remaining
		 * extents in the previous line are finished blobs. Store them.
		 */
		while (le < le_end) {
			extent_to_blobs(bw, ob, le, y, frame);
			le++;
		}
	}

	el->num = e;

	if (y == frame->height - 1) {
		/* All extents of the last line are finished blobs, too. */
		for (extent = el->extents; extent < el->extents + el->num; extent++) {
			extent_to_blobs(bw, ob, extent, y, frame);
		}
	}
}

/*
 * Processes extents from all scanlines in a frame and stores the
 * resulting blobs in ob->blobs.
 */
static void
process_frame(blobwatch *bw, blobservation *ob, struct xrt_frame *frame)
{
	struct extent_line el1;
	struct extent_line el2;

	ob->num_blobs = 0;
	ob->dropped_dark_blobs = 0;

	uint8_t *line = frame->data;
	process_scanline(line, bw, 0, &el1, NULL, frame, ob);

	for (uint32_t y = 1; y < frame->height; y++) {
		process_scanline(line, bw, y, y & 1 ? &el2 : &el1, y & 1 ? &el1 : &el2, frame, ob);
		line += frame->stride;
	}
}

/*
 * Finds the first free tracking slot.
 */
static int
find_free_track(uint8_t *tracked)
{
	int i;

	for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
		if (tracked[i] == 0)
			return i;
	}

	return -1;
}

void
copy_matching_blob(struct blob *to, struct blob *from)
{
	to->blob_id = from->blob_id;
	to->vx = to->x - from->x;
	to->vy = to->y - from->y;
	to->id_age = from->id_age;
	to->led_id = from->led_id;
	to->age = from->age + 1;
}

/*
 * Detects blobs in the current frame and compares them with the observation
 * history. The returned blobservation in the `output` variable must be returned
 * to the blobwatch via blobwatch_release_observation()
 */
void
blobwatch_process(blobwatch *bw, struct xrt_frame *frame, blobservation **output)
{
	blobservation *ob = POP_QUEUE(&bw->observation_q);
	assert(ob != NULL);

	process_frame(bw, ob, frame);

	/* Return observed blobs */
	if (output) {
		*output = ob;
	}

	/* If there is no previous observation, our work is done here - no need to match
	 * blobs against the prior */
	if (bw->last_observation == NULL) {
		bw->last_observation = ob;
		return;
	}

	blobservation *last_ob = bw->last_observation;
	int closest_ob[MAX_BLOBS_PER_FRAME];             // index of last_ob that is closest to each ob
	int closest_last_ob[MAX_BLOBS_PER_FRAME];        // index of ob that is closest to each last_ob
	int closest_last_ob_distsq[MAX_BLOBS_PER_FRAME]; // distsq of ob that is closest to each last_ob
	int i, j;

	/* Clear closest_* */
	for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
		closest_ob[i] = -1;
		closest_last_ob[i] = -1;
		closest_last_ob_distsq[i] = 1000000;
	}

	int scan_again = 1;
	int scan_times = 0;
	while (scan_again) {
		scan_again = 0;

		/* Try to match each blob with the closest blob from the previous frame. */
		for (i = 0; i < ob->num_blobs; i++) {
			if (closest_ob[i] != -1)
				continue; // already has a match

			struct blob *b2 = &ob->blobs[i];
			int closest_j = -1;
			int closest_distsq = -1;

			for (j = 0; j < last_ob->num_blobs; j++) {
				struct blob *b1 = &last_ob->blobs[j];
				int x, y, dx, dy, distsq;

				/* Estimate b1's next position */
				x = b1->x + b1->vx;
				y = b1->y + b1->vy;

				/* Absolute distance */
				dx = abs(x - b2->x);
				dy = abs(y - b2->y);
				distsq = dx * dx + dy * dy;

				if (closest_distsq < 0 || distsq < closest_distsq) {
					if (closest_last_ob[j] != -1 && closest_last_ob_distsq[j] <= distsq) {
						// some blob already claimed this one as closest
						// don't usurp if previous one is closer
						continue;
					}
					closest_j = j;
					closest_distsq = distsq;
				}
			}

			closest_ob[i] = closest_j;

			// didn't find any matching blobs
			if (closest_j < 0)
				continue;

			if (closest_last_ob[closest_j] != -1) {
				// we are usurping some other blob because we are closer
				closest_ob[closest_last_ob[closest_j]] = -1;
				scan_again++;
			}

			closest_last_ob[closest_j] = i;
			closest_last_ob_distsq[closest_j] = closest_distsq;
		}

		if (scan_times++ > 100) {
			U_LOG_W("blob matching looped excessively. scan_times: %d", scan_times);
		}
	}

	/* Copy blobs that found a closest match */
	for (i = 0; i < ob->num_blobs; i++) {
		if (closest_ob[i] < 0)
			continue; // no match

		struct blob *b2 = &ob->blobs[i];
		struct blob *b1 = &last_ob->blobs[closest_ob[i]];

		if (b1->track_index >= 0 && ob->tracked[b1->track_index] == 0) {
			/* Only overwrite tracks that are not already set */
			b2->track_index = b1->track_index;
			ob->tracked[b2->track_index] = i + 1;
		}
		copy_matching_blob(b2, b1);
	}

	/*
	 * Clear the tracking array where blobs have gone missing.
	 */
	for (i = 0; i < MAX_BLOBS_PER_FRAME; i++) {
		int t = ob->tracked[i];
		if (t > 0 && ob->blobs[t - 1].track_index != i)
			ob->tracked[i] = 0;
	}

	/*
	 * Associate newly tracked blobs with a free space in the
	 * tracking array.
	 */
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b2 = &ob->blobs[i];

		if (b2->age > 0 && b2->track_index < 0)
			b2->track_index = find_free_track(ob->tracked);
		if (b2->track_index >= 0)
			ob->tracked[b2->track_index] = i + 1;
	}

#if CONSISTENCY_CHECKS
	/* Check blob <-> tracked array links for consistency */
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b = &ob->blobs[i];

		if (b->track_index >= 0 && ob->tracked[b->track_index] != i + 1) {
			U_LOG_W("Inconsistency! %d != %d", ob->tracked[b->track_index], i + 1);
		}
	}
#endif

	bw->last_observation = ob;
}

struct blob *
blobwatch_find_blob_at(blobwatch *bw, int x, int y)
{
	blobservation *ob = bw->last_observation;
	int i;

	if (ob == NULL) {
		/* No blobs to match against yet */
		return NULL;
	}

	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b = ob->blobs + i;
		int dx = abs(x - b->x);
		int dy = abs(y - b->y);

		/* Check if the target is outside the bounding box */
		if (2 * dx > b->width || 2 * dy > b->height)
			continue;
		return b;
	}

	return NULL;
}

void
blobwatch_update_labels(blobwatch *bw, blobservation *ob, uint8_t device_id)
{
	/* Take the observation in ob and replace any labels for the given device in
	 * the most recent observation with labels from this observation */
	/* FIXME: This is an n^2 match for simplicity. Filtering out blobs with no LED
	 * label and sorting the blobs by ID might make things quicker for larger numbers
	 * of blobs - needs testing. */
	blobservation *last_ob = bw->last_observation;
	int i, l;

	if (last_ob == NULL || last_ob == ob) {
		/* No label transfers needed, increment the ID age */
		for (i = 0; i < ob->num_blobs; i++) {
			struct blob *b = ob->blobs + i;
			if (b->led_id != LED_INVALID_ID && b->led_id == b->prev_led_id)
				b->id_age++;
			else
				b->id_age = 0;
		}
		return;
	}

	/* Clear all labels for the indicated device */
	for (l = 0; l < last_ob->num_blobs; l++) {
		struct blob *new_b = last_ob->blobs + l;
		if (LED_OBJECT_ID(new_b->led_id) == device_id) {
			new_b->prev_led_id = new_b->led_id;
			new_b->led_id = LED_INVALID_ID;
		}
	}

	/* Transfer all labels for matching blob ids */
	for (i = 0; i < ob->num_blobs; i++) {
		struct blob *b = ob->blobs + i;
		if (LED_OBJECT_ID(b->led_id) != device_id)
			continue; /* Not for this device */

		for (l = 0; l < last_ob->num_blobs; l++) {
			struct blob *new_b = last_ob->blobs + l;
			if (new_b->blob_id == b->blob_id) {
				if (bw->debug) {
					U_LOG_D("Found matching blob %u - labelled with LED id %x\n", b->blob_id,
					        b->led_id);
				}
				new_b->led_id = b->led_id;

				if (new_b->led_id == new_b->prev_led_id) {
					new_b->id_age++;
				} else {
					new_b->id_age = 0;
				}
			}
		}
	}
}

void
blobwatch_release_observation(blobwatch *bw, blobservation *ob)
{
	PUSH_QUEUE(&bw->observation_q, ob);
}
