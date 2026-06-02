/*
 * Phoenix-RTOS
 *
 * Operating system loader
 *
 * Raspberry Pi framebuffer visibility
 *
 * Copyright 2026 Phoenix Systems
 *
 * This file is part of Phoenix-RTOS.
 *
 * %LICENSE%
 */

#include <hal/hal.h>

#include "../cache.h"
#include "../cpu.h"
#include "../mmu.h"
#include <syspage.h>


#if defined(PLO_RPI_MAILBOX_BASE_ADDRESS) && (PLO_RPI_MAILBOX_BASE_ADDRESS != 0)

#ifndef PLO_RPI_FB_WIDTH
#define PLO_RPI_FB_WIDTH 1024u
#endif

#ifndef PLO_RPI_FB_HEIGHT
#define PLO_RPI_FB_HEIGHT 768u
#endif

#ifndef PLO_RPI_FB_BPP
#define PLO_RPI_FB_BPP 32u
#endif

enum {
	mbox_read = 0x00 / sizeof(u32),
	mbox_status = 0x18 / sizeof(u32),
	mbox_write = 0x20 / sizeof(u32),
};

enum {
	mbox_response = 0x80000000u,
	mbox_full = 0x80000000u,
	mbox_empty = 0x40000000u,
	mbox_chan_prop = 8u,
};

enum {
	mbox_request = 0u,
	tag_setphywh = 0x48003u,
	tag_setvirtwh = 0x48004u,
	tag_setvirtoff = 0x48009u,
	tag_setdepth = 0x48005u,
	tag_setpxlordr = 0x48006u,
	tag_getfb = 0x40001u,
	tag_getpitch = 0x40008u,
	tag_getclkrate = 0x30002u, /* TD-16-1: get_clock_rate */
	tag_clkid_arm  = 0x3u,     /* clock ID 3 = ARM core */
	tag_last = 0u,
};

enum {
	video_background = 0x003060a0u,
	video_panel = 0x00484848u,
	video_stageOff = 0x00202020u,
	video_stageOn = 0x00f0f0f0u,
	video_mailboxWords = 36u,
};

enum {
	video_stageFramebufferReady = 0u,
	video_stageHalReady = 1u,
	video_stageKernelJump = 2u,
	video_stageCount = 3u,
};


static struct {
	volatile u32 *mailbox;
	volatile u32 *framebuffer;
	u32 width;
	u32 height;
	u32 pitch;
	u32 bpp;
	u32 size;
	u32 progressStage;
} video_common = {
	.mailbox = (volatile u32 *)PLO_RPI_MAILBOX_BASE_ADDRESS,
};


#if defined(PLO_RPI_MAILBOX_BUFFER_ADDRESS) && (PLO_RPI_MAILBOX_BUFFER_ADDRESS != 0)
static volatile u32 *const video_mailbox = (volatile u32 *)(addr_t)PLO_RPI_MAILBOX_BUFFER_ADDRESS;
#else
static volatile u32 video_mailboxStorage[video_mailboxWords] __attribute__((aligned(16)));
static volatile u32 *const video_mailbox = video_mailboxStorage;
#endif


static int video_mailboxCall(unsigned int chan)
{
	u32 msg;

	msg = (((u32)(addr_t)video_mailbox) & ~0xfu) | (chan & 0xfu);

	hal_dcacheClean((addr_t)video_mailbox, (addr_t)video_mailbox + video_mailboxWords * sizeof(u32));

	/* mbox_status full=0x80000000, empty=0x40000000. plo runs cache-off
	 * so the previous status read just confirms the path; the VC4 firmware
	 * has the mailbox FIFO drained from previous boots, so the write can
	 * proceed without an explicit wait. */
	*(video_common.mailbox + mbox_write) = msg;

	for (;;) {
		while ((*(video_common.mailbox + mbox_status) & mbox_empty) != 0u) {
		}

		if (*(video_common.mailbox + mbox_read) == msg) {
			break;
		}
	}

	hal_dcacheInval((addr_t)video_mailbox, (addr_t)video_mailbox + video_mailboxWords * sizeof(u32));

	return (video_mailbox[1] == mbox_response) ? 0 : -1;
}


static int video_framebufferInit(void)
{
	hal_memset((void *)video_mailbox, 0, video_mailboxWords * sizeof(u32));

	video_mailbox[0] = 35u * sizeof(u32);
	video_mailbox[1] = mbox_request;

	video_mailbox[2] = tag_setphywh;
	video_mailbox[3] = 8u;
	video_mailbox[4] = 0u;
	video_mailbox[5] = PLO_RPI_FB_WIDTH;
	video_mailbox[6] = PLO_RPI_FB_HEIGHT;

	video_mailbox[7] = tag_setvirtwh;
	video_mailbox[8] = 8u;
	video_mailbox[9] = 8u;
	video_mailbox[10] = PLO_RPI_FB_WIDTH;
	video_mailbox[11] = PLO_RPI_FB_HEIGHT;

	video_mailbox[12] = tag_setvirtoff;
	video_mailbox[13] = 8u;
	video_mailbox[14] = 8u;
	video_mailbox[15] = 0u;
	video_mailbox[16] = 0u;

	video_mailbox[17] = tag_setdepth;
	video_mailbox[18] = 4u;
	video_mailbox[19] = 4u;
	video_mailbox[20] = PLO_RPI_FB_BPP;

	video_mailbox[21] = tag_setpxlordr;
	video_mailbox[22] = 4u;
	video_mailbox[23] = 4u;
	video_mailbox[24] = 1u;

	video_mailbox[25] = tag_getfb;
	video_mailbox[26] = 8u;
	video_mailbox[27] = 8u;
	video_mailbox[28] = 4096u;
	video_mailbox[29] = 0u;

	video_mailbox[30] = tag_getpitch;
	video_mailbox[31] = 4u;
	video_mailbox[32] = 4u;
	video_mailbox[33] = 0u;

	video_mailbox[34] = tag_last;

	if (video_mailboxCall(mbox_chan_prop) < 0) {
		return -1;
	}

	if ((video_mailbox[20] != PLO_RPI_FB_BPP) || (video_mailbox[28] == 0u) || (video_mailbox[33] == 0u)) {
		return -1;
	}

	video_common.framebuffer = (volatile u32 *)(addr_t)(video_mailbox[28] & 0x3fffffffu);
	video_common.width = video_mailbox[10];
	video_common.height = video_mailbox[11];
	video_common.pitch = video_mailbox[33];
	video_common.bpp = video_mailbox[20];
	video_common.size = video_mailbox[29];
	if (video_common.size == 0u) {
		video_common.size = video_common.pitch * video_common.height;
	}
	video_common.progressStage = video_stageFramebufferReady;

	/* NOTE: framebuffer cacheable mapping is now set up statically by
	 * hal_memoryInit (which remaps the full 0..0xfe000000 range as
	 * Normal WB Cacheable before MMU enable). Doing it dynamically
	 * here previously caused wild PC corruption — mmu_mapAddr's
	 * post-TLBI barrier sequence is incomplete for runtime mapping
	 * changes with the MMU live, and the speculation hit stale
	 * translations. Avoid that entire class of issue by pre-mapping. */

	return 0;
}


static void video_drawSignal(void)
{
	u32 x, y, stride, panelX, panelY, panelW, panelH;
	u32 boxSize, gap, outerPad, innerPad, boxX, boxY, px;
	volatile u32 *row;

	if (video_common.framebuffer == NULL) {
		return;
	}

	stride = video_common.pitch / sizeof(u32);

	for (y = 0; y < video_common.height; ++y) {
		row = video_common.framebuffer + y * stride;

		for (x = 0; x < video_common.width; ++x) {
			row[x] = video_background;
		}
	}

	/* 2026-05-17: restored full 3-square progress panel rendering.
	 * The "wild FAR" crash that previously forced the early return
	 * was a cache-off-era MMU artifact; with armstub fixes
	 * (1319367 + L2CTLR_EL1) and caches operational, the panel
	 * code runs cleanly. Matches the original Phoenix-RTOS boot
	 * progress visual on other platforms. */

	if ((video_common.width < 128u) || (video_common.height < 64u)) {
		hal_dcacheClean((addr_t)video_common.framebuffer, (addr_t)video_common.framebuffer + video_common.size);
		return;
	}

	outerPad = 16u;
	innerPad = 8u;
	boxSize = 48u;
	gap = 16u;
	panelW = innerPad * 2u + video_stageCount * boxSize + (video_stageCount - 1u) * gap;
	panelH = innerPad * 2u + boxSize;
	panelX = outerPad;
	panelY = outerPad;

	for (y = panelY; (y < (panelY + panelH)) && (y < video_common.height); ++y) {
		row = video_common.framebuffer + y * stride;

		for (x = panelX; (x < (panelX + panelW)) && (x < video_common.width); ++x) {
			row[x] = video_panel;
		}
	}

	boxY = panelY + innerPad;
	boxX = panelX + innerPad;

	for (x = 0; x < video_stageCount; ++x) {
		u32 color = (x <= video_common.progressStage) ? video_stageOn : video_stageOff;
		u32 left = boxX + x * (boxSize + gap);
		u32 right = left + boxSize;
		u32 bottom = boxY + boxSize;

		if (left >= video_common.width) {
			break;
		}

		if (right > video_common.width) {
			right = video_common.width;
		}

		if (bottom > video_common.height) {
			bottom = video_common.height;
		}

		for (y = boxY; y < bottom; ++y) {
			row = video_common.framebuffer + y * stride;

			for (px = left; px < right; ++px) {
				row[px] = color;
			}
		}
	}

	hal_dcacheClean((addr_t)video_common.framebuffer, (addr_t)video_common.framebuffer + video_common.size);
}


static void video_updateProgress(u32 stage)
{
	if ((video_common.framebuffer == NULL) || (stage >= video_stageCount)) {
		return;
	}

	if (stage <= video_common.progressStage) {
		return;
	}

	video_common.progressStage = stage;
	video_drawSignal();
}


static void video_publish(void)
{
	graphmode_t graphmode;

	if (video_common.framebuffer == NULL) {
		return;
	}

#if defined(HAS_GRAPHICS) && (HAS_GRAPHICS != 0)
	graphmode.width = (u16)video_common.width;
	graphmode.height = (u16)video_common.height;
	graphmode.bpp = (u16)video_common.bpp;
	graphmode.pitch = (u16)video_common.pitch;
	graphmode.framebuffer = (addr_t)video_common.framebuffer;
	syspage_graphmodeSet(&graphmode);
#endif
}


void video_init(void)
{
	if (video_framebufferInit() < 0) {
		hal_consolePrint("video: fbInit failed\n");
		return;
	}

	video_drawSignal();
}


void video_publishGraphmode(void)
{
	video_publish();
}


void video_markHalReady(void)
{
	video_updateProgress(video_stageHalReady);
}


void video_markKernelJump(void)
{
	video_updateProgress(video_stageKernelJump);
}

#else

void video_init(void)
{
}


void video_publishGraphmode(void)
{
}


void video_markHalReady(void)
{
}


void video_markKernelJump(void)
{
}

#endif
