// Copyright 2020-2025, Collabora, Ltd.
// SPDX-License-Identifier: BSL-1.0
/*!
 * @file
 * @brief Header exposing factory function for AHardwareBuffer backed image
   buffer allocator.
 * @author Rylie Pavlik <rylie.pavlik@collabora.com>
 * @author Simon Zeni <simon.zeni@collabora.com>
 * @ingroup aux_android
 */

#pragma once

#include "xrt/xrt_compositor.h"
#include "xrt/xrt_handles.h"

#ifdef __cplusplus
extern "C" {
#endif

#ifdef XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER

struct xrt_image_native_allocator *
android_ahardwarebuffer_allocator_create(void);

xrt_result_t
ahardwarebuffer_image_allocate(const struct xrt_swapchain_create_info *xsci, xrt_graphics_buffer_handle_t *out_image);

bool
ahardwarebuffer_is_supported(uint64_t format, enum xrt_swapchain_usage_bits xbits);

#endif // XRT_GRAPHICS_BUFFER_HANDLE_IS_AHARDWAREBUFFER

#ifdef __cplusplus
}
#endif
