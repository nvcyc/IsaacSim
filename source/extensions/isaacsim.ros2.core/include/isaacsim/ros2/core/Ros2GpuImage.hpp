// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <cstddef>
#include <cstdint>
#include <memory>
#include <string>

namespace isaacsim
{
namespace ros2
{
namespace core
{
/** Read-only device image metadata, valid only while its lease is retained. */
struct Ros2GpuImageView
{
    const void* data = nullptr;
    size_t size = 0;
    uint32_t width = 0;
    uint32_t height = 0;
    uint32_t step = 0;
    int32_t device = -1;
    bool promotedFromHost = false;
    bool isBigEndian = false;
    double timeStamp = 0.0;
    std::string encoding;
    std::string frameId;
};

/**
 * Owns externally supplied image memory and its synchronization state.
 * Submit all reads on the stream passed to acquireGpuRead before releasing the
 * lease. That stream and its CUDA context must outlive the lease. Do not retain
 * a raw pointer after release. Implementations record completion when released.
 */
class Ros2GpuImageLease
{
public:
    virtual ~Ros2GpuImageLease() = default;
    /** Return immutable metadata for the leased device allocation. */
    virtual const Ros2GpuImageView& getView() const = 0;
};

/**
 * Optional image capability, discovered by dynamic_cast from Ros2ImageMessage.
 * Separate inheritance preserves all existing message/factory vtables. No ROS
 * or CUDA library types cross this interface; stream is a cudaStream_t erased
 * to void*. Calls on one message must be serialized.
 */
class Ros2GpuImage
{
public:
    virtual ~Ros2GpuImage() = default;
    /**
     * Prepare an image from an external device allocation, after writeHeader.
     * A zero size denotes a CUDA mipmapped float32 texture (32FC1 only).
     * The source must be ready for reading; this call completes its D2D copy on
     * the supplied non-null stream before returning, allowing source reuse.
     * Returns false if acceleration is unavailable or preparation failed; the
     * caller may explicitly use its existing CPU path. No device pointer is
     * ever exposed through the legacy getBufferPtr accessor.
     */
    virtual bool prepareDeviceImage(const void* source,
                                    size_t size,
                                    uint32_t width,
                                    uint32_t height,
                                    const std::string& encoding,
                                    int32_t device,
                                    void* stream) = 0;
    /**
     * Transfer a received payload into a lease. CPU input is explicitly promoted
     * to device storage and marked in the view. Returns null on failure.
     * A non-null consumer stream on the specified device is mandatory.
     */
    virtual std::shared_ptr<Ros2GpuImageLease> acquireGpuRead(int32_t device, void* stream) = 0;
};
}
}
}
