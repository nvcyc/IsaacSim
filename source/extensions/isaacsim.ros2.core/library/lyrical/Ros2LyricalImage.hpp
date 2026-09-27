// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#pragma once

#include <pxr/base/gf/declare.h>
#include <pxr/usd/usd/common.h>
#include <sensor_msgs/msg/image.hpp>

// The existing message ABI header expects USD types to have been declared.
#include <isaacsim/ros2/core/Ros2GpuImage.hpp>
#include <isaacsim/ros2/core/Ros2Message.hpp>

namespace isaacsim
{
namespace ros2
{
namespace core
{
/** Backend-private envelope. rcl sees only its generated C++ Image base. */
struct LyricalImageStorage : sensor_msgs::msg::Image
{
    const uint8_t* hostData = nullptr;
    size_t hostSize = 0;
};

/** Lyrical-only generated C++ image; legacy host access is preserved. */
class Ros2LyricalImageMessage : public Ros2ImageMessage, public Ros2GpuImage
{
public:
    Ros2LyricalImageMessage();
    ~Ros2LyricalImageMessage();
    const void* getTypeSupportHandle() override;
    void writeHeader(double timestamp, const std::string& frameId) override;
    void generateBuffer(uint32_t height, uint32_t width, const std::string& encoding, bool pinned) override;
    bool prepareDeviceImage(const void* source,
                            size_t size,
                            uint32_t width,
                            uint32_t height,
                            const std::string& encoding,
                            int32_t device,
                            void* stream) override;
    std::shared_ptr<Ros2GpuImageLease> acquireGpuRead(int32_t device, void* stream) override;

private:
    LyricalImageStorage m_image;
};

/** Return the generated C++ Image handle; never use a generated C handle with it. */
const void* getLyricalImageTypeSupport();
/** Copy legacy host storage into the C++ message immediately before rcl_publish. */
void prepareLyricalImageForPublish(const void* message);
}
}
}
