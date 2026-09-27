// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include "Ros2LyricalImage.hpp"

#include <carb/logging/Log.h>
// Keep the upstream throwing CUDA_CHECK local to its headers. Isaac's macro
// logs instead of throwing and must not change the backend's error contract.
#pragma push_macro("CUDA_CHECK")
#undef CUDA_CHECK
#include <cuda_buffer/cuda_buffer_api.hpp>
#pragma pop_macro("CUDA_CHECK")
#include <rmw/rmw.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/image_encodings.hpp>

#include <cmath>
#include <cstring>
#include <limits>
#include <optional>
#include <stdexcept>
#include <type_traits>

namespace isaacsim
{
namespace ros2
{
namespace core
{
namespace
{
using Image = sensor_msgs::msg::Image;
static_assert(std::is_same_v<Image::_data_type, rosidl::Buffer<uint8_t>>, "Lyrical generated Image is required");

void checkCuda(cudaError_t result)
{
    if (result != cudaSuccess)
    {
        throw std::runtime_error(cudaGetErrorString(result));
    }
}

class DeviceScope
{
public:
    explicit DeviceScope(int device)
    {
        checkCuda(cudaGetDevice(&m_previous));
        checkCuda(cudaSetDevice(device));
    }
    ~DeviceScope()
    {
        (void)cudaSetDevice(m_previous);
    }

private:
    int m_previous = 0;
};

size_t setLayout(Image& image, uint32_t width, uint32_t height, const std::string& encoding)
{
    if (!width || !height)
    {
        throw std::invalid_argument("Empty image");
    }
    const uint64_t step = uint64_t(width) * sensor_msgs::image_encodings::numChannels(encoding) *
                          (sensor_msgs::image_encodings::bitDepth(encoding) / 8);
    if (!step || step > UINT32_MAX || step > SIZE_MAX / height)
    {
        throw std::overflow_error("Image layout exceeds addressable size");
    }
    image.width = width;
    image.height = height;
    image.step = static_cast<uint32_t>(step);
    image.encoding = encoding;
    image.is_bigendian = 0;
    return static_cast<size_t>(step) * height;
}

class ImageLease : public Ros2GpuImageLease
{
public:
    ImageLease(Image&& image, int device, cudaStream_t stream) : m_image(std::move(image)), m_device(device)
    {
        DeviceScope scope(device);
        m_view.promotedFromHost = m_image->data.get_backend_type() != "cuda";
        if (m_view.promotedFromHost)
        {
            if (cuda_buffer_backend::get_or_create_global_pool()->get_device_id() != device)
            {
                throw std::runtime_error("CPU promotion pool belongs to a different CUDA device");
            }
        }
        else
        {
            const auto* implementation =
                dynamic_cast<const cuda_buffer_backend::CudaBufferImpl<uint8_t>*>(m_image->data.get_impl());
            if (!implementation || implementation->get_device_id() != device)
            {
                throw std::runtime_error("Received CUDA image belongs to a different device");
            }
        }
        m_read = cuda_buffer_backend::from_input_buffer(m_image->data, stream);
        cudaPointerAttributes attributes{};
        checkCuda(cudaPointerGetAttributes(&attributes, m_read.get_ptr()));
        if (attributes.type != cudaMemoryTypeDevice || attributes.device != device)
        {
            throw std::runtime_error("Image CUDA device differs from consumer device");
        }
        m_view.data = m_read.get_ptr();
        m_view.size = m_image->data.size();
        m_view.width = m_image->width;
        m_view.height = m_image->height;
        m_view.step = m_image->step;
        m_view.device = device;
        m_view.encoding = m_image->encoding;
        m_view.frameId = m_image->header.frame_id;
        m_view.isBigEndian = m_image->is_bigendian != 0;
        m_view.timeStamp = m_image->header.stamp.sec + m_image->header.stamp.nanosec * 1e-9;
        // CPU promotion may read pageable host storage asynchronously. Retain it
        // until that H2D has completed; the CUDA fast path does not block here.
        if (m_view.promotedFromHost)
        {
            checkCuda(cudaStreamSynchronize(stream));
        }
    }
    ~ImageLease() override
    {
        int previous = 0;
        const auto result = cudaGetDevice(&previous);
        if (result == cudaSuccess)
        {
            (void)cudaSetDevice(m_device);
        }
        // Record consumption before destroying its CudaBuffer/source message.
        m_read = cuda_buffer_backend::ReadHandle{};
        m_image.reset();
        if (result == cudaSuccess)
        {
            (void)cudaSetDevice(previous);
        }
    }
    const Ros2GpuImageView& getView() const override
    {
        return m_view;
    }

private:
    std::optional<Image> m_image;
    int m_device;
    cuda_buffer_backend::ReadHandle m_read;
    Ros2GpuImageView m_view;
};
}

const void* getLyricalImageTypeSupport()
{
    return rosidl_typesupport_cpp::get_message_type_support_handle<Image>();
}

Ros2LyricalImageMessage::Ros2LyricalImageMessage()
{
    // The inherited accessor indexes element zero even before allocation.
    m_buffer.resize(1);
    m_msg = static_cast<Image*>(&m_image);
}

Ros2LyricalImageMessage::~Ros2LyricalImageMessage()
{
    if (m_pinnedBuffer)
    {
        (void)cudaFreeHost(m_pinnedBuffer);
    }
}

const void* Ros2LyricalImageMessage::getTypeSupportHandle()
{
    return getLyricalImageTypeSupport();
}

void Ros2LyricalImageMessage::writeHeader(double timestamp, const std::string& frameId)
{
    m_image.header.frame_id = frameId;
    if (!std::isfinite(timestamp) || timestamp < INT32_MIN || timestamp >= double(INT32_MAX) + 1.0)
    {
        m_image.header.stamp.sec = 0;
        m_image.header.stamp.nanosec = 0;
        CARB_LOG_ERROR("Lyrical Image timestamp is outside the ROS time range");
        return;
    }
    const auto seconds = std::floor(timestamp);
    m_image.header.stamp.sec = static_cast<int32_t>(seconds);
    m_image.header.stamp.nanosec = static_cast<uint32_t>((timestamp - seconds) * 1e9);
}

void Ros2LyricalImageMessage::generateBuffer(uint32_t height, uint32_t width, const std::string& encoding, bool pinned)
{
    m_image.hostData = nullptr;
    m_image.hostSize = 0;
    m_totalBytes = 0;
    if (m_pinnedBuffer)
    {
        (void)cudaFreeHost(m_pinnedBuffer);
        m_pinnedBuffer = nullptr;
    }
    m_isPinnedMemory = false;
    try
    {
        const auto bytes = setLayout(m_image, width, height, encoding);
        m_image.data = rosidl::Buffer<uint8_t>{};
        if (pinned && cudaMallocHost(&m_pinnedBuffer, bytes) == cudaSuccess)
        {
            m_isPinnedMemory = true;
        }
        else
        {
            m_buffer.resize(bytes);
        }
        m_totalBytes = bytes;
        m_image.hostSize = bytes;
        m_image.hostData = static_cast<const uint8_t*>(getBufferPtr());
    }
    catch (const std::exception& error)
    {
        // Keep the inherited nonvirtual host accessor safe even on failure.
        m_buffer.resize(1);
        CARB_LOG_ERROR("Lyrical Image allocation failed: %s", error.what());
    }
}

void prepareLyricalImageForPublish(const void* message)
{
    auto& image = *static_cast<LyricalImageStorage*>(const_cast<Image*>(static_cast<const Image*>(message)));
    if (image.hostData)
    {
        image.data.resize(image.hostSize);
        std::memcpy(image.data.data(), image.hostData, image.hostSize);
    }
}

bool Ros2LyricalImageMessage::prepareDeviceImage(const void* source,
                                                 size_t size,
                                                 uint32_t width,
                                                 uint32_t height,
                                                 const std::string& encoding,
                                                 int32_t device,
                                                 void* streamPointer)
{
    if (!source || !streamPointer || device < 0 ||
        std::strcmp(rmw_get_implementation_identifier(), "rmw_fastrtps_cpp") != 0)
    {
        return false;
    }
    try
    {
        DeviceScope scope(device);
        const auto stream = static_cast<cudaStream_t>(streamPointer);
        const auto bytes = setLayout(m_image, width, height, encoding);
        if ((size && size != bytes) || (!size && encoding != "32FC1"))
        {
            throw std::invalid_argument("Invalid device image size or texture encoding");
        }
        if (cuda_buffer_backend::get_or_create_global_pool()->get_device_id() != device)
        {
            throw std::runtime_error("CUDA buffer pool belongs to a different device");
        }
        auto buffer = cuda_buffer_backend::allocate_buffer(bytes);
        {
            auto write = cuda_buffer_backend::from_output_buffer(buffer, stream);
            cudaPointerAttributes attributes{};
            checkCuda(cudaPointerGetAttributes(&attributes, write.get_ptr()));
            if (attributes.device != device)
            {
                throw std::runtime_error("CUDA buffer pool belongs to a different device");
            }
            if (size)
            {
                checkCuda(cudaPointerGetAttributes(&attributes, source));
                if (attributes.type != cudaMemoryTypeDevice || attributes.device != device)
                {
                    throw std::invalid_argument("Source is not device memory on the requested CUDA device");
                }
                cuda_buffer_backend::to_buffer(source, bytes, write, stream, cudaMemcpyDeviceToDevice);
            }
            else
            {
                cudaArray_t array = nullptr;
                checkCuda(cudaGetMipmappedArrayLevel(
                    &array, reinterpret_cast<cudaMipmappedArray_t>(const_cast<void*>(source)), 0));
                checkCuda(cudaMemcpy2DFromArrayAsync(write.get_ptr(), m_image.step, array, 0, 0, m_image.step, height,
                                                     cudaMemcpyDeviceToDevice, stream));
            }
            // The renderer owns source only during this graph evaluation.
            checkCuda(cudaStreamSynchronize(stream));
        }
        m_image.data = std::move(buffer);
        m_image.hostData = nullptr;
        m_image.hostSize = 0;
        // GPU preparation has its own size in m_image.data. Preserve the
        // legacy host buffer's size and pointer as a consistent pair.
        return true;
    }
    catch (const std::exception& error)
    {
        CARB_LOG_WARN("Lyrical CUDA image unavailable; using CPU path: %s", error.what());
        return false;
    }
}

std::shared_ptr<Ros2GpuImageLease> Ros2LyricalImageMessage::acquireGpuRead(int32_t device, void* stream)
{
    if (!stream || device < 0)
    {
        return {};
    }
    try
    {
        if (!m_image.width || !m_image.height || !m_image.step ||
            uint64_t(m_image.step) * m_image.height != m_image.data.size())
        {
            throw std::invalid_argument("Received image has invalid dimensions or payload length");
        }
        const uint64_t minimumStep = uint64_t(m_image.width) *
                                     sensor_msgs::image_encodings::numChannels(m_image.encoding) *
                                     (sensor_msgs::image_encodings::bitDepth(m_image.encoding) / 8);
        if (!minimumStep || minimumStep > m_image.step)
        {
            throw std::invalid_argument("Received image row stride is smaller than its encoding requires");
        }
        m_image.hostData = nullptr;
        m_image.hostSize = 0;
        return std::make_shared<ImageLease>(
            std::move(static_cast<Image&>(m_image)), device, static_cast<cudaStream_t>(stream));
    }
    catch (const std::exception& error)
    {
        CARB_LOG_ERROR("Cannot acquire Lyrical GPU image: %s", error.what());
        return {};
    }
}
}
}
}
