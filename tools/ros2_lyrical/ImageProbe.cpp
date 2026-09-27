// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <rcl/error_handling.h>
#include <rcl/rcl.h>
#include <rmw/rmw.h>
#include <rosidl_typesupport_cpp/message_type_support.hpp>
#include <sensor_msgs/msg/image.hpp>

#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <type_traits>

#if defined(ISAAC_BRIDGE_IMAGE)
#    include "Ros2LyricalImage.hpp"

#    include <carb/ClientUtils.h>
CARB_FRAMEWORK_GLOBALS("image_bridge_probe");
CARB_LOG_GLOBALS();
CARB_ASSERT_GLOBALS();
using isaacsim::ros2::core::Ros2LyricalImageMessage;
#endif

#pragma push_macro("CUDA_CHECK")
#undef CUDA_CHECK
#include <cuda_buffer/cuda_buffer_api.hpp>
#pragma pop_macro("CUDA_CHECK")

void fillDevicePattern(uint8_t*, size_t, uint32_t, cudaStream_t);
void checkDevicePattern(const uint8_t*, size_t, uint32_t, unsigned int*, cudaStream_t);

namespace
{
using Image = sensor_msgs::msg::Image;
using Clock = std::chrono::steady_clock;
static_assert(std::is_same_v<Image::_data_type, rosidl::Buffer<uint8_t>>, "Build with generated Lyrical messages");

void checkRos(rcl_ret_t result)
{
    if (result != RCL_RET_OK)
    {
        const std::string error = rcl_get_error_string().str;
        rcl_reset_error();
        throw std::runtime_error(error);
    }
}

void checkCuda(cudaError_t result)
{
    if (result != cudaSuccess)
    {
        throw std::runtime_error(cudaGetErrorString(result));
    }
}

void cleanupRos(rcl_ret_t result) noexcept
{
    if (result != RCL_RET_OK)
    {
        std::fprintf(stderr, "ROS cleanup failed: %s\n", rcl_get_error_string().str);
        rcl_reset_error();
    }
}

// Destruction order: message/handles, endpoints, node, context, then CUDA stream.
class Session
{
public:
    Session() = default;
    ~Session()
    {
        if (publisher.impl)
        {
            cleanupRos(rcl_publisher_fini(&publisher, &node));
        }
        if (subscriber.impl)
        {
            cleanupRos(rcl_subscription_fini(&subscriber, &node));
        }
        if (node.impl)
        {
            cleanupRos(rcl_node_fini(&node));
        }
        if (rcl_context_is_valid(&context))
        {
            cleanupRos(rcl_shutdown(&context));
        }
        if (context.impl)
        {
            cleanupRos(rcl_context_fini(&context));
        }
        if (options.impl)
        {
            cleanupRos(rcl_init_options_fini(&options));
        }
        if (stream)
        {
            (void)cudaStreamSynchronize(stream);
            (void)cudaFree(errors);
            (void)cudaStreamDestroy(stream);
        }
    }

    void initialize(bool publish, bool cuda, const std::string& topic)
    {
        if (cuda)
        {
            checkCuda(cudaSetDevice(0));
            checkCuda(cudaStreamCreateWithFlags(&stream, cudaStreamNonBlocking));
            checkCuda(cudaMalloc(reinterpret_cast<void**>(&errors), sizeof(unsigned int)));
        }
        checkRos(rcl_init_options_init(&options, rcl_get_default_allocator()));
        checkRos(rcl_init(0, nullptr, &options, &context));
        auto nodeOptions = rcl_node_get_default_options();
        checkRos(rcl_node_init(&node, publish ? "image_probe_pub" : "image_probe_sub", "", &context, &nodeOptions));
        const auto* type = rosidl_typesupport_cpp::get_message_type_support_handle<Image>();
        if (publish)
        {
            auto endpointOptions = rcl_publisher_get_default_options();
            endpointOptions.qos.depth = 16;
            checkRos(rcl_publisher_init(&publisher, &node, type, topic.c_str(), &endpointOptions));
        }
        else
        {
            auto endpointOptions = rcl_subscription_get_default_options();
            endpointOptions.qos.depth = 16;
            checkRos(rcl_subscription_options_set_acceptable_buffer_backends(cuda ? "cuda" : nullptr, &endpointOptions));
            const auto result = rcl_subscription_init(&subscriber, &node, type, topic.c_str(), &endpointOptions);
            checkRos(result);
            // rcl_subscription_init shallow-copies options; subscription_fini owns the string now.
        }
    }

    rcl_init_options_t options = rcl_get_zero_initialized_init_options();
    rcl_context_t context = rcl_get_zero_initialized_context();
    rcl_node_t node = rcl_get_zero_initialized_node();
    rcl_publisher_t publisher = rcl_get_zero_initialized_publisher();
    rcl_subscription_t subscriber = rcl_get_zero_initialized_subscription();
    cudaStream_t stream = nullptr;
    unsigned int* errors = nullptr;
};

void publish(Session& session, bool cuda, uint32_t frames, size_t expectedSubscribers)
{
    const auto deadline = Clock::now() + std::chrono::seconds(30);
    size_t matches = 0;
    while (matches < expectedSubscribers)
    {
        checkRos(rcl_publisher_get_subscription_count(&session.publisher, &matches));
        if (Clock::now() > deadline)
        {
            throw std::runtime_error("DDS discovery timed out");
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(20));
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    for (uint32_t sequence = 1; sequence <= frames; ++sequence)
    {
#if defined(ISAAC_BRIDGE_IMAGE)
        Ros2LyricalImageMessage bridgeMessage;
        auto& message = *static_cast<Image*>(bridgeMessage.getPtr());
#else
        Image message;
#endif
        message.header.frame_id = std::to_string(sequence);
        // Alternate sizes to expose allocation reuse and stale frame errors.
        message.width = sequence % 2 ? 640 : 1280;
        message.height = sequence % 2 ? 480 : 720;
        message.encoding = "mono8";
        message.step = message.width;
        const size_t bytes = static_cast<size_t>(message.step) * message.height;
        if (cuda)
        {
#if defined(ISAAC_BRIDGE_IMAGE)
            void* source = nullptr;
            checkCuda(cudaMalloc(&source, bytes));
            fillDevicePattern(static_cast<uint8_t*>(source), bytes, sequence, session.stream);
            checkCuda(cudaGetLastError());
            const bool prepared = bridgeMessage.prepareDeviceImage(
                source, bytes, message.width, message.height, message.encoding, 0, session.stream);
            checkCuda(cudaFree(source));
            if (!prepared)
            {
                throw std::runtime_error("Bridge device preparation failed");
            }
#else
            message.data = cuda_buffer_backend::allocate_buffer(bytes);
            {
                auto write = cuda_buffer_backend::from_output_buffer(message.data, session.stream);
                fillDevicePattern(write.get_ptr(), bytes, sequence, session.stream);
                checkCuda(cudaGetLastError());
            } // Record producer completion BEFORE rcl_publish.
#endif
        }
        else
        {
#if defined(ISAAC_BRIDGE_IMAGE)
            bridgeMessage.generateBuffer(message.height, message.width, message.encoding, sequence % 2 == 0);
            auto* host = static_cast<uint8_t*>(bridgeMessage.getBufferPtr());
            for (size_t index = 0; index < bytes; ++index)
            {
                host[index] = static_cast<uint8_t>(index * 13 + sequence * 37);
            }
            isaacsim::ros2::core::prepareLyricalImageForPublish(bridgeMessage.getPtr());
#else
            message.data.resize(bytes);
            for (size_t index = 0; index < bytes; ++index)
            {
                message.data[index] = static_cast<uint8_t>(index * 13 + sequence * 37);
            }
#endif
        }
        checkRos(rcl_publish(&session.publisher, &message, nullptr));
        std::this_thread::sleep_for(std::chrono::milliseconds(33));
    }
    checkRos(rcl_publisher_wait_for_all_acked(&session.publisher, RCL_S_TO_NS(10)));
    std::this_thread::sleep_for(std::chrono::seconds(1));
    std::cout << "published=" << frames << '\n';
}

void subscribe(Session& session, bool cuda, uint32_t frames, const std::string& expectedBackend)
{
    const auto deadline = Clock::now() + std::chrono::seconds(60);
    uint32_t received = 0;
    uint32_t previousSequence = 0;
    while (received < frames)
    {
        if (Clock::now() > deadline)
        {
            throw std::runtime_error("Image receive timed out");
        }
#if defined(ISAAC_BRIDGE_IMAGE)
        auto bridgeMessage = std::make_unique<Ros2LyricalImageMessage>();
        auto& message = *static_cast<Image*>(bridgeMessage->getPtr());
#else
        Image message;
#endif
        rmw_message_info_t info{};
        const auto result = rcl_take(&session.subscriber, &message, &info, nullptr);
        if (result == RCL_RET_SUBSCRIPTION_TAKE_FAILED)
        {
            rcl_reset_error();
            std::this_thread::sleep_for(std::chrono::milliseconds(1));
            continue;
        }
        checkRos(result);
        const auto sequence = static_cast<uint32_t>(std::stoul(message.header.frame_id));
        if (sequence != previousSequence + 1)
        {
            throw std::runtime_error("Missing, duplicate, or reordered image");
        }
        previousSequence = sequence;
        const size_t bytes = static_cast<size_t>(message.step) * message.height;
        if (message.encoding != "mono8" || message.step != message.width || bytes != message.data.size() || !bytes)
        {
            throw std::runtime_error("Invalid image metadata");
        }
        const auto backend = message.data.get_backend_type();
        if (backend != expectedBackend)
        {
            throw std::runtime_error("Expected backend " + expectedBackend + ", received " + backend);
        }
        unsigned int errors = 0;
        if (cuda)
        {
            // The message outlives the read handle and all submitted CUDA work.
#if defined(ISAAC_BRIDGE_IMAGE)
            auto lease = bridgeMessage->acquireGpuRead(0, session.stream);
            if (!lease)
            {
                throw std::runtime_error("Bridge read lease failed");
            }
            if (lease->getView().promotedFromHost != (expectedBackend == "cpu"))
            {
                throw std::runtime_error("Incorrect CPU promotion metadata");
            }
            bridgeMessage.reset(); // The lease must keep the source alive independently.
            auto* readPointer = static_cast<const uint8_t*>(lease->getView().data);
#else
            auto read = cuda_buffer_backend::from_input_buffer(message.data, session.stream);
            const auto* readPointer = read.get_ptr();
#endif
            checkCuda(cudaMemsetAsync(session.errors, 0, sizeof(errors), session.stream));
            checkDevicePattern(readPointer, bytes, sequence, session.errors, session.stream);
            checkCuda(cudaGetLastError());
#if defined(ISAAC_BRIDGE_IMAGE)
            // Release before completion: the backend must retain the allocation
            // until the consumer event has completed, even without the wrapper.
            lease.reset();
#endif
            // Only four validation bytes return to the host; never the image payload.
            checkCuda(cudaMemcpyAsync(&errors, session.errors, sizeof(errors), cudaMemcpyDeviceToHost, session.stream));
            checkCuda(cudaStreamSynchronize(session.stream));
        }
        else
        {
            for (size_t index = 0; index < bytes; ++index)
            {
                errors += message.data[index] != static_cast<uint8_t>(index * 13 + sequence * 37);
            }
        }
        if (errors)
        {
            throw std::runtime_error("Corrupted image payload");
        }
        ++received;
    }
    std::cout << "received=" << received << " backend=" << expectedBackend << " payload=verified\n";
}
}

int main(int argc, char** argv)
{
    try
    {
        if (argc != 6)
        {
            throw std::runtime_error("Usage: image_probe pub|sub cpu|cuda /topic frames expected-subscribers|backend");
        }
        const std::string role = argv[1];
        const std::string storage = argv[2];
        if ((role != "pub" && role != "sub") || (storage != "cpu" && storage != "cuda"))
        {
            throw std::runtime_error("Invalid role or storage");
        }
        const auto frames = std::stoul(argv[4]);
        if (!frames || frames > 1000000)
        {
            throw std::runtime_error("Invalid frame count");
        }
        if (storage == "cuda" && std::string(rmw_get_implementation_identifier()) != "rmw_fastrtps_cpp")
        {
            throw std::runtime_error("CUDA probe requires rmw_fastrtps_cpp");
        }
        Session session;
        session.initialize(role == "pub", storage == "cuda", argv[3]);
        if (role == "pub")
        {
            publish(session, storage == "cuda", frames, std::stoul(argv[5]));
        }
        else
        {
            subscribe(session, storage == "cuda", frames, argv[5]);
        }
        return 0;
    }
    catch (const std::exception& error)
    {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
