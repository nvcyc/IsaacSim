// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <isaacsim/core/includes/ScopedCudaDevice.hpp>
#include <isaacsim/ros2/core/Ros2GpuImage.hpp>
#include <isaacsim/ros2/core/Ros2Node.hpp>

#include <OgnROS2SubscribeImageDatabase.h>

using namespace isaacsim::ros2::core;

class OgnROS2SubscribeImage : public Ros2Node
{
public:
    ~OgnROS2SubscribeImage()
    {
        _releaseResources();
    }

    static bool compute(OgnROS2SubscribeImageDatabase& db)
    {
        auto& state = db.perInstanceState<OgnROS2SubscribeImage>();
        state.m_nodeObj = db.abi_node();
        db.outputs.execOut() = kExecutionAttributeStateDisabled;
        db.outputs.dataPtr() = 0;
        db.outputs.bufferSize() = 0;
        // A lease records completion of reads queued by the previous evaluation.
        state.m_lease.reset();
        if (db.inputs.cudaDeviceIndex() < 0)
        {
            db.logError("ROS2SubscribeImage requires a CUDA device index");
            return false;
        }
        if (state.m_stream && state.m_device != db.inputs.cudaDeviceIndex())
        {
            state.reset();
        }

        if (!state.isInitialized())
        {
            const auto& node = db.abi_node();
            const auto& context = db.abi_context();
            auto stage = pxr::UsdUtilsStageCache::Get().Find(
                pxr::UsdStageCache::Id::FromLongInt(context.iContext->getStageId(context)));
            if (!stage || !state.initializeNodeHandle(
                              std::string(node.iNode->getPrimPath(node)),
                              collectNamespace(db.inputs.nodeNamespace(),
                                               stage->GetPrimAtPath(pxr::SdfPath(node.iNode->getPrimPath(node)))),
                              db.inputs.context()))
            {
                return false;
            }
        }
        if (!state.m_subscriber)
        {
            state.m_message = state.m_factory->createImageMessage();
            state.m_gpuImage = dynamic_cast<Ros2GpuImage*>(state.m_message.get());
            if (!state.m_gpuImage)
            {
                db.logError("ROS2SubscribeImage requires the optional Lyrical GPU image backend");
                return false;
            }
            const auto topic = addTopicPrefix(state.m_namespaceName, db.inputs.topicName());
            if (!state.m_factory->validateTopicName(topic))
            {
                return false;
            }
            Ros2QoSProfile qos;
            qos.depth = db.inputs.queueSize();
            if (!db.inputs.qosProfile().empty() && !jsonToRos2QoSProfile(qos, db.inputs.qosProfile()))
            {
                return false;
            }
            state.m_subscriber = state.m_factory->createSubscriber(
                state.m_nodeHandle.get(), topic.c_str(), state.m_message->getTypeSupportHandle(), qos);
            if (!state.m_subscriber || !state.m_subscriber->isValid())
            {
                state.m_subscriber.reset();
                return false;
            }
            state.m_device = db.inputs.cudaDeviceIndex();
            isaacsim::core::includes::ScopedDevice device(state.m_device);
            const auto result = cudaStreamCreateWithFlags(&state.m_stream, cudaStreamNonBlocking);
            if (result != cudaSuccess)
            {
                db.logError("Cannot create image consumer stream: %s", cudaGetErrorString(result));
                state.reset();
                return false;
            }
        }
        if (!state.m_subscriber->spin(state.m_message->getPtr()))
        {
            return true;
        }
        state.m_lease = state.m_gpuImage->acquireGpuRead(state.m_device, state.m_stream);
        if (!state.m_lease)
        {
            return false;
        }
        const auto& image = state.m_lease->getView();
        db.outputs.dataPtr() = reinterpret_cast<uint64_t>(image.data);
        db.outputs.cudaStream() = reinterpret_cast<uint64_t>(state.m_stream);
        db.outputs.cudaDeviceIndex() = image.device;
        db.outputs.bufferSize() = image.size;
        db.outputs.width() = image.width;
        db.outputs.height() = image.height;
        db.outputs.step() = image.step;
        db.outputs.encoding() = image.encoding;
        db.outputs.frameId() = image.frameId;
        db.outputs.timeStamp() = image.timeStamp;
        db.outputs.isBigEndian() = image.isBigEndian;
        db.outputs.promotedFromHost() = image.promotedFromHost;
        db.outputs.execOut() = kExecutionAttributeStateEnabled;
        return true;
    }

    static void releaseInstance(NodeObj const& node, GraphInstanceID instance)
    {
        OgnROS2SubscribeImageDatabase::sPerInstanceState<OgnROS2SubscribeImage>(node, instance).reset();
    }

    void reset() override
    {
        if (m_nodeObj.iNode)
        {
            GraphObj graph{ m_nodeObj.iNode->getGraph(m_nodeObj) };
            GraphContextObj context{ graph.iGraph->getDefaultGraphContext(graph) };
            for (const char* name : { "outputs:dataPtr", "outputs:cudaStream", "outputs:bufferSize" })
            {
                auto attribute = m_nodeObj.iNode->getAttribute(m_nodeObj, name);
                auto handle = attribute.iAttribute->getAttributeDataHandle(attribute, kAccordingToContextIndex);
                if (auto* value = getDataW<uint64_t>(context, handle))
                {
                    *value = 0;
                }
            }
        }
        _releaseResources();
    }

private:
    void _releaseResources()
    {
        m_lease.reset();
        if (m_stream)
        {
            isaacsim::core::includes::ScopedDevice device(m_device);
            (void)cudaStreamSynchronize(m_stream);
            (void)cudaStreamDestroy(m_stream);
            m_stream = nullptr;
        }
        m_subscriber.reset();
        m_gpuImage = nullptr;
        m_message.reset();
        Ros2Node::reset();
    }

    std::shared_ptr<Ros2Subscriber> m_subscriber;
    std::shared_ptr<Ros2ImageMessage> m_message;
    Ros2GpuImage* m_gpuImage = nullptr;
    std::shared_ptr<Ros2GpuImageLease> m_lease;
    cudaStream_t m_stream = nullptr;
    int m_device = -1;
};

REGISTER_OGN_NODE()
