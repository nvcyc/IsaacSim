// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <cstdint>
#include <cuda_runtime.h>

namespace
{
__global__ void fillPattern(uint8_t* data, size_t size, uint32_t sequence)
{
    for (size_t index = blockIdx.x * blockDim.x + threadIdx.x; index < size; index += gridDim.x * blockDim.x)
    {
        data[index] = static_cast<uint8_t>(index * 13 + sequence * 37);
    }
}

__global__ void checkPattern(const uint8_t* data, size_t size, uint32_t sequence, unsigned int* errors)
{
    for (size_t index = blockIdx.x * blockDim.x + threadIdx.x; index < size; index += gridDim.x * blockDim.x)
    {
        if (data[index] != static_cast<uint8_t>(index * 13 + sequence * 37))
        {
            atomicAdd(errors, 1u);
        }
    }
}
}

void fillDevicePattern(uint8_t* data, size_t size, uint32_t sequence, cudaStream_t stream)
{
    fillPattern<<<128, 256, 0, stream>>>(data, size, sequence);
}

void checkDevicePattern(const uint8_t* data, size_t size, uint32_t sequence, unsigned int* errors, cudaStream_t stream)
{
    checkPattern<<<128, 256, 0, stream>>>(data, size, sequence, errors);
}
