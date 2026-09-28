// SPDX-FileCopyrightText: Copyright (c) 2026 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
// SPDX-License-Identifier: Apache-2.0

#include <cstddef>
#include <cstdint>
#include <cstdio>
#include <cuda.h>
#include <cuda_runtime.h>
#include <stdexcept>

void fillDevicePattern(uint8_t*, size_t, uint32_t, cudaStream_t);
void checkDevicePattern(const uint8_t*, size_t, uint32_t, unsigned int*, cudaStream_t);

namespace
{
class ImportedMemory
{
public:
    ~ImportedMemory()
    {
        if (mapped)
        {
            cuMemUnmap(address, size);
        }
        if (address)
        {
            cuMemAddressFree(address, size);
        }
        if (handle)
        {
            cuMemRelease(handle);
        }
    }

    CUmemGenericAllocationHandle handle{};
    CUdeviceptr address{};
    size_t size{};
    bool mapped{};
};

void checkDriver(CUresult result)
{
    if (result != CUDA_SUCCESS)
    {
        const char* message = nullptr;
        cuGetErrorString(result, &message);
        throw std::runtime_error(message ? message : "CUDA driver call failed");
    }
}
}

/** Test-only C entry points for the Python OmniGraph runtime harness. */
extern "C"
{
    int imageProbeFill(void* data, size_t size, uint32_t sequence, void* stream)
    {
        fillDevicePattern(static_cast<uint8_t*>(data), size, sequence, static_cast<cudaStream_t>(stream));
        return static_cast<int>(cudaGetLastError());
    }

    int imageProbeCheck(const void* data, size_t size, uint32_t sequence, void* errors, void* stream)
    {
        auto result = cudaMemsetAsync(errors, 0, sizeof(unsigned int), static_cast<cudaStream_t>(stream));
        if (result != cudaSuccess)
        {
            return static_cast<int>(result);
        }
        checkDevicePattern(static_cast<const uint8_t*>(data), size, sequence, static_cast<unsigned int*>(errors),
                           static_cast<cudaStream_t>(stream));
        return static_cast<int>(cudaGetLastError());
    }

    // Validate the legacy publisher's exported allocation in the same process.
    // The publisher owns fd; this test borrows it without closing it.
    int imageProbeCheckIpcFd(int fd, size_t size, uint32_t sequence, void* errors, void* stream)
    {
        try
        {
            ImportedMemory memory;
            checkDriver(cuMemImportFromShareableHandle(&memory.handle, reinterpret_cast<void*>(static_cast<intptr_t>(fd)),
                                                       CU_MEM_HANDLE_TYPE_POSIX_FILE_DESCRIPTOR));
            CUmemAllocationProp properties{};
            checkDriver(cuMemGetAllocationPropertiesFromHandle(&properties, memory.handle));
            size_t granularity = 0;
            checkDriver(cuMemGetAllocationGranularity(&granularity, &properties, CU_MEM_ALLOC_GRANULARITY_MINIMUM));
            memory.size = (size + granularity - 1) / granularity * granularity;
            checkDriver(cuMemAddressReserve(&memory.address, memory.size, 0, 0, 0));
            checkDriver(cuMemMap(memory.address, memory.size, 0, memory.handle, 0));
            memory.mapped = true;
            CUmemAccessDesc access{};
            access.location = properties.location;
            access.flags = CU_MEM_ACCESS_FLAGS_PROT_READ;
            checkDriver(cuMemSetAccess(memory.address, memory.size, &access, 1));
            auto result = static_cast<cudaError_t>(
                imageProbeCheck(reinterpret_cast<void*>(memory.address), size, sequence, errors, stream));
            unsigned int mismatches = 0;
            if (result == cudaSuccess)
            {
                result = cudaMemcpyAsync(
                    &mismatches, errors, sizeof(mismatches), cudaMemcpyDeviceToHost, static_cast<cudaStream_t>(stream));
            }
            // Complete reads before unmapping the imported allocation, including
            // the error path where an earlier kernel may already be in flight.
            const auto completion = cudaStreamSynchronize(static_cast<cudaStream_t>(stream));
            if (result == cudaSuccess)
            {
                result = completion;
            }
            if (result == cudaSuccess && mismatches)
            {
                std::fprintf(stderr, "NITROS frame %u: %u corrupted pixels\n", sequence, mismatches);
                result = cudaErrorUnknown;
            }
            return static_cast<int>(result);
        }
        catch (const std::exception& error)
        {
            std::fprintf(stderr, "NITROS allocation validation failed: %s\n", error.what());
            return static_cast<int>(cudaErrorUnknown);
        }
    }
}
