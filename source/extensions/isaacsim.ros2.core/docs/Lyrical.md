# Lyrical image transport investigation

Status: experimental source implementation, built and tested in the real NVIDIA
Isaac Sim 6.1 container. Native linking, Lyrical OmniGraph image transport,
stop/restart, rendered camera output, and targeted Humble/Jazzy/NITROS regressions
passed. This is not a released bundled Lyrical distribution; the remaining
coverage limits are listed below. An additional full process-teardown check
reproduced a simulator crash even without ROS enabled; normal fast-exit test
runs passed.

## Source boundary (6.1.0-rc.26)

The authoritative source directories are `source/extensions/isaacsim.ros2.core`,
`source/extensions/isaacsim.ros2.nodes`, and `source/extensions/isaacsim.ros2.bridge`.
`exts/` is an installation layout, not this checkout's source layout.
The following describes the original bridge before the Lyrical changes.

* `core/plugins/isaacsim.ros2.core/PluginInterface.cpp` loads
  `isaacsim.ros2.core.<ROS_DISTRO>` and resolves `createFactoryC`.
  `core/include/isaacsim/ros2/core/Ros2FactoryImpl.hpp` exports the factory.
  Before this patch, a missing backend fell back to Jazzy, including for Lyrical.
* The factory, message, publisher, and subscriber interfaces are Isaac-owned.
  Backends in `core/library/backend` use `rcl`; there is no native `rclcpp`
  executor. `Ros2SubscriberImpl::spin` polls an `rcl_wait_set_t` then calls
  `rcl_take` into the pointer supplied by the caller.
* Image publication is camera/render-product data →
  `nodes/nodes/OgnROS2PublishImage.cpp::publishImage` / `publishImageHelper` →
  `Ros2ImageMessageImpl` (`sensor_msgs__msg__Image`) →
  `Ros2PublisherImpl::publish` → `rcl_publish`. GPU inputs are copied D2H into
  pinned host memory on the publisher's CUDA stream; that stream is synchronized
  before publication. The separate `nodes/library/ImagePublisher.cpp` helper
  implements the same CPU and GPU-staging behavior for its callers.
* CPU image reception uses the generic `OgnROS2Subscriber`, not an image-specific
  node: `rcl_take` → `Ros2SubscriberImpl::spin` → `Ros2DynamicMessageImpl`
  generated C storage/introspection → `OgnROS2Subscriber::subscriberCallback`
  and `readData(true)` → dynamic CPU output attributes. There is no
  `ROS2SubscribeImage` node in this revision.
* NITROS is an additional **publisher** on `<topic>/nitros_bridge`.
  `OgnROS2PublishImage::publishNitrosBridgeImage` obtains the device pointer from
  `IPCBufferManager::getCurBufferPtr`; `publishNitrosBridgeHelper` copies D2D
  (or copies from a CUDA array), synchronizes its stream, writes the process/FD
  descriptor through `Ros2NitrosBridgeImageMessageImpl::writeData`, and publishes
  the generated C `NitrosBridgeImage`. `IpcBufferManager.hpp` uses CUDA VMM
  allocation/export/map APIs. `Ros2SrtxImagePublisher.cpp` also publishes NITROS
  but its current input is host data and its copy is H2D.
* There is **no NITROS image subscriber, CUDA IPC import, or downstream image GPU
  consumer seam** in these extensions. An inbound convergence point cannot be
  reused because it does not exist here. The outbound convergence point is the
  source device pointer/array plus the publisher stream, before the old D2H copy.

## Compatibility constraints

`Ros2ImageMessage::getBufferPtr()` is a nonvirtual host/pinned-host accessor into
the base object's storage. `Ros2Message::getPtr()` is also nonvirtual. A C++
Image implementation cannot change either method's contract. Legacy vtables
must remain intact. Dynamic C messages should continue negotiating CPU only.

## Upstream findings

Sources inspected on 2026-09-27:

* [rosidl Lyrical](https://github.com/ros2/rosidl/tree/3983d007c1cfde197c63002635c7e45177939aec):
  the generator replaces **unbounded uint8 sequences**, not all primitive
  sequences, with `rosidl::Buffer`. Non-CPU `data()` access throws.
* [CUDA backend](https://github.com/ros2/rosidl_buffer_backends/tree/d7cd9642d77a1d64fd85f25ba0bf96e108401900):
  application APIs live in `cuda_buffer/cuda_buffer_api.hpp`.
  `allocate_buffer(size_t)` returns a buffer; `from_output_buffer(buffer, stream)`
  returns a write handle; `from_input_buffer(const buffer, stream)` returns a
  read handle; `to_buffer(src, bytes, writeHandle, stream, kind)` copies bytes.
  These APIs do not adopt an arbitrary external CUDA allocation.
* [RMW Lyrical](https://github.com/ros2/rmw/tree/2fd339ba950dc70c2f594e0c3fd69d8a1b217d63):
  `rmw_subscription_options_t::acceptable_buffer_backends` accepts `"cuda"`,
  `"any"`, or a comma-separated list. CPU is always implicit; null/empty means
  CPU-only. The documented RMW contract requires an error for an explicitly
  requested missing backend; the tested Fast DDS version instead fell back to
  CPU when its CUDA plugin was hidden, even for an explicit `"cuda"` request.
  At the `rcl` level use
  `rcl_subscription_options_set_acceptable_buffer_backends(value, &options)`.
  Assigning a string literal directly into the RMW field causes an invalid free
  during `rcl_subscription_fini`. This was reproduced and corrected in the
  standalone test. Lyrical `rcl_subscription_init` shallow-copies these options
  and its normal finalization owns that allocated string.
* [Isaac ROS 5.0 image pipeline](https://github.com/NVIDIA-ISAAC-ROS/isaac_ros_image_pipeline/tree/375311f26670455843a3b1f690a1ad54079c6be0):
  `cvcuda_conversions::detail::HandleOwner` carries a buffer handle into tensor
  cleanup; image callbacks keep the source message alive around CUDA submission.
  This is useful lifetime guidance, but its ROS node/executor design is not an
  Isaac Sim integration boundary.

`ReadHandle` references its creating `CudaBuffer`; the source message must
outlive it. Destruction records a read event on the supplied stream and the
buffer recycler waits for that work. A non-null stream is required for the
inspected producer-event wait. CPU promotion through `from_input_buffer` adds
H2D and must be reported as fallback, not zero-copy. Write handles must be
destroyed before publishing so their producer event has been recorded.

## Selected patch and verification gate

1. Build a standalone `rcl` + generated C++ Image publisher/subscriber, using
   `rosidl_typesupport_cpp::get_message_type_support_handle<Image>()`. Test CPU,
   CUDA, and mixed endpoints in separate processes before bridge integration.
2. Add an opt-in Lyrical backend built against a sourced Lyrical prefix. Keep the
   existing Humble/Jazzy dependency packages and defaults. Lyrical must never
   use the Jazzy fallback if its matching backend cannot load.
3. Select a separate Image C++ implementation only in that backend. Keep the
   other wrappers and dynamic-message representation unchanged.
4. The standalone transport test passed. Since the old ABI carries neither a
   GPU lease nor a consumer stream, `Ros2GpuImage.hpp` adds an independent optional
   interface discovered by RTTI. It retains every existing public class layout
   and vtable. The distro enum gains an appended value; the backend-private
   publisher gains a flag only in Lyrical builds. Existing host accessors are unchanged.
5. Publish with one D2D copy into upstream buffer-owned storage (Level A).
   Introduce a narrowly scoped typed receive path with explicit stream/lease
   lifetime; do not route CUDA storage through generic CPU attributes.
6. Validate both bridge directions, old distributions, restart/shutdown, and
   fallback before claiming support. Profile actual copies separately from
   payload checking. Source-level reasoning is not performance evidence.

## Environment findings

The provided `/home/cyc/workspaces/ros2/install` contains a generated Image with
`std::vector<uint8_t>` and no `rosidl_buffer` package. Its RMW header has no
`acceptable_buffer_backends`. It cannot establish Lyrical compatibility.
The initial standalone tests used an isolated official Lyrical container.
Subsequent integration tests used `nvcr.io/nvidia/isaac-sim:6.1.0`, whose installed
version was `6.1.0-rc.26+release.49347.2d230af4.gl`, with Kit
`110.3.0+feature.371399.00c488ae.gl`. The image digest was
`sha256:af1d2b4e75d553bfa27beb5a401198654aa8d607f3b7a6749196e9ce253def20`.
Tests ran on an RTX 4090 with host driver 580.82.07.

The matching source SDK was downloaded and a full native release build completed
in an isolated Ubuntu 24.04 environment with GCC 11, Python 3.12, and the SDK's
CUDA 12.8 toolkit. The Lyrical source installation contained 140 built packages;
its exact repository revisions are recorded in
`_lyrical_validation/container-ros-sources.lock.repos`. The CUDA backend uses the
upstream commit linked above. No host ROS/Python installation or the supplied ROS
workspace was changed.

The rebuilt core plugin, Lyrical backend, node plugin, and generated extension
files were installed into the runtime container. The packaged Humble/Jazzy
backend binaries were retained, and their SHA-256 hashes were unchanged before
and after installation. Runtime checks verified the selected backend was mapped
into each simulator process. The full build exposed a missing `m_nodeObj` member
in the new subscriber's reset path; that compile error was fixed before testing.

## Implementation details

`library/lyrical/Ros2LyricalImage.*` supplies only the Image specialization.
Its generated C++ type-support handle is paired with the C++ message for both
`rcl_publish` and `rcl_take`. The factory, publisher, and subscriber changes are
guarded by `ROS2_BACKEND_LYRICAL`. All other messages, including dynamic messages,
retain their generated C representation. NITROS source remains active for
Humble/Jazzy and is excluded from Lyrical's dependencies.

For GPU publication, `OgnROS2PublishImage` and `ImagePublisher` check the optional
image capability before their existing host-staging path. They pass the device
pointer and a non-null publisher stream to the backend. The backend allocates
upstream CUDA buffer storage, copies D2D, and synchronizes that stream before
returning because the renderer's source expires after graph evaluation. The
write handle records its completion event before publication. Standard Image
messages stay on the ordinary topic; no `/nitros_bridge` descriptor is used.
This is **Level A: one D2D copy plus CUDA IPC**, not source-buffer zero-copy.
Only linear buffers and `32FC1`/`eR32_SFLOAT` mipmapped textures are covered.

The legacy host path still writes host/pinned-host storage through
`getBufferPtr()`. Lyrical's publisher copies this storage into its C++ buffer
before publication. This adds one CPU copy for Lyrical host publication. GPU
preparation keeps the legacy host pointer/size pair separate. It never changes
that accessor to a device pointer. Old distro implementations are unchanged.

For reception, the new `ROS2SubscribeImage` node creates a typed Image subscriber.
Only this C++ Image type advertises `"any"`, and only with Fast DDS. C/introspection
subscriptions remain CPU-only. A received payload is moved into a lease that
owns both its message and read handle. CPU payloads are promoted H2D and set
`promotedFromHost=true`; this is explicitly a fallback path.

The node emits a **read-only** `dataPtr`, image metadata, and `cudaStream`.
Connect its `execOut` to a GPU consumer that enqueues its reads on that stream
before the next evaluation. The lease lasts until the next evaluation or reset;
do not save the pointer for later use or queue host work that submits CUDA reads
after the next evaluation. The read handle records the consumer event before
the source buffer is released. Reset releases the lease, synchronizes only the
consumer stream, destroys it, and clears the output pointer. C++ consumers can
retain the lease themselves for longer lifetimes, but must release it before
disabling the owning core extension. No `cudaDeviceSynchronize` was introduced.

The upstream allocation pool is process-wide and binds to its first device.
The wrapper checks device identity and rejects mismatches before using memory.
Multi-GPU acceleration is not supported by this initial integration.

## Building dependencies and the standalone tests

Use a **Lyrical** base installation with buffer-aware `rcl`, RMW, and generated
interfaces. The verified binary container was
`ros:lyrical-ros-base@sha256:0c19f326a339ed770ef1d4c0646a8b53bdb49dd5ff74b6de41ebdb8ac21e1806`.
Its relevant versions were `rcl 10.4.4`, `rmw_fastrtps_cpp 9.4.9`,
`rosidl_buffer 5.2.1`, and `sensor_msgs 5.9.3`. The CUDA backend source version
was `0.1.2` at the commit linked above. These initial standalone tests used
CUDA 13.2 and GCC 15. Both probe suites were subsequently rebuilt and passed
against the actual Kit SDK, CUDA 12.8, and the native ROS source installation.

For an existing native Lyrical installation, the commands below build an
isolated overlay; change the setup path for a source installation:

```bash
source /opt/ros/lyrical/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
export CUDA_HOME=/usr/local/cuda
export PATH="$CUDA_HOME/bin:$PATH"
mkdir -p /tmp/isaac-lyrical-ws/src
git clone https://github.com/ros2/rosidl_buffer_backends.git \
  /tmp/isaac-lyrical-ws/src/rosidl_buffer_backends
git -C /tmp/isaac-lyrical-ws/src/rosidl_buffer_backends checkout \
  d7cd9642d77a1d64fd85f25ba0bf96e108401900
cd /tmp/isaac-lyrical-ws
colcon build --merge-install --packages-up-to cuda_buffer_backend \
  --cmake-args -DBUILD_TESTING=OFF -DCUDAToolkit_ROOT="$CUDA_HOME" \
  -DCMAKE_BUILD_TYPE=RelWithDebInfo
source install/setup.bash
cd /path/to/IsaacSim6
cmake -S tools/ros2_lyrical -B _lyrical_probe \
  -DCMAKE_CUDA_COMPILER="$CUDA_HOME/bin/nvcc" -DCUDAToolkit_ROOT="$CUDA_HOME"
cmake --build _lyrical_probe -j4
ctest --test-dir _lyrical_probe --output-on-failure
```

Build prerequisites are CMake, a C++/CUDA compiler pair supported by the selected
toolkit, `colcon`, `ament_cmake`, `ament_cmake_auto`, `pluginlib`, `rcutils`, `rmw`,
`rcl`, `sensor_msgs`, `rosidl_buffer`, `rosidl_buffer_backend`,
`rosidl_buffer_backend_registry`, and generated C/C++ type support. The official
ROS base image already supplied the ROS prerequisites. The source overlay builds
`cuda_buffer_backend_msgs`, `cuda_buffer`, and the `cuda_buffer_backend` plugin.
Keep its `AMENT_PREFIX_PATH` sourced so pluginlib can find the plugin XML.

For a full ROS source build, follow the
[official Lyrical source installation](https://docs.ros.org/en/lyrical/Installation/Alternatives/Ubuntu-Development-Setup.html)
using the `lyrical` repository set, including `rmw_fastrtps_cpp`, then build the
overlay above. Do not mix the user's older `rolling-native-buffer` branches
with Lyrical binary interfaces.

## Building Isaac Sim

The backend must use the **same C++ standard-library ABI, CUDA runtime ABI,
compiler/platform baseline, and Python version where applicable as Kit**.
The initial Ubuntu 26.04 container test executables are not installable Isaac
plugins. Build Lyrical and its CUDA overlay from source on the selected Isaac
build baseline, as done for the runtime validation above. Do not copy the
standalone container's ROS binaries into an older host installation.

Install/build the usual bridge message dependencies as well: `ackermann_msgs`,
`vision_msgs`, `tf2_msgs`, `nav_msgs`, `geometry_msgs`, `std_msgs`, `rosgraph_msgs`,
`rcl_interfaces`, `action_msgs`, and `lifecycle_msgs`. No Lyrical NITROS package
is needed. With the project's normal SDK dependencies available:

```bash
source /path/to/kit-compatible-lyrical/install/setup.bash
source /path/to/kit-compatible-cuda-overlay/install/setup.bash
export ISAACSIM_ROS_LYRICAL_PREFIXES="$AMENT_PREFIX_PATH"
cd /path/to/IsaacSim6
./build.sh --release --jobs 4
```

Premake adds `isaacsim.ros2.core.lyrical` when that variable is present. Both
merged and isolated prefixes are accepted in overlay order. The resulting
library is staged in the core extension's `bin` alongside Humble/Jazzy. There
is no invented Packman Lyrical artifact or bundled Lyrical Python directory.
Without the variable, default builds remain Humble/Jazzy-only. The full native
release build passed, including the Lyrical backend link and generated OmniGraph
nodes. This revision already disables nested Docker in `repo.toml`; its build
command does not accept `--no-docker`.

Launch from the same sourced environment:

```bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
./_build/linux-x86_64/release/isaac-sim.sh
```

The existing launch script preserves an already-set `ROS_DISTRO` and library
paths. No launcher changes or Ubuntu-default changes are needed. Never add
Lyrical to the bundled default mapping until a matching packaged dependency
exists. For native Lyrical Python, use a build matching Kit's Python runtime;
the default Ubuntu 26.04 Python package is not presumed compatible with Kit.

## External test applications

`tools/ros2_lyrical/ImageProbe.cpp` builds as `_lyrical_probe/image_probe`. One
small application provides all four endpoint modes. Start the subscriber first
in another terminal with the same sourced environment:

```bash
# CPU publisher and CPU subscriber
_lyrical_probe/image_probe sub cpu /image_probe 30 cpu
_lyrical_probe/image_probe pub cpu /image_probe 30 1

# CUDA publisher and CUDA subscriber
_lyrical_probe/image_probe sub cuda /image_probe 30 cuda
_lyrical_probe/image_probe pub cuda /image_probe 30 1

# CPU publisher to a CUDA consumer: assert transport remains CPU, then promote
_lyrical_probe/image_probe sub cuda /image_probe 30 cpu
_lyrical_probe/image_probe pub cpu /image_probe 30 1

# CUDA publisher to a CPU-only subscriber: assert CPU fallback
_lyrical_probe/image_probe sub cpu /image_probe 30 cpu
_lyrical_probe/image_probe pub cuda /image_probe 30 1
```

The matrix runner also tests one CUDA and one CPU subscriber simultaneously.
Images alternate 640×480 and 1280×720, mono8, approximately 30 Hz. Every byte
depends on its pixel index and frame sequence. The CUDA validator returns only
a four-byte error count to the CPU. Missing, duplicate, reordered, or corrupt
frames cause a nonzero exit.

To test the actual bridge Image implementation independently of Kit, configure
the optional `ISAACSIM_SDK_INCLUDE_DIRS` CMake list with the real Carbonite,
Fabric, nlohmann, and USD include directories. This adds `image_bridge_probe`
and its bidirectional interoperability test. It compiles the production
`Ros2LyricalImage.cpp`, not a mock. The probe also checks the lease after
destroying its wrapper, then releases the lease before GPU completion.

```bash
cmake -S tools/ros2_lyrical -B _lyrical_probe \
  -DISAACSIM_SDK_INCLUDE_DIRS="/path/to/carb/include;/path/to/fabric/include;/path/to/nlohmann/include;/path/to/usd/include"
cmake --build _lyrical_probe -j4
ctest --test-dir _lyrical_probe --output-on-failure
ISAAC_IMAGE_PROBE_FRAMES=1000 python3 tools/ros2_lyrical/test_transport.py \
  _lyrical_probe/image_probe _lyrical_probe/image_bridge_probe --case 2
```

## Results and copy evidence

| Validation | Result |
|---|---|
| Standalone `rcl` C++ Image, five CPU/CUDA/mixed cases | Passed, 30 frames per publisher |
| Production bridge Image wrapper ↔ external `rcl`, same five cases both directions | Passed, ten combinations |
| CUDA buffer reuse, changing sizes, early wrapper/lease destruction | Passed, 1,000 frames each direction |
| CUDA plugin hidden from the ament index | Passed, CPU transport and H2D promotion in both directions, 30 frames each |
| All eleven production backend sources, C++17 syntax with Lyrical | Passed using cached SDK headers; two GCC 15 diagnostics in old USD/TBB headers downgraded for this check only |
| Python, OGN JSON, Premake Lua syntax | Passed |
| Native Ubuntu 24.04 / GCC 11 / Python 3.12 Lyrical dependency build | Passed, 140 packages; standalone five-case matrix also passed |
| Runtime harness GPU pattern helper | Passed on RTX 4090; valid pixels accepted and deliberately incorrect pixels rejected |
| Full native release build, including core backends and generated nodes | Passed against the actual Kit SDK, GCC 11, CUDA 12.8 |
| Real Isaac Sim Lyrical subscriber: CUDA transport and CPU promotion | Passed, 60 frames per case over two stop/play cycles |
| Real Isaac Sim Lyrical publisher: CPU/CUDA source × CPU/CUDA peer | Passed, 60 frames per case over two stop/play cycles |
| Packaged Humble/Jazzy backends with rebuilt core/nodes: CPU and GPU source publication | Passed, four cases, 60 frames each; external CPU Image receiver |
| Humble/Jazzy NITROS publication | Passed, 60 frames each; descriptor metadata and every GPU pixel checked by same-process VMM import |
| Lyrical camera integration | Passed, five distinct rendered 320×240 RGBA frames, every byte matched the CPU receiver |
| Real Isaac Sim CUDA transport profiling | Passed in both directions, 30 frames each; exact copy counts below |
| Full process exit with `fast_shutdown=False` | Failed in the packaged `simulation_manager` plugin, reproduced without ROS or `cuda_buffer` loaded |

Nsight Systems 2025.6.3 first recorded the standalone production-wrapper case
in both directions, 30 frames each, separate processes, same host/user and
RTX 4090:

| Copy kind | Calls | Total bytes | Interpretation |
|---|---:|---:|---|
| Device → Device | 30 | 18,432,000 | Exactly one payload copy per bridge publication |
| Device → Host | 60 | 240 | Exactly four validation bytes per received frame |
| Host → Device | 0 | 0 | No payload host staging in this test |

The trace also contains `cuMemImportFromShareableHandle`, VMM map/export calls,
and stream/event synchronization; ROS logs show CUDA descriptor serialization.
No `cudaDeviceSynchronize` occurred. The D2D GPU copy durations averaged about
1.35 microseconds in this run; these are copy durations, **not end-to-end latency**.
No CPU utilization, memory-bandwidth, multi-camera throughput, 1080p/60 Hz,
long-duration leak, CPU-baseline or NITROS-baseline performance claim is made.
Those require additional controlled workloads.

Separate profiles then captured the **actual Isaac Sim OmniGraph nodes** and
their external ROS peers. Capture starts after application warmup and ends after
the measured stop cycle. All recorded memcpy operations are counted, without
filtering out renderer copies or restricting to selected payload sizes:

| Real-runtime direction | D2D payload copies | D2H copies | H2D copies |
|---|---:|---:|---:|
| External CUDA publisher → Isaac `ROS2SubscribeImage` | 0 | 30 × 4-byte validation result | 0 |
| Isaac `ROS2PublishImage` → external CUDA subscriber | 30, totaling 18,432,000 bytes | 30 × 4-byte validation result | 0 |

Outbound D2D consists of fifteen 307,200-byte and fifteen 921,600-byte copies,
all in the simulator process. Inbound scalar D2H is in the simulator; outbound
scalar D2H is in the external validator. Both traces contain CUDA VMM imports
and exports and no `cudaDeviceSynchronize` or `cuCtxSynchronize`. Stream/event
synchronization remains present. Thus the measured inbound path has no payload
copy; publication is Level A with one D2D copy, and neither accelerated path
stages the payload through host memory. This evidence covers the synthetic
linear image workload, not every renderer or texture format.

Local evidence is saved under `_lyrical_validation/` (ignored build artifacts),
including `cuda-images.nsys-rep`, `cuda-images.sqlite`, `cuda-copy-stats.txt`,
`copy-verification.json`, the matrix/stress logs, and source compilation logs.
Real simulator JSON results, peer logs, and both raw and converted Nsight
captures are in `_lyrical_validation/runtime-results/`. The final native build
log is `_lyrical_validation/isaac61-source-rebuild.log`.
Early runtime JSON files labeled the application version `6.1.0` as
`kit_version`; the corrected profiling reports record Kit's actual build string
`110.3.0+feature.371399.00c488ae.gl` separately from `app_version`.
Reproduce profiling after building both probes:

```bash
nsys profile --trace=cuda,nvtx --sample=none --cpuctxsw=none \
  --trace-fork-before-exec=true -o cuda-images \
  python3 tools/ros2_lyrical/test_transport.py \
  _lyrical_probe/image_probe _lyrical_probe/image_bridge_probe --case 2
nsys stats --report cuda_gpu_mem_size_sum,cuda_gpu_mem_time_sum,cuda_api_sum \
  cuda-images.nsys-rep
python3 tools/ros2_lyrical/verify_trace.py cuda-images.sqlite
```

## Real Isaac Sim runtime harness

`tools/ros2_lyrical/test_runtime.py` creates an actual publisher or subscriber
OmniGraph node in `SimulationApp`, checks that the dedicated Lyrical backend is
loaded, and exchanges changing image sizes with a separate `image_probe`
process. It checks every payload byte, then stops and restarts the graph.
Subscriber stop checks reject stale output pointers and streams. All six
Lyrical combinations passed in the real 6.1 runtime: two inbound transport modes
and four outbound source/receiver combinations, totaling 360 checked frames.

After rebuilding and installing the modified core and nodes extensions into a
real 6.1 runtime, source the matching Lyrical installation before launching:

```bash
source /path/to/ros-lyrical/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
/path/to/isaacsim/python.sh tools/ros2_lyrical/test_runtime.py \
  --isaac-root /path/to/isaacsim \
  --peer "$PWD/_lyrical_probe/image_probe" \
  --pattern-library "$PWD/_lyrical_probe/libruntime_image_pattern.so" \
  --direction subscribe --peer-storage cuda \
  --frames 30 --cycles 2 --output _lyrical_validation/runtime-subscribe-cuda.json
```

Also run `--direction subscribe --peer-storage cpu`, and the four combinations
of `--direction publish --storage cpu|cuda --peer-storage cpu|cuda`, using a
distinct output filename for each run. Inspect the JSON `status` in addition to
the process exit code: Kit's default fast shutdown can otherwise mask a Python
exception. The harness explicitly forwards failure status to `SimulationApp.close`.

An extra `--full-shutdown` check disables Kit's default fast exit. The inbound
test passed 60 frames, two stop/play cycles, graph removal, and returned from
`SimulationApp.close`, then segfaulted during C++ process-exit destruction.
GDB located the fault in the packaged
`libisaacsim.core.simulation_manager.plugin.so`. A minimal `SimulationApp`
without any ROS graph reproduced the same crash location; its process maps
confirmed that neither `libisaacsim.ros2.core` nor `libcuda_buffer.so` was loaded.
This isolates the observed crash from the new transport, but full process exit
is still a failed acceptance check in this runtime. The Lyrical run also emitted
a pluginlib warning about live backend objects during unload, which remains
separate cleanup evidence and is not dismissed by the baseline reproduction.
Both debugger logs and the minimal reproducer are saved under
`_lyrical_validation/`. A JSON `shutdown: returned` means only that the close
call returned; require a successful process exit as well before calling complete
teardown passed.

The harness warms up the simulator before starting the external stream and
releases its final subscriber lease before waiting for the publisher to exit.
Starting a live publisher during the first renderer/physics initialization
caused stale upstream CUDA buffer identifiers when the consumer lagged behind;
this is not a claim of lossless delivery under arbitrary startup/backpressure.
The final lease must be released so the external CUDA allocation pool can finish
teardown. GPU validators submit their reads on the node's exposed stream.

For measured runtime copies, add `--profile --cycles 1` and wrap the command:

```bash
nsys profile --trace=cuda,nvtx --sample=none --cpuctxsw=none \
  --trace-fork-before-exec=true --capture-range=cudaProfilerApi \
  --capture-range-end=stop --export=sqlite -o profile-runtime-subscribe \
  /path/to/isaacsim/python.sh tools/ros2_lyrical/test_runtime.py \
  --isaac-root /path/to/isaacsim \
  --peer "$PWD/_lyrical_probe/image_probe" \
  --pattern-library "$PWD/_lyrical_probe/libruntime_image_pattern.so" \
  --direction subscribe --storage cuda --peer-storage cuda \
  --frames 30 --cycles 1 --profile --output profile-runtime-subscribe.json
python3 tools/ros2_lyrical/verify_trace.py profile-runtime-subscribe.sqlite \
  --runtime-direction subscribe
```

Repeat with `publish` and a distinct report path. The verifier checks every
memcpy, requires VMM import evidence, and rejects device-wide synchronization.
The container's CLI-only Nsight copy produced `.qdstrm` captures; the host's
matching 2025.6.3 `QdstrmImporter` converted these to `.nsys-rep`, followed by
`nsys export --type sqlite`.

For legacy publication, launch with the packaged Humble or Jazzy environment,
`--distro humble|jazzy --direction publish --peer-storage cpu`, and
`--peer-ros-prefix /path/to/ros-lyrical/install`. The peer runs its Lyrical
libraries in a separate process. Both CPU and GPU input passed for each legacy
distro. Add `--nitros --storage cuda` to check the extra NITROS publisher;
both distros passed. This observer imports the exported GPU FD in the simulator
process and checks every pixel. It does **not** test an external Isaac ROS
NITROS receiver's handle-transfer/acknowledgment protocol. NITROS uses fixed
640×480 images because its existing allocation pool is fixed at initialization.

`test_camera_runtime.py` separately renders a rotating cube with a CUDA camera
annotator and sends five changing RGBA frames through `ROS2PublishImage` to a
standard CPU ROS Image subscriber. Every received byte matched the reference.
The test takes a D2D snapshot of the rendered allocation to stabilize its
lifetime; reference extraction and the CPU receiver deliberately involve host
copies. This confirms rendered image correctness, while direct renderer-buffer
lifetime and a camera-to-GPU end-to-end profile remain separate coverage.

The isolated environments are the Docker containers `isaac61-lyrical-build`
and `isaac61-lyrical-runtime`, sharing source/dependency files in the
`isaac61-lyrical-workspace` volume. The runtime image contains the rebuilt
extensions; `/workspace/isaac61-runtime-env.sh` selects the native Lyrical
installation and Kit Python. Runtime library dependencies and Python packages
were installed only in those environments. Source manifests and build/test logs are in
`_lyrical_validation/`. The optional upstream `rclcpp` test dependencies were
excluded; the bridge and transport probe continue to use `rcl`.

## Remaining acceptance gates and limitations

* This is an experimental Linux source-build path, not a released bundled distro.
  No Windows packaging or CI image for Lyrical is provided.
* Full native build, graph wiring, and stop/restart passed in Kit 6.1. Broader
  renderer source ordering and downstream applications with independently
  scheduled GPU consumers still need coverage beyond these tests.
* Non-fast process teardown fails in this simulator build even with ROS disabled,
  as described above. The transport matrix uses the normal fast-exit setting;
  it does not establish clean static destruction or plugin hot unload.
* Fast DDS is the accelerated baseline. Other RMWs use CPU-only subscription
  negotiation and publication fallback; alternative RMW runtime tests remain.
* Same-host, same-user, same-device VMM/IPC was tested. Cross-host, permission
  failures, unsupported VMM, and publisher/subscriber crashes need separate
  failure tests. Legacy-to-Lyrical CPU image interoperability passed; this is
  not a general cross-distribution compatibility guarantee. Follow upstream
  negotiation/fallback semantics; do not infer cross-host acceleration.
  Hiding the CUDA plugin with `AMENT_PREFIX_PATH=/opt/ros/lyrical` while keeping
  its libraries loadable produced correct CPU fallback in both probe directions.
  A missing `libcuda_buffer.so` is different: the linked backend cannot load.
* Multi-GPU mismatches are rejected rather than treated as interchangeable device
  pointers. The upstream process-wide pool restricts allocation-device changes.
* Only typed Image is converted. CompressedImage, PointCloud2, tensors, arbitrary
  primitive buffers, and runtime dynamic messages remain future work.
* No source-allocation adoption is implemented. Publication uses one D2D copy;
  CPU fallback and Lyrical host publication include the copies described above.
* Existing class vtables and layouts are preserved; the GPU capability is a
  separate additive interface. Its pointers/leases are not serializable and
  cannot outlive the extension that implements them.
