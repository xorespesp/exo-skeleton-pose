# Pose protocol

Network protocol between the `exo-skeleton-pose` server (`serve` subcommand) and its clients. 
`exo_pose_proto.fbs` is the single source of truth; all bindings are generated from it with [FlatBuffers](https://flatbuffers.dev/) `flatc`.

## Transport

- WebSocket, binary frames (`ws://`, no TLS)
- Every frame is one `Message` whose `payload` union carries exactly one table.

## Message flow

| Direction | Messages |
|-----------|----------|
| Client -> Server | `OpenSource`, `CloseSource`, `CalibrateRestPose`, `ClearRestPose` |
| Server -> Client | `PoseFrame` (per frame, broadcast), `SourceStatus` (on change), `Ack` (per command) |

`OpenSource.source` is a plain string parsed by the server: 
an unsigned integer selects a live device index, anything else is a recording file path. 
Camera image data is intentionally not part of the protocol.

## Codegen

The C++ header is generated automatically during the CMake build. To generate
bindings for other languages from the same schema:

```
flatc --ts     -o <out> exo_pose_proto.fbs   # JavaScript / TypeScript (webview)
flatc --csharp -o <out> exo_pose_proto.fbs   # C#
```
