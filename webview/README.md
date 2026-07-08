# webview

three.js demo that drives an Xbot rig from the `exo-skeleton-pose` WebSocket protocol.   
(A test client for the protocol in `../proto/exo_pose_proto.fbs`)

## Setup

```
$ npm install
$ npm run dev
```

## Use

1. Start the server: `exo-skeleton-pose serve --port 9002`.
2. The page auto-connects to `ws://localhost:9002` (status shown in the panel).
3. In `Source`, enter a device index (e.g. `0`) or a recording path, then `Open`.
4. Detected joints rotate the rig live. Use `Calibrate` in a neutral stance so the
   rig zeroes out to that pose, then move.
