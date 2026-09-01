# Mission Control packet contract

[`packet.schema.json`](packet.schema.json) is the versioned source of truth for
the WebSocket protocol. It is based on Mission Control's TypeScript types and
README plus the packet handlers implemented in Resurgence.

Packet statuses mean:

- `active`: the current repositories agree on the packet shape.
- `legacy-compatible`: the bridge preserves a legacy packet that current
  Mission Control omits or exposes under a disjoint set of device names.
- `planned`: a known schema or behavior mismatch must be resolved before the
  packet is implemented in the new bridge.
- `unsupported`: intentionally outside this protocol version. No v1 packets
  currently have this status.

`x-bridge-support` marks every request and report routed by `mc_bridge`.
Commands use typed ROS messages, and reports use typed subscriptions rather
than JSON inside `std_msgs/String`.

Physical joint positions and servo positions use degrees. Typed ROS joint
position messages carry an explicit unit; legacy `ikForward` and `ikUp`
positions use meters. Camera stream data is one H.264 frame represented as
Annex-B NAL byte arrays; camera frame data is a base64 JPEG with its GPS and
orientation fields. The ROS boundary caps decoded JPEGs at 12 MiB, stream
frames at 4 MiB, and stream frames at 4096 NAL units.

Mission Control currently exposes only its ten arm joint names and the `mast`
servo. The bridge accepts the legacy science names but filters their reports so
the unchanged client cannot index missing UI state. Its current mast-servo UI
also has an upstream `range`/`limits` mismatch; bridge support is nevertheless
covered by protocol fixtures.
