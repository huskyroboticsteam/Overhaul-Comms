# Mission Control packet contract

[`packet.schema.json`](packet.schema.json) is the versioned source of truth for
the WebSocket protocol. It is based on Mission Control's TypeScript types and
README plus the packet handlers implemented in Resurgence.

Packet statuses mean:

- `active`: the current repositories agree on the packet shape.
- `legacy-compatible`: Resurgence implements the packet, but current Mission
  Control does not expose it.
- `planned`: a known schema or behavior mismatch must be resolved before the
  packet is implemented in the new bridge.
- `unsupported`: intentionally outside this protocol version. No v1 packets
  currently have this status.

Only drive, tank drive, and emergency stop are routed by `mc_bridge` in the
first vertical slice; `x-bridge-support` marks those definitions. Other valid
packets are recognized but not yet published to ROS.

Known follow-ups are joint-position units and behavior, the disjoint servo
names, and the camera-stream byte framing. New captured fixtures should be
added under `fixtures/` before changing those definitions.
