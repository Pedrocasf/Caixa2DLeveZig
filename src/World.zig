const std = @import("std");
const Body = @import("Body.zig").Body;
const Joint = @import("Joint.zig").Joint;
const Arbiter = @import("Arbiter.zig").Arbiter;
const ArbiterKey = @import("Arbiter.zig").ArbiterKey;
pub export fn World(comptime T: type) type {
    return struct {
        const Self = @This();
        const bodies = std.ArrayList(*Body(T));
        const joints = std.ArrayHashMap(ArbiterKey(T), Arbiter(T), comptime Context: type, comptime max_load_percentage: u64)
        pub const static = struct {
            var accumulateImpulses = false;
            var warmStarting = false;
            var positionCorrection = false;
        };
    };
}
