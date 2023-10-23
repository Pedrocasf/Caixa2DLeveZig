pub export fn World(comptime T: type) type {
    _ = T;
    return struct {
        pub const static = struct {
            var accumulateImpulses = false;
            var warmStarting = false;
            var positionCorrection = false;
        };
    };
}
