const std = @import("std");
const MathUtils = @import("MathUtils.zig");
const NumT = f32;
pub const Vec2 = MathUtils.Vec2(NumT);
pub const Mat22 = MathUtils.Mat22(NumT);
pub const Arbiter = @import("Arbiter.zig").Arbiter(NumT);
pub const ArbiterKey = @import("Arbiter.zig").ArbiterKey(NumT);
pub const Body = @import("Body.zig").Body(NumT);
pub const Joint = @import("Joint.zig").Joint(NumT);
pub const World = @import("World.zig").World(NumT);

const testing = std.testing;

export fn add(a: i32, b: i32) i32 {
    return a + b;
}

test "basic add functionality" {
    try testing.expect(add(3, 7) == 10);
}
