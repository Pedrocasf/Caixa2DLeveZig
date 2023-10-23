const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const Body = @import("Body.zig");
pub export fn Joint(comptime T: type) type {
    return struct {
        const Self = @This();
        M: Mat22(T),
        localAnchor1: Vec2(T),
        localAnchor2: Vec2(T),
        r1: Vec2(T),
        r2: Vec2(T),
        bias: Vec2(T),

        body1: Body(T),
        body2: Body(T),
    };
}
