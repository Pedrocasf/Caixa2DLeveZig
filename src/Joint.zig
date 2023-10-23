const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const Body = @import("Body.zig");
pub export fn Joint(comptime T: type) type {
    return struct {
        const Self = @This();
        body1: Body(T),
        body2: Body(T),
    };
}
