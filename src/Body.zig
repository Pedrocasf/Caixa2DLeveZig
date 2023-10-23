const std = @import("std");
const maxInt = std.math.maxInt;
const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
pub export fn Body(comptime T: type) type {
    return struct {
        const Self = @This();
        position: Vec2(T),
        rotation: T,
        velocity: Vec2(T),
        angularVelocity: T,
        force: Vec2(T),
        torque: T,
        width: Vec2(T),
        friction: T,
        mass: T,
        invMass: T,
        I: T,
        invI: T,
        pub fn init() Self {
            Self{ .position = Vec2(T).init(0, 0), .rotation = 0, .velocity = Vec2(T).init(0, 0), .angularVelocity = 0, .force = Vec2(T).init(0, 0), .torque = 0, .friction = 1, .width = Vec2(T).init(1, 1), .mass = maxInt(T), .invMass = 0, .I = maxInt(T), .invI = 0 };
        }
        pub fn addForce(self: Self, f: Vec2(T)) void {
            self.force.acc(f);
        }
        pub fn Set(self: Self, w: Vec2(T), m: T) void {
            self.position.set(0, 0);
            self.rotation = 0;
            self.velocity.set(0, 0);
            self.angularVelocity = 0;
            self.force.set(0, 0);
            self.torque = 0;
            self.friction = 1;
            self.width = w;
            self.mass = m;
            if (self.mass < maxInt(T)) {
                self.invMass = 1 / self.mass;
                self.I = self.mass * (self.width.x * self.width.x + self.width.y * self.width.y) / 12;
                self.invI = 1 / self.I;
            } else {
                self.invMass = 0;
                self.I = maxInt(T);
                self.invI = 0;
            }
        }
    };
}
