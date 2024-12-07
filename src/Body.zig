const std = @import("std");
const Math = @import("MathUtils.zig");
const maxFloat = std.math.floatMax;
const Vec2 = Math.Vec2;
pub const Body = struct {
    const Self = @This();
    position: Vec2,
    rotation: f32,
    velocity: Vec2,
    angularVelocity: f32,
    force: Vec2,
    torque: f32,
    width: Vec2,
    friction: f32,
    mass: f32,
    invMass: f32,
    I: f32,
    invI: f32,
    pub fn init() Self {
        return .{
            .position = Vec2.init(0, 0),
            .rotation = 0,
            .velocity = Vec2.init(0, 0),
            .angularVelocity = 0,
            .force = Vec2.init(0, 0),
            .torque = 0,
            .friction = 1,
            .width = Vec2.init(1, 1),
            .mass = maxFloat,
            .invMass = 0,
            .I = maxFloat,
            .invI = 0,
        };
    }
    pub fn addForce(self: *Self, f: Vec2) void {
        self.force.acc(f);
    }
    pub fn Set(self: *Self, w: Vec2, m: f32) void {
        self.position.set(0, 0);
        self.rotation = 0;
        self.velocity.set(0, 0);
        self.angularVelocity = 0;
        self.force.set(0, 0);
        self.torque = 0;
        self.friction = 1;
        self.width = w;
        self.mass = m;
        if (self.mass < maxFloat(f32)) {
            self.invMass = 1 / self.mass;
            self.I = self.mass * (self.width.x * self.width.x + self.width.y * self.width.y) / 12;
            self.invI = 1 / self.I;
        } else {
            self.invMass = 0;
            self.I = maxFloat(f32);
            self.invI = 0;
        }
    }
};
