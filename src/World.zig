const std = @import("std");
const Body = @import("Body.zig").Body;
const Joint = @import("Joint.zig").Joint;
const Arbiter = @import("Arbiter.zig").Arbiter;
const ArbiterKey = @import("Arbiter.zig").ArbiterKey;
const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const FixedBufferAllocator = std.heap.FixedBufferAllocator;
pub const World = struct {
    const Self = @This();
    bodies: std.ArrayList(*Body),
    joints: std.ArrayList(*Joint),
    arbiters: std.AutoArrayHashMap(ArbiterKey, Arbiter),
    gravity: Vec2,
    iterations: usize,
    pub var static = struct {
        pub var accumulateImpulses = false;
        pub var warmStarting = false;
        pub var positionCorrection = false;
    };
    pub fn init(a: std.mem.Allocator, gravityVec: Vec2, iter: usize) Self {
        return .{ .bodies = std.ArrayList(*Body).init(a), .joints = std.ArrayList(*Joint).init(a), .arbiters = std.AutoArrayHashMap(ArbiterKey, Arbiter).init(a), .gravity = gravityVec, .iterations = iter };
    }
    pub fn AddBody(self: *Self, body: *Body) void {
        self.bodies.appendAssumeCapacity(body);
    }
    pub fn AddJoint(self: *Self, joint: *Joint) void {
        self.joints.appendAssumeCapacity(joint);
    }
    pub fn clear(self: *Self) void {
        self.bodies.clearAndFree();
        self.joints.clearAndFree();
        self.arbiters.clearAndFree();
    }
    pub fn BoardPhase(self: *Self) void {
        for (self.bodies.items) |bi| {
            for (self.bodies.items) |bj| {
                if ((bi.invMass == 0) and (bj.invMass == 0)) {
                    continue;
                }
                const newArb = Arbiter.init(bi, bj);
                const key = ArbiterKey.init(bi, bj);
                if (newArb.numContacts > 0) {
                    self.arbiters.put(key, newArb);
                } else {
                    self.arbiters.remove(key);
                }
            }
        }
    }
    pub fn Step(self: *Self, dt: f32) void {
        var inv_dt: f32 = 0;
        if (dt > 0) {
            inv_dt = 1 / dt;
        }
        self.BoardPhase();
        for (self.bodies.items) |b| {
            if (b.invMass == 0) {
                continue;
            }
            b.velocity.acc(Math.MultSV(dt, Math.AddV(self.gravity, Math.MultSV(b.invMass, b.force))));
            b.angularVelocity += dt * b.invI * b.torque;
        }
        for (self.arbiters.values()) |v| {
            v.PreStep(inv_dt);
        }
        for (self.joints.items) |j| {
            j.PreStep(inv_dt);
        }
        for (0..self.iterations) |_| {
            for (self.arbiters.values()) |v| {
                v.ApplyImpulse();
            }
            for (self.joints.items) |j| {
                j.ApplyImpulse();
            }
        }
        for (self.bodies.items) |b| {
            b.position.acc(Math.MultSV(dt, b.velocity));
            b.rotation += dt * b.angularVelocity;
            b.force.set(0, 0);
            b.torque = 0;
        }
    }
};
