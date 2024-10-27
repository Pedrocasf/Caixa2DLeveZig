const std = @import("std");
const Body = @import("Body.zig").Body;
const Joint = @import("Joint.zig").Joint;
const Arbiter = @import("Arbiter.zig").Arbiter;
const ArbiterKey = @import("Arbiter.zig").ArbiterKey;
const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const FixedBufferAllocator = std.heap.FixedBufferAllocator;
pub fn World(comptime T: type) type {
    return struct {
        const Self = @This();
        bodies: std.ArrayList(*Body(T)),
        joints: std.ArrayList(*Joint(T)),
        arbiters: std.AutoArrayHashMap(ArbiterKey(T), Arbiter(T)),
        gravity: Vec2(T),
        iterations: usize,
        pub var static = struct {
            pub var accumulateImpulses = false;
            pub var warmStarting = false;
            pub var positionCorrection = false;
        };
        pub fn init(a: std.mem.Allocator, gravityVec: Vec2(T), iter: usize) Self {
            return .{ .bodies = std.ArrayList(*Body(T)).init(a), .joints = std.ArrayList(*Joint(T)).init(a), .arbiters = std.AutoArrayHashMap(ArbiterKey(T), Arbiter(T)).init(a), .gravity = gravityVec, .iterations = iter };
        }
        pub fn AddBody(self: *Self, body: *Body(T)) void {
            self.bodies.appendAssumeCapacity(body);
        }
        pub fn AddJoint(self: *Self, joint: *Joint(T)) void {
            self.joints.appendAssumeCapacity(joint);
        }
        pub fn clear(self: *Self) void {
            self.bodies.clearAndFree();
            self.joints.clearAndFree();
            self.arbiters.clearAndFree();
        }
        pub fn BoardPhase(self: *Self) void {
            for (self.bodies) |bi| {
                for (self.bodies) |bj| {
                    if (bi.invMass == 0 & (bj.invMass == 0)) {
                        continue;
                    }
                    const newArb = Arbiter(T).init(bi, bj);
                    const key = ArbiterKey(T).init(bi, bj);
                    if (newArb.numContacts > 0) {
                        self.arbiters.put(key, newArb);
                    } else {
                        self.arbiters.remove(key);
                    }
                }
            }
        }
        pub fn Step(self: *Self, dt: T) void {
            var inv_dt: T = 0;
            if (dt > 0) {
                inv_dt = 1 / dt;
            }
            self.BoardPhase();
            for (self.bodies) |b| {
                if (b.invMass == 0) {
                    continue;
                }
                b.velocity.acc(Math.MultSV(T, dt, Math.AddV(T, self.gravity, Math.MultSV(T, b.invMass, b.force))));
                b.angularVelocity += dt * b.invI * b.torque;
            }
            for (self.arbiters.values()) |v| {
                v.PreStep(inv_dt);
            }
            for (self.joints) |j| {
                j.PreStep(inv_dt);
            }
            for (0..self.iterations) |_| {
                for (self.arbiters.values()) |v| {
                    v.ApplyImpulse();
                }
                for (self.joints) |j| {
                    j.ApplyImpulse();
                }
            }
            for (self.bodies) |b| {
                b.position.acc(Math.MultSV(T, dt, b.velocity));
                b.rotation += dt * b.angularVelocity;
                b.force.set(0, 0);
                b.torque = 0;
            }
        }
    };
}
