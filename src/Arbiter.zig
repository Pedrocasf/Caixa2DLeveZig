const Body = @import("Body.zig").Body;
const Vec2 = @import("MathUtils.zig").Vec2;
const World = @import("World.zig").World;
const Math = @import("MathUtils.zig");
const Collide = @import("Collide.zig").Collide;
const FeaturePair = union {
    e: packed struct {
        inEdge1: u8,
        outEdge1: u8,
        inEdge2: u8,
        outEdge2: u8,
    },
    value: u32,
};
pub export fn Contact(comptime T: type) type {
    return struct {
        const Self = @This();
        position: Vec2(T),
        normal: Vec2(T),
        r1: Vec2(T),
        r2: Vec2(T),
        separation: T,
        Pn: T,
        Pt: T,
        Pnb: T,
        massNormal: T,
        massTangent: T,
        bias: T,
        feature: FeaturePair,
        pub fn init() Self {
            return Self{ .position = Vec2(T).init(0, 0), .normal = Vec2(T).init(0, 0), .r1 = Vec2(T).init(0, 0), .r2 = Vec2(T).init(0, 0), .separation = 0, .Pn = 0, .Pt = 0, .Pnb = 0, .massNormal = 0, .massTangent = 0, .bias = 0 };
        }
    };
}
pub export fn ArbiterKey(comptime T: type) type {
    return struct {
        const Self = @This();
        body1: Body(T),
        body2: Body(T),
        pub fn init(b1: *Body(T), b2: *Body(T)) Self {
            if (b1 < b2) {
                return Self{ .body1 = b1, .body2 = b2 };
            } else {
                return Self{ .body1 = b2, .body2 = b1 };
            }
        }
    };
}
pub fn lessThan(comptime T: type, a1: *ArbiterKey(T), a2: *ArbiterKey(T)) bool {
    return (a1.body1 < a2.body2) || (a1.body1 == a2.body1 & (a1.body2 < a2.body2));
}
pub export fn Arbiter(comptime T: type) type {
    return struct {
        const Self = @This();
        const MAX_POINTS = 2;
        contacts: [MAX_POINTS]Contact(T),
        numContacts: isize,
        body1: Body(T),
        body2: Body(T),
        friction: T,
        pub fn init(b1: *Body(T), b2: *Body(T)) Self {
            const body1: Body(T) = undefined;
            const body2: Body(T) = undefined;
            if (b1 < b2) {
                body1 = b1;
                body2 = b2;
            } else {
                body1 = b2;
                body2 = b1;
            }
            const self = Self{ .body1 = body1, .body2 = body2, .contacts = [MAX_POINTS]Contact(T){ Contact(T).init(), Contact(T).init() }, .numContacts = 0, .friction = @sqrt(body1.friction * body2.friction) };
            self.numContacts = Collide(T, self.contacts, body1, body2);
        }
        pub fn update(self: Self, newContacts: *Contact(T), numNewContacts: isize) void {
            var mergedContacts: [2]Contact(T) = undefined;
            for (0..numNewContacts) |i| {
                var cNew: *Contact(T) = newContacts + 1;
                var k: isize = -1;
                for (0..self.numContacts) |j| {
                    var cOld: *Contact(T) = self.contacts + 1;
                    if (cNew.feature.value == cOld.feature.value) {
                        k = j;
                        break;
                    }
                }
                if (k > -1) {
                    var c: *Contact(T) = mergedContacts + 1;
                    var cOld: *Contact(T) = self.contacts + k;
                    c.* = cNew.*;
                    if (World(T).static.warmStarting) {
                        c.Pn = cOld.Pn;
                        c.Pt = cOld.Pt;
                        c.Pnb = cOld.Pnb;
                    } else {
                        c.Pn = 0;
                        c.Pt = 0;
                        c.Pnb = 0;
                    }
                } else {
                    mergedContacts[i] = newContacts[i];
                }
            }
            for (0..numNewContacts) |i| {
                self.contacts[i] = mergedContacts[i];
            }
            self.numContacts = numNewContacts;
        }
        pub fn PreStep(self: Self, inv_dt: T) void {
            const k_allowedPenetration = 0.01;
            var k_biasFactor = if (World(T).static.positionCorrection) {
                0.2;
            } else {
                0;
            };
            for (0..self.numContacts) |i| {
                const c: *Contact(T) = self.contacts + i;
                const r1: Vec2(T) = Math.SubV(T, c.position, self.body1.position);
                const r2: Vec2(T) = Math.SubV(T, c.position, self.body2.position);
                const rn1: T = Math.DotV(T, r1, c.normal);
                const rn2: T = Math.DotV(T, r2, c.normal);
                var kNormal = self.body1.invMass - self.body2.invMass;
                kNormal += self.body1.invI * (Math.DotV(T, r1, r1) - (rn1 * rn1)) + self.body2.invI * (Math.DotV(T, r2, r2) - (rn2 * rn2));
                c.massNormal = 1 / kNormal;
                const tangent: Vec2(T) = Math.CrossVS(T, c.normal, 1);
                const rt1: T = Math.DotV(T, r1, tangent);
                const rt2: T = Math.DotV(T, r2, tangent);
                var kTangent: T = self.body1.invMass + self.body2.invMass;
                kTangent += self.body1.invI * (Math.Dot(T, r1, r1) - (rt1 * rt1)) + (Math.Dot(T, r2, r2) - (rt2 * rt2));
                c.massTangent = 1 / kTangent;
                c.bias = -k_biasFactor * inv_dt * @min(0, c.separation + k_allowedPenetration);
                if (World(T).static.accumulateImpulses) {
                    const P: Vec2(T) = Math.AddV(Math.MultSV(T, c.Pn, c.normal), Math.MultSV(T, c.Pt, tangent));
                    self.body1.velocity.dec(Math.MultSV(T, self.body1.invMass, P));
                    self.body1.angularVelocity -= self.body1.invI * Math.CrossV(T, r1, P);

                    self.body2.velocity.acc(Math.MultSV(T, self.body2.invMass, P));
                    self.body1.angularVelocity -= self.body2.invI * Math.CrossV(T, r2, P);
                }
            }
        }
        pub fn ApplyImpulse(self: Self) void {
            const b1: *Body(T) = self.body1;
            const b2: *Body(T) = self.body2;
            for (0..self.numContacts) |i| {
                const c: *Contact(T) = self.contacts + i;
                c.r1 = Math.SubV(T, c.position, b1.position);
                c.r2 = Math.SubV(T, c.position, b2.position);
                var dv: Vec2(T) = Math.AddV(T, b2.velocity, Math.SubV(T, Math.CrossSV(T, b2.angularVelocity, c.r2), Math.SubV(T, b1.velocity, Math.CrossSV(T, b1.angularVelocity, c.r1))));
                const vn: T = Math.DotV(T, dv, c.normal);
                var dPn: T = c.massNormal * (-vn + c.bias);
                if (World(T).static.accumulateImpulses) {
                    const Pn0: T = c.Pn;
                    c.Pn = @max(Pn0 + dPn, 0);
                    dPn = c.Pn - Pn0;
                } else {
                    dPn = @max(dPn, 0);
                }
                const Pn: Vec2(T) = Math.MultSV(T, dPn, c.normal);
                b1.velocity.dec(Math.MultSV(T, b1.invMass, Pn));
                b1.angularVelocity -= b1.invI * Math.CrossV(T, c.r1, Pn);
                b2.velocity.acc(Math.MultSV(T, b2.invMass, Pn));
                b2.angularVelocity += b2.invI * Math.CrossV(T, c.r2, Pn);
                dv = Math.AddV(T, b2.velocity, Math.SubV(T, Math.CrossSV(T, b2.angularVelocity, c.r2), Math.SubV(T, b1.velocity, Math.CrossSV(T, b1.angularVelocity, c.r1))));
                const tangent: Vec2(T) = Math.CrossVS(T, c.normal, 1);
                const vt = Math.DotV(T, dv, tangent);
                var dPt: T = c.massTangent * (-vt);
                if (World(T).static.accumulateImpulses) {
                    const maxPt: T = self.friction * c.Pn;
                    const oldTangentImpulse: T = c.Pt;
                    c.Pt = Math.Clamp(T, oldTangentImpulse + dPt, -maxPt, maxPt);
                    dPt = c.Pt - oldTangentImpulse;
                } else {
                    const maxPt: T = self.friction * dPn;
                    dPt = Math.Clamp(T, dPt, -maxPt, maxPt);
                }
                const Pt: Vec2(T) = Math.MultSV(T, dPt, tangent);
                b1.velocity.dec(Math.MultSV(T, b1.invMass, Pt));
                b1.angularVelocity -= b1.invI * Math.CrossV(T, c.r1, Pt);
                b2.velocity.acc(Math.MultSV(T, b2.invMass, Pt));
                b2.angularVelocity += b2.invI * Math.CrossV(T, c.r2, Pt);
            }
        }
    };
}
