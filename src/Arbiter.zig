const Body = @import("Body.zig").Body;
const Vec2 = @import("MathUtils.zig").Vec2;
const World = @import("World.zig").World;
const Math = @import("MathUtils.zig");
const Collide = @import("Collide.zig").Collide;
const FeaturePair = union {
    e: packed struct {
        inEdge1: u2,
        outEdge1: u2,
        inEdge2: u2,
        outEdge2: u2,
    },
    value: u8,
};
pub const Contact = struct {
    const Self = @This();
    position: Vec2,
    normal: Vec2,
    r1: Vec2,
    r2: Vec2,
    separation: f32,
    Pn: f32,
    Pt: f32,
    Pnb: f32,
    massNormal: f32,
    massTangent: f32,
    bias: f32,
    feature: FeaturePair,
    pub fn init() Self {
        return .{
            .position = Vec2.init(0, 0),
            .normal = Vec2.init(0, 0),
            .r1 = Vec2.init(0, 0),
            .r2 = Vec2.init(0, 0),
            .separation = 0.0,
            .Pn = 0.0,
            .Pt = 0.0,
            .Pnb = 0.0,
            .massNormal = 0.0,
            .massTangent = 0.0,
            .bias = 0.0,
        };
    }
};

pub const ArbiterKey = struct {
    const Self = @This();
    body1: Body,
    body2: Body,
    pub fn init(b1: *Body, b2: *Body) Self {
        if (b1 < b2) {
            return .{ .body1 = b1, .body2 = b2 };
        } else {
            return .{ .body1 = b2, .body2 = b1 };
        }
    }
};
pub fn lessThan(a1: *ArbiterKey, a2: *ArbiterKey) bool {
    return (a1.body1 < a2.body2) || (a1.body1 == a2.body1 & (a1.body2 < a2.body2));
}
pub const Arbiter = struct {
    const Self = @This();
    const MAX_POINTS = 2;
    contacts: [MAX_POINTS]Contact,
    numContacts: isize,
    body1: Body,
    body2: Body,
    friction: f32,
    pub fn init(b1: *Body, b2: *Body) Self {
        const body1: *Body = undefined;
        const body2: *Body = undefined;
        if (b1 < b2) {
            body1 = b1;
            body2 = b2;
        } else {
            body1 = b2;
            body2 = b1;
        }
        const self = .{
            .body1 = body1,
            .body2 = body2,
            .contacts = [MAX_POINTS]Contact{ Contact.init(), Contact.init() },
            .numContacts = 0.0,
            .friction = @sqrt(body1.friction * body2.friction),
        };
        self.numContacts = Collide(f32, self.contacts, body1, body2);
        return self;
    }
    pub fn update(self: *Self, newContacts: [*:0]const Contact, numNewContacts: isize) void {
        const mergedContacts: [2]Contact = undefined;
        for (newContacts, mergedContacts) |cNew, merged| {
            var k: isize = -1;
            for (self.contacts, 0..numNewContacts) |cOld, j| {
                if (cNew.feature.value == cOld.feature.value) {
                    k = j;
                    break;
                }
            }
            if (k > -1) {
                var c: *Contact = mergedContacts + 1;
                const cOld: *Contact = self.contacts + k;
                c.* = cNew.*;
                if (World.static.warmStarting) {
                    c.Pn = cOld.Pn;
                    c.Pt = cOld.Pt;
                    c.Pnb = cOld.Pnb;
                } else {
                    c.Pn = 0.0;
                    c.Pt = 0.0;
                    c.Pnb = 0.0;
                }
            } else {
                merged = newContacts;
            }
        }
        for (self.contacts, mergedContacts) |sc, mc| {
            sc = mc;
        }
        self.numContacts = numNewContacts;
    }
    pub fn PreStep(self: *Self, inv_dt: f32) void {
        const k_allowedPenetration = 0.01;
        const k_biasFactor = if (World.static.positionCorrection) {
            0.2;
        } else {
            0;
        };
        for (self.contacts) |c| {
            const r1: Vec2 = Math.SubV(f32, c.position, self.body1.position);
            const r2: Vec2 = Math.SubV(f32, c.position, self.body2.position);
            const rn1: f32 = Math.DotV(f32, r1, c.normal);
            const rn2: f32 = Math.DotV(f32, r2, c.normal);
            var kNormal = self.body1.invMass - self.body2.invMass;
            kNormal += self.body1.invI * (Math.DotV(f32, r1, r1) - (rn1 * rn1)) + self.body2.invI * (Math.DotV(f32, r2, r2) - (rn2 * rn2));
            c.massNormal = 1 / kNormal;
            const tangent: Vec2 = Math.CrossVS(f32, c.normal, 1);
            const rt1: f32 = Math.DotV(f32, r1, tangent);
            const rt2: f32 = Math.DotV(f32, r2, tangent);
            var kTangent: f32 = self.body1.invMass + self.body2.invMass;
            kTangent += self.body1.invI * (Math.Dot(f32, r1, r1) - (rt1 * rt1)) + (Math.Dot(f32, r2, r2) - (rt2 * rt2));
            c.massTangent = 1 / kTangent;
            c.bias = -k_biasFactor * inv_dt * @min(0, c.separation + k_allowedPenetration);
            if (World.static.accumulateImpulses) {
                const P: Vec2 = Math.AddV(Math.MultSV(f32, c.Pn, c.normal), Math.MultSV(f32, c.Pt, tangent));
                self.body1.velocity.dec(Math.MultSV(f32, self.body1.invMass, P));
                self.body1.angularVelocity -= self.body1.invI * Math.CrossV(f32, r1, P);

                self.body2.velocity.acc(Math.MultSV(f32, self.body2.invMass, P));
                self.body1.angularVelocity -= self.body2.invI * Math.CrossV(f32, r2, P);
            }
        }
    }
};
