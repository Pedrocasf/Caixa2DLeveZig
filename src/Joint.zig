const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const Body = @import("Body.zig").Body;
const World = @import("World.zig").World;
pub const Joint = struct {
    const Self = @This();
    M: Mat22 = Mat22.initV(Vec2.init(1, 0), Vec2.init(0, 1)),
    localAnchor1: Vec2 = Vec2.init(1, 0),
    localAnchor2: Vec2 = Vec2.init(0, 1),
    r1: Vec2 = Vec2.init(1, 0),
    r2: Vec2 = Vec2.init(0, 1),
    bias: Vec2 = Vec2.init(0, 0),
    P: Vec2,
    body1: Body(),
    body2: Body(),
    biasFactor: f32,
    softness: f32,
    pub fn init() Self {
        return .{
            .body1 = Body().init(),
            .body2 = Body().init(),
            .p = Vec2.init(0, 0),
            .biasFactor = 0.2,
            .softness = 0,
        };
    }
    pub fn set(self: *Self, b1: Body, b2: Body, anchor: Vec2) void {
        self.body1 = b1;
        self.body2 = b2;
        const Rot1 = Mat22.initAngle(b1.rotation);
        const Rot2 = Mat22.initAngle(b2.rotation);
        const Rot1T = Rot1.transpose();
        const Rot2T = Rot2.transpose();
        self.localAnchor1 = Math.MultMV(f32, Rot1T, Math.SubV(f32, anchor, b1.position));
        self.localAnchor2 = Math.MultMV(f32, Rot2T, Math.SubV(f32, anchor, b2.position));
        self.P = Vec2.init(0, 0);
        self.softness = 0;
        self.biasFactor = 0.2;
    }
    pub fn preStep(self: *Self, inv_dt: f32) void {
        const Rot1 = Mat22.initAngle(self.body1.rotation);
        const Rot2 = Mat22.initAngle(self.body1.rotation);
        self.r1 = Math.MultMV(f32, Rot1, self.localAnchor1);
        self.r2 = Math.MultMV(f32, Rot2, self.localAnchor2);
        const K1 = Mat22.initV(Vec2.init(self.body1.invMass + self.body2.invMass, 0), Vec2.init(0, self.body1.invMass + self.body2.invMass));
        const K2 = Mat22.initV(Vec2.init(self.body1.invI * self.r1.y * self.r1.y, -self.body1.invI * self.r1.x * self.r1.y), Vec2.init(-self.body1.invI * self.r1.x * self.r1.y, self.body1.invI * self.r1.x * self.r1.x));
        const K3 = Mat22.initV(Vec2.init(self.body2.invI * self.r2.y * self.r2.y, -self.body2.invI * self.r2.x * self.r2.y), Vec2.init(-self.body2.invI * self.r2.x * self.r2.y, self.body2.invI * self.r2.x * self.r2.x));
        var K = Math.AddMM(f32, K1, Math.AddMM(f32, K2, K3));
        K.col1.x += self.softness;
        K.col1.y += self.softness;
        self.M = K.invert();
        const p1 = Math.AddV(f32, self.body1.position, self.r1);
        const p2 = Math.AddV(f32, self.body2.position, self.r2);
        const dp = Math.SubV(f32, p2, p1);
        if (World().static.positionCorrection) {
            self.bias = Math.MultSV(-self.biasFactor * inv_dt, dp);
        } else {
            self.bias.set(0, 0);
        }
        if (World().static.warmStarting) {
            self.body1.velocity.dec(Math.MultSV(f32, self.body1.invMass, self.P));
            self.body1.angularVelocity -= self.body1.invI * Math.CrossVS(f32, self.r1, self.P);
            self.body2.velocity.acc(Math.MultSV(f32, self.body2.invMass, self.P));
            self.body2.angularVelocity -= self.body2.invI * Math.CrossVS(f32, self.r2, self.P);
        } else {
            self.P.set(0, 0);
        }
    }
    pub fn ApplyImpulse(self: *Self) void {
        const dv = Math.SubV(Math.SubV(f32, Math.AddV(f32, self.body2.velocity, Math.CrossSV(f32, self.body2.angularVelocity, self.r2)), self.body1.velocity), Math.CrossSV(f32, self.body1.angularVelocity, self.r1));
        const impulse = Math.MultMV(f32, self.M, Math.MultSV(f32, self.P, Math.SubV(f32, Math.SubV(f32, self.bias, dv), self.softness)));
        self.body1.velocity.dec(Math.MultSV(f32, self.invMass, impulse));
        self.body1.angularVelocity -= self.body1.invI * Math.CrossV(f32, self.r1, impulse);
        self.body2.velocity.dec(Math.MultSV(f32, self.invMass, impulse));
        self.body2.angularVelocity -= self.body2.invI * Math.CrossV(f32, self.r2, impulse);
        self.P.acc(impulse);
    }
};
