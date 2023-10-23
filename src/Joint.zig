const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const Body = @import("Body.zig").Body;
const World = @import("World.zig").World;
pub export fn Joint(comptime T: type) type {
    return struct {
        const Self = @This();
        M: Mat22(T),
        localAnchor1: Vec2(T),
        localAnchor2: Vec2(T),
        r1: Vec2(T),
        r2: Vec2(T),
        bias: Vec2(T),
        P: Vec2(T),
        body1: Body(T),
        body2: Body(T),
        biasFactor: T,
        softness: T,
        pub fn init(b1: Body(T), b2: Body(T), anchor: Vec2(T)) Self {
            const Rot1 = Mat22(T).initAngle(b1.rotation);
            const Rot2 = Mat22(T).initAngle(b2.rotation);
            const Rot1T = Rot1.transpose();
            const Rot2T = Rot2.transpose();
            const localAnchor1 = Math.MultMV(T, Rot1T, Math.SubV(T, anchor, b1.position));
            const localAnchor2 = Math.MultMV(T, Rot2T, Math.SubV(T, anchor, b2.position));

            return Self{ .M = Mat22(T).initV(Vec2(T).init(0, 0), Vec2(T).init(0, 0)), .localAnchor1 = localAnchor1, .localAnchor2 = localAnchor2, .r1 = Vec2(T).init(0, 0), .r2 = Vec2(T).init(0, 0), .bias = Vec2(T).init(0, 0), .P = Vec2(T).init(0, 0), .body1 = b1, .body2 = b2, .biasFactor = 2, .softness = 0 };
        }
        pub fn preStep(self: Self, inv_dt: T) void {
            const Rot1 = Mat22(T).initAngle(self.body1.rotation);
            const Rot2 = Mat22(T).initAngle(self.body1.rotation);
            self.r1 = Math.MultMV(T, Rot1, self.localAnchor1);
            self.r2 = Math.MultMV(T, Rot2, self.localAnchor2);
            const K1 = Mat22(T).initV(Vec2(T).init(self.body1.invMass + self.body2.invMass, 0), Vec2(T).init(0, self.body1.invMass + self.body2.invMass));
            const K2 = Mat22(T).initV(Vec2(T).init(self.body1.invI * self.r1.y * self.r1.y, -self.body1.invI * self.r1.x * self.r1.y), Vec2(T).init(-self.body1.invI * self.r1.x * self.r1.y, self.body1.invI * self.r1.x * self.r1.x));
            const K3 = Mat22(T).initV(Vec2(T).init(self.body2.invI * self.r2.y * self.r2.y, -self.body2.invI * self.r2.x * self.r2.y), Vec2(T).init(-self.body2.invI * self.r2.x * self.r2.y, self.body2.invI * self.r2.x * self.r2.x));
            var K = Math.AddMM(T, K1, Math.AddMM(T, K2, K3));
            K.col1.x += self.softness;
            K.col1.y += self.softness;
            self.M = K.invert();
            const p1 = Math.AddV(T, self.body1.position, self.r1);
            const p2 = Math.AddV(T, self.body2.position, self.r2);
            const dp = Math.SubV(T, p2, p1);
            if (World(T).static.positionCorrection) {
                self.bias = Math.MultSV(-self.biasFactor * inv_dt, dp);
            } else {
                self.bias.set(0, 0);
            }
            if (World(T).static.warmStarting) {
                self.body1.velocity.dec(Math.MultSV(T, self.body1.invMass, self.P));
                self.body1.angularVelocity -= self.body1.invI * Math.CrossVS(T, self.r1, self.P);
                self.body2.velocity.acc(Math.MultSV(T, self.body2.invMass, self.P));
                self.body2.angularVelocity -= self.body2.invI * Math.CrossVS(T, self.r2, self.P);
            } else {
                self.P.set(0, 0);
            }
        }
        pub fn ApplyImpulse(self: Self) void {
            const dv = Math.SubV(Math.SubV(T, Math.AddV(T, self.body2.velocity, Math.CrossSV(T, self.body2.angularVelocity, self.r2)), self.body1.velocity), Math.CrossSV(T, self.body1.angularVelocity, self.r1));
            const impulse = Math.MultMV(T, self.M, Math.MultSV(T, self.P, Math.SubV(T, Math.SubV(T, self.bias, dv), self.softness)));
            self.body1.velocity.dec(Math.MultSV(T, self.invMass, impulse));
            self.body1.angularVelocity -= self.body1.invI * Math.CrossV(T, self.r1, impulse);
            self.body2.velocity.dec(Math.MultSV(T, self.invMass, impulse));
            self.body2.angularVelocity -= self.body2.invI * Math.CrossV(T, self.r2, impulse);
            self.P.acc(impulse);
        }
    };
}
