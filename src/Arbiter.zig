const Body = @import("Body.zig").Body;
const Vec2 = @import("MathUtils.zig").Vec2;
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
            return Self{
                .body1 = body1,
                .body2 = body2,
            };
        }
    };
}
