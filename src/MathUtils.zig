pub fn Vec2(comptime T: type) type {
    return struct {
        const Self = @This();
        x: T,
        y: T,
        pub fn init(x: T, y: T) Self {
            return .{ .x = x, .y = y };
        }
        pub fn set(self: *Self, x: T, y: T) void {
            self.x = x;
            self.y = y;
        }
        pub fn neg(self: *Self) void {
            self.x = -Self.x;
            self.y = -Self.y;
        }
        pub fn acc(self: *Self, v: Vec2(T)) void {
            self.x += v.x;
            self.y += v.y;
        }
        pub fn dec(self: *Self, v: Vec2(T)) void {
            self.x -= v.x;
            self.y -= v.y;
        }
        pub fn mul(self: *Self, a: T) void {
            self.x *= a;
            self.y *= a;
        }
        pub fn Lenght(self: *Self) T {
            return @sqrt((self.x * self.x) + (self.y * self.y));
        }
    };
}
fn assert(ok: bool) void {
    if (!ok) unreachable; // assertion failure
}
pub fn Mat22(comptime T: type) type {
    return struct {
        const Self = @This();
        col1: Vec2(T),
        col2: Vec2(T),
        pub fn initAngle(angle: T) Self {
            const c: T = @cos(angle);
            const s: T = @sin(angle);
            return .{
                .col1 = Vec2(T).init(c, s),
                .col2 = Vec2(T).init(-s, c),
            };
        }
        pub fn initV(col1: Vec2(T), col2: Vec2(T)) Self {
            return .{
                .col1 = col1,
                .col2 = col2,
            };
        }
        pub fn transpose(self: *Self) Self {
            return Self.initV(Vec2(T).init(self.col1.x, self.col2.x), Vec2(T).init(self.col1.y, self.col2.y));
        }
        pub fn invert(self: *Self) Self {
            const a: T = self.col1.x;
            const b: T = self.col2.x;
            const c: T = self.col1.y;
            const d: T = self.col2.y;
            const det: T = (a * d) - (b * c);
            assert(det != 0);
            const idet: T = 1 / det;
            const B: Self = Self.initV(Vec2(T).init(idet * d, -idet * c), Vec2(T).init(-idet * b, idet * a));
            return B;
        }
    };
}
pub fn DotV(comptime T: type, a: Vec2(T), b: Vec2(T)) T {
    return (a.x * b.x) + (a.y * b.y);
}
pub fn CrossV(comptime T: type, a: Vec2(T), b: Vec2(T)) T {
    return (a.x * b.y) - (a.y * b.x);
}
pub fn CrossVS(comptime T: type, a: Vec2(T), s: T) Vec2(T) {
    return Vec2(T).init(s * a.y, -s * a.x);
}
pub fn CrossSV(comptime T: type, s: T, a: Vec2(T)) Vec2(T) {
    return Vec2(T).init(-s * a.y, s * a.x);
}
pub fn MultMV(comptime T: type, A: Mat22(T), v: Vec2(T)) Vec2(T) {
    return Vec2(T).init(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
}
pub fn AddV(comptime T: type, a: Vec2(T), b: Vec2(T)) Vec2(T) {
    return Vec2(T).init(a.x + b.x, a.y + b.y);
}
pub fn SubV(comptime T: type, a: Vec2(T), b: Vec2(T)) Vec2(T) {
    return Vec2(T).init(a.x - b.x, a.y - b.y);
}
pub fn MultSV(comptime T: type, s: T, v: Vec2(T)) Vec2(T) {
    return Vec2(T).init(s * v.x, s * v.y);
}
pub fn DivSV(comptime T: type, s: T, v: Vec2(T)) Vec2(T) {
    return Vec2(T).init(v.x / s, v.y / s);
}
pub fn AddMM(comptime T: type, A: Mat22(T), B: Mat22(T)) Mat22(T) {
    return Mat22(T).initV(AddV(T, A.col1, B.col1), AddV(T, A.col2, B.col2));
}
pub fn MultMM(comptime T: type, A: Mat22(T), B: Mat22(T)) Mat22(T) {
    return Mat22(T).initV(MultMV(T, A, B.col1), MultMV(T, A, B.col2));
}
pub fn AbsV(comptime T: type, a: Vec2(T)) Vec2(T) {
    return Vec2(T).init(@abs(a.x), @abs(a.y));
}
pub fn AbsM(comptime T: type, a: Mat22(T)) Mat22(T) {
    return Mat22(T).initV(AbsV(T, a.col1), AbsV(T, a.col2));
}
pub fn Clamp(comptime T: type, a: T, low: T, high: T) T {
    return @max(low, @min(a, high));
}
pub fn Sign(comptime T: type, x: T) T {
    if (x < 0) {
        return -1;
    } else {
        return 1;
    }
}
pub fn Swap(comptime T: type, a: *T, b: *T) void {
    const tmp: T = a;
    a = b;
    b = tmp;
}
