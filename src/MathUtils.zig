pub const Vec2 = struct {
    const Self = @This();
    x: f32,
    y: f32,
    pub fn init(x: f32, y: f32) Self {
        return .{ .x = x, .y = y };
    }
    pub fn set(self: *Self, x: f32, y: f32) void {
        self.x = x;
        self.y = y;
    }
    pub fn neg(self: *Self) void {
        self.x = -Self.x;
        self.y = -Self.y;
    }
    pub fn acc(self: *Self, v: Vec2) void {
        self.x += v.x;
        self.y += v.y;
    }
    pub fn dec(self: *Self, v: Vec2) void {
        self.x -= v.x;
        self.y -= v.y;
    }
    pub fn mul(self: *Self, a: f32) void {
        self.x *= a;
        self.y *= a;
    }
    pub fn Lenght(self: *Self) f32 {
        return @sqrt((self.x * self.x) + (self.y * self.y));
    }
};

fn assert(ok: bool) void {
    if (!ok) unreachable; // assertion failure
}
pub const Mat22 = struct {
    const Self = @This();
    col1: Vec2,
    col2: Vec2,
    pub fn initAngle(angle: f32) Self {
        const c: f32 = @cos(angle);
        const s: f32 = @sin(angle);
        return .{
            .col1 = Vec2.init(c, s),
            .col2 = Vec2.init(-s, c),
        };
    }
    pub fn initV(col1: Vec2, col2: Vec2) Self {
        return .{
            .col1 = col1,
            .col2 = col2,
        };
    }
    pub fn transpose(self: *Self) Self {
        return Self.initV(Vec2.init(self.col1.x, self.col2.x), Vec2.init(self.col1.y, self.col2.y));
    }
    pub fn invert(self: *Self) Self {
        const a: f32 = self.col1.x;
        const b: f32 = self.col2.x;
        const c: f32 = self.col1.y;
        const d: f32 = self.col2.y;
        const det: f32 = (a * d) - (b * c);
        assert(det != 0);
        const idet: f32 = 1 / det;
        const B: Self = Self.initV(Vec2.init(idet * d, -idet * c), Vec2.init(-idet * b, idet * a));
        return B;
    }
};
pub fn DotV(a: Vec2, b: Vec2) f32 {
    return (a.x * b.x) + (a.y * b.y);
}
pub fn CrossV(a: Vec2, b: Vec2) f32 {
    return (a.x * b.y) - (a.y * b.x);
}
pub fn CrossVS(a: Vec2, s: f32) Vec2 {
    return Vec2.init(s * a.y, -s * a.x);
}
pub fn CrossSV(s: f32, a: Vec2) Vec2 {
    return Vec2.init(-s * a.y, s * a.x);
}
pub fn MultMV(A: Mat22, v: Vec2) Vec2 {
    return Vec2.init(A.col1.x * v.x + A.col2.x * v.y, A.col1.y * v.x + A.col2.y * v.y);
}
pub fn AddV(a: Vec2, b: Vec2) Vec2 {
    return Vec2.init(a.x + b.x, a.y + b.y);
}
pub fn SubV(a: Vec2, b: Vec2) Vec2 {
    return Vec2.init(a.x - b.x, a.y - b.y);
}
pub fn MultSV(s: f32, v: Vec2) Vec2 {
    return Vec2.init(s * v.x, s * v.y);
}
pub fn DivSV(s: f32, v: Vec2) Vec2 {
    return Vec2.init(v.x / s, v.y / s);
}
pub fn AddMM(A: Mat22, B: Mat22) Mat22 {
    return Mat22.initV(AddV(f32, A.col1, B.col1), AddV(f32, A.col2, B.col2));
}
pub fn MultMM(A: Mat22, B: Mat22) Mat22 {
    return Mat22.initV(MultMV(f32, A, B.col1), MultMV(f32, A, B.col2));
}
pub fn AbsV(a: Vec2) Vec2 {
    return Vec2.init(@abs(a.x), @abs(a.y));
}
pub fn AbsM(a: Mat22) Mat22 {
    return Mat22.initV(AbsV(f32, a.col1), AbsV(f32, a.col2));
}
pub fn Clamp(a: f32, low: f32, high: f32) f32 {
    return @max(low, @min(a, high));
}
pub fn Sign(x: f32) f32 {
    if (x < 0) {
        return -1;
    } else {
        return 1;
    }
}
pub fn Swap(a: *f32, b: *f32) void {
    const tmp: f32 = a;
    a = b;
    b = tmp;
}
