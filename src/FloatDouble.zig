pub fn NativeFloatDoubleWrapper(comptime T: type) type {
    return struct {
        const Self = @This();
        n: T,
        pub fn init(n: T) Self {
            return Self{ .n = n };
        }
        pub fn set(self: Self, n: T) void {
            self.n = n;
        }
        pub fn get(self: Self) T {
            return self.n;
        }
        pub fn addition(self: Self, o: Self) Self {
            return self.n + o;
        }
        pub fn subtraction(self: Self, o: T) Self {
            return self - o;
        }
    };
}
pub const Float = NativeFloatDoubleWrapper(f32);
pub const Double = NativeFloatDoubleWrapper(f64);
