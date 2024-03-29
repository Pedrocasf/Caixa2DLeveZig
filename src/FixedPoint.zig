fn FixedPoint(comptime T: type, comptime BinaryScaling: comptime_int, comptime InitInt: type, comptime InitFloat: type) type {
    return struct {
        const FP = @This();
        raw: T,
        pub fn init(v: T) FP {
            return .{ .raw = v << BinaryScaling };
        }
        pub fn initFromFloat(v: InitFloat) FP {
            return .{ .raw = @floatCast(v * (1 << BinaryScaling)) };
        }
        pub fn unscale(fp: FP) InitInt {
            return fp.raw >> BinaryScaling;
        } 
    };
}
