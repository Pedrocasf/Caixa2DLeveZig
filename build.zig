const std = @import("std");

pub fn build(b: *std.Build) void {
    const target = b.standardTargetOptions(.{});

    const optimize = std.builtin.OptimizeMode.ReleaseSmall;

    const lib = b.addStaticLibrary(.{
        .name = "C2DLZ",
        .root_source_file = .{ .cwd_relative = "src/lib.zig" },
        .target = target,
        .optimize = optimize,
    });

    b.installArtifact(lib);
}
