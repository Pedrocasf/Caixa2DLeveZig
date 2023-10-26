const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const Contact = @import("Arbiter.zig").Contact;
const FeaturePair = @import("Arbiter.zig").FeaturePair;
const Body = @import("Body.zig").Body;
pub const Axis = enum { FACE_A_X, FACE_A_Y, FACE_B_X, FACE_B_Y };
pub const EdgeNumbers = enum { NO_EDGE, EDGE1, EDGE2, EDGE3, EDGE4 };
pub fn ClipVertex(comptime T: type) type {
    return struct {
        const Self = @This();
        v: Vec2(T),
        fp: FeaturePair,
        pub fn init() Self {
            return Self{ .v = Vec2(T).init(0, 0), .fp = 0 };
        }
    };
}

pub fn Flip(fp: *FeaturePair) void {
    Math.Swap(u8, fp.e.inEdge1, fp.e.inEdge2);
    Math.Swap(u8, fp.e.outEdge1, fp.e.outEdge2);
}
pub fn ClipSegmentToLine(
    comptime T: type,
    vOut: [2]ClipVertex(T),
    vIn: [2]ClipVertex(T),
    normal: Vec2(T),
    offset: T,
    clipEdge: u8,
) isize {
    var numOut: isize = 0;
    var distance0: T = Math.DotV(T, normal, vIn[0]) - offset;
    var distance1: T = Math.DotV(T, normal, vIn[1]) - offset;
    if (distance0 <= 0) {
        vOut[numOut] = vIn[0];
        numOut += 1;
    }
    if (distance1 <= 0) {
        vOut[numOut] = vIn[1];
        numOut += 1;
    }
    if (distance0 * distance1 < 0) {
        const interp: T = distance0 / (distance0 - distance1);
        vOut[numOut].v = Math.AddV(T, vIn[0], Math.MultSV(T, interp, Math.SubV(T, vIn[1].v, vIn[0].v)));
        if (distance0 > 0) {
            vOut[numOut].fp = vIn[0].fp;
            vOut[numOut].fp.e.inEdge1 = clipEdge;
            vOut[numOut].fp.e.inEdge2 = EdgeNumbers.NO_EDGE;
        } else {
            vOut[numOut].fp = vIn[1].fp;
            vOut[numOut].fp.e.outEdge1 = clipEdge;
            vOut[numOut].fp.e.outEdge2 = EdgeNumbers.NO_EDGE;
        }
        numOut += 1;
    }
    return numOut;
}
pub fn ComputeIncidentEdge(comptime T: type, c: [2]ClipVertex(T), h: *Vec2(T), pos: *Vec2(T), Rot: *Mat22(T), normal: *Vec2(T)) void {
    const RotT = Rot.transpose();
    var n = Math.MultMV(T, RotT, normal);
    n.neg();
    const nAbs = Math.AbsV(T, n);
    if (nAbs.x > nAbs.y) {
        if (Math.Sign(T, n.x) > 0) {
            c[0].v.set(h.x, -h.y);
            c[0].fp.e.inEdge2 = EdgeNumbers.EDGE3;
            c[0].fp.e.outEdge2 = EdgeNumbers.EDGE4;

            c[1].v.set(h.x, h.y);
            c[1].fp.e.inEdge2 = EdgeNumbers.EDGE4;
            c[1].fp.e.outEdge2 = EdgeNumbers.EDGE1;
        } else {
            c[0].v.set(-h.x, h.y);
            c[0].fp.e.inEdge2 = EdgeNumbers.EDGE1;
            c[0].fp.e.outEdge2 = EdgeNumbers.EDGE2;

            c[1].v.set(-h.x, -h.y);
            c[1].fp.e.inEdge2 = EdgeNumbers.EDGE2;
            c[1].fp.e.outEdge2 = EdgeNumbers.EDGE3;
        }
    } else {
        if (Math.Sign(T, n.y) > 0) {
            c[0].v.set(h.x, h.y);
            c[0].fp.e.inEdge2 = EdgeNumbers.EDGE4;
            c[0].fp.e.outEdge2 = EdgeNumbers.EDGE1;

            c[1].v.set(-h.x, h.y);
            c[1].fp.e.inEdge2 = EdgeNumbers.EDGE1;
            c[1].fp.e.outEdge2 = EdgeNumbers.EDGE2;
        } else {
            c[0].v.set(-h.x, -h.y);
            c[0].fp.e.inEdge2 = EdgeNumbers.EDGE2;
            c[0].fp.e.outEdge2 = EdgeNumbers.EDGE3;

            c[1].v.set(h.x, -h.y);
            c[1].fp.e.inEdge2 = EdgeNumbers.EDGE3;
            c[1].fp.e.outEdge2 = EdgeNumbers.EDGE4;
        }
    }
    c[0].v = Math.AddV(T, pos, Math.MultMV(T, Rot, c[0].v));
    c[1].v = Math.AddV(T, pos, Math.MultMV(T, Rot, c[1].v));
}
pub fn Collide(comptime T: type, contacts: [*:0]Contact(T), bodyA: *Body(T), bodyB: *Body(T)) isize {
    const hA = Math.DivSV(T, 2, bodyA.width);
    const hB = Math.DivSV(T, 2, bodyB.width);
    const posA = bodyA.position;
    const posB = bodyB.position;
    const RotA = Mat22(T).initAngle(bodyA.rotation);
    const RotB = Mat22(T).initAngle(bodyB.rotation);
    const RotAT = RotA.transpose();
    const RotBT = RotB.transpose();
    const dp = Math.SubV(T, posA, posB);
    const dA = Math.MultMV(T, RotAT, dp);
    const dB = Math.MultMV(T, RotBT, dp);
    const C = Math.MultMM(T, RotAT, RotB);
    const absC = Math.AbsM(T, C);
    const absCT = absC.transpose();
    const faceA = Math.SubV(T, Math.SubV(T, Math.AbsV(T, dA), hA), Math.MultMV(T, absC, hB));
    if (faceA.x > 0 || (faceA.y > 0)) {
        return 0;
    }
    const faceB = Math.SubV(T, Math.AbsV(T, dB), Math.MultMV(T, absCT, Math.SubV(T, hA, hB)));
    if (faceB.x > 0 || (faceB.y > 0)) {
        return 0;
    }
    var axis: Axis = Axis.FACE_A_X;
    var separation: T = faceA.x;
    var normal: Vec2(T) = RotA.col1;
    if (dA.x < 0) {
        normal.neg();
    }
    const relativeTol = 0.95;
    const absoluteTol = 0.01;
    if (faceA.y > (relativeTol * separation + absoluteTol * hA.y)) {
        axis = Axis.FACE_A_Y;
        separation = faceA.y;
        normal = RotA.col2;
        if (dA.y < 0) {
            normal.neg();
        }
    }
    if (faceB.x > (relativeTol * separation + absoluteTol * hB.x)) {
        axis = Axis.FACE_B_X;
        separation = faceB.x;
        normal = RotB.col1;
        if (dB.x < 0) {
            normal.neg();
        }
    }
    if (faceB.y > (relativeTol * separation + absoluteTol * hB.y)) {
        axis = Axis.FACE_B_Y;
        separation = faceB.y;
        normal = RotB.col2;
        if (dB.y < 0) {
            normal.neg();
        }
    }
    var frontNormal: Vec2(T) = undefined;
    var sideNormal: Vec2(T) = undefined;
    var incidentEdge: [2]ClipVertex(T) = undefined;
    var front: T = undefined;
    var negSide: T = undefined;
    var posSide: T = undefined;
    var negEdge: u8 = undefined;
    var posEdge: u8 = undefined;
    switch (axis) {
        Axis.FACE_A_X => {
            frontNormal = normal;
            front = Math.DotV(T, posA, frontNormal) + hA.x;
            sideNormal = RotA.col2;
            const side: T = Math.DotV(T, posA, sideNormal);
            negSide = -side + hA.y;
            posSide = side + hA.y;
            negEdge = EdgeNumbers.EDGE3;
            posEdge = EdgeNumbers.EDGE1;
            ComputeIncidentEdge(T, incidentEdge, hB, posB, RotB, frontNormal);
        },
        Axis.FACE_A_Y => {
            frontNormal = normal;
            front = Math.DotV(T, posA, frontNormal) + hA.x;
            sideNormal = RotA.col1;
            const side: T = Math.DotV(T, posA, sideNormal);
            negSide = -side + hA.x;
            posSide = side + hA.x;
            negEdge = EdgeNumbers.EDGE2;
            posEdge = EdgeNumbers.EDGE4;
            ComputeIncidentEdge(T, incidentEdge, hB, posB, RotB, frontNormal);
        },
        Axis.FACE_B_X => {
            frontNormal = -normal;
            front = Math.DotV(T, posB, frontNormal) + hB.x;
            sideNormal = RotB.col2;
            const side: T = Math.DotV(T, posB, sideNormal);
            negSide = -side + hB.y;
            posSide = side + hB.y;
            negEdge = EdgeNumbers.EDGE3;
            posEdge = EdgeNumbers.EDGE1;
            ComputeIncidentEdge(T, incidentEdge, hA, posA, RotA, frontNormal);
        },
        Axis.FACE_B_Y => {
            frontNormal = -normal;
            front = Math.DotV(T, posB, frontNormal) + hB.y;
            sideNormal = RotA.col1;
            const side: T = Math.DotV(T, posB, sideNormal);
            negSide = -side + hB.x;
            posSide = side + hB.x;
            negEdge = EdgeNumbers.EDGE2;
            posEdge = EdgeNumbers.EDGE4;
            ComputeIncidentEdge(T, incidentEdge, hA, posA, RotA, frontNormal);
        },
    }
    var clipPoints1: [2]ClipVertex(T) = undefined;
    var clipPoints2: [2]ClipVertex(T) = undefined;
    var np: isize = undefined;
    np = ClipSegmentToLine(T, clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);
    if (np < 2) {
        return 0;
    }
    np = ClipSegmentToLine(T, clipPoints2, clipPoints1, sideNormal, posSide, posEdge);
    if (np < 2) {
        return 0;
    }
    var numContacts: isize = 0;
    for (clipPoints2, contacts) |clipPoint2, contact| {
        const separationInner: T = Math.DotV(T, frontNormal, clipPoint2.v) - front;
        if (separation <= 0) {
            contact.separation = separationInner;
            contact.normal = normal;
            contact.position = Math.SubV(T, clipPoint2.v, Math.MultSV(T, separationInner, frontNormal));
            contact.feature = clipPoint2.fp;
            if (axis == Axis.FACE_B_X || (axis == Axis.FACE_B_Y)) {
                Flip(contact.feature);
            }
            numContacts += 1;
        }
    }
    return numContacts;
}
