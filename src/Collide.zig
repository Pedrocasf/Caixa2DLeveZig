const Math = @import("MathUtils.zig");
const Vec2 = Math.Vec2;
const Mat22 = Math.Mat22;
const Contact = @import("Arbiter.zig").Contact;
const FeaturePair = @import("Arbiter.zig").FeaturePair;
const Body = @import("Body.zig").Body;
pub const Axis = enum { FACE_A_X, FACE_A_Y, FACE_B_X, FACE_B_Y };
pub const EdgeNumbers = enum { NO_EDGE, EDGE1, EDGE2, EDGE3, EDGE4 };
pub const ClipVertex = struct {
    const Self = @This();
    v: Vec2,
    fp: FeaturePair,
    pub fn init() Self {
        return .{
            .v = Vec2.init(0, 0),
            .fp = 0,
        };
    }
};

pub fn Flip(fp: *FeaturePair) void {
    Math.Swap(u8, fp.e.inEdge1, fp.e.inEdge2);
    Math.Swap(u8, fp.e.outEdge1, fp.e.outEdge2);
}
pub fn ClipSegmentToLine(
    vOut: [2]ClipVertex,
    vIn: [2]ClipVertex,
    normal: Vec2,
    offset: f32,
    clipEdge: u8,
) isize {
    var numOut: isize = 0;
    const distance0: f32 = Math.DotV(f32, normal, vIn[0]) - offset;
    const distance1: f32 = Math.DotV(f32, normal, vIn[1]) - offset;
    if (distance0 <= 0) {
        vOut[numOut] = vIn[0];
        numOut += 1;
    }
    if (distance1 <= 0) {
        vOut[numOut] = vIn[1];
        numOut += 1;
    }
    if (distance0 * distance1 < 0) {
        const interp: f32 = distance0 / (distance0 - distance1);
        vOut[numOut].v = Math.AddV(f32, vIn[0], Math.MultSV(f32, interp, Math.SubV(f32, vIn[1].v, vIn[0].v)));
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
pub fn ComputeIncidentEdge(c: [2]ClipVertex, h: *Vec2, pos: *Vec2, Rot: *Mat22, normal: *Vec2) void {
    const RotT = Rot.transpose();
    var n = Math.MultMV(f32, RotT, normal);
    n.neg();
    const nAbs = Math.AbsV(f32, n);
    if (nAbs.x > nAbs.y) {
        if (Math.Sign(f32, n.x) > 0) {
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
        if (Math.Sign(f32, n.y) > 0) {
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
    c[0].v = Math.AddV(f32, pos, Math.MultMV(f32, Rot, c[0].v));
    c[1].v = Math.AddV(f32, pos, Math.MultMV(f32, Rot, c[1].v));
}
pub fn Collide(comptime T: type, contacts: [*:0]Contact, bodyA: *Body, bodyB: *Body(T)) isize {
    const hA = Math.DivSV(f32, 2, bodyA.width);
    const hB = Math.DivSV(f32, 2, bodyB.width);
    const posA = bodyA.position;
    const posB = bodyB.position;
    const RotA = Mat22.initAngle(bodyA.rotation);
    const RotB = Mat22.initAngle(bodyB.rotation);
    const RotAT = RotA.transpose();
    const RotBT = RotB.transpose();
    const dp = Math.SubV(f32, posA, posB);
    const dA = Math.MultMV(f32, RotAT, dp);
    const dB = Math.MultMV(f32, RotBT, dp);
    const C = Math.MultMM(f32, RotAT, RotB);
    const absC = Math.AbsM(f32, C);
    const absCT = absC.transpose();
    const faceA = Math.SubV(f32, Math.SubV(f32, Math.AbsV(f32, dA), hA), Math.MultMV(f32, absC, hB));
    if (faceA.x > 0 || (faceA.y > 0)) {
        return 0;
    }
    const faceB = Math.SubV(f32, Math.AbsV(f32, dB), Math.MultMV(f32, absCT, Math.SubV(f32, hA, hB)));
    if (faceB.x > 0 || (faceB.y > 0)) {
        return 0;
    }
    var axis: Axis = Axis.FACE_A_X;
    var separation: f32 = faceA.x;
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
    const incidentEdge: [2]ClipVertex(T) = undefined;
    var front: f32 = undefined;
    var negSide: f32 = undefined;
    var posSide: f32 = undefined;
    var negEdge: u8 = undefined;
    var posEdge: u8 = undefined;
    switch (axis) {
        Axis.FACE_A_X => {
            frontNormal = normal;
            front = Math.DotV(f32, posA, frontNormal) + hA.x;
            sideNormal = RotA.col2;
            const side: f32 = Math.DotV(f32, posA, sideNormal);
            negSide = -side + hA.y;
            posSide = side + hA.y;
            negEdge = EdgeNumbers.EDGE3;
            posEdge = EdgeNumbers.EDGE1;
            ComputeIncidentEdge(f32, incidentEdge, hB, posB, RotB, frontNormal);
        },
        Axis.FACE_A_Y => {
            frontNormal = normal;
            front = Math.DotV(f32, posA, frontNormal) + hA.x;
            sideNormal = RotA.col1;
            const side: f32 = Math.DotV(f32, posA, sideNormal);
            negSide = -side + hA.x;
            posSide = side + hA.x;
            negEdge = EdgeNumbers.EDGE2;
            posEdge = EdgeNumbers.EDGE4;
            ComputeIncidentEdge(f32, incidentEdge, hB, posB, RotB, frontNormal);
        },
        Axis.FACE_B_X => {
            frontNormal = -normal;
            front = Math.DotV(f32, posB, frontNormal) + hB.x;
            sideNormal = RotB.col2;
            const side: f32 = Math.DotV(f32, posB, sideNormal);
            negSide = -side + hB.y;
            posSide = side + hB.y;
            negEdge = EdgeNumbers.EDGE3;
            posEdge = EdgeNumbers.EDGE1;
            ComputeIncidentEdge(f32, incidentEdge, hA, posA, RotA, frontNormal);
        },
        Axis.FACE_B_Y => {
            frontNormal = -normal;
            front = Math.DotV(f32, posB, frontNormal) + hB.y;
            sideNormal = RotA.col1;
            const side: f32 = Math.DotV(f32, posB, sideNormal);
            negSide = -side + hB.x;
            posSide = side + hB.x;
            negEdge = EdgeNumbers.EDGE2;
            posEdge = EdgeNumbers.EDGE4;
            ComputeIncidentEdge(f32, incidentEdge, hA, posA, RotA, frontNormal);
        },
    }
    const clipPoints1: [2]ClipVertex(T) = undefined;
    const clipPoints2: [2]ClipVertex(T) = undefined;
    var np: isize = undefined;
    np = ClipSegmentToLine(f32, clipPoints1, incidentEdge, -sideNormal, negSide, negEdge);
    if (np < 2) {
        return 0;
    }
    np = ClipSegmentToLine(f32, clipPoints2, clipPoints1, sideNormal, posSide, posEdge);
    if (np < 2) {
        return 0;
    }
    var numContacts: isize = 0;
    for (clipPoints2, contacts) |clipPoint2, contact| {
        const separationInner: f32 = Math.DotV(f32, frontNormal, clipPoint2.v) - front;
        if (separation <= 0) {
            contact.separation = separationInner;
            contact.normal = normal;
            contact.position = Math.SubV(f32, clipPoint2.v, Math.MultSV(f32, separationInner, frontNormal));
            contact.feature = clipPoint2.fp;
            if (axis == Axis.FACE_B_X || (axis == Axis.FACE_B_Y)) {
                Flip(contact.feature);
            }
            numContacts += 1;
        }
    }
    return numContacts;
}
