// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonCollisionInfoRecord {
	
    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        MemoryLayout.sequenceLayout(4, MemoryLayout.sequenceLayout(4, Constants$root.C_FLOAT$LAYOUT)).withName("m_offsetMatrix"),
        MemoryLayout.structLayout(
            Constants$root.C_LONG_LONG$LAYOUT.withName("m_userId"),
            MemoryLayout.unionLayout(
                Constants$root.C_POINTER$LAYOUT.withName("m_ptr"),
                Constants$root.C_LONG_LONG$LAYOUT.withName("m_int"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_float")
            ).withName("m_userData"),
            MemoryLayout.sequenceLayout(6, MemoryLayout.unionLayout(
                Constants$root.C_POINTER$LAYOUT.withName("m_ptr"),
                Constants$root.C_LONG_LONG$LAYOUT.withName("m_int"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_float")
            )).withName("m_userParam")
        ).withName("m_collisionMaterial"),
        Constants$root.C_LONG$LAYOUT.withName("m_collisionType"),
        MemoryLayout.paddingLayout(32),
        MemoryLayout.unionLayout(
            MemoryLayout.structLayout(
                Constants$root.C_FLOAT$LAYOUT.withName("m_x"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_y"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_z")
            ).withName("m_box"),
            MemoryLayout.structLayout(
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_height")
            ).withName("m_cone"),
            MemoryLayout.structLayout(
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio")
            ).withName("m_sphere"),
            MemoryLayout.structLayout(
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio0"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio1"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_height")
            ).withName("m_capsule"),
            MemoryLayout.structLayout(
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio0"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio1"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_height")
            ).withName("m_cylinder"),
            MemoryLayout.structLayout(
                Constants$root.C_FLOAT$LAYOUT.withName("m_radio"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_height")
            ).withName("m_chamferCylinder"),
            MemoryLayout.structLayout(
                Constants$root.C_LONG$LAYOUT.withName("m_vertexCount"),
                Constants$root.C_LONG$LAYOUT.withName("m_vertexStrideInBytes"),
                Constants$root.C_LONG$LAYOUT.withName("m_faceCount"),
                MemoryLayout.paddingLayout(32),
                Constants$root.C_POINTER$LAYOUT.withName("m_vertex")
            ).withName("m_convexHull"),
            MemoryLayout.structLayout(
                Constants$root.C_LONG$LAYOUT.withName("m_vertexCount"),
                Constants$root.C_LONG$LAYOUT.withName("m_triangleCount"),
                Constants$root.C_LONG$LAYOUT.withName("m_vrtexStrideInBytes"),
                MemoryLayout.paddingLayout(32),
                Constants$root.C_POINTER$LAYOUT.withName("m_indexList"),
                Constants$root.C_POINTER$LAYOUT.withName("m_vertexList")
            ).withName("m_deformableMesh"),
            MemoryLayout.structLayout(
                Constants$root.C_LONG$LAYOUT.withName("m_chidrenCount")
            ).withName("m_compoundCollision"),
            MemoryLayout.structLayout(
                Constants$root.C_LONG$LAYOUT.withName("m_vertexCount"),
                Constants$root.C_LONG$LAYOUT.withName("m_indexCount")
            ).withName("m_collisionTree"),
            MemoryLayout.structLayout(
                Constants$root.C_LONG$LAYOUT.withName("m_width"),
                Constants$root.C_LONG$LAYOUT.withName("m_height"),
                Constants$root.C_LONG$LAYOUT.withName("m_gridsDiagonals"),
                Constants$root.C_LONG$LAYOUT.withName("m_elevationDataType"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_verticalScale"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_horizonalScale_x"),
                Constants$root.C_FLOAT$LAYOUT.withName("m_horizonalScale_z"),
                MemoryLayout.paddingLayout(32),
                Constants$root.C_POINTER$LAYOUT.withName("m_vertialElevation"),
                Constants$root.C_POINTER$LAYOUT.withName("m_atributes")
            ).withName("m_heightField"),
            MemoryLayout.structLayout(
                Constants$root.C_LONG$LAYOUT.withName("m_childrenProxyCount")
            ).withName("m_sceneCollision"),
            MemoryLayout.sequenceLayout(64, Constants$root.C_FLOAT$LAYOUT).withName("m_paramArray")
        ).withName("$anon$0")
    ).withName("NewtonCollisionInfoRecord");
    public static MemoryLayout $LAYOUT() {
        return NewtonCollisionInfoRecord.$struct$LAYOUT;
    }
    public static MemorySegment m_offsetMatrix$slice(MemorySegment seg) {
        return seg.asSlice(0, 64);
    }
    public static MemorySegment m_collisionMaterial$slice(MemorySegment seg) {
        return seg.asSlice(64, 64);
    }
    static final VarHandle m_collisionType$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_collisionType"));
    public static VarHandle m_collisionType$VH() {
        return NewtonCollisionInfoRecord.m_collisionType$VH;
    }
    public static int m_collisionType$get(MemorySegment seg) {
        return (int)NewtonCollisionInfoRecord.m_collisionType$VH.get(seg);
    }
    public static void m_collisionType$set( MemorySegment seg, int x) {
        NewtonCollisionInfoRecord.m_collisionType$VH.set(seg, x);
    }
    public static int m_collisionType$get(MemorySegment seg, long index) {
        return (int)NewtonCollisionInfoRecord.m_collisionType$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_collisionType$set(MemorySegment seg, long index, int x) {
        NewtonCollisionInfoRecord.m_collisionType$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static MemorySegment m_box$slice(MemorySegment seg) {
        return seg.asSlice(136, 12);
    }
    public static MemorySegment m_cone$slice(MemorySegment seg) {
        return seg.asSlice(136, 8);
    }
    public static MemorySegment m_sphere$slice(MemorySegment seg) {
        return seg.asSlice(136, 4);
    }
    public static MemorySegment m_capsule$slice(MemorySegment seg) {
        return seg.asSlice(136, 12);
    }
    public static MemorySegment m_cylinder$slice(MemorySegment seg) {
        return seg.asSlice(136, 12);
    }
    public static MemorySegment m_chamferCylinder$slice(MemorySegment seg) {
        return seg.asSlice(136, 8);
    }
    public static MemorySegment m_convexHull$slice(MemorySegment seg) {
        return seg.asSlice(136, 24);
    }
    public static MemorySegment m_deformableMesh$slice(MemorySegment seg) {
        return seg.asSlice(136, 32);
    }
    public static MemorySegment m_compoundCollision$slice(MemorySegment seg) {
        return seg.asSlice(136, 4);
    }
    public static MemorySegment m_collisionTree$slice(MemorySegment seg) {
        return seg.asSlice(136, 8);
    }
    public static MemorySegment m_heightField$slice(MemorySegment seg) {
        return seg.asSlice(136, 48);
    }
    public static MemorySegment m_sceneCollision$slice(MemorySegment seg) {
        return seg.asSlice(136, 4);
    }
    public static MemorySegment m_paramArray$slice(MemorySegment seg) {
        return seg.asSlice(136, 256);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(int len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment allocate(ResourceScope scope) { return allocate(SegmentAllocator.nativeAllocator(scope)); }
    public static MemorySegment allocateArray(int len, ResourceScope scope) {
        return allocateArray(len, SegmentAllocator.nativeAllocator(scope));
    }
    public static MemorySegment ofAddress(MemoryAddress addr, ResourceScope scope) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, scope); }
}


