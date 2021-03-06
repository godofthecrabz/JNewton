// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonUserContactPoint {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        MemoryLayout.sequenceLayout(4, Constants$root.C_FLOAT$LAYOUT).withName("m_point"),
        MemoryLayout.sequenceLayout(4, Constants$root.C_FLOAT$LAYOUT).withName("m_normal"),
        Constants$root.C_LONG_LONG$LAYOUT.withName("m_shapeId0"),
        Constants$root.C_LONG_LONG$LAYOUT.withName("m_shapeId1"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_penetration"),
        MemoryLayout.sequenceLayout(3, Constants$root.C_LONG$LAYOUT).withName("m_unused")
    ).withName("NewtonUserContactPoint");
    public static MemoryLayout $LAYOUT() {
        return NewtonUserContactPoint.$struct$LAYOUT;
    }
    public static MemorySegment m_point$slice(MemorySegment seg) {
        return seg.asSlice(0, 16);
    }
    public static MemorySegment m_normal$slice(MemorySegment seg) {
        return seg.asSlice(16, 16);
    }
    static final VarHandle m_shapeId0$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_shapeId0"));
    public static VarHandle m_shapeId0$VH() {
        return NewtonUserContactPoint.m_shapeId0$VH;
    }
    public static long m_shapeId0$get(MemorySegment seg) {
        return (long)NewtonUserContactPoint.m_shapeId0$VH.get(seg);
    }
    public static void m_shapeId0$set( MemorySegment seg, long x) {
        NewtonUserContactPoint.m_shapeId0$VH.set(seg, x);
    }
    public static long m_shapeId0$get(MemorySegment seg, long index) {
        return (long)NewtonUserContactPoint.m_shapeId0$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_shapeId0$set(MemorySegment seg, long index, long x) {
        NewtonUserContactPoint.m_shapeId0$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_shapeId1$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_shapeId1"));
    public static VarHandle m_shapeId1$VH() {
        return NewtonUserContactPoint.m_shapeId1$VH;
    }
    public static long m_shapeId1$get(MemorySegment seg) {
        return (long)NewtonUserContactPoint.m_shapeId1$VH.get(seg);
    }
    public static void m_shapeId1$set( MemorySegment seg, long x) {
        NewtonUserContactPoint.m_shapeId1$VH.set(seg, x);
    }
    public static long m_shapeId1$get(MemorySegment seg, long index) {
        return (long)NewtonUserContactPoint.m_shapeId1$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_shapeId1$set(MemorySegment seg, long index, long x) {
        NewtonUserContactPoint.m_shapeId1$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_penetration$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_penetration"));
    public static VarHandle m_penetration$VH() {
        return NewtonUserContactPoint.m_penetration$VH;
    }
    public static float m_penetration$get(MemorySegment seg) {
        return (float)NewtonUserContactPoint.m_penetration$VH.get(seg);
    }
    public static void m_penetration$set( MemorySegment seg, float x) {
        NewtonUserContactPoint.m_penetration$VH.set(seg, x);
    }
    public static float m_penetration$get(MemorySegment seg, long index) {
        return (float)NewtonUserContactPoint.m_penetration$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_penetration$set(MemorySegment seg, long index, float x) {
        NewtonUserContactPoint.m_penetration$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static MemorySegment m_unused$slice(MemorySegment seg) {
        return seg.asSlice(52, 12);
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


