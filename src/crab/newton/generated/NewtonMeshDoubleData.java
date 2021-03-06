// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class NewtonMeshDoubleData {

    static final  GroupLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_POINTER$LAYOUT.withName("m_data"),
        Constants$root.C_POINTER$LAYOUT.withName("m_indexList"),
        Constants$root.C_LONG$LAYOUT.withName("m_strideInBytes"),
        MemoryLayout.paddingLayout(32)
    ).withName("NewtonMeshDoubleData");
    public static MemoryLayout $LAYOUT() {
        return NewtonMeshDoubleData.$struct$LAYOUT;
    }
    static final VarHandle m_data$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_data"));
    public static VarHandle m_data$VH() {
        return NewtonMeshDoubleData.m_data$VH;
    }
    public static MemoryAddress m_data$get(MemorySegment seg) {
        return (jdk.incubator.foreign.MemoryAddress)NewtonMeshDoubleData.m_data$VH.get(seg);
    }
    public static void m_data$set( MemorySegment seg, MemoryAddress x) {
        NewtonMeshDoubleData.m_data$VH.set(seg, x);
    }
    public static MemoryAddress m_data$get(MemorySegment seg, long index) {
        return (jdk.incubator.foreign.MemoryAddress)NewtonMeshDoubleData.m_data$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_data$set(MemorySegment seg, long index, MemoryAddress x) {
        NewtonMeshDoubleData.m_data$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_indexList$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_indexList"));
    public static VarHandle m_indexList$VH() {
        return NewtonMeshDoubleData.m_indexList$VH;
    }
    public static MemoryAddress m_indexList$get(MemorySegment seg) {
        return (jdk.incubator.foreign.MemoryAddress)NewtonMeshDoubleData.m_indexList$VH.get(seg);
    }
    public static void m_indexList$set( MemorySegment seg, MemoryAddress x) {
        NewtonMeshDoubleData.m_indexList$VH.set(seg, x);
    }
    public static MemoryAddress m_indexList$get(MemorySegment seg, long index) {
        return (jdk.incubator.foreign.MemoryAddress)NewtonMeshDoubleData.m_indexList$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_indexList$set(MemorySegment seg, long index, MemoryAddress x) {
        NewtonMeshDoubleData.m_indexList$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_strideInBytes$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_strideInBytes"));
    public static VarHandle m_strideInBytes$VH() {
        return NewtonMeshDoubleData.m_strideInBytes$VH;
    }
    public static int m_strideInBytes$get(MemorySegment seg) {
        return (int)NewtonMeshDoubleData.m_strideInBytes$VH.get(seg);
    }
    public static void m_strideInBytes$set( MemorySegment seg, int x) {
        NewtonMeshDoubleData.m_strideInBytes$VH.set(seg, x);
    }
    public static int m_strideInBytes$get(MemorySegment seg, long index) {
        return (int)NewtonMeshDoubleData.m_strideInBytes$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_strideInBytes$set(MemorySegment seg, long index, int x) {
        NewtonMeshDoubleData.m_strideInBytes$VH.set(seg.asSlice(index*sizeof()), x);
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


