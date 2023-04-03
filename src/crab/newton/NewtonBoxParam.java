// Generated by jextract

package crab.newton;

import java.lang.invoke.VarHandle;
import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * struct NewtonBoxParam {
 *     float m_x;
 *     float m_y;
 *     float m_z;
 * };
 * }
 */
public class NewtonBoxParam {

    static final StructLayout $struct$LAYOUT = MemoryLayout.structLayout(
        Constants$root.C_FLOAT$LAYOUT.withName("m_x"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_y"),
        Constants$root.C_FLOAT$LAYOUT.withName("m_z")
    ).withName("NewtonBoxParam");
    public static MemoryLayout $LAYOUT() {
        return NewtonBoxParam.$struct$LAYOUT;
    }
    static final VarHandle m_x$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_x"));
    public static VarHandle m_x$VH() {
        return NewtonBoxParam.m_x$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * float m_x;
     * }
     */
    public static float m_x$get(MemorySegment seg) {
        return (float)NewtonBoxParam.m_x$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * float m_x;
     * }
     */
    public static void m_x$set(MemorySegment seg, float x) {
        NewtonBoxParam.m_x$VH.set(seg, x);
    }
    public static float m_x$get(MemorySegment seg, long index) {
        return (float)NewtonBoxParam.m_x$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_x$set(MemorySegment seg, long index, float x) {
        NewtonBoxParam.m_x$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_y$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_y"));
    public static VarHandle m_y$VH() {
        return NewtonBoxParam.m_y$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * float m_y;
     * }
     */
    public static float m_y$get(MemorySegment seg) {
        return (float)NewtonBoxParam.m_y$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * float m_y;
     * }
     */
    public static void m_y$set(MemorySegment seg, float x) {
        NewtonBoxParam.m_y$VH.set(seg, x);
    }
    public static float m_y$get(MemorySegment seg, long index) {
        return (float)NewtonBoxParam.m_y$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_y$set(MemorySegment seg, long index, float x) {
        NewtonBoxParam.m_y$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_z$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_z"));
    public static VarHandle m_z$VH() {
        return NewtonBoxParam.m_z$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * float m_z;
     * }
     */
    public static float m_z$get(MemorySegment seg) {
        return (float)NewtonBoxParam.m_z$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * float m_z;
     * }
     */
    public static void m_z$set(MemorySegment seg, float x) {
        NewtonBoxParam.m_z$VH.set(seg, x);
    }
    public static float m_z$get(MemorySegment seg, long index) {
        return (float)NewtonBoxParam.m_z$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_z$set(MemorySegment seg, long index, float x) {
        NewtonBoxParam.m_z$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(long len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment ofAddress(MemorySegment addr, SegmentScope scope) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, scope); }
}


