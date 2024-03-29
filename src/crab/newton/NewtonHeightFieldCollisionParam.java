// Generated by jextract

package crab.newton;

import java.lang.invoke.VarHandle;
import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * struct NewtonHeightFieldCollisionParam {
 *     int m_width;
 *     int m_height;
 *     int m_gridsDiagonals;
 *     int m_elevationDataType;
 *     float m_verticalScale;
 *     float m_horizonalScale_x;
 *     float m_horizonalScale_z;
 *     void* m_vertialElevation;
 *     char* m_atributes;
 * };
 * }
 */
public class NewtonHeightFieldCollisionParam {

    static final StructLayout $struct$LAYOUT = MemoryLayout.structLayout(
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
    ).withName("NewtonHeightFieldCollisionParam");
    public static MemoryLayout $LAYOUT() {
        return NewtonHeightFieldCollisionParam.$struct$LAYOUT;
    }
    static final VarHandle m_width$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_width"));
    public static VarHandle m_width$VH() {
        return NewtonHeightFieldCollisionParam.m_width$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * int m_width;
     * }
     */
    public static int m_width$get(MemorySegment seg) {
        return (int)NewtonHeightFieldCollisionParam.m_width$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * int m_width;
     * }
     */
    public static void m_width$set(MemorySegment seg, int x) {
        NewtonHeightFieldCollisionParam.m_width$VH.set(seg, x);
    }
    public static int m_width$get(MemorySegment seg, long index) {
        return (int)NewtonHeightFieldCollisionParam.m_width$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_width$set(MemorySegment seg, long index, int x) {
        NewtonHeightFieldCollisionParam.m_width$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_height$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_height"));
    public static VarHandle m_height$VH() {
        return NewtonHeightFieldCollisionParam.m_height$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * int m_height;
     * }
     */
    public static int m_height$get(MemorySegment seg) {
        return (int)NewtonHeightFieldCollisionParam.m_height$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * int m_height;
     * }
     */
    public static void m_height$set(MemorySegment seg, int x) {
        NewtonHeightFieldCollisionParam.m_height$VH.set(seg, x);
    }
    public static int m_height$get(MemorySegment seg, long index) {
        return (int)NewtonHeightFieldCollisionParam.m_height$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_height$set(MemorySegment seg, long index, int x) {
        NewtonHeightFieldCollisionParam.m_height$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_gridsDiagonals$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_gridsDiagonals"));
    public static VarHandle m_gridsDiagonals$VH() {
        return NewtonHeightFieldCollisionParam.m_gridsDiagonals$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * int m_gridsDiagonals;
     * }
     */
    public static int m_gridsDiagonals$get(MemorySegment seg) {
        return (int)NewtonHeightFieldCollisionParam.m_gridsDiagonals$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * int m_gridsDiagonals;
     * }
     */
    public static void m_gridsDiagonals$set(MemorySegment seg, int x) {
        NewtonHeightFieldCollisionParam.m_gridsDiagonals$VH.set(seg, x);
    }
    public static int m_gridsDiagonals$get(MemorySegment seg, long index) {
        return (int)NewtonHeightFieldCollisionParam.m_gridsDiagonals$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_gridsDiagonals$set(MemorySegment seg, long index, int x) {
        NewtonHeightFieldCollisionParam.m_gridsDiagonals$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_elevationDataType$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_elevationDataType"));
    public static VarHandle m_elevationDataType$VH() {
        return NewtonHeightFieldCollisionParam.m_elevationDataType$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * int m_elevationDataType;
     * }
     */
    public static int m_elevationDataType$get(MemorySegment seg) {
        return (int)NewtonHeightFieldCollisionParam.m_elevationDataType$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * int m_elevationDataType;
     * }
     */
    public static void m_elevationDataType$set(MemorySegment seg, int x) {
        NewtonHeightFieldCollisionParam.m_elevationDataType$VH.set(seg, x);
    }
    public static int m_elevationDataType$get(MemorySegment seg, long index) {
        return (int)NewtonHeightFieldCollisionParam.m_elevationDataType$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_elevationDataType$set(MemorySegment seg, long index, int x) {
        NewtonHeightFieldCollisionParam.m_elevationDataType$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_verticalScale$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_verticalScale"));
    public static VarHandle m_verticalScale$VH() {
        return NewtonHeightFieldCollisionParam.m_verticalScale$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * float m_verticalScale;
     * }
     */
    public static float m_verticalScale$get(MemorySegment seg) {
        return (float)NewtonHeightFieldCollisionParam.m_verticalScale$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * float m_verticalScale;
     * }
     */
    public static void m_verticalScale$set(MemorySegment seg, float x) {
        NewtonHeightFieldCollisionParam.m_verticalScale$VH.set(seg, x);
    }
    public static float m_verticalScale$get(MemorySegment seg, long index) {
        return (float)NewtonHeightFieldCollisionParam.m_verticalScale$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_verticalScale$set(MemorySegment seg, long index, float x) {
        NewtonHeightFieldCollisionParam.m_verticalScale$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_horizonalScale_x$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_horizonalScale_x"));
    public static VarHandle m_horizonalScale_x$VH() {
        return NewtonHeightFieldCollisionParam.m_horizonalScale_x$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * float m_horizonalScale_x;
     * }
     */
    public static float m_horizonalScale_x$get(MemorySegment seg) {
        return (float)NewtonHeightFieldCollisionParam.m_horizonalScale_x$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * float m_horizonalScale_x;
     * }
     */
    public static void m_horizonalScale_x$set(MemorySegment seg, float x) {
        NewtonHeightFieldCollisionParam.m_horizonalScale_x$VH.set(seg, x);
    }
    public static float m_horizonalScale_x$get(MemorySegment seg, long index) {
        return (float)NewtonHeightFieldCollisionParam.m_horizonalScale_x$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_horizonalScale_x$set(MemorySegment seg, long index, float x) {
        NewtonHeightFieldCollisionParam.m_horizonalScale_x$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_horizonalScale_z$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_horizonalScale_z"));
    public static VarHandle m_horizonalScale_z$VH() {
        return NewtonHeightFieldCollisionParam.m_horizonalScale_z$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * float m_horizonalScale_z;
     * }
     */
    public static float m_horizonalScale_z$get(MemorySegment seg) {
        return (float)NewtonHeightFieldCollisionParam.m_horizonalScale_z$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * float m_horizonalScale_z;
     * }
     */
    public static void m_horizonalScale_z$set(MemorySegment seg, float x) {
        NewtonHeightFieldCollisionParam.m_horizonalScale_z$VH.set(seg, x);
    }
    public static float m_horizonalScale_z$get(MemorySegment seg, long index) {
        return (float)NewtonHeightFieldCollisionParam.m_horizonalScale_z$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_horizonalScale_z$set(MemorySegment seg, long index, float x) {
        NewtonHeightFieldCollisionParam.m_horizonalScale_z$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_vertialElevation$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_vertialElevation"));
    public static VarHandle m_vertialElevation$VH() {
        return NewtonHeightFieldCollisionParam.m_vertialElevation$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * void* m_vertialElevation;
     * }
     */
    public static MemorySegment m_vertialElevation$get(MemorySegment seg) {
        return (java.lang.foreign.MemorySegment)NewtonHeightFieldCollisionParam.m_vertialElevation$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * void* m_vertialElevation;
     * }
     */
    public static void m_vertialElevation$set(MemorySegment seg, MemorySegment x) {
        NewtonHeightFieldCollisionParam.m_vertialElevation$VH.set(seg, x);
    }
    public static MemorySegment m_vertialElevation$get(MemorySegment seg, long index) {
        return (java.lang.foreign.MemorySegment)NewtonHeightFieldCollisionParam.m_vertialElevation$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_vertialElevation$set(MemorySegment seg, long index, MemorySegment x) {
        NewtonHeightFieldCollisionParam.m_vertialElevation$VH.set(seg.asSlice(index*sizeof()), x);
    }
    static final VarHandle m_atributes$VH = $struct$LAYOUT.varHandle(MemoryLayout.PathElement.groupElement("m_atributes"));
    public static VarHandle m_atributes$VH() {
        return NewtonHeightFieldCollisionParam.m_atributes$VH;
    }
    /**
     * Getter for field:
     * {@snippet :
     * char* m_atributes;
     * }
     */
    public static MemorySegment m_atributes$get(MemorySegment seg) {
        return (java.lang.foreign.MemorySegment)NewtonHeightFieldCollisionParam.m_atributes$VH.get(seg);
    }
    /**
     * Setter for field:
     * {@snippet :
     * char* m_atributes;
     * }
     */
    public static void m_atributes$set(MemorySegment seg, MemorySegment x) {
        NewtonHeightFieldCollisionParam.m_atributes$VH.set(seg, x);
    }
    public static MemorySegment m_atributes$get(MemorySegment seg, long index) {
        return (java.lang.foreign.MemorySegment)NewtonHeightFieldCollisionParam.m_atributes$VH.get(seg.asSlice(index*sizeof()));
    }
    public static void m_atributes$set(MemorySegment seg, long index, MemorySegment x) {
        NewtonHeightFieldCollisionParam.m_atributes$VH.set(seg.asSlice(index*sizeof()), x);
    }
    public static long sizeof() { return $LAYOUT().byteSize(); }
    public static MemorySegment allocate(SegmentAllocator allocator) { return allocator.allocate($LAYOUT()); }
    public static MemorySegment allocateArray(long len, SegmentAllocator allocator) {
        return allocator.allocate(MemoryLayout.sequenceLayout(len, $LAYOUT()));
    }
    public static MemorySegment ofAddress(MemorySegment addr, SegmentScope scope) { return RuntimeHelper.asArray(addr, $LAYOUT(), 1, scope); }
}


