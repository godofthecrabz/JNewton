// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonCollisionForEachPolygonDo$callback)(void*,int,float*,int);
 * }
 */
public interface NewtonCollisionForEachPolygonDo$callback {

    void apply(java.lang.foreign.MemorySegment userData, int vertexCount, java.lang.foreign.MemorySegment faceArray, int faceId);
    static MemorySegment allocate(NewtonCollisionForEachPolygonDo$callback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$76.NewtonCollisionForEachPolygonDo$callback_UP$MH, fi, constants$76.NewtonCollisionForEachPolygonDo$callback$FUNC, scope);
    }
    static NewtonCollisionForEachPolygonDo$callback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _userData, int _vertexCount, java.lang.foreign.MemorySegment _faceArray, int _faceId) -> {
            try {
                constants$76.NewtonCollisionForEachPolygonDo$callback_DOWN$MH.invokeExact(symbol, _userData, _vertexCount, _faceArray, _faceId);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

