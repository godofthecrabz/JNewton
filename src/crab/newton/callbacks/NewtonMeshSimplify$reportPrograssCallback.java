// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * int (*NewtonMeshSimplify$reportPrograssCallback)(float,void*);
 * }
 */
public interface NewtonMeshSimplify$reportPrograssCallback {

    int apply(float normalizedProgressPercent, java.lang.foreign.MemorySegment userData);
    static MemorySegment allocate(NewtonMeshSimplify$reportPrograssCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$117.NewtonMeshSimplify$reportPrograssCallback_UP$MH, fi, constants$117.NewtonMeshSimplify$reportPrograssCallback$FUNC, scope);
    }
    static NewtonMeshSimplify$reportPrograssCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (float _normalizedProgressPercent, java.lang.foreign.MemorySegment _userData) -> {
            try {
                return (int)constants$117.NewtonMeshSimplify$reportPrograssCallback_DOWN$MH.invokeExact(symbol, _normalizedProgressPercent, _userData);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


