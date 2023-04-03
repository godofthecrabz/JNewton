// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback)(struct NewtonBody*,struct NewtonFracturedCompoundMeshPart*,struct NewtonCollision*);
 * }
 */
public interface NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback {

    void apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment mainMesh, java.lang.foreign.MemorySegment fracturedCompountCollision);
    static MemorySegment allocate(NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$57.NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_UP$MH, fi, constants$57.NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback$FUNC, scope);
    }
    static NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _mainMesh, java.lang.foreign.MemorySegment _fracturedCompountCollision) -> {
            try {
                constants$57.NewtonCreateFracturedCompoundCollision$regenerateMainMeshCallback_DOWN$MH.invokeExact(symbol, _body, _mainMesh, _fracturedCompountCollision);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


