// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * void (*NewtonStaticCollisionSetDebugCallback$userCallback)(struct NewtonBody*,struct NewtonBody*,int,int,float*,int);
 * }
 */
public interface NewtonStaticCollisionSetDebugCallback$userCallback {

    void apply(java.lang.foreign.MemorySegment bodyWithTreeCollision, java.lang.foreign.MemorySegment body, int faceID, int vertexCount, java.lang.foreign.MemorySegment vertex, int vertexStrideInBytes);
    static MemorySegment allocate(NewtonStaticCollisionSetDebugCallback$userCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$71.NewtonStaticCollisionSetDebugCallback$userCallback_UP$MH, fi, constants$71.NewtonStaticCollisionSetDebugCallback$userCallback$FUNC, scope);
    }
    static NewtonStaticCollisionSetDebugCallback$userCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _bodyWithTreeCollision, java.lang.foreign.MemorySegment _body, int _faceID, int _vertexCount, java.lang.foreign.MemorySegment _vertex, int _vertexStrideInBytes) -> {
            try {
                constants$71.NewtonStaticCollisionSetDebugCallback$userCallback_DOWN$MH.invokeExact(symbol, _bodyWithTreeCollision, _body, _faceID, _vertexCount, _vertex, _vertexStrideInBytes);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


