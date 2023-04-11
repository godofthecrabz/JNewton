// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * float (*NewtonWorldRayCast$filter)(struct NewtonBody*,struct NewtonCollision*,float*,float*,long long,void*,float);
 * }
 */
public interface NewtonWorldRayCast$filter {

    float apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment shapeHit, java.lang.foreign.MemorySegment hitContact, java.lang.foreign.MemorySegment hitNormal, long collisionID, java.lang.foreign.MemorySegment userData, float intersectParam);
    static MemorySegment allocate(NewtonWorldRayCast$filter fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$41.NewtonWorldRayCast$filter_UP$MH, fi, constants$41.NewtonWorldRayCast$filter$FUNC, scope);
    }
    static NewtonWorldRayCast$filter ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _shapeHit, java.lang.foreign.MemorySegment _hitContact, java.lang.foreign.MemorySegment _hitNormal, long _collisionID, java.lang.foreign.MemorySegment _userData, float _intersectParam) -> {
            try {
                return (float)constants$41.NewtonWorldRayCast$filter_DOWN$MH.invokeExact(symbol, _body, _shapeHit, _hitContact, _hitNormal, _collisionID, _userData, _intersectParam);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}

