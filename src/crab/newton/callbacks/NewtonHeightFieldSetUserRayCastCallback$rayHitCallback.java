// Generated by jextract

package crab.newton.callbacks;

import java.lang.foreign.*;
import crab.newton.internal.*;
/**
 * {@snippet :
 * float (*NewtonHeightFieldSetUserRayCastCallback$rayHitCallback)(struct NewtonBody*,struct NewtonCollision*,float,int,int,float*,int,void*);
 * }
 */
public interface NewtonHeightFieldSetUserRayCastCallback$rayHitCallback {

    float apply(java.lang.foreign.MemorySegment body, java.lang.foreign.MemorySegment heightFieldCollision, float intersection, int row, int col, java.lang.foreign.MemorySegment normal, int faceId, java.lang.foreign.MemorySegment usedData);
    static MemorySegment allocate(NewtonHeightFieldSetUserRayCastCallback$rayHitCallback fi, SegmentScope scope) {
        return RuntimeHelper.upcallStub(constants$68.NewtonHeightFieldSetUserRayCastCallback$rayHitCallback_UP$MH, fi, constants$68.NewtonHeightFieldSetUserRayCastCallback$rayHitCallback$FUNC, scope);
    }
    static NewtonHeightFieldSetUserRayCastCallback$rayHitCallback ofAddress(MemorySegment addr, SegmentScope scope) {
        MemorySegment symbol = MemorySegment.ofAddress(addr.address(), 0, scope);
        return (java.lang.foreign.MemorySegment _body, java.lang.foreign.MemorySegment _heightFieldCollision, float _intersection, int _row, int _col, java.lang.foreign.MemorySegment _normal, int _faceId, java.lang.foreign.MemorySegment _usedData) -> {
            try {
                return (float)constants$68.NewtonHeightFieldSetUserRayCastCallback$rayHitCallback_DOWN$MH.invokeExact(symbol, _body, _heightFieldCollision, _intersection, _row, _col, _normal, _faceId, _usedData);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


