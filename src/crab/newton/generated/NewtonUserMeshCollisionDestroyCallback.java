// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public interface NewtonUserMeshCollisionDestroyCallback {

    void apply(jdk.incubator.foreign.MemoryAddress x0);
    static NativeSymbol allocate(NewtonUserMeshCollisionDestroyCallback fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonUserMeshCollisionDestroyCallback.class, fi, constants$6.NewtonUserMeshCollisionDestroyCallback$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;)V", scope);
    }
    static NewtonUserMeshCollisionDestroyCallback ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonUserMeshCollisionDestroyCallback::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0) -> {
            try {
                constants$6.NewtonUserMeshCollisionDestroyCallback$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


