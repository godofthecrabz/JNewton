// Generated by jextract

package crab.newton.generated;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public interface NewtonConstraintDestructor {

    void apply(jdk.incubator.foreign.MemoryAddress x0);
    static NativeSymbol allocate(NewtonConstraintDestructor fi, ResourceScope scope) {
        return RuntimeHelper.upcallStub(NewtonConstraintDestructor.class, fi, constants$17.NewtonConstraintDestructor$FUNC, "(Ljdk/incubator/foreign/MemoryAddress;)V", scope);
    }
    static NewtonConstraintDestructor ofAddress(MemoryAddress addr, ResourceScope scope) {
        NativeSymbol symbol = NativeSymbol.ofAddress("NewtonConstraintDestructor::" + Long.toHexString(addr.toRawLongValue()), addr, scope);
return (jdk.incubator.foreign.MemoryAddress x0) -> {
            try {
                constants$17.NewtonConstraintDestructor$MH.invokeExact(symbol, (jdk.incubator.foreign.Addressable)x0);
            } catch (Throwable ex$) {
                throw new AssertionError("should not reach here", ex$);
            }
        };
    }
}


