// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.invoke.VarHandle;
import java.nio.ByteOrder;
import jdk.incubator.foreign.*;
import static jdk.incubator.foreign.ValueLayout.*;
public class constants$11 {

	public static final FunctionDescriptor NewtonIslandUpdate$FUNC = FunctionDescriptor.of(Constants$root.C_LONG$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
	public static final MethodHandle NewtonIslandUpdate$MH = RuntimeHelper.downcallHandle(
        constants$11.NewtonIslandUpdate$FUNC, false
    );
	public static final FunctionDescriptor NewtonFractureCompoundCollisionOnEmitCompoundFractured$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFractureCompoundCollisionOnEmitCompoundFractured$MH = RuntimeHelper.downcallHandle(
        constants$11.NewtonFractureCompoundCollisionOnEmitCompoundFractured$FUNC, false
    );
	public static final FunctionDescriptor NewtonFractureCompoundCollisionOnEmitChunk$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
	public static final MethodHandle NewtonFractureCompoundCollisionOnEmitChunk$MH = RuntimeHelper.downcallHandle(
        constants$11.NewtonFractureCompoundCollisionOnEmitChunk$FUNC, false
    );
}


