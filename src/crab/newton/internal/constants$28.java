// Generated by jextract

package crab.newton.internal;

import java.lang.invoke.MethodHandle;
import java.lang.foreign.*;
import crab.newton.callbacks.*;
public final class constants$28 {

    // Suppresses default constructor, ensuring non-instantiability.
    private constants$28() {}
    public static final FunctionDescriptor NewtonDeserializeScene$serializeCallback$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final FunctionDescriptor NewtonDeserializeScene$serializeCallback_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonDeserializeScene$serializeCallback_UP$MH = RuntimeHelper.upcallHandle(NewtonDeserializeScene$serializeCallback.class, "apply", constants$28.NewtonDeserializeScene$serializeCallback_UP$FUNC);
    public static final FunctionDescriptor NewtonDeserializeScene$serializeCallback_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonDeserializeScene$serializeCallback_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$28.NewtonDeserializeScene$serializeCallback_DOWN$FUNC
    );
    public static final FunctionDescriptor NewtonDeserializeScene$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonDeserializeScene$MH = RuntimeHelper.downcallHandle(
        "NewtonDeserializeScene",
        constants$28.NewtonDeserializeScene$FUNC
    );
    public static final FunctionDescriptor NewtonFindSerializedBody$FUNC = FunctionDescriptor.of(Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_LONG$LAYOUT
    );
    public static final MethodHandle NewtonFindSerializedBody$MH = RuntimeHelper.downcallHandle(
        "NewtonFindSerializedBody",
        constants$28.NewtonFindSerializedBody$FUNC
    );
    public static final FunctionDescriptor NewtonSetJointSerializationCallbacks$serializeJoint$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final FunctionDescriptor NewtonSetJointSerializationCallbacks$serializeJoint_UP$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSetJointSerializationCallbacks$serializeJoint_UP$MH = RuntimeHelper.upcallHandle(NewtonSetJointSerializationCallbacks$serializeJoint.class, "apply", constants$28.NewtonSetJointSerializationCallbacks$serializeJoint_UP$FUNC);
    public static final FunctionDescriptor NewtonSetJointSerializationCallbacks$serializeJoint_DOWN$FUNC = FunctionDescriptor.ofVoid(
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT,
        Constants$root.C_POINTER$LAYOUT
    );
    public static final MethodHandle NewtonSetJointSerializationCallbacks$serializeJoint_DOWN$MH = RuntimeHelper.downcallHandle(
        constants$28.NewtonSetJointSerializationCallbacks$serializeJoint_DOWN$FUNC
    );
}


