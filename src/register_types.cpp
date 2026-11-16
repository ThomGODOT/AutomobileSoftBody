#include "register_types.h"
#include "soft_body_solver_3d.h"

#include <gdextension_interface.h>
#include <godot_cpp/core/defs.hpp>
#include <godot_cpp/godot.hpp>

using namespace godot;

void initialize_mobile_softbody_module(ModuleInitializationLevel level) {
    if (level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }

    GDREGISTER_CLASS(SoftBodySolver3D);
}

void uninitialize_mobile_softbody_module(ModuleInitializationLevel level) {
    if (level != MODULE_INITIALIZATION_LEVEL_SCENE) {
        return;
    }
}

extern "C" {

GDExtensionBool GDE_EXPORT mobile_softbody_library_init(
    GDExtensionInterfaceGetProcAddress get_proc_address,
    GDExtensionClassLibraryPtr library,
    GDExtensionInitialization *initialization
) {
    GDExtensionBinding::InitObject init_obj(
        get_proc_address,
        library,
        initialization
    );

    init_obj.register_initializer(initialize_mobile_softbody_module);
    init_obj.register_terminator(uninitialize_mobile_softbody_module);
    init_obj.set_minimum_library_initialization_level(
        MODULE_INITIALIZATION_LEVEL_SCENE
    );

    return init_obj.init();
}

}
