#pragma once

#include <cstdint>
#include <vector>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

// Stage 2a: hygiene + instrumentation.
//
// The simulation numerics (integration, constraint solving, plasticity,
// breakage, impulse application) are intentionally IDENTICAL to the previous
// version. Known solver problems are left in place on purpose so they can be
// measured with the new diagnostics before Stage 2b/2c changes them:
//   - apply_impulse delivers J * sum(weights), not J        (Stage 2c)
//   - plasticity / breakage / max_strain use pre-solve strain (Stage 3)
//   - stiffness depends on solver_iterations (PBD)          (Stage 2b)
//   - damping acts on global motion                         (Stage 2b)
//
// All coordinates are in this node's local space (the node transform is not
// applied to particles).
class SoftBodySolver3D : public Node3D {
    GDCLASS(SoftBodySolver3D, Node3D)

private:
    struct Particle {
        Vector3 position;
        Vector3 previous_position;
        Vector3 rest_position; // Pose at creation; reference for deformation.
        real_t inverse_mass = 1.0;
    };

    struct Beam {
        int32_t a = 0;
        int32_t b = 0;
        real_t rest_length = 0.0;
        real_t initial_rest_length = 0.0;
        real_t stiffness = 0.5;
        real_t yield_strain = 0.12;
        real_t break_strain = 0.85;
        real_t plastic_creep = 0.012;
        bool broken = false;
    };

    // Cached measurements, recomputed lazily only when a getter needs them
    // after the state changed. Costs nothing per tick if never queried.
    struct Diagnostics {
        real_t total_mass = 0.0;
        Vector3 center_of_mass;
        Vector3 rest_center_of_mass;
        Vector3 linear_velocity;
        Vector3 linear_momentum;
        Vector3 angular_momentum; // About the center of mass.
        real_t kinetic_energy = 0.0;
        real_t internal_kinetic_energy = 0.0;
        Transform3D rigid_frame; // Best-fit rigid transform: rest pose -> current.
        real_t rms_deformation = 0.0;
        real_t max_deformation = 0.0;
        real_t max_strain_post_solve = 0.0;
        int32_t yielded_beam_count = 0;
    };

    static constexpr int32_t MAX_STEPS_PER_FRAME = 4;

    std::vector<Particle> particles;
    std::vector<Beam> beams;

    Vector3 gravity = Vector3(0.0, -9.81, 0.0);
    real_t damping = 0.995;
    real_t fixed_step = real_t(1.0 / 60.0);
    real_t accumulator = 0.0;
    real_t max_strain = 0.0; // Pre-solve, last step only (unchanged behaviour).

    int32_t solver_iterations = 4;
    int32_t broken_beam_count = 0;

    // Defaults applied to every beam (existing and future).
    real_t default_yield_strain = real_t(0.12);
    real_t default_break_strain = real_t(0.85);
    real_t default_plastic_creep = real_t(0.012);

    mutable Diagnostics diagnostics;
    mutable bool diagnostics_dirty = true;
    mutable Quaternion frame_rotation; // Warm start for the rigid-fit solver.

    void simulate_step(real_t delta);
    void solve_constraints();

    void compute_diagnostics() const;
    void ensure_diagnostics() const;

    // Frees data only. Never calls Node methods, so it is safe from the destructor.
    void clear_data();
    // clear_data() plus disabling physics processing. Not for the destructor.
    void clear_body();

protected:
    static void _bind_methods();

public:
    SoftBodySolver3D();
    ~SoftBodySolver3D();

    void create_box(
        const Vector3 &size,
        const Vector3 &center,
        int32_t nodes_x,
        int32_t nodes_y,
        int32_t nodes_z
    );

    // Returns the total linear momentum actually delivered to the particles
    // (useful to verify impulse scaling).
    Vector3 apply_impulse(
        const Vector3 &local_position,
        const Vector3 &impulse,
        real_t radius
    );

    // Restores the creation pose, original rest lengths, un-breaks all beams
    // and zeroes velocities. For repeatable tests.
    void reset_to_rest();

    void set_gravity(const Vector3 &value);
    Vector3 get_gravity() const;

    void set_damping(real_t value);
    real_t get_damping() const;

    void set_solver_iterations(int32_t value);
    int32_t get_solver_iterations() const;

    void set_fixed_step(real_t value);
    real_t get_fixed_step() const;

    void set_yield_strain(real_t value);
    real_t get_yield_strain() const;

    void set_break_strain(real_t value);
    real_t get_break_strain() const;

    void set_plastic_creep(real_t value);
    real_t get_plastic_creep() const;

    PackedVector3Array get_node_positions() const;
    PackedInt32Array get_beam_indices() const;

    int32_t get_node_count() const;
    int32_t get_beam_count() const;
    int32_t get_broken_beam_count() const;
    real_t get_max_strain() const;

    // --- Diagnostics -------------------------------------------------------
    real_t get_total_mass() const;
    Vector3 get_center_of_mass() const;
    Vector3 get_linear_velocity() const;
    Vector3 get_linear_momentum() const;
    Vector3 get_angular_momentum() const;
    real_t get_kinetic_energy() const;
    // Total kinetic energy minus the translational energy of the center of
    // mass. Still includes rigid rotation of the whole body.
    real_t get_internal_kinetic_energy() const;

    // Best-fit rigid transform (rest pose -> current pose), mass weighted.
    Transform3D get_rigid_frame() const;
    // Per-node offset from the rigid-fit pose, i.e. deformation with the
    // body's overall translation and rotation removed.
    PackedVector3Array get_node_deformations() const;
    real_t get_rms_deformation() const;
    real_t get_max_deformation() const;

    // Max |length - rest| / rest over unbroken beams, measured AFTER solving.
    // Compare with get_max_strain(), which is measured before solving.
    real_t get_max_strain_post_solve() const;
    // Unbroken beams whose rest length has changed by more than 0.5 percent.
    int32_t get_yielded_beam_count() const;

    void _physics_process(double delta) override;
};

}
