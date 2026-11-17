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

// Stage 2b: XPBD substep solver.
//
// Solver summary
//   - Each physics tick (fixed_step) is split into `substeps` small steps.
//   - Beams are XPBD distance constraints with real compliance (1 / stiffness),
//     so stiffness no longer depends on iteration count or step size.
//   - Velocities are explicit. Beam damping acts on RELATIVE axial velocity only,
//     so a moving body keeps its momentum. `global_drag` is separate (default 0).
//   - Plasticity and breakage are evaluated AFTER solving, on the settled strain.
//     (Simple placeholder rules; Stage 3 refines them.)
//   - Solve order alternates direction to avoid directional bias.
//   - Safety: speed clamp, NaN/Inf detection with rollback to the last good tick.
//
// Units are arbitrary but consistent: mass (kg), length (m), stiffness (N/m).
// All coordinates are in this node's local space.
//
// Still intentionally unchanged (Stage 2c): apply_impulse delivers J * sum(weights).
class SoftBodySolver3D : public Node3D {
    GDCLASS(SoftBodySolver3D, Node3D)

private:
    struct Particle {
        Vector3 position;
        Vector3 previous_position; // Position at the start of the current substep.
        Vector3 velocity;
        Vector3 rest_position;
        real_t inverse_mass = 1.0; // 0 = pinned.
    };

    struct Beam {
        int32_t a = 0;
        int32_t b = 0;
        real_t rest_length = 0.0;
        real_t initial_rest_length = 0.0;
        real_t compliance = 0.0; // 1 / (beam_stiffness * stiffness_scale)
        real_t lambda = 0.0;     // XPBD multiplier (only used when iterations > 1).
        // Per-beam multipliers on the global material values.
        real_t stiffness_scale = 1.0;
        real_t damping_scale = 1.0;
        real_t yield_scale = 1.0;
        real_t break_scale = 1.0;
        real_t plastic_scale = 1.0;
        uint8_t overload = 0; // Consecutive substeps beyond the break limit.
        bool broken = false;
    };

    struct Diagnostics {
        real_t total_mass = 0.0;
        Vector3 center_of_mass;
        Vector3 rest_center_of_mass;
        Vector3 linear_velocity;
        Vector3 linear_momentum;
        Vector3 angular_momentum; // About the center of mass.
        real_t kinetic_energy = 0.0;
        real_t internal_kinetic_energy = 0.0;
        real_t elastic_energy = 0.0;
        Transform3D rigid_frame; // Best-fit rigid transform: rest pose -> current.
        real_t rms_deformation = 0.0;
        real_t max_deformation = 0.0;
        real_t max_strain_post_solve = 0.0;
        int32_t yielded_beam_count = 0;
    };

    static constexpr int32_t MAX_NODES = 4096;
    static constexpr int32_t MAX_BEAMS = 16384;
    static constexpr int32_t MAX_TICKS_PER_FRAME = 4;
    static constexpr real_t MIN_REST_RATIO = real_t(0.35);
    static constexpr real_t MAX_REST_RATIO = real_t(1.60);

    std::vector<Particle> particles;
    std::vector<Beam> beams;
    std::vector<Vector3> good_position; // State at the start of the last tick (rollback).
    std::vector<Vector3> good_velocity;

    // Global settings.
    Vector3 gravity = Vector3(0.0, -9.81, 0.0);
    real_t fixed_step = real_t(1.0 / 60.0);
    int32_t substeps = 8;
    int32_t solver_iterations = 1;
    real_t global_drag = 0.0;   // 1/s, whole-body velocity decay.
    real_t max_speed = 100.0;   // m/s safety clamp.

    // Global material values (beams scale these).
    real_t beam_stiffness = real_t(4.0e5); // N/m
    real_t beam_damping = 20.0;            // 1/s, relative axial velocity decay.
    real_t yield_strain = real_t(0.03);
    real_t break_strain = real_t(0.5);     // Tension: extension over initial length.
    real_t break_hold = real_t(0.004);     // s the limit must be exceeded to break.
    real_t plastic_rate = 150.0;           // 1/s, how fast rest length follows.

    // Runtime state.
    real_t accumulator = 0.0;
    uint32_t substep_counter = 0;
    real_t max_strain = 0.0;      // Peak |strain| during the last tick (post-solve).
    real_t tick_max_strain = 0.0;
    int32_t broken_beam_count = 0;
    int32_t fault_count = 0;
    double plastic_work = 0.0;
    double damper_work = 0.0;
    double step_time_us = 0.0;    // Smoothed cost of one _physics_process call.

    // Test wall (analytic half-space: dot(x, normal) >= offset).
    bool wall_enabled = false;
    Vector3 wall_normal = Vector3(0.0, 0.0, -1.0);
    real_t wall_offset = 0.0;
    real_t wall_friction = 0.0;
    int32_t wall_contacts = 0;
    Vector3 tick_wall_impulse;
    Vector3 wall_force;
    Vector3 wall_impulse_total;
    real_t wall_peak_force = 0.0;

    mutable Diagnostics diagnostics;
    mutable bool diagnostics_dirty = true;
    mutable Quaternion frame_rotation; // Warm start for the rigid-fit solver.

    void simulate_tick();
    void integrate(real_t h);
    void solve_beams(real_t inv_h2, int32_t iteration, bool reverse);
    void solve_wall(real_t h);
    void update_beams(real_t h);

    void refresh_compliance();

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

    // --- Building ----------------------------------------------------------
    // Convenience test fixture (same topology as before).
    void create_box(
        const Vector3 &size,
        const Vector3 &center,
        int32_t nodes_x,
        int32_t nodes_y,
        int32_t nodes_z
    );

    // Authoring: clear_structure(), add_node()..., add_beam()..., finish_build().
    void clear_structure();
    // mass <= 0 pins the node. Returns the node index or -1 if full.
    int32_t add_node(const Vector3 &position, real_t mass);
    // Rest length = current distance. Returns the beam index or -1.
    int32_t add_beam(int32_t a, int32_t b, real_t stiffness_scale);
    void set_beam_scales(
        int32_t beam_index,
        real_t stiffness_scale,
        real_t damping_scale,
        real_t yield_scale,
        real_t break_scale,
        real_t plastic_scale
    );
    void finish_build();

    // --- Mass and motion ---------------------------------------------------
    void set_node_mass(int32_t node_index, real_t mass); // mass <= 0 pins.
    real_t get_node_mass(int32_t node_index) const;      // 0 if pinned.
    void set_total_mass(real_t mass);                    // Rescales unpinned nodes.
    void set_linear_velocity(const Vector3 &velocity);   // All unpinned nodes.

    // Returns the total linear momentum actually delivered to the particles.
    Vector3 apply_impulse(
        const Vector3 &local_position,
        const Vector3 &impulse,
        real_t radius
    );

    // Restores the creation pose, original rest lengths, un-breaks all beams,
    // zeroes velocities, energy counters and wall statistics.
    void reset_to_rest();

    // --- Test wall ---------------------------------------------------------
    // Half-space dot(x, normal) >= offset. Inelastic, optional friction.
    void set_wall(bool enabled, const Vector3 &normal, real_t offset, real_t friction);
    real_t get_wall_force() const;       // Magnitude of the last tick's contact force.
    real_t get_wall_peak_force() const;
    Vector3 get_wall_impulse() const;    // Cumulative since reset.
    int32_t get_wall_contact_count() const;

    // --- Settings ----------------------------------------------------------
    void set_gravity(const Vector3 &value);
    Vector3 get_gravity() const;

    void set_global_drag(real_t value);
    real_t get_global_drag() const;

    void set_beam_damping(real_t value);
    real_t get_beam_damping() const;

    void set_solver_iterations(int32_t value);
    int32_t get_solver_iterations() const;

    void set_substeps(int32_t value);
    int32_t get_substeps() const;

    void set_fixed_step(real_t value);
    real_t get_fixed_step() const;

    void set_max_speed(real_t value);
    real_t get_max_speed() const;

    void set_beam_stiffness(real_t value);
    real_t get_beam_stiffness() const;

    void set_yield_strain(real_t value);
    real_t get_yield_strain() const;

    void set_break_strain(real_t value);
    real_t get_break_strain() const;

    void set_break_hold(real_t value);
    real_t get_break_hold() const;

    void set_plastic_rate(real_t value);
    real_t get_plastic_rate() const;

    // --- Output ------------------------------------------------------------
    PackedVector3Array get_node_positions() const;
    PackedInt32Array get_beam_indices() const;

    int32_t get_node_count() const;
    int32_t get_beam_count() const;
    int32_t get_broken_beam_count() const;
    // Peak |strain| of any beam during the last tick, measured after solving.
    real_t get_max_strain() const;
    // Number of times NaN/Inf was detected and the last tick rolled back.
    int32_t get_fault_count() const;
    // Smoothed microseconds spent in _physics_process (all ticks of a frame).
    real_t get_step_time_us() const;

    // --- Diagnostics -------------------------------------------------------
    real_t get_total_mass() const;
    Vector3 get_center_of_mass() const;
    Vector3 get_rest_center_of_mass() const;
    Vector3 get_linear_velocity() const;
    Vector3 get_linear_momentum() const;
    Vector3 get_angular_momentum() const;
    real_t get_kinetic_energy() const;
    // Total kinetic energy minus translational energy of the center of mass.
    // Still includes rigid rotation of the whole body.
    real_t get_internal_kinetic_energy() const;
    // Energy stored in beam elasticity right now.
    real_t get_elastic_energy() const;
    // Cumulative energy converted by plastic deformation / removed by beam damping.
    real_t get_plastic_work() const;
    real_t get_damper_work() const;

    // Best-fit rigid transform (rest pose -> current pose), mass weighted.
    Transform3D get_rigid_frame() const;
    // Per-node offset from the rigid-fit pose.
    PackedVector3Array get_node_deformations() const;
    real_t get_rms_deformation() const;
    real_t get_max_deformation() const;

    // Max |length - rest| / rest over unbroken beams right now.
    real_t get_max_strain_post_solve() const;
    // Unbroken beams whose rest length has changed by more than 0.5 percent.
    int32_t get_yielded_beam_count() const;

    void _physics_process(double delta) override;
};

}
