#pragma once

#include <cstdint>
#include <utility>
#include <vector>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/basis.hpp>
#include <godot_cpp/variant/dictionary.hpp>
#include <godot_cpp/variant/packed_float32_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/quaternion.hpp>
#include <godot_cpp/variant/transform3d.hpp>
#include <godot_cpp/variant/vector3.hpp>

namespace godot {

class SoftBodySolver3D : public Node3D {
    GDCLASS(SoftBodySolver3D, Node3D)

private:
    struct Particle {
        Vector3 position;
        Vector3 previous_position;
        Vector3 velocity;
        Vector3 rest_position;
        real_t inverse_mass = 1.0;
    };

    struct Beam {
        int32_t a = 0;
        int32_t b = 0;
        real_t rest_length = 0.0;
        real_t initial_rest_length = 0.0;
        real_t compliance = 0.0;
        real_t lambda = 0.0;
        real_t stiffness_scale = 1.0;
        real_t damping_scale = 1.0;
        real_t yield_scale = 1.0;
        real_t break_scale = 1.0;
        real_t plastic_scale = 1.0;
        uint8_t overload = 0;
        bool broken = false;
    };

    struct Diagnostics {
        real_t total_mass = 0.0;
        Vector3 center_of_mass;
        Vector3 rest_center_of_mass;
        Vector3 linear_velocity;
        Vector3 linear_momentum;
        Vector3 angular_momentum;
        real_t kinetic_energy = 0.0;
        real_t internal_kinetic_energy = 0.0;
        real_t elastic_energy = 0.0;
        Transform3D rigid_frame;
        real_t rms_deformation = 0.0;
        real_t max_deformation = 0.0;
        real_t max_strain_post_solve = 0.0;
        int32_t yielded_beam_count = 0;
    };

    static constexpr int32_t MAX_NODES = 4096;
    static constexpr int32_t MAX_BEAMS = 16384;
    static constexpr int32_t MAX_TICKS_PER_FRAME = 4;
    static constexpr int32_t MIN_ACTIVE_SUBSTEPS = 2;

    std::vector<Particle> particles;
    std::vector<Beam> beams;
    std::vector<Vector3> good_position;
    std::vector<Vector3> good_velocity;
    std::vector<std::pair<real_t, int32_t>> scratch_order;
    std::vector<real_t> scratch_weights;

    Vector3 gravity = Vector3(0.0, -9.81, 0.0);
    real_t fixed_step = real_t(1.0 / 60.0);
    int32_t substeps = 8;
    int32_t active_substeps = 8;
    int32_t solver_iterations = 1;
    real_t global_drag = 0.0;
    real_t settle_damping = 8.0;
    real_t max_speed = 100.0;

    real_t beam_stiffness = real_t(4.0e5);
    real_t beam_damping = 20.0;
    real_t yield_strain = real_t(0.02);
    real_t break_strain = real_t(0.5);
    real_t break_hold = real_t(0.004);
    real_t plastic_rate = 150.0;
    real_t hardening = real_t(0.4);
    real_t buckle_ratio = real_t(0.7);
    real_t crush_limit = real_t(0.5);
    real_t stretch_limit = real_t(1.4);
    real_t plastic_hold = real_t(0.5);
    real_t plastic_timer = 0.0;
    bool plastic_allowed = false;

    bool lock_momentum = true;
    bool sleep_enabled = true;
    real_t sleep_speed = real_t(0.03);
    int32_t sleep_ticks = 20;
    int32_t sleep_counter = 0;
    bool sleeping = false;

    real_t time_budget_us = 0.0;
    int32_t budget_frames = 0;

    real_t accumulator = 0.0;
    uint32_t substep_counter = 0;
    real_t max_strain = 0.0;
    real_t tick_max_strain = 0.0;
    real_t tick_peak_speed_sq = 0.0;
    int32_t broken_beam_count = 0;
    int32_t fault_count = 0;
    double plastic_work = 0.0;
    double damper_work = 0.0;
    double step_time_us = 0.0;

    bool wall_enabled = false;
    Vector3 wall_normal = Vector3(0.0, 0.0, -1.0);
    real_t wall_offset = 0.0;
    real_t wall_friction = 0.0;
    Vector3 wall_center;
    real_t wall_radius = 0.0;
    bool crash_active = false;
    real_t crash_time_left = 0.0;
    real_t crash_release_speed = real_t(0.25);
    int32_t wall_contacts = 0;
    Vector3 tick_wall_impulse;
    Vector3 wall_force;
    Vector3 wall_impulse_total;
    real_t wall_peak_force = 0.0;

    mutable Diagnostics diagnostics;
    mutable bool diagnostics_dirty = true;
    mutable Quaternion frame_rotation;

    void simulate_tick();
    void integrate(real_t h);
    void solve_beams(real_t inv_h2, int32_t iteration, bool reverse);
    void solve_wall(real_t h);
    void update_beams(real_t h);
    void update_crash();
    void remove_mean_velocity();

    void refresh_compliance();
    void enter_sleep();
    void touch(real_t max_delta_v_squared);
    real_t total_kinetic_energy() const;

    void compute_diagnostics() const;
    void ensure_diagnostics() const;

    void clear_data();
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

    void clear_structure();
    int32_t add_node(const Vector3 &position, real_t mass);
    int32_t add_beam(
        int32_t a,
        int32_t b,
        real_t stiffness_scale,
        real_t yield_scale,
        real_t break_scale
    );
    void set_beam_scales(
        int32_t beam_index,
        real_t stiffness_scale,
        real_t damping_scale,
        real_t yield_scale,
        real_t break_scale,
        real_t plastic_scale
    );
    void finish_build();

    void set_node_mass(int32_t node_index, real_t mass);
    real_t get_node_mass(int32_t node_index) const;
    void set_total_mass(real_t mass);
    void set_linear_velocity(const Vector3 &velocity);

    Vector3 apply_impulse(
        const Vector3 &local_position,
        const Vector3 &impulse,
        real_t radius
    );
    void apply_node_impulse(int32_t node_index, const Vector3 &impulse);
    real_t apply_impact(
        const Vector3 &local_position,
        const Vector3 &direction,
        real_t delta_v,
        int32_t spread_nodes,
        bool balance
    );

    real_t apply_crash(
        const Vector3 &local_point,
        const Vector3 &inward_normal,
        real_t speed,
        real_t duration,
        real_t contact_radius,
        real_t friction
    );
    bool is_crashing() const;

    void reset_to_rest();
    void wake();
    bool is_sleeping() const;

    void set_wall(bool enabled, const Vector3 &normal, real_t offset, real_t friction);
    real_t get_wall_force() const;
    real_t get_wall_peak_force() const;
    Vector3 get_wall_impulse() const;
    int32_t get_wall_contact_count() const;

    void set_gravity(const Vector3 &value);
    Vector3 get_gravity() const;

    void set_global_drag(real_t value);
    real_t get_global_drag() const;

    void set_settle_damping(real_t value);
    real_t get_settle_damping() const;

    void set_beam_damping(real_t value);
    real_t get_beam_damping() const;

    void set_solver_iterations(int32_t value);
    int32_t get_solver_iterations() const;

    void set_substeps(int32_t value);
    int32_t get_substeps() const;
    int32_t get_active_substeps() const;

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

    void set_hardening(real_t value);
    real_t get_hardening() const;

    void set_buckle_ratio(real_t value);
    real_t get_buckle_ratio() const;

    void set_crush_limit(real_t value);
    real_t get_crush_limit() const;

    void set_stretch_limit(real_t value);
    real_t get_stretch_limit() const;

    void set_plastic_hold(real_t value);
    real_t get_plastic_hold() const;

    void set_lock_momentum(bool value);
    bool get_lock_momentum() const;

    void set_sleep_enabled(bool value);
    bool get_sleep_enabled() const;

    void set_sleep_speed(real_t value);
    real_t get_sleep_speed() const;

    void set_sleep_ticks(int32_t value);
    int32_t get_sleep_ticks() const;

    void set_time_budget_us(real_t value);
    real_t get_time_budget_us() const;

    void set_crash_release_speed(real_t value);
    real_t get_crash_release_speed() const;

    PackedVector3Array get_node_positions() const;
    PackedInt32Array get_beam_indices() const;
    PackedFloat32Array get_node_damage() const;

    int32_t get_node_count() const;
    int32_t get_beam_count() const;
    int32_t get_broken_beam_count() const;
    real_t get_max_strain() const;
    int32_t get_fault_count() const;
    real_t get_step_time_us() const;
    Dictionary get_stats() const;

    real_t get_total_mass() const;
    Vector3 get_center_of_mass() const;
    Vector3 get_rest_center_of_mass() const;
    Vector3 get_linear_velocity() const;
    Vector3 get_linear_momentum() const;
    Vector3 get_angular_momentum() const;
    real_t get_kinetic_energy() const;
    real_t get_internal_kinetic_energy() const;
    real_t get_elastic_energy() const;
    real_t get_plastic_work() const;
    real_t get_damper_work() const;

    Transform3D get_rigid_frame() const;
    PackedVector3Array get_node_deformations() const;
    real_t get_rms_deformation() const;
    real_t get_max_deformation() const;

    real_t get_max_strain_post_solve() const;
    int32_t get_yielded_beam_count() const;

    void _physics_process(double delta) override;
};

}
