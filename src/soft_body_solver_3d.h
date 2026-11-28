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
        real_t saved_inverse_mass = 1.0;
        int32_t part = 0;
        bool detached = false;
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
        bool compression_only = false;
    };

    struct BreakEvent {
        int32_t beam = 0;
        int32_t part_a = 0;
        int32_t part_b = 0;
        Vector3 position;
    };

    struct DetachEvent {
        int32_t part = 0;
        Vector3 position;
        Vector3 velocity;
    };

    struct PartState {
        int32_t node_count = 0;
        int32_t joints_total = 0;
        int32_t joints_alive = 0;
        bool detached = false;
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

    enum ColliderType : uint8_t {
        COLLIDER_PLANE,
        COLLIDER_SPHERE,
        COLLIDER_CAPSULE,
        COLLIDER_BOX
    };

    struct Collider {
        int32_t id = 0;
        ColliderType type = COLLIDER_PLANE;
        Vector3 position;
        Basis basis;
        Vector3 local_normal = Vector3(0.0, 1.0, 0.0);
        Vector3 local_segment;
        Vector3 extents;
        real_t radius = 0.0;
        Vector3 velocity;
        real_t friction = real_t(0.3);
        real_t life = 0.0;
        Vector3 tick_impulse;
        Vector3 impulse;
    };

    struct SkinTriangle {
        int32_t a = 0;
        int32_t b = 0;
        int32_t c = 0;
    };

    struct SelfPair {
        int32_t node = 0;
        int32_t other = 0;
    };

    struct SkinEdge {
        int32_t a = 0;
        int32_t b = 0;
    };

    struct TriEntry {
        int32_t triangle = 0;
        int32_t next = -1;
    };

    static constexpr int32_t MAX_NODES = 4096;
    static constexpr int32_t MAX_BEAMS = 16384;
    static constexpr int32_t MAX_TICKS_PER_FRAME = 4;
    static constexpr int32_t MIN_ACTIVE_SUBSTEPS = 2;
    static constexpr int32_t MAX_PARTS = 64;
    static constexpr int32_t MAX_BREAK_EVENTS = 64;
    static constexpr int32_t MAX_COLLIDERS = 32;
    static constexpr int32_t MAX_SKIN_TRIANGLES = 8192;
    static constexpr int32_t MAX_SELF_PAIRS = 8192;
    static constexpr int32_t SELF_HASH_SIZE = 4096;

    std::vector<Particle> particles;
    std::vector<Beam> beams;
    std::vector<Vector3> good_position;
    std::vector<Vector3> good_velocity;
    std::vector<std::pair<real_t, int32_t>> scratch_order;
    std::vector<real_t> scratch_weights;
    std::vector<PartState> parts;
    std::vector<BreakEvent> pending_breaks;
    std::vector<DetachEvent> pending_detaches;
    std::vector<int32_t> detach_queue;
    bool parts_dirty = true;
    int32_t detached_part_count = 0;
    int32_t dropped_break_events = 0;

    Vector3 gravity = Vector3(0.0, -9.81, 0.0);
    real_t fixed_step = real_t(1.0 / 60.0);
    int32_t substeps = 8;
    int32_t active_substeps = 8;
    int32_t solver_iterations = 1;
    int32_t effective_iterations = 1;
    bool auto_stabilize = true;
    bool stability_dirty = true;
    real_t omega_max = 0.0;
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

    std::vector<Collider> colliders;
    int32_t next_collider_id = 1;
    real_t contact_margin = real_t(0.02);
    int32_t contact_count = 0;
    real_t tick_contact_depth = 0.0;
    real_t contact_peak_force = 0.0;
    Vector3 tick_contact_impulse;
    Vector3 contact_force;

    bool self_collision = false;
    real_t self_collision_radius = 0.0;
    real_t self_collision_thickness = real_t(0.03);
    real_t self_collision_friction = real_t(0.2);
    real_t auto_self_radius = 0.0;
    std::vector<SkinTriangle> skin;
    std::vector<SelfPair> node_pairs;
    std::vector<SelfPair> triangle_pairs;
    std::vector<int32_t> hash_node_head;
    std::vector<int32_t> hash_node_next;
    std::vector<int32_t> hash_triangle_head;
    std::vector<TriEntry> hash_triangle_entries;
    std::vector<int32_t> triangle_mark;
    int32_t mark_counter = 0;
    int32_t self_contact_count = 0;
    int32_t self_overflow = 0;
    int32_t self_idle_ticks = 0;
    bool self_collision_edges = true;
    bool skin_edges_dirty = true;
    std::vector<SkinEdge> skin_edges;
    std::vector<SelfPair> edge_pairs;
    std::vector<int32_t> hash_edge_head;
    std::vector<TriEntry> hash_edge_entries;
    std::vector<int32_t> edge_mark;
    std::vector<Vector3> edge_low;
    std::vector<Vector3> edge_high;
    int32_t edge_mark_counter = 0;

    double energy_last = 0.0;
    double energy_dissipated = 0.0;
    double energy_injected = 0.0;
    double absorbed_energy_tick = 0.0;
    Vector3 tick_com;
    Vector3 tick_net_torque;
    Vector3 net_contact_force;
    Vector3 net_contact_torque;

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
    double elastic_energy() const;

    bool collide_point(const Collider &collider, const Vector3 &point, Vector3 &normal, real_t &depth) const;
    Collider *find_collider(int32_t id);
    int32_t push_collider(Collider collider);
    void solve_colliders(real_t h);
    void age_colliders();
    bool collider_touches_body(const Collider &collider) const;
    real_t self_radius();
    void build_self_contacts(real_t travel);
    void rebuild_skin_edges();
    void solve_self_collision(real_t h);

    void rebuild_parts();
    void register_break(int32_t beam_index);
    void process_detachments();
    void emit_pending_events();

    void refresh_compliance();
    void compute_stability();
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

    Dictionary get_damage_state() const;
    bool set_damage_state(const Dictionary &state);

    int32_t add_support_beam(int32_t a, int32_t b, real_t stiffness_scale, real_t yield_scale);
    void set_beam_compression_only(int32_t beam_index, bool value);
    bool is_beam_compression_only(int32_t beam_index) const;

    real_t get_absorbed_energy() const;
    real_t get_injected_energy() const;
    real_t get_remaining_energy() const;
    real_t get_rebound_speed_limit() const;
    real_t get_absorbed_energy_last_tick() const;
    real_t get_absorbed_delta_v() const;
    Vector3 get_net_contact_force() const;
    Vector3 get_net_contact_torque() const;

    int32_t add_plane_collider(
        const Vector3 &point,
        const Vector3 &normal,
        real_t extent,
        real_t friction,
        real_t life
    );
    int32_t add_sphere_collider(const Vector3 &center, real_t radius, real_t friction, real_t life);
    int32_t add_capsule_collider(
        const Vector3 &from,
        const Vector3 &to,
        real_t radius,
        real_t friction,
        real_t life
    );
    int32_t add_box_collider(
        const Vector3 &center,
        const Basis &orientation,
        const Vector3 &half_extents,
        real_t friction,
        real_t life
    );
    void update_collider(
        int32_t id,
        const Vector3 &position,
        const Basis &orientation,
        const Vector3 &velocity
    );
    void remove_collider(int32_t id);
    void clear_colliders();
    int32_t get_collider_count() const;
    Vector3 get_collider_force(int32_t id) const;
    int32_t get_contact_count() const;
    Vector3 get_contact_force() const;
    real_t get_contact_peak_force() const;

    int32_t add_collision_triangle(int32_t a, int32_t b, int32_t c);
    void set_collision_triangles(const PackedInt32Array &indices);
    int32_t build_skin_from_mesh(const PackedVector3Array &vertices, const PackedInt32Array &indices);
    void clear_collision_triangles();
    int32_t get_collision_triangle_count() const;
    int32_t get_self_contact_count() const;

    void set_self_collision(bool value);
    bool get_self_collision() const;
    void set_self_collision_radius(real_t value);
    real_t get_self_collision_radius() const;
    void set_self_collision_thickness(real_t value);
    real_t get_self_collision_thickness() const;
    void set_self_collision_friction(real_t value);
    real_t get_self_collision_friction() const;
    void set_self_collision_edges(bool value);
    bool get_self_collision_edges() const;
    int32_t get_skin_edge_count() const;
    void set_contact_margin(real_t value);
    real_t get_contact_margin() const;

    void set_node_part(int32_t node_index, int32_t part);
    void set_nodes_part(const PackedInt32Array &node_indices, int32_t part);
    int32_t get_node_part(int32_t node_index) const;
    PackedInt32Array get_node_parts() const;
    PackedInt32Array get_part_nodes(int32_t part) const;
    int32_t get_part_count() const;
    void set_part_joint_strength(
        int32_t part,
        real_t stiffness_scale,
        real_t yield_scale,
        real_t break_scale
    );
    real_t get_part_attachment(int32_t part) const;
    real_t get_part_damage(int32_t part) const;
    bool is_part_detached(int32_t part) const;
    int32_t get_detached_part_count() const;

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

    void set_auto_stabilize(bool value);
    bool get_auto_stabilize() const;
    real_t get_stability_ratio() const;
    int32_t get_effective_iterations() const;

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
