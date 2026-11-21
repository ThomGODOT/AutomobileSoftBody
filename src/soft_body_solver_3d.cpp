#include "soft_body_solver_3d.h"

#include <algorithm>
#include <cmath>
#include <cstddef>

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/string.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

SoftBodySolver3D::SoftBodySolver3D() {
    set_physics_process(false);
}

SoftBodySolver3D::~SoftBodySolver3D() {
}

void SoftBodySolver3D::_bind_methods() {
    ClassDB::bind_method(
        D_METHOD("create_box", "size", "center", "nodes_x", "nodes_y", "nodes_z"),
        &SoftBodySolver3D::create_box
    );
    ClassDB::bind_method(D_METHOD("clear_structure"), &SoftBodySolver3D::clear_structure);
    ClassDB::bind_method(
        D_METHOD("add_node", "position", "mass"),
        &SoftBodySolver3D::add_node,
        DEFVAL(1.0)
    );
    ClassDB::bind_method(
        D_METHOD("add_beam", "a", "b", "stiffness_scale", "yield_scale", "break_scale"),
        &SoftBodySolver3D::add_beam,
        DEFVAL(1.0),
        DEFVAL(1.0),
        DEFVAL(1.0)
    );
    ClassDB::bind_method(
        D_METHOD("set_beam_scales", "beam_index", "stiffness_scale", "damping_scale",
                 "yield_scale", "break_scale", "plastic_scale"),
        &SoftBodySolver3D::set_beam_scales
    );
    ClassDB::bind_method(D_METHOD("finish_build"), &SoftBodySolver3D::finish_build);

    ClassDB::bind_method(D_METHOD("set_node_mass", "node_index", "mass"), &SoftBodySolver3D::set_node_mass);
    ClassDB::bind_method(D_METHOD("get_node_mass", "node_index"), &SoftBodySolver3D::get_node_mass);
    ClassDB::bind_method(D_METHOD("set_total_mass", "mass"), &SoftBodySolver3D::set_total_mass);
    ClassDB::bind_method(
        D_METHOD("set_linear_velocity", "velocity"),
        &SoftBodySolver3D::set_linear_velocity
    );
    ClassDB::bind_method(
        D_METHOD("apply_impulse", "local_position", "impulse", "radius"),
        &SoftBodySolver3D::apply_impulse,
        DEFVAL(0.0)
    );
    ClassDB::bind_method(
        D_METHOD("apply_node_impulse", "node_index", "impulse"),
        &SoftBodySolver3D::apply_node_impulse
    );
    ClassDB::bind_method(
        D_METHOD("apply_impact", "local_position", "direction", "delta_v", "spread_nodes", "balance"),
        &SoftBodySolver3D::apply_impact,
        DEFVAL(3),
        DEFVAL(true)
    );
    ClassDB::bind_method(
        D_METHOD("apply_crash", "local_point", "inward_normal", "speed", "duration",
                 "contact_radius", "friction"),
        &SoftBodySolver3D::apply_crash,
        DEFVAL(0.4),
        DEFVAL(0.0),
        DEFVAL(0.2)
    );
    ClassDB::bind_method(D_METHOD("is_crashing"), &SoftBodySolver3D::is_crashing);
    ClassDB::bind_method(D_METHOD("reset_to_rest"), &SoftBodySolver3D::reset_to_rest);
    ClassDB::bind_method(D_METHOD("wake"), &SoftBodySolver3D::wake);
    ClassDB::bind_method(D_METHOD("is_sleeping"), &SoftBodySolver3D::is_sleeping);

    ClassDB::bind_method(
        D_METHOD("set_wall", "enabled", "normal", "offset", "friction"),
        &SoftBodySolver3D::set_wall,
        DEFVAL(0.0)
    );
    ClassDB::bind_method(D_METHOD("get_wall_force"), &SoftBodySolver3D::get_wall_force);
    ClassDB::bind_method(D_METHOD("get_wall_peak_force"), &SoftBodySolver3D::get_wall_peak_force);
    ClassDB::bind_method(D_METHOD("get_wall_impulse"), &SoftBodySolver3D::get_wall_impulse);
    ClassDB::bind_method(
        D_METHOD("get_wall_contact_count"),
        &SoftBodySolver3D::get_wall_contact_count
    );

    ClassDB::bind_method(D_METHOD("set_gravity", "value"), &SoftBodySolver3D::set_gravity);
    ClassDB::bind_method(D_METHOD("get_gravity"), &SoftBodySolver3D::get_gravity);

    ClassDB::bind_method(D_METHOD("set_global_drag", "value"), &SoftBodySolver3D::set_global_drag);
    ClassDB::bind_method(D_METHOD("get_global_drag"), &SoftBodySolver3D::get_global_drag);

    ClassDB::bind_method(
        D_METHOD("set_settle_damping", "value"),
        &SoftBodySolver3D::set_settle_damping
    );
    ClassDB::bind_method(D_METHOD("get_settle_damping"), &SoftBodySolver3D::get_settle_damping);

    ClassDB::bind_method(D_METHOD("set_beam_damping", "value"), &SoftBodySolver3D::set_beam_damping);
    ClassDB::bind_method(D_METHOD("get_beam_damping"), &SoftBodySolver3D::get_beam_damping);

    ClassDB::bind_method(
        D_METHOD("set_solver_iterations", "value"),
        &SoftBodySolver3D::set_solver_iterations
    );
    ClassDB::bind_method(
        D_METHOD("get_solver_iterations"),
        &SoftBodySolver3D::get_solver_iterations
    );

    ClassDB::bind_method(
        D_METHOD("set_auto_stabilize", "value"),
        &SoftBodySolver3D::set_auto_stabilize
    );
    ClassDB::bind_method(D_METHOD("get_auto_stabilize"), &SoftBodySolver3D::get_auto_stabilize);
    ClassDB::bind_method(D_METHOD("get_stability_ratio"), &SoftBodySolver3D::get_stability_ratio);
    ClassDB::bind_method(
        D_METHOD("get_effective_iterations"),
        &SoftBodySolver3D::get_effective_iterations
    );

    ClassDB::bind_method(D_METHOD("set_substeps", "value"), &SoftBodySolver3D::set_substeps);
    ClassDB::bind_method(D_METHOD("get_substeps"), &SoftBodySolver3D::get_substeps);
    ClassDB::bind_method(D_METHOD("get_active_substeps"), &SoftBodySolver3D::get_active_substeps);

    ClassDB::bind_method(D_METHOD("set_fixed_step", "value"), &SoftBodySolver3D::set_fixed_step);
    ClassDB::bind_method(D_METHOD("get_fixed_step"), &SoftBodySolver3D::get_fixed_step);

    ClassDB::bind_method(D_METHOD("set_max_speed", "value"), &SoftBodySolver3D::set_max_speed);
    ClassDB::bind_method(D_METHOD("get_max_speed"), &SoftBodySolver3D::get_max_speed);

    ClassDB::bind_method(
        D_METHOD("set_beam_stiffness", "value"),
        &SoftBodySolver3D::set_beam_stiffness
    );
    ClassDB::bind_method(D_METHOD("get_beam_stiffness"), &SoftBodySolver3D::get_beam_stiffness);

    ClassDB::bind_method(D_METHOD("set_yield_strain", "value"), &SoftBodySolver3D::set_yield_strain);
    ClassDB::bind_method(D_METHOD("get_yield_strain"), &SoftBodySolver3D::get_yield_strain);

    ClassDB::bind_method(D_METHOD("set_break_strain", "value"), &SoftBodySolver3D::set_break_strain);
    ClassDB::bind_method(D_METHOD("get_break_strain"), &SoftBodySolver3D::get_break_strain);

    ClassDB::bind_method(D_METHOD("set_break_hold", "value"), &SoftBodySolver3D::set_break_hold);
    ClassDB::bind_method(D_METHOD("get_break_hold"), &SoftBodySolver3D::get_break_hold);

    ClassDB::bind_method(D_METHOD("set_plastic_rate", "value"), &SoftBodySolver3D::set_plastic_rate);
    ClassDB::bind_method(D_METHOD("get_plastic_rate"), &SoftBodySolver3D::get_plastic_rate);

    ClassDB::bind_method(D_METHOD("set_hardening", "value"), &SoftBodySolver3D::set_hardening);
    ClassDB::bind_method(D_METHOD("get_hardening"), &SoftBodySolver3D::get_hardening);

    ClassDB::bind_method(D_METHOD("set_buckle_ratio", "value"), &SoftBodySolver3D::set_buckle_ratio);
    ClassDB::bind_method(D_METHOD("get_buckle_ratio"), &SoftBodySolver3D::get_buckle_ratio);

    ClassDB::bind_method(D_METHOD("set_crush_limit", "value"), &SoftBodySolver3D::set_crush_limit);
    ClassDB::bind_method(D_METHOD("get_crush_limit"), &SoftBodySolver3D::get_crush_limit);

    ClassDB::bind_method(D_METHOD("set_stretch_limit", "value"), &SoftBodySolver3D::set_stretch_limit);
    ClassDB::bind_method(D_METHOD("get_stretch_limit"), &SoftBodySolver3D::get_stretch_limit);

    ClassDB::bind_method(D_METHOD("set_plastic_hold", "value"), &SoftBodySolver3D::set_plastic_hold);
    ClassDB::bind_method(D_METHOD("get_plastic_hold"), &SoftBodySolver3D::get_plastic_hold);

    ClassDB::bind_method(D_METHOD("set_lock_momentum", "value"), &SoftBodySolver3D::set_lock_momentum);
    ClassDB::bind_method(D_METHOD("get_lock_momentum"), &SoftBodySolver3D::get_lock_momentum);

    ClassDB::bind_method(D_METHOD("set_sleep_enabled", "value"), &SoftBodySolver3D::set_sleep_enabled);
    ClassDB::bind_method(D_METHOD("get_sleep_enabled"), &SoftBodySolver3D::get_sleep_enabled);

    ClassDB::bind_method(D_METHOD("set_sleep_speed", "value"), &SoftBodySolver3D::set_sleep_speed);
    ClassDB::bind_method(D_METHOD("get_sleep_speed"), &SoftBodySolver3D::get_sleep_speed);

    ClassDB::bind_method(D_METHOD("set_sleep_ticks", "value"), &SoftBodySolver3D::set_sleep_ticks);
    ClassDB::bind_method(D_METHOD("get_sleep_ticks"), &SoftBodySolver3D::get_sleep_ticks);

    ClassDB::bind_method(
        D_METHOD("set_time_budget_us", "value"),
        &SoftBodySolver3D::set_time_budget_us
    );
    ClassDB::bind_method(D_METHOD("get_time_budget_us"), &SoftBodySolver3D::get_time_budget_us);

    ClassDB::bind_method(
        D_METHOD("set_crash_release_speed", "value"),
        &SoftBodySolver3D::set_crash_release_speed
    );
    ClassDB::bind_method(
        D_METHOD("get_crash_release_speed"),
        &SoftBodySolver3D::get_crash_release_speed
    );

    ClassDB::bind_method(D_METHOD("get_node_positions"), &SoftBodySolver3D::get_node_positions);
    ClassDB::bind_method(D_METHOD("get_beam_indices"), &SoftBodySolver3D::get_beam_indices);
    ClassDB::bind_method(D_METHOD("get_node_damage"), &SoftBodySolver3D::get_node_damage);
    ClassDB::bind_method(D_METHOD("get_node_count"), &SoftBodySolver3D::get_node_count);
    ClassDB::bind_method(D_METHOD("get_beam_count"), &SoftBodySolver3D::get_beam_count);
    ClassDB::bind_method(
        D_METHOD("get_broken_beam_count"),
        &SoftBodySolver3D::get_broken_beam_count
    );
    ClassDB::bind_method(D_METHOD("get_max_strain"), &SoftBodySolver3D::get_max_strain);
    ClassDB::bind_method(D_METHOD("get_fault_count"), &SoftBodySolver3D::get_fault_count);
    ClassDB::bind_method(D_METHOD("get_step_time_us"), &SoftBodySolver3D::get_step_time_us);
    ClassDB::bind_method(D_METHOD("get_stats"), &SoftBodySolver3D::get_stats);

    ClassDB::bind_method(D_METHOD("get_total_mass"), &SoftBodySolver3D::get_total_mass);
    ClassDB::bind_method(D_METHOD("get_center_of_mass"), &SoftBodySolver3D::get_center_of_mass);
    ClassDB::bind_method(
        D_METHOD("get_rest_center_of_mass"),
        &SoftBodySolver3D::get_rest_center_of_mass
    );
    ClassDB::bind_method(D_METHOD("get_linear_velocity"), &SoftBodySolver3D::get_linear_velocity);
    ClassDB::bind_method(D_METHOD("get_linear_momentum"), &SoftBodySolver3D::get_linear_momentum);
    ClassDB::bind_method(
        D_METHOD("get_angular_momentum"),
        &SoftBodySolver3D::get_angular_momentum
    );
    ClassDB::bind_method(D_METHOD("get_kinetic_energy"), &SoftBodySolver3D::get_kinetic_energy);
    ClassDB::bind_method(
        D_METHOD("get_internal_kinetic_energy"),
        &SoftBodySolver3D::get_internal_kinetic_energy
    );
    ClassDB::bind_method(D_METHOD("get_elastic_energy"), &SoftBodySolver3D::get_elastic_energy);
    ClassDB::bind_method(D_METHOD("get_plastic_work"), &SoftBodySolver3D::get_plastic_work);
    ClassDB::bind_method(D_METHOD("get_damper_work"), &SoftBodySolver3D::get_damper_work);
    ClassDB::bind_method(D_METHOD("get_rigid_frame"), &SoftBodySolver3D::get_rigid_frame);
    ClassDB::bind_method(
        D_METHOD("get_node_deformations"),
        &SoftBodySolver3D::get_node_deformations
    );
    ClassDB::bind_method(D_METHOD("get_rms_deformation"), &SoftBodySolver3D::get_rms_deformation);
    ClassDB::bind_method(D_METHOD("get_max_deformation"), &SoftBodySolver3D::get_max_deformation);
    ClassDB::bind_method(
        D_METHOD("get_max_strain_post_solve"),
        &SoftBodySolver3D::get_max_strain_post_solve
    );
    ClassDB::bind_method(
        D_METHOD("get_yielded_beam_count"),
        &SoftBodySolver3D::get_yielded_beam_count
    );

    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "global_drag", PROPERTY_HINT_RANGE, "0,10,0.01"),
        "set_global_drag",
        "get_global_drag"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "settle_damping", PROPERTY_HINT_RANGE, "0,60,0.1"),
        "set_settle_damping",
        "get_settle_damping"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "beam_damping", PROPERTY_HINT_RANGE, "0,200,0.5"),
        "set_beam_damping",
        "get_beam_damping"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "solver_iterations", PROPERTY_HINT_RANGE, "1,8,1"),
        "set_solver_iterations",
        "get_solver_iterations"
    );
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "auto_stabilize"), "set_auto_stabilize", "get_auto_stabilize");
    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "substeps", PROPERTY_HINT_RANGE, "1,32,1"),
        "set_substeps",
        "get_substeps"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "fixed_step", PROPERTY_HINT_RANGE, "0.004,0.034,0.0001"),
        "set_fixed_step",
        "get_fixed_step"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "max_speed", PROPERTY_HINT_RANGE, "1,1000,1"),
        "set_max_speed",
        "get_max_speed"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "beam_stiffness", PROPERTY_HINT_RANGE, "1000,100000000,1000,exp"),
        "set_beam_stiffness",
        "get_beam_stiffness"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "yield_strain", PROPERTY_HINT_RANGE, "0.001,1,0.001"),
        "set_yield_strain",
        "get_yield_strain"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "break_strain", PROPERTY_HINT_RANGE, "0.01,10,0.01"),
        "set_break_strain",
        "get_break_strain"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "break_hold", PROPERTY_HINT_RANGE, "0,0.1,0.001"),
        "set_break_hold",
        "get_break_hold"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "plastic_rate", PROPERTY_HINT_RANGE, "0,2000,1"),
        "set_plastic_rate",
        "get_plastic_rate"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "hardening", PROPERTY_HINT_RANGE, "0,5,0.01"),
        "set_hardening",
        "get_hardening"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "buckle_ratio", PROPERTY_HINT_RANGE, "0.1,1,0.01"),
        "set_buckle_ratio",
        "get_buckle_ratio"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "crush_limit", PROPERTY_HINT_RANGE, "0.1,0.95,0.01"),
        "set_crush_limit",
        "get_crush_limit"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "stretch_limit", PROPERTY_HINT_RANGE, "1.05,3,0.01"),
        "set_stretch_limit",
        "get_stretch_limit"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "plastic_hold", PROPERTY_HINT_RANGE, "0,5,0.05"),
        "set_plastic_hold",
        "get_plastic_hold"
    );
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "lock_momentum"), "set_lock_momentum", "get_lock_momentum");
    ADD_PROPERTY(PropertyInfo(Variant::BOOL, "sleep_enabled"), "set_sleep_enabled", "get_sleep_enabled");
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "sleep_speed", PROPERTY_HINT_RANGE, "0.001,1,0.001"),
        "set_sleep_speed",
        "get_sleep_speed"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "sleep_ticks", PROPERTY_HINT_RANGE, "1,600,1"),
        "set_sleep_ticks",
        "get_sleep_ticks"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "time_budget_us", PROPERTY_HINT_RANGE, "0,20000,50"),
        "set_time_budget_us",
        "get_time_budget_us"
    );
    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "crash_release_speed", PROPERTY_HINT_RANGE, "0.01,5,0.01"),
        "set_crash_release_speed",
        "get_crash_release_speed"
    );
}

void SoftBodySolver3D::clear_data() {
    particles.clear();
    beams.clear();
    good_position.clear();
    good_velocity.clear();
    scratch_order.clear();
    scratch_weights.clear();
    accumulator = 0.0;
    substep_counter = 0;
    max_strain = 0.0;
    tick_max_strain = 0.0;
    tick_peak_speed_sq = 0.0;
    broken_beam_count = 0;
    fault_count = 0;
    plastic_work = 0.0;
    damper_work = 0.0;
    wall_contacts = 0;
    tick_wall_impulse = Vector3();
    wall_force = Vector3();
    wall_impulse_total = Vector3();
    wall_peak_force = 0.0;
    wall_enabled = false;
    wall_radius = 0.0;
    crash_active = false;
    crash_time_left = 0.0;
    plastic_timer = 0.0;
    plastic_allowed = false;
    frame_rotation = Quaternion();
    diagnostics = Diagnostics();
    diagnostics_dirty = true;
    sleeping = false;
    sleep_counter = 0;
    budget_frames = 0;
    active_substeps = substeps;
}

void SoftBodySolver3D::clear_body() {
    clear_data();
    set_physics_process(false);
}

void SoftBodySolver3D::clear_structure() {
    clear_body();
}

void SoftBodySolver3D::refresh_compliance() {
    for (Beam &beam : beams) {
        const real_t k = std::max(beam_stiffness * beam.stiffness_scale, real_t(1.0));
        beam.compliance = real_t(1.0) / k;
    }

    stability_dirty = true;
}

void SoftBodySolver3D::compute_stability() {
    std::vector<real_t> stiffness_sum(particles.size(), real_t(0.0));

    for (const Beam &beam : beams) {
        if (beam.broken) {
            continue;
        }

        const real_t k = real_t(1.0) / std::max(beam.compliance, real_t(1.0e-12));
        stiffness_sum[beam.a] += k;
        stiffness_sum[beam.b] += k;
    }

    real_t worst = 0.0;

    for (size_t i = 0; i < particles.size(); ++i) {
        worst = std::max(worst, stiffness_sum[i] * particles[i].inverse_mass);
    }

    omega_max = std::sqrt(worst);
    stability_dirty = false;
}

void SoftBodySolver3D::enter_sleep() {
    for (Particle &particle : particles) {
        particle.velocity = Vector3();
        particle.previous_position = particle.position;
    }

    sleeping = true;
    sleep_counter = 0;
    accumulator = 0.0;
    diagnostics_dirty = true;
    set_physics_process(false);
}

void SoftBodySolver3D::touch(real_t p_max_delta_v_squared) {
    if (p_max_delta_v_squared > real_t(0.25)) {
        plastic_timer = plastic_hold;
    }

    const real_t threshold = sleep_speed * real_t(0.5);

    if (p_max_delta_v_squared <= threshold * threshold) {
        return;
    }

    sleep_counter = 0;

    if (sleeping) {
        wake();
    }
}

void SoftBodySolver3D::wake() {
    sleep_counter = 0;

    if (particles.empty()) {
        return;
    }

    if (sleeping) {
        sleeping = false;
        accumulator = 0.0;
    }

    set_physics_process(true);
}

bool SoftBodySolver3D::is_sleeping() const {
    return sleeping;
}

real_t SoftBodySolver3D::total_kinetic_energy() const {
    real_t kinetic = 0.0;

    for (const Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        kinetic += real_t(0.5) * particle.velocity.length_squared() / particle.inverse_mass;
    }

    return kinetic;
}

int32_t SoftBodySolver3D::add_node(const Vector3 &p_position, real_t p_mass) {
    if (static_cast<int32_t>(particles.size()) >= MAX_NODES) {
        return -1;
    }

    Particle particle;
    particle.position = p_position;
    particle.previous_position = p_position;
    particle.rest_position = p_position;
    particle.inverse_mass = p_mass <= real_t(0.0) ? real_t(0.0)
                                                  : real_t(1.0) / std::max(p_mass, real_t(0.001));
    particles.push_back(particle);

    diagnostics_dirty = true;
    return static_cast<int32_t>(particles.size()) - 1;
}

int32_t SoftBodySolver3D::add_beam(
    int32_t p_a,
    int32_t p_b,
    real_t p_stiffness_scale,
    real_t p_yield_scale,
    real_t p_break_scale
) {
    const int32_t node_count = static_cast<int32_t>(particles.size());

    if (p_a < 0 || p_b < 0 || p_a >= node_count || p_b >= node_count || p_a == p_b) {
        return -1;
    }
    if (static_cast<int32_t>(beams.size()) >= MAX_BEAMS) {
        return -1;
    }

    const real_t length = particles[p_a].position.distance_to(particles[p_b].position);
    if (length <= real_t(0.0001)) {
        return -1;
    }

    Beam beam;
    beam.a = p_a;
    beam.b = p_b;
    beam.rest_length = length;
    beam.initial_rest_length = length;
    beam.stiffness_scale = std::max(p_stiffness_scale, real_t(0.001));
    beam.yield_scale = std::max(p_yield_scale, real_t(0.001));
    beam.break_scale = std::max(p_break_scale, real_t(0.001));
    beam.compliance = real_t(1.0) / std::max(beam_stiffness * beam.stiffness_scale, real_t(1.0));
    beams.push_back(beam);

    diagnostics_dirty = true;
    return static_cast<int32_t>(beams.size()) - 1;
}

void SoftBodySolver3D::set_beam_scales(
    int32_t p_beam_index,
    real_t p_stiffness_scale,
    real_t p_damping_scale,
    real_t p_yield_scale,
    real_t p_break_scale,
    real_t p_plastic_scale
) {
    if (p_beam_index < 0 || p_beam_index >= static_cast<int32_t>(beams.size())) {
        return;
    }

    Beam &beam = beams[p_beam_index];
    beam.stiffness_scale = std::max(p_stiffness_scale, real_t(0.001));
    beam.damping_scale = std::max(p_damping_scale, real_t(0.0));
    beam.yield_scale = std::max(p_yield_scale, real_t(0.001));
    beam.break_scale = std::max(p_break_scale, real_t(0.001));
    beam.plastic_scale = std::max(p_plastic_scale, real_t(0.0));
    beam.compliance = real_t(1.0) / std::max(beam_stiffness * beam.stiffness_scale, real_t(1.0));
    stability_dirty = true;
}

void SoftBodySolver3D::finish_build() {
    if (particles.empty()) {
        return;
    }

    good_position.resize(particles.size());
    good_velocity.resize(particles.size());

    accumulator = 0.0;
    frame_rotation = Quaternion();
    diagnostics_dirty = true;
    stability_dirty = true;
    sleeping = false;
    sleep_counter = 0;
    active_substeps = substeps;
    set_physics_process(true);
}

void SoftBodySolver3D::create_box(
    const Vector3 &p_size,
    const Vector3 &p_center,
    int32_t p_nodes_x,
    int32_t p_nodes_y,
    int32_t p_nodes_z
) {
    clear_body();

    const int32_t nx = std::clamp(p_nodes_x, 2, 6);
    const int32_t ny = std::clamp(p_nodes_y, 2, 6);
    const int32_t nz = std::clamp(p_nodes_z, 2, 6);

    const Vector3 size(
        std::max(real_t(0.05), p_size.x),
        std::max(real_t(0.05), p_size.y),
        std::max(real_t(0.05), p_size.z)
    );

    auto index = [nx, ny](int32_t x, int32_t y, int32_t z) {
        return x + nx * (y + ny * z);
    };

    particles.reserve(static_cast<size_t>(nx) * ny * nz);
    beams.reserve(static_cast<size_t>(nx) * ny * nz * 16);

    for (int32_t z = 0; z < nz; ++z) {
        for (int32_t y = 0; y < ny; ++y) {
            for (int32_t x = 0; x < nx; ++x) {
                const real_t fx = real_t(x) / real_t(nx - 1);
                const real_t fy = real_t(y) / real_t(ny - 1);
                const real_t fz = real_t(z) / real_t(nz - 1);

                add_node(
                    p_center + Vector3(
                        (fx - real_t(0.5)) * size.x,
                        (fy - real_t(0.5)) * size.y,
                        (fz - real_t(0.5)) * size.z
                    ),
                    real_t(1.0)
                );
            }
        }
    }

    auto link = [&](int32_t x, int32_t y, int32_t z,
                    int32_t dx, int32_t dy, int32_t dz,
                    real_t stiffness_scale) {
        const int32_t tx = x + dx;
        const int32_t ty = y + dy;
        const int32_t tz = z + dz;

        if (tx < 0 || tx >= nx || ty < 0 || ty >= ny || tz < 0 || tz >= nz) {
            return;
        }

        add_beam(index(x, y, z), index(tx, ty, tz), stiffness_scale, real_t(1.0), real_t(1.0));
    };

    for (int32_t z = 0; z < nz; ++z) {
        for (int32_t y = 0; y < ny; ++y) {
            for (int32_t x = 0; x < nx; ++x) {
                for (int32_t dx = 0; dx <= 1; ++dx) {
                    for (int32_t dy = -1; dy <= 1; ++dy) {
                        for (int32_t dz = -1; dz <= 1; ++dz) {
                            if (dx == 0 && dy < 0) {
                                continue;
                            }
                            if (dx == 0 && dy == 0 && dz <= 0) {
                                continue;
                            }

                            const int32_t offset_count =
                                std::abs(dx) + std::abs(dy) + std::abs(dz);

                            const real_t stiffness_scale =
                                offset_count == 1 ? real_t(0.82) :
                                offset_count == 2 ? real_t(0.58) :
                                                     real_t(0.46);

                            link(x, y, z, dx, dy, dz, stiffness_scale);
                        }
                    }
                }

                link(x, y, z, 2, 0, 0, real_t(0.18));
                link(x, y, z, 0, 2, 0, real_t(0.18));
                link(x, y, z, 0, 0, 2, real_t(0.18));
            }
        }
    }

    finish_build();
}

void SoftBodySolver3D::set_node_mass(int32_t p_node_index, real_t p_mass) {
    if (p_node_index < 0 || p_node_index >= static_cast<int32_t>(particles.size())) {
        return;
    }

    particles[p_node_index].inverse_mass =
        p_mass <= real_t(0.0) ? real_t(0.0)
                              : real_t(1.0) / std::max(p_mass, real_t(0.001));
    diagnostics_dirty = true;
    stability_dirty = true;
}

real_t SoftBodySolver3D::get_node_mass(int32_t p_node_index) const {
    if (p_node_index < 0 || p_node_index >= static_cast<int32_t>(particles.size())) {
        return 0.0;
    }

    const real_t inverse_mass = particles[p_node_index].inverse_mass;
    return inverse_mass > real_t(0.0) ? real_t(1.0) / inverse_mass : real_t(0.0);
}

void SoftBodySolver3D::set_total_mass(real_t p_mass) {
    if (p_mass <= real_t(0.0)) {
        return;
    }

    real_t current = 0.0;
    for (const Particle &particle : particles) {
        if (particle.inverse_mass > real_t(0.0)) {
            current += real_t(1.0) / particle.inverse_mass;
        }
    }

    if (current <= real_t(0.0)) {
        return;
    }

    const real_t factor = p_mass / current;
    for (Particle &particle : particles) {
        if (particle.inverse_mass > real_t(0.0)) {
            particle.inverse_mass /= factor;
        }
    }

    diagnostics_dirty = true;
    stability_dirty = true;
}

void SoftBodySolver3D::set_linear_velocity(const Vector3 &p_velocity) {
    for (Particle &particle : particles) {
        if (particle.inverse_mass > real_t(0.0)) {
            particle.velocity = p_velocity;
        }
    }

    diagnostics_dirty = true;
    wake();
}

Vector3 SoftBodySolver3D::apply_impulse(
    const Vector3 &p_local_position,
    const Vector3 &p_impulse,
    real_t p_radius
) {
    Vector3 delivered;

    if (particles.empty()) {
        return delivered;
    }

    real_t max_delta_v_squared = 0.0;

    if (p_radius <= real_t(0.0001)) {
        int32_t nearest = -1;
        real_t best = 0.0;

        for (size_t i = 0; i < particles.size(); ++i) {
            if (particles[i].inverse_mass <= real_t(0.0)) {
                continue;
            }

            const real_t distance_squared =
                particles[i].position.distance_squared_to(p_local_position);

            if (nearest < 0 || distance_squared < best) {
                nearest = static_cast<int32_t>(i);
                best = distance_squared;
            }
        }

        if (nearest < 0) {
            return delivered;
        }

        const Vector3 delta_v = p_impulse * particles[nearest].inverse_mass;
        particles[nearest].velocity += delta_v;
        max_delta_v_squared = delta_v.length_squared();
        delivered = p_impulse;
    } else {
        scratch_weights.assign(particles.size(), real_t(0.0));
        real_t weight_sum = 0.0;
        int32_t nearest = -1;
        real_t nearest_distance = 0.0;

        for (size_t i = 0; i < particles.size(); ++i) {
            if (particles[i].inverse_mass <= real_t(0.0)) {
                continue;
            }

            const real_t distance = particles[i].position.distance_to(p_local_position);

            if (nearest < 0 || distance < nearest_distance) {
                nearest = static_cast<int32_t>(i);
                nearest_distance = distance;
            }

            if (distance >= p_radius) {
                continue;
            }

            const real_t weight = real_t(1.0) - distance / p_radius;
            scratch_weights[i] = weight;
            weight_sum += weight;
        }

        if (nearest < 0) {
            return delivered;
        }

        if (weight_sum <= real_t(0.000001)) {
            scratch_weights[nearest] = real_t(1.0);
            weight_sum = real_t(1.0);
        }

        for (size_t i = 0; i < particles.size(); ++i) {
            if (scratch_weights[i] <= real_t(0.0)) {
                continue;
            }

            const Vector3 share = p_impulse * (scratch_weights[i] / weight_sum);
            const Vector3 delta_v = share * particles[i].inverse_mass;
            particles[i].velocity += delta_v;
            max_delta_v_squared = std::max(max_delta_v_squared, delta_v.length_squared());
            delivered += share;
        }
    }

    diagnostics_dirty = true;
    touch(max_delta_v_squared);
    return delivered;
}

void SoftBodySolver3D::apply_node_impulse(int32_t p_node_index, const Vector3 &p_impulse) {
    if (p_node_index < 0 || p_node_index >= static_cast<int32_t>(particles.size())) {
        return;
    }

    Particle &particle = particles[p_node_index];

    if (particle.inverse_mass <= real_t(0.0)) {
        return;
    }

    const Vector3 delta_v = p_impulse * particle.inverse_mass;
    particle.velocity += delta_v;

    diagnostics_dirty = true;
    touch(delta_v.length_squared());
}

real_t SoftBodySolver3D::apply_impact(
    const Vector3 &p_local_position,
    const Vector3 &p_direction,
    real_t p_delta_v,
    int32_t p_spread_nodes,
    bool p_balance
) {
    if (particles.empty() || p_delta_v <= real_t(0.0001) ||
        p_direction.length_squared() <= real_t(0.00000001)) {
        return 0.0;
    }

    const Vector3 direction = p_direction.normalized();
    const real_t delta_v = std::min(p_delta_v, max_speed * real_t(0.5));

    scratch_order.clear();
    scratch_order.reserve(particles.size());
    real_t total_mass = 0.0;

    for (size_t i = 0; i < particles.size(); ++i) {
        if (particles[i].inverse_mass <= real_t(0.0)) {
            continue;
        }

        total_mass += real_t(1.0) / particles[i].inverse_mass;
        scratch_order.emplace_back(
            particles[i].position.distance_squared_to(p_local_position),
            static_cast<int32_t>(i)
        );
    }

    if (scratch_order.empty()) {
        return 0.0;
    }

    const size_t count = std::min(
        static_cast<size_t>(std::clamp(p_spread_nodes, 1, 64)),
        scratch_order.size()
    );

    std::partial_sort(
        scratch_order.begin(),
        scratch_order.begin() + static_cast<std::ptrdiff_t>(count),
        scratch_order.end()
    );

    const real_t energy_before = total_kinetic_energy();

    const real_t max_weight = real_t(1.0) / (std::sqrt(scratch_order[0].first) + real_t(0.1));
    Vector3 added_momentum;
    real_t max_delta_v_squared = 0.0;

    for (size_t i = 0; i < count; ++i) {
        Particle &particle = particles[scratch_order[i].second];
        const real_t weight =
            real_t(1.0) / (std::sqrt(scratch_order[i].first) + real_t(0.1));
        const Vector3 delta = direction * (delta_v * weight / max_weight);

        particle.velocity += delta;
        added_momentum += delta / particle.inverse_mass;
        max_delta_v_squared = std::max(max_delta_v_squared, delta.length_squared());
    }

    if (p_balance && total_mass > real_t(0.0)) {
        const Vector3 shift = added_momentum / total_mass;

        for (Particle &particle : particles) {
            if (particle.inverse_mass > real_t(0.0)) {
                particle.velocity -= shift;
            }
        }
    }

    diagnostics_dirty = true;
    touch(max_delta_v_squared);

    return std::max(real_t(0.0), total_kinetic_energy() - energy_before);
}

void SoftBodySolver3D::reset_to_rest() {
    if (particles.empty()) {
        return;
    }

    for (Particle &particle : particles) {
        particle.position = particle.rest_position;
        particle.previous_position = particle.rest_position;
        particle.velocity = Vector3();
    }

    for (Beam &beam : beams) {
        beam.rest_length = beam.initial_rest_length;
        beam.lambda = 0.0;
        beam.overload = 0;
        beam.broken = false;
    }

    broken_beam_count = 0;
    fault_count = 0;
    plastic_work = 0.0;
    damper_work = 0.0;
    accumulator = 0.0;
    max_strain = 0.0;
    tick_max_strain = 0.0;
    tick_peak_speed_sq = 0.0;
    wall_contacts = 0;
    tick_wall_impulse = Vector3();
    wall_force = Vector3();
    wall_impulse_total = Vector3();
    wall_peak_force = 0.0;
    crash_active = false;
    crash_time_left = 0.0;
    plastic_timer = 0.0;
    frame_rotation = Quaternion();
    diagnostics_dirty = true;
    active_substeps = substeps;
    wake();
}

void SoftBodySolver3D::set_wall(
    bool p_enabled,
    const Vector3 &p_normal,
    real_t p_offset,
    real_t p_friction
) {
    if (p_normal.length_squared() <= real_t(0.000001)) {
        wall_enabled = false;
        return;
    }

    wall_enabled = p_enabled;
    wall_normal = p_normal.normalized();
    wall_offset = p_offset;
    wall_friction = std::clamp(p_friction, real_t(0.0), real_t(2.0));
    wall_radius = 0.0;
    crash_active = false;
    wake();
}

real_t SoftBodySolver3D::apply_crash(
    const Vector3 &p_local_point,
    const Vector3 &p_inward_normal,
    real_t p_speed,
    real_t p_duration,
    real_t p_contact_radius,
    real_t p_friction
) {
    if (particles.empty() || p_speed <= real_t(0.0001) ||
        p_inward_normal.length_squared() <= real_t(0.00000001)) {
        return 0.0;
    }

    const Vector3 normal = p_inward_normal.normalized();
    const real_t speed = std::min(p_speed, max_speed * real_t(0.5));
    const real_t radius = std::max(p_contact_radius, real_t(0.0));
    const real_t radius_squared = radius * radius;

    bool found = false;
    real_t lowest = 0.0;
    real_t total_mass = 0.0;

    for (int pass = 0; pass < 2 && !found; ++pass) {
        for (const Particle &particle : particles) {
            if (particle.inverse_mass <= real_t(0.0)) {
                continue;
            }

            if (pass == 0 && radius > real_t(0.0)) {
                const Vector3 relative = particle.position - p_local_point;
                const Vector3 lateral = relative - normal * relative.dot(normal);

                if (lateral.length_squared() > radius_squared) {
                    continue;
                }
            }

            const real_t height = particle.position.dot(normal);

            if (!found || height < lowest) {
                lowest = height;
                found = true;
            }
        }

        if (!found && radius <= real_t(0.0)) {
            break;
        }
    }

    if (!found) {
        return 0.0;
    }

    const real_t energy_before = total_kinetic_energy();

    for (Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        particle.velocity -= normal * speed;
        total_mass += real_t(1.0) / particle.inverse_mass;
    }

    wall_enabled = true;
    wall_normal = normal;
    wall_offset = lowest;
    wall_center = p_local_point;
    wall_radius = radius;
    wall_friction = std::clamp(p_friction, real_t(0.0), real_t(2.0));
    wall_impulse_total = Vector3();
    wall_peak_force = 0.0;

    crash_active = true;
    crash_time_left = std::clamp(p_duration, real_t(0.02), real_t(3.0));
    plastic_timer = plastic_hold;

    diagnostics_dirty = true;
    sleep_counter = 0;
    wake();

    return std::max(real_t(0.0), total_kinetic_energy() - energy_before);
}

bool SoftBodySolver3D::is_crashing() const {
    return crash_active;
}

real_t SoftBodySolver3D::get_wall_force() const {
    return wall_force.length();
}

real_t SoftBodySolver3D::get_wall_peak_force() const {
    return wall_peak_force;
}

Vector3 SoftBodySolver3D::get_wall_impulse() const {
    return wall_impulse_total;
}

int32_t SoftBodySolver3D::get_wall_contact_count() const {
    return wall_contacts;
}

void SoftBodySolver3D::set_gravity(const Vector3 &value) {
    gravity = value;
    wake();
}

Vector3 SoftBodySolver3D::get_gravity() const {
    return gravity;
}

void SoftBodySolver3D::set_global_drag(real_t value) {
    global_drag = std::clamp(value, real_t(0.0), real_t(10.0));
}

real_t SoftBodySolver3D::get_global_drag() const {
    return global_drag;
}

void SoftBodySolver3D::set_settle_damping(real_t value) {
    settle_damping = std::clamp(value, real_t(0.0), real_t(60.0));
}

real_t SoftBodySolver3D::get_settle_damping() const {
    return settle_damping;
}

void SoftBodySolver3D::set_beam_damping(real_t value) {
    beam_damping = std::clamp(value, real_t(0.0), real_t(200.0));
}

real_t SoftBodySolver3D::get_beam_damping() const {
    return beam_damping;
}

void SoftBodySolver3D::set_solver_iterations(int32_t value) {
    solver_iterations = std::clamp(value, 1, 8);
}

void SoftBodySolver3D::set_auto_stabilize(bool value) {
    auto_stabilize = value;
}

bool SoftBodySolver3D::get_auto_stabilize() const {
    return auto_stabilize;
}

real_t SoftBodySolver3D::get_stability_ratio() const {
    return omega_max * fixed_step / real_t(std::max(active_substeps, MIN_ACTIVE_SUBSTEPS));
}

int32_t SoftBodySolver3D::get_effective_iterations() const {
    return effective_iterations;
}

int32_t SoftBodySolver3D::get_solver_iterations() const {
    return solver_iterations;
}

void SoftBodySolver3D::set_substeps(int32_t value) {
    substeps = std::clamp(value, 1, 32);
    active_substeps = substeps;
}

int32_t SoftBodySolver3D::get_substeps() const {
    return substeps;
}

int32_t SoftBodySolver3D::get_active_substeps() const {
    return active_substeps;
}

void SoftBodySolver3D::set_fixed_step(real_t value) {
    fixed_step = std::clamp(value, real_t(1.0 / 240.0), real_t(1.0 / 30.0));
}

real_t SoftBodySolver3D::get_fixed_step() const {
    return fixed_step;
}

void SoftBodySolver3D::set_max_speed(real_t value) {
    max_speed = std::clamp(value, real_t(1.0), real_t(1000.0));
}

real_t SoftBodySolver3D::get_max_speed() const {
    return max_speed;
}

void SoftBodySolver3D::set_beam_stiffness(real_t value) {
    beam_stiffness = std::clamp(value, real_t(1000.0), real_t(1.0e8));
    refresh_compliance();
}

real_t SoftBodySolver3D::get_beam_stiffness() const {
    return beam_stiffness;
}

void SoftBodySolver3D::set_yield_strain(real_t value) {
    yield_strain = std::clamp(value, real_t(0.001), real_t(1.0));
}

real_t SoftBodySolver3D::get_yield_strain() const {
    return yield_strain;
}

void SoftBodySolver3D::set_break_strain(real_t value) {
    break_strain = std::clamp(value, real_t(0.01), real_t(10.0));
}

real_t SoftBodySolver3D::get_break_strain() const {
    return break_strain;
}

void SoftBodySolver3D::set_break_hold(real_t value) {
    break_hold = std::clamp(value, real_t(0.0), real_t(0.1));
}

real_t SoftBodySolver3D::get_break_hold() const {
    return break_hold;
}

void SoftBodySolver3D::set_plastic_rate(real_t value) {
    plastic_rate = std::clamp(value, real_t(0.0), real_t(2000.0));
}

real_t SoftBodySolver3D::get_plastic_rate() const {
    return plastic_rate;
}

void SoftBodySolver3D::set_hardening(real_t value) {
    hardening = std::clamp(value, real_t(0.0), real_t(5.0));
}

real_t SoftBodySolver3D::get_hardening() const {
    return hardening;
}

void SoftBodySolver3D::set_buckle_ratio(real_t value) {
    buckle_ratio = std::clamp(value, real_t(0.1), real_t(1.0));
}

real_t SoftBodySolver3D::get_buckle_ratio() const {
    return buckle_ratio;
}

void SoftBodySolver3D::set_crush_limit(real_t value) {
    crush_limit = std::clamp(value, real_t(0.1), real_t(0.95));
}

real_t SoftBodySolver3D::get_crush_limit() const {
    return crush_limit;
}

void SoftBodySolver3D::set_stretch_limit(real_t value) {
    stretch_limit = std::clamp(value, real_t(1.05), real_t(3.0));
}

real_t SoftBodySolver3D::get_stretch_limit() const {
    return stretch_limit;
}

void SoftBodySolver3D::set_plastic_hold(real_t value) {
    plastic_hold = std::clamp(value, real_t(0.0), real_t(5.0));
}

real_t SoftBodySolver3D::get_plastic_hold() const {
    return plastic_hold;
}

void SoftBodySolver3D::set_lock_momentum(bool value) {
    lock_momentum = value;
}

bool SoftBodySolver3D::get_lock_momentum() const {
    return lock_momentum;
}

void SoftBodySolver3D::set_sleep_enabled(bool value) {
    sleep_enabled = value;

    if (!value) {
        wake();
    }
}

bool SoftBodySolver3D::get_sleep_enabled() const {
    return sleep_enabled;
}

void SoftBodySolver3D::set_sleep_speed(real_t value) {
    sleep_speed = std::clamp(value, real_t(0.001), real_t(1.0));
}

real_t SoftBodySolver3D::get_sleep_speed() const {
    return sleep_speed;
}

void SoftBodySolver3D::set_sleep_ticks(int32_t value) {
    sleep_ticks = std::clamp(value, 1, 600);
}

int32_t SoftBodySolver3D::get_sleep_ticks() const {
    return sleep_ticks;
}

void SoftBodySolver3D::set_crash_release_speed(real_t value) {
    crash_release_speed = std::clamp(value, real_t(0.01), real_t(5.0));
}

real_t SoftBodySolver3D::get_crash_release_speed() const {
    return crash_release_speed;
}

void SoftBodySolver3D::set_time_budget_us(real_t value) {
    time_budget_us = std::clamp(value, real_t(0.0), real_t(20000.0));

    if (time_budget_us <= real_t(0.0)) {
        active_substeps = substeps;
    }
}

real_t SoftBodySolver3D::get_time_budget_us() const {
    return time_budget_us;
}

void SoftBodySolver3D::integrate(real_t h) {
    const real_t max_speed_sq = max_speed * max_speed;
    const Vector3 gravity_dv = gravity * h;

    for (Particle &particle : particles) {
        particle.previous_position = particle.position;

        if (particle.inverse_mass <= real_t(0.0)) {
            particle.velocity = Vector3();
            continue;
        }

        particle.velocity += gravity_dv;

        const real_t speed_sq = particle.velocity.length_squared();
        if (speed_sq > max_speed_sq) {
            particle.velocity *= max_speed / std::sqrt(speed_sq);
        }

        particle.position += particle.velocity * h;
    }
}

void SoftBodySolver3D::solve_beams(real_t inv_h2, int32_t iteration, bool reverse) {
    const int32_t count = static_cast<int32_t>(beams.size());

    for (int32_t i = 0; i < count; ++i) {
        Beam &beam = beams[reverse ? count - 1 - i : i];

        if (beam.broken) {
            continue;
        }

        Particle &pa = particles[beam.a];
        Particle &pb = particles[beam.b];

        const Vector3 delta = pb.position - pa.position;
        const real_t distance = delta.length();

        if (distance <= real_t(0.000001)) {
            continue;
        }

        const real_t alpha = beam.compliance * inv_h2;
        const real_t w_sum = pa.inverse_mass + pb.inverse_mass;
        const real_t constraint = distance - beam.rest_length;
        const real_t lambda = iteration == 0 ? real_t(0.0) : beam.lambda;

        const real_t d_lambda = (-constraint - alpha * lambda) / (w_sum + alpha);
        beam.lambda = lambda + d_lambda;

        const Vector3 direction = delta / distance;
        pa.position -= direction * (pa.inverse_mass * d_lambda);
        pb.position += direction * (pb.inverse_mass * d_lambda);
    }
}

void SoftBodySolver3D::solve_wall(real_t h) {
    const real_t inv_h = real_t(1.0) / h;
    int32_t contacts = 0;

    for (Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t penetration = wall_offset - particle.position.dot(wall_normal);

        if (penetration <= real_t(0.0)) {
            continue;
        }

        if (wall_radius > real_t(0.0)) {
            const Vector3 relative = particle.position - wall_center;
            const Vector3 lateral = relative - wall_normal * relative.dot(wall_normal);

            if (lateral.length_squared() > wall_radius * wall_radius) {
                continue;
            }
        }

        ++contacts;
        Vector3 correction = wall_normal * penetration;

        if (wall_friction > real_t(0.0)) {
            const Vector3 move = particle.position - particle.previous_position;
            const Vector3 tangent = move - wall_normal * move.dot(wall_normal);
            const real_t tangent_length = tangent.length();

            if (tangent_length > real_t(0.000000001)) {
                const real_t factor =
                    std::min(real_t(1.0), wall_friction * penetration / tangent_length);
                correction -= tangent * factor;
            }
        }

        particle.position += correction;
        tick_wall_impulse += correction * (inv_h / particle.inverse_mass);
    }

    wall_contacts = contacts;
}

void SoftBodySolver3D::update_beams(real_t h) {
    const int32_t hold_substeps = std::clamp(
        static_cast<int32_t>(std::ceil(break_hold / h)), 1, 255
    );

    for (Beam &beam : beams) {
        if (beam.broken) {
            continue;
        }

        Particle &pa = particles[beam.a];
        Particle &pb = particles[beam.b];

        const Vector3 delta = pb.position - pa.position;
        const real_t distance = delta.length();

        if (distance <= real_t(0.000001)) {
            continue;
        }

        const Vector3 direction = delta / distance;
        const real_t w_sum = pa.inverse_mass + pb.inverse_mass;

        if (w_sum > real_t(0.0) && beam_damping > real_t(0.0) && beam.damping_scale > real_t(0.0)) {
            const real_t factor =
                std::min(beam_damping * beam.damping_scale * h, real_t(0.9));
            const real_t relative = (pb.velocity - pa.velocity).dot(direction);
            const real_t impulse = relative * factor / w_sum;

            pa.velocity += direction * (impulse * pa.inverse_mass);
            pb.velocity -= direction * (impulse * pb.inverse_mass);

            damper_work += 0.5 * (1.0 / double(w_sum)) * double(relative) * double(relative)
                * double(factor) * (2.0 - double(factor));
        }

        const real_t strain = (distance - beam.rest_length) / beam.rest_length;
        tick_max_strain = std::max(tick_max_strain, std::abs(strain));

        const real_t deviation =
            std::abs(beam.rest_length / beam.initial_rest_length - real_t(1.0));
        const real_t tension_yield = std::min(
            yield_strain * beam.yield_scale + hardening * deviation, real_t(0.9)
        );
        const real_t compression_yield = std::max(
            std::min(tension_yield * buckle_ratio, real_t(0.9)), real_t(0.0005)
        );
        const real_t rate = std::min(plastic_rate * beam.plastic_scale * h, real_t(1.0));

        if (rate > real_t(0.0) && plastic_allowed) {
            real_t target = beam.rest_length;
            bool flow = false;

            if (strain > tension_yield) {
                target = distance / (real_t(1.0) + tension_yield);
                flow = true;
            } else if (strain < -compression_yield) {
                target = distance / (real_t(1.0) - compression_yield);
                flow = true;
            }

            if (flow) {
                real_t new_rest = beam.rest_length + (target - beam.rest_length) * rate;
                new_rest = std::clamp(
                    new_rest,
                    beam.initial_rest_length * crush_limit,
                    beam.initial_rest_length * stretch_limit
                );

                const double stiffness = 1.0 / double(beam.compliance);
                const double before = double(distance - beam.rest_length);
                const double after = double(distance - new_rest);
                plastic_work += 0.5 * stiffness * (before * before - after * after);

                beam.rest_length = new_rest;
            }
        }

        const real_t break_limit = break_strain * beam.break_scale;

        if (distance > beam.initial_rest_length * (real_t(1.0) + break_limit)) {
            if (beam.overload < 255) {
                ++beam.overload;
            }

            if (beam.overload >= hold_substeps) {
                beam.broken = true;
                ++broken_beam_count;
            }
        } else {
            beam.overload = 0;
        }
    }
}

void SoftBodySolver3D::remove_mean_velocity() {
    real_t mass_sum = 0.0;
    Vector3 center;
    Vector3 momentum;

    for (const Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t mass = real_t(1.0) / particle.inverse_mass;
        mass_sum += mass;
        center += particle.position * mass;
        momentum += particle.velocity * mass;
    }

    if (mass_sum <= real_t(0.0)) {
        return;
    }

    center /= mass_sum;
    const Vector3 mean = momentum / mass_sum;

    double inertia[3][3] = {{0, 0, 0}, {0, 0, 0}, {0, 0, 0}};
    Vector3 angular;

    for (const Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t mass = real_t(1.0) / particle.inverse_mass;
        const Vector3 r = particle.position - center;
        const Vector3 v = particle.velocity - mean;
        const double rr = double(r.length_squared());
        const double rv[3] = {double(r.x), double(r.y), double(r.z)};

        angular += r.cross(v) * mass;

        for (int i = 0; i < 3; ++i) {
            for (int j = 0; j < 3; ++j) {
                inertia[i][j] += double(mass) * ((i == j ? rr : 0.0) - rv[i] * rv[j]);
            }
        }
    }

    const double a = inertia[0][0], b = inertia[0][1], c0 = inertia[0][2];
    const double d = inertia[1][0], e = inertia[1][1], f = inertia[1][2];
    const double g = inertia[2][0], h = inertia[2][1], k = inertia[2][2];

    const double cof00 = e * k - f * h;
    const double cof01 = -(d * k - f * g);
    const double cof02 = d * h - e * g;
    const double determinant = a * cof00 + b * cof01 + c0 * cof02;

    Vector3 omega;

    if (std::abs(determinant) > 1.0e-9) {
        const double inv = 1.0 / determinant;
        const double i00 = cof00 * inv;
        const double i01 = -(b * k - c0 * h) * inv;
        const double i02 = (b * f - c0 * e) * inv;
        const double i10 = cof01 * inv;
        const double i11 = (a * k - c0 * g) * inv;
        const double i12 = -(a * f - c0 * d) * inv;
        const double i20 = cof02 * inv;
        const double i21 = -(a * h - b * g) * inv;
        const double i22 = (a * e - b * d) * inv;

        omega = Vector3(
            real_t(i00 * angular.x + i01 * angular.y + i02 * angular.z),
            real_t(i10 * angular.x + i11 * angular.y + i12 * angular.z),
            real_t(i20 * angular.x + i21 * angular.y + i22 * angular.z)
        );
    }

    for (Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        particle.velocity -= mean + omega.cross(particle.position - center);
    }
}

void SoftBodySolver3D::update_crash() {
    crash_time_left -= fixed_step;

    real_t mass_sum = 0.0;
    real_t toward_wall = 0.0;

    for (const Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t mass = real_t(1.0) / particle.inverse_mass;
        mass_sum += mass;
        toward_wall -= particle.velocity.dot(wall_normal) * mass;
    }

    const real_t mean_toward_wall = mass_sum > real_t(0.0) ? toward_wall / mass_sum : real_t(0.0);

    if (crash_time_left <= real_t(0.0) || mean_toward_wall <= crash_release_speed) {
        crash_active = false;
        wall_enabled = false;
        wall_radius = 0.0;
        wall_contacts = 0;

        if (lock_momentum) {
            remove_mean_velocity();
        }
    }
}

void SoftBodySolver3D::simulate_tick() {
    const int32_t steps = std::clamp(active_substeps, MIN_ACTIVE_SUBSTEPS, std::max(substeps, MIN_ACTIVE_SUBSTEPS));
    const real_t h = fixed_step / real_t(steps);
    const real_t inv_h = real_t(1.0) / h;
    const real_t inv_h2 = inv_h * inv_h;
    const real_t drag_factor = std::max(real_t(0.0), real_t(1.0) - global_drag * h);
    const real_t settle_factor = std::min(settle_damping * h, real_t(0.9));
    const real_t max_speed_sq = max_speed * max_speed;

    if (stability_dirty) {
        compute_stability();
    }

    const real_t stability = omega_max * h;
    int32_t iterations = solver_iterations;

    if (auto_stabilize) {
        if (stability > real_t(16.0)) {
            iterations = std::max(iterations, 4);
        } else if (stability > real_t(8.0)) {
            iterations = std::max(iterations, 3);
        } else if (stability > real_t(3.0)) {
            iterations = std::max(iterations, 2);
        }
    }

    effective_iterations = iterations;

    tick_max_strain = 0.0;
    tick_peak_speed_sq = 0.0;
    tick_wall_impulse = Vector3();
    plastic_allowed = plastic_hold <= real_t(0.0) || crash_active || plastic_timer > real_t(0.0);

    if (good_position.size() != particles.size()) {
        good_position.resize(particles.size());
        good_velocity.resize(particles.size());
    }
    for (size_t i = 0; i < particles.size(); ++i) {
        good_position[i] = particles[i].position;
        good_velocity[i] = particles[i].velocity;
    }

    for (int32_t step = 0; step < steps; ++step) {
        integrate(h);

        for (int32_t iteration = 0; iteration < iterations; ++iteration) {
            solve_beams(inv_h2, iteration, ((substep_counter + iteration) & 1u) != 0u);
        }

        if (wall_enabled) {
            solve_wall(h);
        }

        real_t mass_sum = 0.0;
        Vector3 momentum;

        for (Particle &particle : particles) {
            if (particle.inverse_mass <= real_t(0.0)) {
                continue;
            }

            particle.velocity = (particle.position - particle.previous_position) * inv_h;
            particle.velocity *= drag_factor;

            real_t speed_sq = particle.velocity.length_squared();
            if (speed_sq > max_speed_sq) {
                particle.velocity *= max_speed / std::sqrt(speed_sq);
                speed_sq = max_speed_sq;
            }

            tick_peak_speed_sq = std::max(tick_peak_speed_sq, speed_sq);

            if (settle_factor > real_t(0.0)) {
                const real_t mass = real_t(1.0) / particle.inverse_mass;
                mass_sum += mass;
                momentum += particle.velocity * mass;
            }
        }

        if (settle_factor > real_t(0.0) && mass_sum > real_t(0.0)) {
            const Vector3 mean_velocity = momentum / mass_sum;

            for (Particle &particle : particles) {
                if (particle.inverse_mass <= real_t(0.0)) {
                    continue;
                }

                particle.velocity -= (particle.velocity - mean_velocity) * settle_factor;
            }
        }

        update_beams(h);
        ++substep_counter;
    }

    bool finite = true;
    for (const Particle &particle : particles) {
        const real_t sum = particle.position.x + particle.position.y + particle.position.z
            + particle.velocity.x + particle.velocity.y + particle.velocity.z;
        if (!std::isfinite(sum)) {
            finite = false;
            break;
        }
    }

    if (!finite) {
        for (size_t i = 0; i < particles.size(); ++i) {
            particles[i].position = good_position[i];
            particles[i].previous_position = good_position[i];
            particles[i].velocity = Vector3();
        }
        for (Beam &beam : beams) {
            if (!std::isfinite(beam.rest_length) || beam.rest_length <= real_t(0.0)) {
                beam.rest_length = beam.initial_rest_length;
            }
            beam.lambda = 0.0;
        }
        ++fault_count;
        tick_max_strain = 0.0;
        tick_peak_speed_sq = 0.0;
        tick_wall_impulse = Vector3();
    }

    if (finite) {
        real_t drift_squared = 0.0;

        for (size_t i = 0; i < particles.size(); ++i) {
            if (particles[i].inverse_mass <= real_t(0.0)) {
                continue;
            }

            drift_squared = std::max(
                drift_squared, (particles[i].position - good_position[i]).length_squared()
            );
        }

        tick_peak_speed_sq = drift_squared / (fixed_step * fixed_step);
    }

    if (tick_peak_speed_sq > real_t(0.25)) {
        plastic_timer = plastic_hold;
    } else if (plastic_timer > real_t(0.0)) {
        plastic_timer = std::max(real_t(0.0), plastic_timer - fixed_step);
    }

    max_strain = tick_max_strain;

    wall_force = tick_wall_impulse / fixed_step;
    wall_impulse_total += tick_wall_impulse;
    wall_peak_force = std::max(wall_peak_force, wall_force.length());

    diagnostics_dirty = true;

    if (crash_active) {
        update_crash();
    } else if (lock_momentum && !wall_enabled) {
        remove_mean_velocity();
    }

    if (sleep_enabled && finite && !crash_active) {
        if (tick_peak_speed_sq <= sleep_speed * sleep_speed) {
            ++sleep_counter;
        } else {
            sleep_counter = 0;
        }

        if (sleep_counter >= sleep_ticks) {
            enter_sleep();
        }
    } else {
        sleep_counter = 0;
    }
}

void SoftBodySolver3D::_physics_process(double p_delta) {
    if (particles.empty() || sleeping) {
        return;
    }

    const uint64_t start_us = Time::get_singleton()->get_ticks_usec();

    accumulator += std::min(real_t(p_delta), real_t(0.1));

    int32_t ticks = 0;

    while (accumulator >= fixed_step && ticks < MAX_TICKS_PER_FRAME && !sleeping) {
        simulate_tick();
        accumulator -= fixed_step;
        ++ticks;
    }

    if (ticks == MAX_TICKS_PER_FRAME && accumulator > fixed_step) {
        accumulator = fixed_step;
    }

    if (ticks > 0) {
        const double elapsed =
            static_cast<double>(Time::get_singleton()->get_ticks_usec() - start_us);
        step_time_us = step_time_us <= 0.0 ? elapsed : step_time_us * 0.9 + elapsed * 0.1;

        if (time_budget_us > real_t(0.0)) {
            if (++budget_frames >= 10) {
                budget_frames = 0;

                if (step_time_us > double(time_budget_us) && active_substeps > MIN_ACTIVE_SUBSTEPS) {
                    --active_substeps;
                } else if (step_time_us < double(time_budget_us) * 0.6 && active_substeps < substeps) {
                    ++active_substeps;
                }
            }
        }
    }
}

void SoftBodySolver3D::compute_diagnostics() const {
    Diagnostics d;

    const real_t epsilon = real_t(0.0001);

    real_t total_mass = 0.0;
    Vector3 com;
    Vector3 rest_com;
    Vector3 momentum;

    for (const Particle &p : particles) {
        if (p.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t m = real_t(1.0) / p.inverse_mass;
        total_mass += m;
        com += p.position * m;
        rest_com += p.rest_position * m;
        momentum += p.velocity * m;
    }

    d.total_mass = total_mass;

    if (total_mass <= epsilon) {
        diagnostics = d;
        diagnostics_dirty = false;
        return;
    }

    com /= total_mass;
    rest_com /= total_mass;

    d.center_of_mass = com;
    d.rest_center_of_mass = rest_com;
    d.linear_momentum = momentum;
    d.linear_velocity = momentum / total_mass;

    Vector3 a_col[3];
    Vector3 angular;
    real_t kinetic = 0.0;

    for (const Particle &p : particles) {
        if (p.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t m = real_t(1.0) / p.inverse_mass;
        const Vector3 r = p.position - com;
        const Vector3 q = p.rest_position - rest_com;

        angular += r.cross(p.velocity) * m;
        kinetic += real_t(0.5) * m * p.velocity.length_squared();

        a_col[0] += r * (m * q.x);
        a_col[1] += r * (m * q.y);
        a_col[2] += r * (m * q.z);
    }

    d.angular_momentum = angular;
    d.kinetic_energy = kinetic;
    d.internal_kinetic_energy = std::max(
        real_t(0.0),
        kinetic - real_t(0.5) * total_mass * d.linear_velocity.length_squared()
    );

    Quaternion rotation = frame_rotation;

    for (int32_t iteration = 0; iteration < 24; ++iteration) {
        const Basis r_basis(rotation);
        const Vector3 r0 = r_basis.get_column(0);
        const Vector3 r1 = r_basis.get_column(1);
        const Vector3 r2 = r_basis.get_column(2);

        const Vector3 numerator =
            r0.cross(a_col[0]) + r1.cross(a_col[1]) + r2.cross(a_col[2]);
        const real_t denominator =
            std::abs(r0.dot(a_col[0]) + r1.dot(a_col[1]) + r2.dot(a_col[2])) +
            real_t(1.0e-9);

        const Vector3 omega = numerator / denominator;
        const real_t angle = omega.length();

        if (angle < real_t(1.0e-9)) {
            break;
        }

        rotation = (Quaternion(omega / angle, angle) * rotation).normalized();
    }

    frame_rotation = rotation;

    const Basis frame_basis(rotation);
    d.rigid_frame = Transform3D(frame_basis, com);

    real_t sum_squared = 0.0;
    real_t max_squared = 0.0;

    for (const Particle &p : particles) {
        const Vector3 expected = com + frame_basis.xform(p.rest_position - rest_com);
        const real_t squared = (p.position - expected).length_squared();

        sum_squared += squared;
        max_squared = std::max(max_squared, squared);
    }

    if (!particles.empty()) {
        d.rms_deformation = std::sqrt(sum_squared / real_t(particles.size()));
    }
    d.max_deformation = std::sqrt(max_squared);

    double elastic = 0.0;

    for (const Beam &beam : beams) {
        if (beam.broken || beam.rest_length <= epsilon) {
            continue;
        }

        const real_t distance = particles[beam.a].position.distance_to(
            particles[beam.b].position
        );
        const real_t extension = distance - beam.rest_length;
        const real_t strain = std::abs(extension) / beam.rest_length;
        d.max_strain_post_solve = std::max(d.max_strain_post_solve, strain);

        elastic += 0.5 * double(extension) * double(extension) / double(beam.compliance);

        if (beam.initial_rest_length > epsilon &&
            std::abs(beam.rest_length / beam.initial_rest_length - real_t(1.0)) >
                real_t(0.005)) {
            ++d.yielded_beam_count;
        }
    }

    d.elastic_energy = static_cast<real_t>(elastic);

    diagnostics = d;
    diagnostics_dirty = false;
}

void SoftBodySolver3D::ensure_diagnostics() const {
    if (diagnostics_dirty) {
        compute_diagnostics();
    }
}

PackedVector3Array SoftBodySolver3D::get_node_positions() const {
    PackedVector3Array result;
    result.resize(static_cast<int64_t>(particles.size()));

    Vector3 *write = result.ptrw();

    for (size_t i = 0; i < particles.size(); ++i) {
        write[i] = particles[i].position;
    }

    return result;
}

PackedInt32Array SoftBodySolver3D::get_beam_indices() const {
    PackedInt32Array result;

    const int64_t active_count =
        static_cast<int64_t>(beams.size()) - static_cast<int64_t>(broken_beam_count);

    if (active_count <= 0) {
        return result;
    }

    result.resize(active_count * 2);

    int32_t *write = result.ptrw();
    int64_t cursor = 0;

    for (const Beam &beam : beams) {
        if (beam.broken) {
            continue;
        }

        write[cursor++] = beam.a;
        write[cursor++] = beam.b;
    }

    return result;
}

PackedFloat32Array SoftBodySolver3D::get_node_damage() const {
    PackedFloat32Array result;
    result.resize(static_cast<int64_t>(particles.size()));

    float *write = result.ptrw();

    for (size_t i = 0; i < particles.size(); ++i) {
        write[i] = 0.0f;
    }

    const real_t reference = std::max(real_t(0.05), real_t(1.0) - crush_limit);

    for (const Beam &beam : beams) {
        real_t damage = 1.0;

        if (!beam.broken && beam.initial_rest_length > real_t(0.0001)) {
            damage = std::min(
                std::abs(beam.rest_length / beam.initial_rest_length - real_t(1.0)) / reference,
                real_t(1.0)
            );
        }

        write[beam.a] = std::max(write[beam.a], static_cast<float>(damage));
        write[beam.b] = std::max(write[beam.b], static_cast<float>(damage));
    }

    return result;
}

int32_t SoftBodySolver3D::get_node_count() const {
    return static_cast<int32_t>(particles.size());
}

int32_t SoftBodySolver3D::get_beam_count() const {
    return static_cast<int32_t>(beams.size());
}

int32_t SoftBodySolver3D::get_broken_beam_count() const {
    return broken_beam_count;
}

real_t SoftBodySolver3D::get_max_strain() const {
    return max_strain;
}

int32_t SoftBodySolver3D::get_fault_count() const {
    return fault_count;
}

real_t SoftBodySolver3D::get_step_time_us() const {
    return static_cast<real_t>(step_time_us);
}

Dictionary SoftBodySolver3D::get_stats() const {
    ensure_diagnostics();

    Dictionary stats;
    stats[String("sleeping")] = sleeping;
    stats[String("active_substeps")] = active_substeps;
    stats[String("iterations")] = effective_iterations;
    stats[String("stability_ratio")] = get_stability_ratio();
    stats[String("step_time_us")] = static_cast<real_t>(step_time_us);
    stats[String("fault_count")] = fault_count;
    stats[String("broken_beams")] = broken_beam_count;
    stats[String("yielded_beams")] = diagnostics.yielded_beam_count;
    stats[String("max_strain")] = max_strain;
    stats[String("plastic_work")] = static_cast<real_t>(plastic_work);
    stats[String("damper_work")] = static_cast<real_t>(damper_work);
    stats[String("elastic_energy")] = diagnostics.elastic_energy;
    stats[String("kinetic_energy")] = diagnostics.kinetic_energy;
    stats[String("max_deformation")] = diagnostics.max_deformation;
    return stats;
}

real_t SoftBodySolver3D::get_total_mass() const {
    ensure_diagnostics();
    return diagnostics.total_mass;
}

Vector3 SoftBodySolver3D::get_center_of_mass() const {
    ensure_diagnostics();
    return diagnostics.center_of_mass;
}

Vector3 SoftBodySolver3D::get_rest_center_of_mass() const {
    ensure_diagnostics();
    return diagnostics.rest_center_of_mass;
}

Vector3 SoftBodySolver3D::get_linear_velocity() const {
    ensure_diagnostics();
    return diagnostics.linear_velocity;
}

Vector3 SoftBodySolver3D::get_linear_momentum() const {
    ensure_diagnostics();
    return diagnostics.linear_momentum;
}

Vector3 SoftBodySolver3D::get_angular_momentum() const {
    ensure_diagnostics();
    return diagnostics.angular_momentum;
}

real_t SoftBodySolver3D::get_kinetic_energy() const {
    ensure_diagnostics();
    return diagnostics.kinetic_energy;
}

real_t SoftBodySolver3D::get_internal_kinetic_energy() const {
    ensure_diagnostics();
    return diagnostics.internal_kinetic_energy;
}

real_t SoftBodySolver3D::get_elastic_energy() const {
    ensure_diagnostics();
    return diagnostics.elastic_energy;
}

real_t SoftBodySolver3D::get_plastic_work() const {
    return static_cast<real_t>(plastic_work);
}

real_t SoftBodySolver3D::get_damper_work() const {
    return static_cast<real_t>(damper_work);
}

Transform3D SoftBodySolver3D::get_rigid_frame() const {
    ensure_diagnostics();
    return diagnostics.rigid_frame;
}

PackedVector3Array SoftBodySolver3D::get_node_deformations() const {
    ensure_diagnostics();

    PackedVector3Array result;
    result.resize(static_cast<int64_t>(particles.size()));

    Vector3 *write = result.ptrw();

    if (diagnostics.total_mass <= real_t(0.0001)) {
        for (size_t i = 0; i < particles.size(); ++i) {
            write[i] = Vector3();
        }
        return result;
    }

    const Basis &frame_basis = diagnostics.rigid_frame.basis;
    const Vector3 &origin = diagnostics.rigid_frame.origin;

    for (size_t i = 0; i < particles.size(); ++i) {
        const Particle &p = particles[i];
        const Vector3 expected =
            origin + frame_basis.xform(p.rest_position - diagnostics.rest_center_of_mass);
        write[i] = p.position - expected;
    }

    return result;
}

real_t SoftBodySolver3D::get_rms_deformation() const {
    ensure_diagnostics();
    return diagnostics.rms_deformation;
}

real_t SoftBodySolver3D::get_max_deformation() const {
    ensure_diagnostics();
    return diagnostics.max_deformation;
}

real_t SoftBodySolver3D::get_max_strain_post_solve() const {
    ensure_diagnostics();
    return diagnostics.max_strain_post_solve;
}

int32_t SoftBodySolver3D::get_yielded_beam_count() const {
    ensure_diagnostics();
    return diagnostics.yielded_beam_count;
}
