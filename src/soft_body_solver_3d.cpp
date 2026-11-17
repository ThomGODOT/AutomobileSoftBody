#include "soft_body_solver_3d.h"

#include <algorithm>
#include <cmath>

#include <godot_cpp/classes/time.hpp>
#include <godot_cpp/core/class_db.hpp>
#include <godot_cpp/variant/variant.hpp>

using namespace godot;

SoftBodySolver3D::SoftBodySolver3D() {
    set_physics_process(false);
}

SoftBodySolver3D::~SoftBodySolver3D() {
    // Do not call Node methods here: when the extension instance is freed the
    // Node part of the object may already be destroyed. The std::vector
    // members release their memory automatically.
}

void SoftBodySolver3D::_bind_methods() {
    // Building.
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
        D_METHOD("add_beam", "a", "b", "stiffness_scale"),
        &SoftBodySolver3D::add_beam,
        DEFVAL(1.0)
    );
    ClassDB::bind_method(
        D_METHOD("set_beam_scales", "beam_index", "stiffness_scale", "damping_scale",
                 "yield_scale", "break_scale", "plastic_scale"),
        &SoftBodySolver3D::set_beam_scales
    );
    ClassDB::bind_method(D_METHOD("finish_build"), &SoftBodySolver3D::finish_build);

    // Mass and motion.
    ClassDB::bind_method(D_METHOD("set_node_mass", "node_index", "mass"), &SoftBodySolver3D::set_node_mass);
    ClassDB::bind_method(D_METHOD("get_node_mass", "node_index"), &SoftBodySolver3D::get_node_mass);
    ClassDB::bind_method(D_METHOD("set_total_mass", "mass"), &SoftBodySolver3D::set_total_mass);
    ClassDB::bind_method(
        D_METHOD("set_linear_velocity", "velocity"),
        &SoftBodySolver3D::set_linear_velocity
    );
    ClassDB::bind_method(
        D_METHOD("apply_impulse", "local_position", "impulse", "radius"),
        &SoftBodySolver3D::apply_impulse
    );
    ClassDB::bind_method(D_METHOD("reset_to_rest"), &SoftBodySolver3D::reset_to_rest);

    // Test wall.
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

    // Settings.
    ClassDB::bind_method(D_METHOD("set_gravity", "value"), &SoftBodySolver3D::set_gravity);
    ClassDB::bind_method(D_METHOD("get_gravity"), &SoftBodySolver3D::get_gravity);

    ClassDB::bind_method(D_METHOD("set_global_drag", "value"), &SoftBodySolver3D::set_global_drag);
    ClassDB::bind_method(D_METHOD("get_global_drag"), &SoftBodySolver3D::get_global_drag);

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

    ClassDB::bind_method(D_METHOD("set_substeps", "value"), &SoftBodySolver3D::set_substeps);
    ClassDB::bind_method(D_METHOD("get_substeps"), &SoftBodySolver3D::get_substeps);

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

    // Output.
    ClassDB::bind_method(D_METHOD("get_node_positions"), &SoftBodySolver3D::get_node_positions);
    ClassDB::bind_method(D_METHOD("get_beam_indices"), &SoftBodySolver3D::get_beam_indices);
    ClassDB::bind_method(D_METHOD("get_node_count"), &SoftBodySolver3D::get_node_count);
    ClassDB::bind_method(D_METHOD("get_beam_count"), &SoftBodySolver3D::get_beam_count);
    ClassDB::bind_method(
        D_METHOD("get_broken_beam_count"),
        &SoftBodySolver3D::get_broken_beam_count
    );
    ClassDB::bind_method(D_METHOD("get_max_strain"), &SoftBodySolver3D::get_max_strain);
    ClassDB::bind_method(D_METHOD("get_fault_count"), &SoftBodySolver3D::get_fault_count);
    ClassDB::bind_method(D_METHOD("get_step_time_us"), &SoftBodySolver3D::get_step_time_us);

    // Diagnostics.
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

    // Properties.
    ADD_PROPERTY(PropertyInfo(Variant::VECTOR3, "gravity"), "set_gravity", "get_gravity");

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "global_drag", PROPERTY_HINT_RANGE, "0,10,0.01"),
        "set_global_drag",
        "get_global_drag"
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
}

// ------------------------------------------------------------- Building ----

void SoftBodySolver3D::clear_data() {
    particles.clear();
    beams.clear();
    good_position.clear();
    good_velocity.clear();
    accumulator = 0.0;
    substep_counter = 0;
    max_strain = 0.0;
    tick_max_strain = 0.0;
    broken_beam_count = 0;
    fault_count = 0;
    plastic_work = 0.0;
    damper_work = 0.0;
    wall_contacts = 0;
    tick_wall_impulse = Vector3();
    wall_force = Vector3();
    wall_impulse_total = Vector3();
    wall_peak_force = 0.0;
    frame_rotation = Quaternion();
    diagnostics = Diagnostics();
    diagnostics_dirty = true;
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

int32_t SoftBodySolver3D::add_beam(int32_t p_a, int32_t p_b, real_t p_stiffness_scale) {
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

        add_beam(index(x, y, z), index(tx, ty, tz), stiffness_scale);
    };

    // Structural and shear beams: one-neighbour offsets in 3D.
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

                // Longer span beams (weak; real bending resistance comes later).
                link(x, y, z, 2, 0, 0, real_t(0.18));
                link(x, y, z, 0, 2, 0, real_t(0.18));
                link(x, y, z, 0, 0, 2, real_t(0.18));
            }
        }
    }

    finish_build();
}

// ------------------------------------------------------ Mass and motion ----

void SoftBodySolver3D::set_node_mass(int32_t p_node_index, real_t p_mass) {
    if (p_node_index < 0 || p_node_index >= static_cast<int32_t>(particles.size())) {
        return;
    }

    particles[p_node_index].inverse_mass =
        p_mass <= real_t(0.0) ? real_t(0.0)
                              : real_t(1.0) / std::max(p_mass, real_t(0.001));
    diagnostics_dirty = true;
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
}

void SoftBodySolver3D::set_linear_velocity(const Vector3 &p_velocity) {
    for (Particle &particle : particles) {
        if (particle.inverse_mass > real_t(0.0)) {
            particle.velocity = p_velocity;
        }
    }

    diagnostics_dirty = true;
}

Vector3 SoftBodySolver3D::apply_impulse(
    const Vector3 &p_local_position,
    const Vector3 &p_impulse,
    real_t p_radius
) {
    Vector3 delivered;

    if (p_radius <= real_t(0.0001)) {
        return delivered;
    }

    for (Particle &particle : particles) {
        if (particle.inverse_mass <= real_t(0.0)) {
            continue;
        }

        const real_t distance = particle.position.distance_to(p_local_position);

        if (distance >= p_radius) {
            continue;
        }

        const real_t weight = real_t(1.0) - distance / p_radius;

        // Delta-v = J * weight / mass. Momentum delivered = J * weight.
        particle.velocity += p_impulse * (weight * particle.inverse_mass);
        delivered += p_impulse * weight;
    }

    diagnostics_dirty = true;
    return delivered;
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
    wall_contacts = 0;
    tick_wall_impulse = Vector3();
    wall_force = Vector3();
    wall_impulse_total = Vector3();
    wall_peak_force = 0.0;
    frame_rotation = Quaternion();
    diagnostics_dirty = true;
}

// --------------------------------------------------------------- Wall ------

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

// ----------------------------------------------------------- Settings ------

void SoftBodySolver3D::set_gravity(const Vector3 &value) {
    gravity = value;
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

void SoftBodySolver3D::set_beam_damping(real_t value) {
    beam_damping = std::clamp(value, real_t(0.0), real_t(200.0));
}

real_t SoftBodySolver3D::get_beam_damping() const {
    return beam_damping;
}

void SoftBodySolver3D::set_solver_iterations(int32_t value) {
    solver_iterations = std::clamp(value, 1, 8);
}

int32_t SoftBodySolver3D::get_solver_iterations() const {
    return solver_iterations;
}

void SoftBodySolver3D::set_substeps(int32_t value) {
    substeps = std::clamp(value, 1, 32);
}

int32_t SoftBodySolver3D::get_substeps() const {
    return substeps;
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

// --------------------------------------------------------- Simulation ------

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

// XPBD distance constraint: C = |pb - pa| - rest.
//   dlambda = (-C - alpha * lambda) / (wa + wb + alpha),  alpha = compliance / h^2
// With one iteration lambda starts at 0 each substep, so no reset pass is needed.
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

// Runs once per substep after velocities are updated:
//   1. relative axial damping (momentum conserving)
//   2. plasticity on the settled strain
//   3. tension breakage with a short hold time
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

        // 1. Axial damping: remove a fraction of the relative velocity along the beam.
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

        // 2. Plasticity (elastic-perfectly-plastic, rate limited).
        const real_t strain = (distance - beam.rest_length) / beam.rest_length;
        tick_max_strain = std::max(tick_max_strain, std::abs(strain));

        const real_t yield_limit = std::min(yield_strain * beam.yield_scale, real_t(0.9));
        const real_t rate = std::min(plastic_rate * beam.plastic_scale * h, real_t(1.0));

        if (rate > real_t(0.0) && std::abs(strain) > yield_limit) {
            const real_t target = strain > real_t(0.0)
                ? distance / (real_t(1.0) + yield_limit)
                : distance / (real_t(1.0) - yield_limit);

            real_t new_rest = beam.rest_length + (target - beam.rest_length) * rate;
            new_rest = std::clamp(
                new_rest,
                beam.initial_rest_length * MIN_REST_RATIO,
                beam.initial_rest_length * MAX_REST_RATIO
            );

            const double stiffness = 1.0 / double(beam.compliance);
            const double before = double(distance - beam.rest_length);
            const double after = double(distance - new_rest);
            plastic_work += 0.5 * stiffness * (before * before - after * after);

            beam.rest_length = new_rest;
        }

        // 3. Tension breakage. Compression never breaks a beam (crush is limited
        // by MIN_REST_RATIO instead).
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

void SoftBodySolver3D::simulate_tick() {
    const real_t h = fixed_step / real_t(substeps);
    const real_t inv_h = real_t(1.0) / h;
    const real_t inv_h2 = inv_h * inv_h;
    const real_t drag_factor = std::max(real_t(0.0), real_t(1.0) - global_drag * h);
    const real_t max_speed_sq = max_speed * max_speed;

    tick_max_strain = 0.0;
    tick_wall_impulse = Vector3();

    // Snapshot for rollback.
    if (good_position.size() != particles.size()) {
        good_position.resize(particles.size());
        good_velocity.resize(particles.size());
    }
    for (size_t i = 0; i < particles.size(); ++i) {
        good_position[i] = particles[i].position;
        good_velocity[i] = particles[i].velocity;
    }

    for (int32_t step = 0; step < substeps; ++step) {
        integrate(h);

        // Alternate the solve direction so no beam ordering is favoured.
        for (int32_t iteration = 0; iteration < solver_iterations; ++iteration) {
            solve_beams(inv_h2, iteration, ((substep_counter + iteration) & 1u) != 0u);
        }

        if (wall_enabled) {
            solve_wall(h);
        }

        // Velocities from the settled positions.
        for (Particle &particle : particles) {
            if (particle.inverse_mass <= real_t(0.0)) {
                continue;
            }

            particle.velocity = (particle.position - particle.previous_position) * inv_h;
            particle.velocity *= drag_factor;

            const real_t speed_sq = particle.velocity.length_squared();
            if (speed_sq > max_speed_sq) {
                particle.velocity *= max_speed / std::sqrt(speed_sq);
            }
        }

        update_beams(h);
        ++substep_counter;
    }

    // Safety: detect NaN/Inf and roll back this tick.
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
        tick_wall_impulse = Vector3();
    }

    max_strain = tick_max_strain;

    wall_force = tick_wall_impulse / fixed_step;
    wall_impulse_total += tick_wall_impulse;
    wall_peak_force = std::max(wall_peak_force, wall_force.length());

    diagnostics_dirty = true;
}

void SoftBodySolver3D::_physics_process(double p_delta) {
    if (particles.empty()) {
        return;
    }

    const uint64_t start_us = Time::get_singleton()->get_ticks_usec();

    accumulator += std::min(real_t(p_delta), real_t(0.1));

    int32_t ticks = 0;

    while (accumulator >= fixed_step && ticks < MAX_TICKS_PER_FRAME) {
        simulate_tick();
        accumulator -= fixed_step;
        ++ticks;
    }

    // Discard excess backlog rather than allowing an expensive catch-up spiral.
    if (ticks == MAX_TICKS_PER_FRAME && accumulator > fixed_step) {
        accumulator = fixed_step;
    }

    if (ticks > 0) {
        const double elapsed =
            static_cast<double>(Time::get_singleton()->get_ticks_usec() - start_us);
        step_time_us = step_time_us <= 0.0 ? elapsed : step_time_us * 0.9 + elapsed * 0.1;
    }
}

// -------------------------------------------------------- Diagnostics ------

void SoftBodySolver3D::compute_diagnostics() const {
    Diagnostics d;

    const real_t epsilon = real_t(0.0001);

    // Pass 1: mass, centers of mass, momentum. Pinned particles are excluded.
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

    // Pass 2: angular momentum, kinetic energy and the shape-matching
    // covariance A = sum(m * (p - c) * (q - c0)^T), stored by columns.
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

    // Extract the rotation of A iteratively (Mueller et al. 2016), warm
    // started from the previous result.
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

    // Deformation relative to the rigid-fit pose, over all particles.
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

    // Beam measurements: strain, elastic energy, plastic beam count.
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
