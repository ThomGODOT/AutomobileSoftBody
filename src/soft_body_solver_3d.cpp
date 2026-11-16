#include "soft_body_solver_3d.h"

#include <algorithm>
#include <cmath>

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
    ClassDB::bind_method(
        D_METHOD("create_box", "size", "center", "nodes_x", "nodes_y", "nodes_z"),
        &SoftBodySolver3D::create_box
    );

    ClassDB::bind_method(
        D_METHOD("apply_impulse", "local_position", "impulse", "radius"),
        &SoftBodySolver3D::apply_impulse
    );
    ClassDB::bind_method(D_METHOD("reset_to_rest"), &SoftBodySolver3D::reset_to_rest);

    ClassDB::bind_method(D_METHOD("set_gravity", "value"), &SoftBodySolver3D::set_gravity);
    ClassDB::bind_method(D_METHOD("get_gravity"), &SoftBodySolver3D::get_gravity);

    ClassDB::bind_method(D_METHOD("set_damping", "value"), &SoftBodySolver3D::set_damping);
    ClassDB::bind_method(D_METHOD("get_damping"), &SoftBodySolver3D::get_damping);

    ClassDB::bind_method(
        D_METHOD("set_solver_iterations", "value"),
        &SoftBodySolver3D::set_solver_iterations
    );
    ClassDB::bind_method(
        D_METHOD("get_solver_iterations"),
        &SoftBodySolver3D::get_solver_iterations
    );

    ClassDB::bind_method(D_METHOD("set_fixed_step", "value"), &SoftBodySolver3D::set_fixed_step);
    ClassDB::bind_method(D_METHOD("get_fixed_step"), &SoftBodySolver3D::get_fixed_step);

    ClassDB::bind_method(D_METHOD("set_yield_strain", "value"), &SoftBodySolver3D::set_yield_strain);
    ClassDB::bind_method(D_METHOD("get_yield_strain"), &SoftBodySolver3D::get_yield_strain);

    ClassDB::bind_method(D_METHOD("set_break_strain", "value"), &SoftBodySolver3D::set_break_strain);
    ClassDB::bind_method(D_METHOD("get_break_strain"), &SoftBodySolver3D::get_break_strain);

    ClassDB::bind_method(
        D_METHOD("set_plastic_creep", "value"),
        &SoftBodySolver3D::set_plastic_creep
    );
    ClassDB::bind_method(D_METHOD("get_plastic_creep"), &SoftBodySolver3D::get_plastic_creep);

    ClassDB::bind_method(
        D_METHOD("get_node_positions"),
        &SoftBodySolver3D::get_node_positions
    );
    ClassDB::bind_method(
        D_METHOD("get_beam_indices"),
        &SoftBodySolver3D::get_beam_indices
    );

    ClassDB::bind_method(D_METHOD("get_node_count"), &SoftBodySolver3D::get_node_count);
    ClassDB::bind_method(D_METHOD("get_beam_count"), &SoftBodySolver3D::get_beam_count);
    ClassDB::bind_method(
        D_METHOD("get_broken_beam_count"),
        &SoftBodySolver3D::get_broken_beam_count
    );
    ClassDB::bind_method(D_METHOD("get_max_strain"), &SoftBodySolver3D::get_max_strain);

    // Diagnostics.
    ClassDB::bind_method(D_METHOD("get_total_mass"), &SoftBodySolver3D::get_total_mass);
    ClassDB::bind_method(D_METHOD("get_center_of_mass"), &SoftBodySolver3D::get_center_of_mass);
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

    ADD_PROPERTY(
        PropertyInfo(Variant::VECTOR3, "gravity"),
        "set_gravity",
        "get_gravity"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "damping"),
        "set_damping",
        "get_damping"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::INT, "solver_iterations"),
        "set_solver_iterations",
        "get_solver_iterations"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "fixed_step", PROPERTY_HINT_RANGE, "0.004,0.034,0.0001"),
        "set_fixed_step",
        "get_fixed_step"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "yield_strain", PROPERTY_HINT_RANGE, "0.001,10,0.001"),
        "set_yield_strain",
        "get_yield_strain"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "break_strain", PROPERTY_HINT_RANGE, "0.001,10,0.001"),
        "set_break_strain",
        "get_break_strain"
    );

    ADD_PROPERTY(
        PropertyInfo(Variant::FLOAT, "plastic_creep", PROPERTY_HINT_RANGE, "0,1,0.001"),
        "set_plastic_creep",
        "get_plastic_creep"
    );
}

void SoftBodySolver3D::clear_data() {
    particles.clear();
    beams.clear();
    accumulator = 0.0;
    max_strain = 0.0;
    broken_beam_count = 0;
    frame_rotation = Quaternion();
    diagnostics = Diagnostics();
    diagnostics_dirty = true;
}

void SoftBodySolver3D::clear_body() {
    clear_data();
    set_physics_process(false);
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

    for (int32_t z = 0; z < nz; ++z) {
        for (int32_t y = 0; y < ny; ++y) {
            for (int32_t x = 0; x < nx; ++x) {
                Particle particle;

                const real_t fx = real_t(x) / real_t(nx - 1);
                const real_t fy = real_t(y) / real_t(ny - 1);
                const real_t fz = real_t(z) / real_t(nz - 1);

                particle.position = p_center + Vector3(
                    (fx - 0.5) * size.x,
                    (fy - 0.5) * size.y,
                    (fz - 0.5) * size.z
                );

                particle.previous_position = particle.position;
                particle.rest_position = particle.position;
                particles.push_back(particle);
            }
        }
    }

    auto add_beam = [&](int32_t x, int32_t y, int32_t z,
                        int32_t dx, int32_t dy, int32_t dz,
                        real_t stiffness) {
        const int32_t tx = x + dx;
        const int32_t ty = y + dy;
        const int32_t tz = z + dz;

        if (tx < 0 || tx >= nx || ty < 0 || ty >= ny || tz < 0 || tz >= nz) {
            return;
        }

        Beam beam;
        beam.a = index(x, y, z);
        beam.b = index(tx, ty, tz);
        beam.rest_length = particles[beam.a].position.distance_to(
            particles[beam.b].position
        );
        beam.initial_rest_length = beam.rest_length;
        beam.stiffness = stiffness;
        beam.yield_strain = default_yield_strain;
        beam.break_strain = default_break_strain;
        beam.plastic_creep = default_plastic_creep;

        if (beam.rest_length > real_t(0.0001)) {
            beams.push_back(beam);
        }
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

                            const real_t stiffness =
                                offset_count == 1 ? real_t(0.82) :
                                offset_count == 2 ? real_t(0.58) :
                                                     real_t(0.46);

                            add_beam(x, y, z, dx, dy, dz, stiffness);
                        }
                    }
                }

                // Longer span beams (weak; see review notes on bending).
                add_beam(x, y, z, 2, 0, 0, real_t(0.18));
                add_beam(x, y, z, 0, 2, 0, real_t(0.18));
                add_beam(x, y, z, 0, 0, 2, real_t(0.18));
            }
        }
    }

    diagnostics_dirty = true;
    set_physics_process(true);
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
        const real_t distance = particle.position.distance_to(p_local_position);

        if (distance >= p_radius) {
            continue;
        }

        const real_t weight = real_t(1.0) - distance / p_radius;

        particle.previous_position -=
            p_impulse * (weight * particle.inverse_mass * fixed_step);

        // Momentum change = mass * delta-v = (1 / inverse_mass) * (J * w * inverse_mass).
        // Particles with zero inverse mass do not move and receive nothing.
        if (particle.inverse_mass > real_t(0.0001)) {
            delivered += p_impulse * weight;
        }
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
    }

    for (Beam &beam : beams) {
        beam.rest_length = beam.initial_rest_length;
        beam.broken = false;
    }

    broken_beam_count = 0;
    accumulator = 0.0;
    max_strain = 0.0;
    frame_rotation = Quaternion();
    diagnostics_dirty = true;
}

void SoftBodySolver3D::set_gravity(const Vector3 &value) {
    gravity = value;
}

Vector3 SoftBodySolver3D::get_gravity() const {
    return gravity;
}

void SoftBodySolver3D::set_damping(real_t value) {
    damping = std::clamp(value, real_t(0.0), real_t(1.0));
}

real_t SoftBodySolver3D::get_damping() const {
    return damping;
}

void SoftBodySolver3D::set_solver_iterations(int32_t value) {
    solver_iterations = std::clamp(value, 1, 8);
}

int32_t SoftBodySolver3D::get_solver_iterations() const {
    return solver_iterations;
}

void SoftBodySolver3D::set_fixed_step(real_t value) {
    fixed_step = std::clamp(value, real_t(1.0 / 240.0), real_t(1.0 / 30.0));
}

real_t SoftBodySolver3D::get_fixed_step() const {
    return fixed_step;
}

void SoftBodySolver3D::set_yield_strain(real_t value) {
    default_yield_strain = std::clamp(value, real_t(0.001), real_t(10.0));

    for (Beam &beam : beams) {
        beam.yield_strain = default_yield_strain;
    }
}

real_t SoftBodySolver3D::get_yield_strain() const {
    return default_yield_strain;
}

void SoftBodySolver3D::set_break_strain(real_t value) {
    default_break_strain = std::clamp(value, real_t(0.001), real_t(10.0));

    for (Beam &beam : beams) {
        beam.break_strain = default_break_strain;
    }
}

real_t SoftBodySolver3D::get_break_strain() const {
    return default_break_strain;
}

void SoftBodySolver3D::set_plastic_creep(real_t value) {
    default_plastic_creep = std::clamp(value, real_t(0.0), real_t(1.0));

    for (Beam &beam : beams) {
        beam.plastic_creep = default_plastic_creep;
    }
}

real_t SoftBodySolver3D::get_plastic_creep() const {
    return default_plastic_creep;
}

void SoftBodySolver3D::solve_constraints() {
    for (int32_t iteration = 0; iteration < solver_iterations; ++iteration) {
        for (Beam &beam : beams) {
            if (beam.broken) {
                continue;
            }

            Particle &a = particles[beam.a];
            Particle &b = particles[beam.b];

            Vector3 difference = b.position - a.position;
            const real_t distance = difference.length();

            if (distance <= real_t(0.0001)) {
                continue;
            }

            const real_t error = distance - beam.rest_length;
            const Vector3 correction =
                difference / distance * error * beam.stiffness;

            const real_t inverse_total = a.inverse_mass + b.inverse_mass;

            if (inverse_total <= real_t(0.0001)) {
                continue;
            }

            a.position += correction * (a.inverse_mass / inverse_total);
            b.position -= correction * (b.inverse_mass / inverse_total);
        }
    }
}

void SoftBodySolver3D::simulate_step(real_t delta) {
    max_strain = 0.0;

    for (Particle &particle : particles) {
        const Vector3 current = particle.position;
        const Vector3 velocity = (particle.position - particle.previous_position) * damping;

        particle.position += velocity + gravity * delta * delta;
        particle.previous_position = current;
    }

    // Plasticity and break checks use the uncorrected frame strain.
    // (Unchanged in Stage 2a; revisited in Stage 3.)
    for (Beam &beam : beams) {
        if (beam.broken) {
            continue;
        }

        const real_t distance = particles[beam.a].position.distance_to(
            particles[beam.b].position
        );

        if (beam.rest_length <= real_t(0.0001)) {
            continue;
        }

        const real_t strain = std::abs(distance - beam.rest_length) / beam.rest_length;
        max_strain = std::max(max_strain, strain);

        if (strain >= beam.break_strain) {
            beam.broken = true;
            ++broken_beam_count;
            continue;
        }

        if (strain >= beam.yield_strain) {
            const real_t creep = beam.plastic_creep * (delta / fixed_step);

            beam.rest_length += (distance - beam.rest_length) * creep;

            beam.rest_length = std::clamp(
                beam.rest_length,
                beam.initial_rest_length * real_t(0.65),
                beam.initial_rest_length * real_t(1.50)
            );
        }
    }

    solve_constraints();
    diagnostics_dirty = true;
}

void SoftBodySolver3D::_physics_process(double p_delta) {
    if (particles.empty()) {
        return;
    }

    accumulator += std::min(real_t(p_delta), real_t(0.1));

    int32_t steps = 0;

    while (accumulator >= fixed_step && steps < MAX_STEPS_PER_FRAME) {
        simulate_step(fixed_step);
        accumulator -= fixed_step;
        ++steps;
    }

    // Discard excess backlog rather than allowing an expensive catch-up spiral.
    if (steps == MAX_STEPS_PER_FRAME && accumulator > fixed_step) {
        accumulator = fixed_step;
    }
}

void SoftBodySolver3D::compute_diagnostics() const {
    Diagnostics d;

    const real_t epsilon = real_t(0.0001);
    const real_t inv_dt = real_t(1.0) / fixed_step;

    // Pass 1: mass, centers of mass, momentum. Particles with zero inverse
    // mass are treated as pinned and excluded.
    real_t total_mass = 0.0;
    Vector3 com;
    Vector3 rest_com;
    Vector3 momentum;

    for (const Particle &p : particles) {
        if (p.inverse_mass <= epsilon) {
            continue;
        }

        const real_t m = real_t(1.0) / p.inverse_mass;
        total_mass += m;
        com += p.position * m;
        rest_com += p.rest_position * m;
        momentum += (p.position - p.previous_position) * (inv_dt * m);
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
        if (p.inverse_mass <= epsilon) {
            continue;
        }

        const real_t m = real_t(1.0) / p.inverse_mass;
        const Vector3 velocity = (p.position - p.previous_position) * inv_dt;
        const Vector3 r = p.position - com;
        const Vector3 q = p.rest_position - rest_com;

        angular += r.cross(velocity) * m;
        kinetic += real_t(0.5) * m * velocity.length_squared();

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

    // Post-solve strain and plastic beam count.
    for (const Beam &beam : beams) {
        if (beam.broken || beam.rest_length <= epsilon) {
            continue;
        }

        const real_t distance = particles[beam.a].position.distance_to(
            particles[beam.b].position
        );
        const real_t strain = std::abs(distance - beam.rest_length) / beam.rest_length;
        d.max_strain_post_solve = std::max(d.max_strain_post_solve, strain);

        if (beam.initial_rest_length > epsilon &&
            std::abs(beam.rest_length / beam.initial_rest_length - real_t(1.0)) >
                real_t(0.005)) {
            ++d.yielded_beam_count;
        }
    }

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

real_t SoftBodySolver3D::get_total_mass() const {
    ensure_diagnostics();
    return diagnostics.total_mass;
}

Vector3 SoftBodySolver3D::get_center_of_mass() const {
    ensure_diagnostics();
    return diagnostics.center_of_mass;
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
