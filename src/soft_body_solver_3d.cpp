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
    clear_body();
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
}

void SoftBodySolver3D::clear_body() {
    particles.clear();
    beams.clear();
    accumulator = 0.0;
    max_strain = 0.0;
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
                            if (dx == 0 && dy == 0 && dz == 0) {
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

                // Longer bending beams.
                add_beam(x, y, z, 2, 0, 0, real_t(0.18));
                add_beam(x, y, z, 0, 2, 0, real_t(0.18));
                add_beam(x, y, z, 0, 0, 2, real_t(0.18));
            }
        }
    }

    set_physics_process(true);
}

void SoftBodySolver3D::apply_impulse(
    const Vector3 &p_local_position,
    const Vector3 &p_impulse,
    real_t p_radius
) {
    if (p_radius <= real_t(0.0001)) {
        return;
    }

    for (Particle &particle : particles) {
        const real_t distance = particle.position.distance_to(p_local_position);

        if (distance >= p_radius) {
            continue;
        }

        const real_t weight = real_t(1.0) - distance / p_radius;

        particle.previous_position -=
            p_impulse * (weight * particle.inverse_mass * fixed_step);
    }
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
}

void SoftBodySolver3D::_physics_process(double p_delta) {
    if (particles.empty()) {
        return;
    }

    accumulator += std::min(real_t(p_delta), real_t(0.1));

    int32_t steps = 0;

    while (accumulator >= fixed_step && steps < 4) {
        simulate_step(fixed_step);
        accumulator -= fixed_step;
        ++steps;
    }

    // Discard excess backlog rather than allowing an expensive catch-up spiral.
    if (steps == 4 && accumulator > fixed_step) {
        accumulator = fixed_step;
    }
}

PackedVector3Array SoftBodySolver3D::get_node_positions() const {
    PackedVector3Array result;

    for (const Particle &particle : particles) {
        result.push_back(particle.position);
    }

    return result;
}

PackedInt32Array SoftBodySolver3D::get_beam_indices() const {
    PackedInt32Array result;

    for (const Beam &beam : beams) {
        if (!beam.broken) {
            result.push_back(beam.a);
            result.push_back(beam.b);
        }
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
    int32_t count = 0;

    for (const Beam &beam : beams) {
        if (beam.broken) {
            ++count;
        }
    }

    return count;
}

real_t SoftBodySolver3D::get_max_strain() const {
    return max_strain;
}
