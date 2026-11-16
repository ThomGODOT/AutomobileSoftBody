#pragma once

#include <cstdint>
#include <vector>

#include <godot_cpp/classes/node3d.hpp>
#include <godot_cpp/variant/vector3.hpp>
#include <godot_cpp/variant/packed_vector3_array.hpp>
#include <godot_cpp/variant/packed_int32_array.hpp>

namespace godot {

class SoftBodySolver3D : public Node3D {
    GDCLASS(SoftBodySolver3D, Node3D)

private:
    struct Particle {
        Vector3 position;
        Vector3 previous_position;
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

    std::vector<Particle> particles;
    std::vector<Beam> beams;

    Vector3 gravity = Vector3(0.0, -9.81, 0.0);
    real_t damping = 0.995;
    real_t fixed_step = real_t(1.0 / 60.0);
    real_t accumulator = 0.0;
    real_t max_strain = 0.0;

    int32_t solver_iterations = 4;

    void simulate_step(real_t delta);
    void solve_constraints();

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

    void apply_impulse(
        const Vector3 &local_position,
        const Vector3 &impulse,
        real_t radius
    );

    void set_gravity(const Vector3 &value);
    Vector3 get_gravity() const;

    void set_damping(real_t value);
    real_t get_damping() const;

    void set_solver_iterations(int32_t value);
    int32_t get_solver_iterations() const;

    PackedVector3Array get_node_positions() const;
    PackedInt32Array get_beam_indices() const;

    int32_t get_node_count() const;
    int32_t get_beam_count() const;
    int32_t get_broken_beam_count() const;
    real_t get_max_strain() const;

    void _physics_process(double delta) override;
};

}
