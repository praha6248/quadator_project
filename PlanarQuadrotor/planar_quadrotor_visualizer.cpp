#include "planar_quadrotor_visualizer.h"

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

/**
 * TODO: Improve visualizetion
 * 1. Transform coordinates from quadrotor frame to image frame so the circle is in the middle of the screen
 * 2. Use more shapes to represent quadrotor (e.x. try replicate http://underactuated.mit.edu/acrobot.html#section3 or do something prettier)
 * 3. Animate proppelers (extra points)
 */
void drawBody(SDL_Renderer* renderer, int visual_x, int visual_y) {
    // GREY
    roundedBoxRGBA(renderer, visual_x - 20, visual_y - 28, visual_x + 20, visual_y + 20, 4, 100, 100, 100, 255);
    roundedBoxRGBA(renderer, visual_x - 20, visual_y - 28, visual_x + 15, visual_y + 20, 4, 126, 126, 126, 255);
    roundedBoxRGBA(renderer, visual_x - 60, visual_y - 5, visual_x + 60, visual_y + 5, 4, 100, 100, 100, 255);
    roundedBoxRGBA(renderer, visual_x - 68, visual_y - 20, visual_x - 62, visual_y + 10, 2, 100, 100, 100, 255);
    roundedBoxRGBA(renderer, visual_x + 68, visual_y - 20, visual_x + 62, visual_y + 10, 2, 100, 100, 100, 255);
    // BODY
    roundedBoxRGBA(renderer, visual_x - 30, visual_y - 20, visual_x + 30, visual_y + 20, 7, 255, 78, 171, 255);
    roundedBoxRGBA(renderer, visual_x - 30, visual_y - 20, visual_x + 20, visual_y + 20, 7, 255, 131, 196, 255);
    roundedBoxRGBA(renderer, visual_x - 25, visual_y - 10, visual_x - 15, visual_y - 16, 2, 255, 205, 231, 255);
    roundedBoxRGBA(renderer, visual_x - 12, visual_y - 10, visual_x - 7, visual_y - 16, 2, 255, 205, 231, 255);
    // WINGS
    roundedBoxRGBA(renderer, visual_x - 75, visual_y - 10, visual_x - 55, visual_y + 10, 4, 255, 78, 171, 255);
    roundedBoxRGBA(renderer, visual_x - 75, visual_y - 10, visual_x - 60, visual_y + 10, 4, 255, 131, 196, 255);
    roundedBoxRGBA(renderer, visual_x + 75, visual_y - 10, visual_x + 55, visual_y + 10, 4, 255, 78, 171, 255);
    roundedBoxRGBA(renderer, visual_x + 70, visual_y - 10, visual_x + 55, visual_y + 10, 4, 255, 131, 196, 255);
}

void drawPropellers(SDL_Renderer* renderer, int visual_x, int visual_y, float propeller_angle, bool clockwise) {
    int propeller_center_x = visual_x;
    int propeller_center_y = visual_y - 20;

    float rotation_direction = clockwise ? 1.0f : -1.0f;
    float angle = propeller_angle * rotation_direction;

    auto rotate_point = [](float x, float y, float cx, float cy, float angle) {
        float s = sin(angle);
        float c = cos(angle);
        x -= cx;
        y -= cy;
        float xnew = x * c - y * s;
        float ynew = x * s + y * c;
        x = xnew + cx;
        y = ynew + cy;

        return std::make_pair(static_cast<int>(x), static_cast<int>(y));
        };

    std::vector<std::pair<int, int>> propeller_blades = {
        {propeller_center_x - 5, propeller_center_y}, {propeller_center_x + 5, propeller_center_y}
    };

    for (const auto& blade_center : propeller_blades) {
        auto rotated_blade = rotate_point(blade_center.first, blade_center.second, propeller_center_x, propeller_center_y, angle);
        filledEllipseRGBA(renderer, rotated_blade.first, rotated_blade.second, 12, 5, 255, 180, 219, 255);
    }
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    /* x, y, theta coordinates */
    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];
    // Transform coordinates to the center of the screen
    int visual_x = static_cast<int>(q_x * 128.0f) + 640;
    int visual_y = 360 - static_cast<int>(q_y * 128.0f);

    visual_x = std::max(40, std::min(visual_x, 1280 - 40));
    visual_y = std::max(40, std::min(visual_y, 720 - 40));

    static float propeller_angle = 0.0f; // Initial angle
    propeller_angle += 0.04f; // Increment angle for animation

    drawBody(gRenderer.get(), visual_x, visual_y);
    drawPropellers(gRenderer.get(), visual_x - 65, visual_y, propeller_angle, true);
    drawPropellers(gRenderer.get(), visual_x + 65, visual_y, propeller_angle, false);
}