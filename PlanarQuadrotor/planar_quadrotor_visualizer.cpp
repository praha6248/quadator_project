#include "planar_quadrotor_visualizer.h"
#include <cmath>

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr) : quadrotor_ptr(quadrotor_ptr) {}

float rotate_x(float x, float y, float cx, float cy, float angle) {
    float s = sin(angle);
    float c = cos(angle);
    x -= cx;
    y -= cy;
    return x * c - y * s + cx;
}

float rotate_y(float x, float y, float cx, float cy, float angle) {
    float s = sin(angle);
    float c = cos(angle);
    x -= cx;
    y -= cy;
    return x * s + y * c + cy;
}

void draw_rect(SDL_Renderer* renderer, int visual_x, int visual_y, float theta, int r, int g, int b, int x1, int x2, int y1, int y2) {
    std::vector<std::pair<int, int>> rect_points = {
        {x2, y1},
        {x1, y1},
        {x1, y2},
        {x2, y2}
    };

    std::vector<std::pair<int, int>> rotated_rect_points;
    float cos_theta = cos(theta);
    float sin_theta = sin(theta);
    for (const auto& point : rect_points) {
        int x = point.first;
        int y = point.second;
        int rotated_x = static_cast<int>(x * cos_theta - y * sin_theta);
        int rotated_y = static_cast<int>(x * sin_theta + y * cos_theta);
        rotated_rect_points.push_back({ rotated_x, rotated_y });
    }

    std::vector<Sint16> polyX;
    std::vector<Sint16> polyY;
    for (const auto& point : rotated_rect_points) {
        int x = point.first + visual_x;
        int y = point.second + visual_y;
        polyX.push_back(x);
        polyY.push_back(y);
    }

    filledPolygonRGBA(renderer, polyX.data(), polyY.data(), polyX.size(), r, g, b, 255);
}

void drawBody(SDL_Renderer* renderer, int visual_x, int visual_y, float theta) {
    draw_rect(renderer, visual_x, visual_y, theta, 100, 100, 100, 20, -20, -28, 20);
    draw_rect(renderer, visual_x, visual_y, theta, 126, 126, 126, 15, -20, -28, 20);
    draw_rect(renderer, visual_x, visual_y, theta, 100, 100, 100, 60, -60, -5, 5);
    draw_rect(renderer, visual_x, visual_y, theta, 100, 100, 100, -62, -68, -20, 10);
    draw_rect(renderer, visual_x, visual_y, theta, 100, 100, 100, 62, 68, -20, 10);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 78, 171, 30, -30, -20, 20);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 131, 196, 20, -30, -20, 20);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 205, 231, -15, -25, -10, -16);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 205, 231, -7, -12, -10, -16);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 78, 171, -55, -75, -10, 10);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 131, 196, -60, -75, -10, 10);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 78, 171, 55, 75, -10, 10);
    draw_rect(renderer, visual_x, visual_y, theta, 255, 131, 196, 55, 70, -10, 10);
}

void drawPropellers(SDL_Renderer* renderer, int visual_x, int visual_y, float propeller_angle, bool clockwise) {
    int propeller_center_x = visual_x;
    int propeller_center_y = visual_y;

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
        filledEllipseRGBA(renderer, rotated_blade.first, rotated_blade.second, 10, 6, 255, 180, 219, 255);
    }
}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x, q_y, q_theta;

    q_x = state[0];
    q_y = state[1];
    q_theta = state[2];

    int visual_x = static_cast<int>(q_x * 256.0f) + 640;
    int visual_y = 360 - static_cast<int>(q_y * 256.0f);

    static float propeller_angle = 0.0f;
    propeller_angle += 0.04f;
    propeller_angle += 0.04f;

    float propeller1_rotated_x = rotate_x(visual_x - 65, visual_y-20, visual_x, visual_y, q_theta);
    float propeller1_rotated_y = rotate_y(visual_x - 65, visual_y-20, visual_x, visual_y, q_theta);

    float propeller2_rotated_x = rotate_x(visual_x + 65, visual_y-20, visual_x, visual_y, q_theta);
    float propeller2_rotated_y = rotate_y(visual_x + 65, visual_y-20, visual_x, visual_y, q_theta);

    drawBody(gRenderer.get(), visual_x, visual_y, q_theta);
    drawPropellers(gRenderer.get(), static_cast<int>(propeller1_rotated_x), static_cast<int>(propeller1_rotated_y), propeller_angle, true);
    drawPropellers(gRenderer.get(), static_cast<int>(propeller2_rotated_x), static_cast<int>(propeller2_rotated_y), propeller_angle, false);
}
