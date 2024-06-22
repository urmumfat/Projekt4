#include "planar_quadrotor_visualizer.h"
#include <SDL2_gfx/SDL2_gfxPrimitives.h>

const int MAX_SCREEN_WIDTH = 1280;
const int MAX_SCREEN_HEIGHT = 720;

PlanarQuadrotorVisualizer::PlanarQuadrotorVisualizer(PlanarQuadrotor* quadrotor_ptr, int screen_w, int screen_h)
    : quadrotor_ptr(quadrotor_ptr), screen_w(screen_w), screen_h(screen_h) {}

void PlanarQuadrotorVisualizer::render(std::shared_ptr<SDL_Renderer>& gRenderer) {
    Eigen::VectorXf state = quadrotor_ptr->GetState();
    float q_x = state[0] * (screen_w / 2) / 1.0; // Scaling to meters
    float q_y = state[1] * (screen_h / 2) / 1.0; // Scaling to meters
    float q_theta = -state[2]; // Inverting the tilt angle
    // Transform the quadrotor's position to the screen center
    int screen_center_x = screen_w / 2;
    int screen_center_y = screen_h / 2;
    int rect_x = static_cast<int>(screen_center_x + q_x);
    int rect_y = static_cast<int>(screen_center_y - q_y);

    // New dimensions for the quadrotor's body
    int body_width = 80;
    int body_height = 10;

    // Calculate the coordinates of the body rectangle's corners
    float half_body_width = body_width / 2.0f;
    float half_body_height = body_height / 2.0f;

    float cos_theta = cos(q_theta);
    float sin_theta = sin(q_theta);

    // Coordinates for the body rectangle
    Sint16 body_vx[4];
    Sint16 body_vy[4];

    body_vx[0] = rect_x + (-half_body_width * cos_theta - (-half_body_height) * sin_theta);
    body_vy[0] = rect_y + (-half_body_width * sin_theta + (-half_body_height) * cos_theta);

    body_vx[1] = rect_x + (half_body_width * cos_theta - (-half_body_height) * sin_theta);
    body_vy[1] = rect_y + (half_body_width * sin_theta + (-half_body_height) * cos_theta);

    body_vx[2] = rect_x + (half_body_width * cos_theta - (half_body_height)*sin_theta);
    body_vy[2] = rect_y + (half_body_width * sin_theta + (half_body_height)*cos_theta);

    body_vx[3] = rect_x + (-half_body_width * cos_theta - (half_body_height)*sin_theta);
    body_vy[3] = rect_y + (-half_body_width * sin_theta + (half_body_height)*cos_theta);

    // Set the color for the quadrotor's body
    SDL_SetRenderDrawColor(gRenderer.get(), 0x70, 0x70, 0x70, 0xFF);

    // Draw the filled polygon (body rectangle)
    filledPolygonRGBA(gRenderer.get(), body_vx, body_vy, 4, 0x70, 0x70, 0x70, 0xFF);


    // Draw the thrust vectors u1 and u2
    float thrust_length = 20.0f;
    int thrust_thickness = 3; // Thickness of the thrust vector

    // Thrust vector u1 (left)
    int u1_x = rect_x - static_cast<int>(half_body_width * cos_theta);
    int u1_y = rect_y - static_cast<int>(half_body_width * sin_theta);
    int u1_end_x = u1_x - static_cast<int>(thrust_length * sin_theta);
    int u1_end_y = u1_y - static_cast<int>(thrust_length * cos_theta);

    // Thrust vector u2 (right)
    int u2_x = rect_x + static_cast<int>(half_body_width * cos_theta);
    int u2_y = rect_y + static_cast<int>(half_body_width * sin_theta);
    int u2_end_x = u2_x - static_cast<int>(thrust_length * sin_theta);
    int u2_end_y = u2_y - static_cast<int>(thrust_length * cos_theta);

    // Set the color for the thrust vectors
    SDL_SetRenderDrawColor(gRenderer.get(), 0x00, 0x00, 0x00, 0xFF);

    // Draw thick thrust vectors as filled rectangles
    // Thrust vector u1 (left)
    SDL_Rect u1_rect;
    u1_rect.x = u1_x - thrust_thickness / 2;
    u1_rect.y = u1_y - thrust_length;
    u1_rect.w = thrust_thickness;
    u1_rect.h = thrust_length;
    SDL_RenderFillRect(gRenderer.get(), &u1_rect);

    // Thrust vector u2 (right)
    SDL_Rect u2_rect;
    u2_rect.x = u2_x - thrust_thickness / 2;
    u2_rect.y = u2_y - thrust_length;
    u2_rect.w = thrust_thickness;
    u2_rect.h = thrust_length;
    SDL_RenderFillRect(gRenderer.get(), &u2_rect);

    float rotor_radiusY = 4.0f;

    // Rotor 1 (left)
    filledEllipseRGBA(gRenderer.get(), u1_end_x, u1_end_y, rotor_radiusX, rotor_radiusY, 0x00, 0x00, 0xFF, 0xFF);

    // Rotor 2 (right)
    filledEllipseRGBA(gRenderer.get(), u2_end_x, u2_end_y, rotor_radiusX, rotor_radiusY, 0x00, 0x00, 0xFF, 0xFF);

    // Update propeller width for next frame
    rotor_radiusX += 0.1f; // Adjust this increment value as needed

    // Ensure the width does not exceed a maximum value
    if (rotor_radiusX >= 30.0f) {
        rotor_radiusX = 15.0f;
    }
}

void PlanarQuadrotorVisualizer::MouseClickfollow(int x, int y) {
    // Transform the mouse click position to quadrotor coordinates
    float q_x = static_cast<float>(x - screen_w / 2) / (screen_w / 2) * 1.0; // Scaling to meters
    float q_y = static_cast<float>(screen_h / 2 - y) / (screen_h / 2) * 1.0; // Scaling to meters
    // Limit quadrotor movement within the screen boundaries
    if (q_x < -1.0)
        q_x = -1.0;
    else if (q_x > 1.0)
        q_x = 1.0;

    if (q_y < -1.0)
        q_y = -1.0;
    else if (q_y > 1.0)
        q_y = 1.0;

    // Set the new goal state for the quadrotor
    Eigen::VectorXf new_goal = Eigen::VectorXf::Zero(6);
    new_goal[0] = q_x;
    new_goal[1] = q_y;
    new_goal[2] = 0;
    quadrotor_ptr->SetGoal(new_goal);
}