/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include <matplot/matplot.h>

std::vector<float> x_history;
std::vector<float> y_history;
std::vector<float> theta_history;

Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    // Calculate LQR gain matrix
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 10, 10, 10, 1, 10, 1.5 / 2 / M_PI;
    R.row(0) << 0.1, 0.05;
    R.row(1) << 0.05, 0.1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void plotTrajectory() {
    if (x_history.empty() || y_history.empty()) {
        std::cerr << "Error: History vectors are empty. Cannot plot trajectory." << std::endl;
        return;
    }
    if (x_history.size() != y_history.size()) {
        std::cerr << "Error: History vectors have different sizes. Cannot plot trajectory." << std::endl;
        return;
    }

    matplot::title("Quadrotor Trajectory");
    matplot::plot(x_history, y_history)->color({ 1.0f, 0.08f, 0.58f });
    matplot::xlabel("X position");
    matplot::ylabel("Y position");
    matplot::show();
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void drawFlag(SDL_Renderer* renderer, int visual_x, int visual_y) {
    Sint16 x1 = visual_x;
    Sint16 y1 = visual_y - 20;
    Sint16 x2 = visual_x;
    Sint16 y2 = visual_y - 50;
    Sint16 x3 = visual_x + 30;
    Sint16 y3 = visual_y - 35;
    filledTrigonRGBA(renderer, x1, y1, x2, y2, x3, y3, 255, 78, 171, 255);

    SDL_SetRenderDrawColor(renderer, 100, 100, 100, 255); // Gray color for the pole
    SDL_RenderDrawLine(renderer, visual_x, visual_y - 50, visual_x, visual_y); // Flag pole
    SDL_RenderDrawLine(renderer, visual_x - 1, visual_y - 50, visual_x - 1, visual_y); // Thickening the flag pole
    SDL_RenderDrawLine(renderer, visual_x + 1, visual_y - 50, visual_x + 1, visual_y); // Thickening the flag pole
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor);

    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);

    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        int x, y;
        bool destinationReached = false;

        // Main loop
        while (!quit)
        {
            // Handle SDL events
            while (SDL_PollEvent(&e) != 0)
            {
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN)
                {
                    // Handle mouse clicks for setting the goal state
                    if (e.button.button == SDL_BUTTON_LEFT) {
                        SDL_GetMouseState(&x, &y);
                        float quad_x = (static_cast<float>(x) - SCREEN_WIDTH / 2) / 128.0f;
                        float quad_y = (SCREEN_HEIGHT / 2 - static_cast<float>(y)) / 128.0f;
                        goal_state << quad_x, quad_y, 0, 0, 0, 0;
                        quadrotor.SetGoal(goal_state);
                        destinationReached = false; // Reset flag when goal is updated
                    }
                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p)
                {
                    // Plot trajectory
                    plotTrajectory();
                }
            }

            // Update quadrotor simulation
            quadrotor.Update(dt);
            x_history.push_back(quadrotor.GetState()[0]);
            y_history.push_back(quadrotor.GetState()[1]);
            theta_history.push_back(quadrotor.GetState()[2]);

            // Check if destination is reached
            if (!destinationReached) {
                Eigen::Vector2f currentPos = quadrotor.GetState().head<2>();
                Eigen::Vector2f goalPos = goal_state.head<2>();
                float distance = (currentPos - goalPos).norm();
                if (distance < 0.1) { // Adjust the threshold as needed
                    destinationReached = true; // Set flag when destination is reached
                }
            }

            // Render scene
            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            quadrotor_visualizer.render(gRenderer);

            // Draw flag only if destination is not reached
            if (!destinationReached) {
                int visual_goal_x = static_cast<int>(goal_state[0] * 128.0f) + SCREEN_WIDTH / 2;
                int visual_goal_y = SCREEN_HEIGHT / 2 - static_cast<int>(goal_state[1] * 128.0f);
                drawFlag(gRenderer.get(), visual_goal_x, visual_goal_y);
            }

            SDL_RenderPresent(gRenderer.get());

            // Control quadrotor using LQR
            control(quadrotor, K);
            quadrotor.Update(dt);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO) >= 0)
    {
        SDL_SetHint(SDL_HINT_RENDER_SCALE_QUALITY, "1");
        gWindow = std::shared_ptr<SDL_Window>(SDL_CreateWindow("Planar Quadrotor", SDL_WINDOWPOS_UNDEFINED, SDL_WINDOWPOS_UNDEFINED, SCREEN_WIDTH, SCREEN_HEIGHT, SDL_WINDOW_SHOWN), SDL_DestroyWindow);
        gRenderer = std::shared_ptr<SDL_Renderer>(SDL_CreateRenderer(gWindow.get(), -1, SDL_RENDERER_ACCELERATED), SDL_DestroyRenderer);
        SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
    }
    else
    {
        std::cout << "SDL_ERROR: " << SDL_GetError() << std::endl;
        return -1;
    }
    return 0;
}