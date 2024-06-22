/**
 * SDL window creation adapted from https://github.com/isJuhn/DoublePendulum
*/
#include "simulate.h"
#include<matplot/matplot.h>
#include <cmath>
#include <Eigen/Dense>
#include<SDL.h>
#include<fstream>
#include <stdio.h>
#include<SDL_audio.h>

// Structure to hold the audio data
struct AudioData {
    Uint8* pos;
    Uint32 length;
};

// Callback function to play the audio
void MyAudioCallback(void* userdata, Uint8* stream, int len) {
    AudioData* audio = (AudioData*)userdata;

    if (audio->length == 0) {
        return;
    }

    len = (len > audio->length ? audio->length : len);
    SDL_memcpy(stream, audio->pos, len);

    audio->pos += len;
    audio->length -= len;
}

// Function to load the audio file
bool LoadWAV(const char* file, AudioData& audio) {
    SDL_AudioSpec wav_spec;
    Uint8* wav_buffer;
    Uint32 wav_length;

    if (SDL_LoadWAV(file, &wav_spec, &wav_buffer, &wav_length) == nullptr) {
        std::cerr << "Error: Could not load WAV file." << std::endl;
        return false;
    }

    wav_spec.callback = MyAudioCallback;
    wav_spec.userdata = &audio;

    audio.pos = wav_buffer;
    audio.length = wav_length;

    if (SDL_OpenAudio(&wav_spec, nullptr) < 0) {
        std::cerr << "Error: Could not open audio." << std::endl;
        return false;
    }

    return true;
}
Eigen::MatrixXf LQR(PlanarQuadrotor& quadrotor, float dt) {
    /* Calculate LQR gain matrix */
    Eigen::MatrixXf Eye = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf A = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf A_discrete = Eigen::MatrixXf::Zero(6, 6);
    Eigen::MatrixXf B(6, 2);
    Eigen::MatrixXf B_discrete(6, 2);
    Eigen::MatrixXf Q = Eigen::MatrixXf::Identity(6, 6);
    Eigen::MatrixXf R = Eigen::MatrixXf::Identity(2, 2);
    Eigen::MatrixXf K = Eigen::MatrixXf::Zero(6, 6);
    Eigen::Vector2f input = quadrotor.GravityCompInput();

    Q.diagonal() << 10, 10, 10, 1, 10, 0.25 / 2 / M_PI;
    R.row(0) << 0.1, 0.05;
    R.row(1) << 0.05, 0.1;

    std::tie(A, B) = quadrotor.Linearize();
    A_discrete = Eye + dt * A;
    B_discrete = dt * B;

    return LQR(A_discrete, B_discrete, Q, R);
}

void control(PlanarQuadrotor& quadrotor, const Eigen::MatrixXf& K) {
    Eigen::Vector2f input = quadrotor.GravityCompInput();
    quadrotor.SetInput(input - K * quadrotor.GetControlState());
}

void plotQuadrotorPath(const std::vector<float>& x_coords, const std::vector<float>& y_coords) {
    using namespace matplot;
    auto fig = figure(true);
    plot(x_coords, y_coords);
    title("Quadrotor Flight Path");
    xlabel("X Position");
    ylabel("Y Position");
    show();

}


void limitQuadrotorPosition(Eigen::VectorXf& position, int screen_width, int screen_height) {
    // Limit quadrotor position within the screen boundaries
    if (position[0] < -screen_width / 2)
        position[0] = -screen_width / 2;
    else if (position[0] > screen_width / 2)
        position[0] = screen_width / 2;

    if (position[1] < -screen_height / 2)
        position[1] = -screen_height / 2;
    else if (position[1] > screen_height / 2)
        position[1] = screen_height / 2;
}

int main(int argc, char* args[])
{
    std::shared_ptr<SDL_Window> gWindow = nullptr;
    std::shared_ptr<SDL_Renderer> gRenderer = nullptr;
    const int SCREEN_WIDTH = 1280;
    const int SCREEN_HEIGHT = 720;
    int a = 0;
    
    // AudioData structure to hold audio information
    AudioData audio;
    if (!LoadWAV("C:/Users/adamj/Desktop/tp_projekt_4/build/Debug/tg80.wav", audio)) {
        std::cerr << "Failed to load audio file." << std::endl;
        return 1;
    }

    Eigen::VectorXf initial_state = Eigen::VectorXf::Zero(6);
    PlanarQuadrotor quadrotor(initial_state);
    PlanarQuadrotorVisualizer quadrotor_visualizer(&quadrotor, 1280, 720);
    /**
     * Goal pose for the quadrotor
     * [x, y, theta, x_dot, y_dot, theta_dot]
     * For implemented LQR controller, it has to be [x, y, 0, 0, 0, 0]
    */
    Eigen::VectorXf goal_state = Eigen::VectorXf::Zero(6);
    goal_state << 0, 0, 0, 0, 0, 0;
    quadrotor.SetGoal(goal_state);
    /* Timestep for the simulation */
    const float dt = 0.001;
    Eigen::MatrixXf K = LQR(quadrotor, dt);

    std::vector<float> x_coords;
    std::vector<float> y_coords;
    x_coords.push_back(0);
    y_coords.push_back(0);

    if (init(gWindow, gRenderer, SCREEN_WIDTH, SCREEN_HEIGHT) >= 0)
    {
        SDL_Event e;
        bool quit = false;
        float q_x = 0;
        float q_y = 0;
        
        SDL_PauseAudio(0);

        while (!quit)
        {
            //events
            while (SDL_PollEvent(&e) != 0)
            {
                
                if (e.type == SDL_QUIT)
                {
                    quit = true;
                }
                else if (e.type == SDL_MOUSEBUTTONDOWN && e.button.button == SDL_BUTTON_LEFT) {
                    int mouseX, mouseY;

                    SDL_GetMouseState(&mouseX, &mouseY);
                    

                    // Transform the mouse click position to quadrotor coordinates
                    q_x = static_cast<float>(SCREEN_WIDTH / 2 - mouseX);
                    q_y = static_cast<float>(SCREEN_HEIGHT / 2 - mouseY);

                    // Limit quadrotor movement within the screen boundaries
                    Eigen::VectorXf quadrotorPosition = Eigen::VectorXf::Zero(2);
                    
                    limitQuadrotorPosition(quadrotorPosition, SCREEN_WIDTH, SCREEN_HEIGHT);

                    quadrotor_visualizer.MouseClickfollow(mouseX, mouseY);

                }
                else if (e.type == SDL_KEYDOWN && e.key.keysym.sym == SDLK_p) {
                    plotQuadrotorPath(x_coords, y_coords);
                    quit = true;
                }
                
                
            }
            


            SDL_Delay((int)dt * 1000);

            SDL_SetRenderDrawColor(gRenderer.get(), 0xFF, 0xFF, 0xFF, 0xFF);
            SDL_RenderClear(gRenderer.get());

            /* Quadrotor rendering step */
            quadrotor_visualizer.render(gRenderer);

            SDL_RenderPresent(gRenderer.get());

            /* Simulate quadrotor forward in time */
            control(quadrotor, K);
            quadrotor.Update(dt);
            
            char movement;

            if (quadrotor.GetState()[1] > y_coords.back()) {
                movement = 'u';
                std::cout << "up";
            }
            else if (quadrotor.GetState()[1] < y_coords.back()) {
                movement = 'd';
                std::cout << "down";
            }
            else {
                movement = 's';
                std::cout << "stay";
            }

            x_coords.push_back(quadrotor.GetState()[0]);
            y_coords.push_back(quadrotor.GetState()[1]);
        }
    }
    SDL_Quit();
    return 0;
}

int init(std::shared_ptr<SDL_Window>& gWindow, std::shared_ptr<SDL_Renderer>& gRenderer, const int SCREEN_WIDTH, const int SCREEN_HEIGHT)
{
    if (SDL_Init(SDL_INIT_VIDEO | SDL_INIT_AUDIO) >= 0)
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

