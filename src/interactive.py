import pygame
import pygame_gui
from scipy.integrate import solve_ivp
import numpy as np
from controllers.controller import ControllerFSF
from dynamics import drone_dynamics
from plots import plot_results

# pygame setup
pygame.init()
resolution = (1280, 720)
screen = pygame.display.set_mode(resolution)

bg_image = pygame.image.load("misc/background.jpg").convert()
drone_img = pygame.image.load("misc/drone.png").convert_alpha()
bg_image = pygame.transform.scale(bg_image, (resolution[0], resolution[1]))

manager = pygame_gui.UIManager(resolution)
clock = pygame.time.Clock()
running = True
dt = 0

t_i = 0
SCALE = 40
drawing = False
target_x = 0
target_y = 0

control_objective = 'point'

def change_control_objective(selected_value):
    global control_objective
    print(f"Control objective changed: {selected_value}")
    if selected_value == 'Target Point (Click)':
        control_objective = 'point'
    elif selected_value == 'Trajectory (Draw)':
        control_objective = 'trajectory'


dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=[
        'Target Point (Click)',
        'Trajectory (Draw)'
    ],
    starting_option="Target Point (Click)", 
    relative_rect=pygame.Rect(10, 10, 200, 30),
    manager=manager,
)

# system parameters (2D Drone)
m = 2 # mass (kg)
L = 2 # length (m)
g = 9.81 # gravity (m/s^2)
I = (1/12) * m * L**2 # moment of inertia (kg*m^2)

params = {"m": m, "L": L, "g": g, "I": I}

# rescale drone image
drone_width = int(SCALE * L)
drone_height = int(drone_width * (drone_img.get_height() / drone_img.get_width())) # Mantém a proporção da imagem
drone_img = pygame.transform.smoothscale(drone_img, (drone_width, drone_height))

# time interval
t_start = 0
t_end = 30
t = np.linspace(t_start, t_end, 500)

z = [0,0,0,0,0,0]
x, y, theta, x_dot, y_dot, theta_dot = z

def restart():
    global z
    z = rerun([15,0,0,0,0,0], lambda t : (15,12))

def rerun(z0, target_traj):
    global x, y, theta, x_dot, y_dot, theta_dot

    # Instantiate the controller
    controller = ControllerFSF(type='lqr', target_fn=target_traj)

    # solve ODE
    sol = solve_ivp(drone_dynamics, (t_start, t_end), z0, t_eval=t, args=(params, controller), rtol=1e-3, atol=1e-6)
    print(sol)

    # update states globally
    x, y, theta, x_dot, y_dot, theta_dot = sol.y

    return sol.y

pygame.font.init()
normal_font = pygame.font.Font(None, 32)
big_font = pygame.font.Font(None, 50)

restart()

simulation_time_elapsed = 0.0
drawing_start_time = 0

traj_x = []
traj_y = []

while running:

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

    manager.update(dt)

    if t_i < len(t) - 1:
        dt_target = t[t_i + 1] - t[t_i]
        simulation_time_elapsed += dt
        
        while simulation_time_elapsed >= dt_target:
            t_i += 1
            simulation_time_elapsed -= dt_target
    # poll for events
    # pygame.QUIT event means the user clicked X to close the window
    events = pygame.event.get()
    for event in events:

        # process mouse event for select box (GUI)
        manager.process_events(event)

        if event.type == pygame.QUIT:
            running = False

        # If control objective was changed   
        elif event.type == pygame_gui.UI_DROP_DOWN_MENU_CHANGED:
            if event.ui_element == dropdown: 
                change_control_objective(event.text) 

        elif event.type == pygame.MOUSEBUTTONUP and drawing:
            drawing = False

            if len(traj_x) > 1:
                drawing_duration = max(1.0, (pygame.time.get_ticks() - drawing_start_time) / 1000.0)
                effective_duration = max(3.0, drawing_duration * 2.0) # Dá mais tempo para o drone acompanhar
                x_pts = np.array(traj_x)
                y_pts = np.array(traj_y)
                
                t_draw = np.linspace(t_start, effective_duration, len(x_pts))

                def target_traj(current_t):
                    clamped_t = np.clip(current_t, t_start, effective_duration)
                    
                    x_target = np.interp(clamped_t, t_draw, x_pts) / SCALE
                    y_target = (screen.get_height() - np.interp(clamped_t, t_draw, y_pts)) / SCALE
                    return x_target, y_target

                z = rerun(z[:,t_i], target_traj)
                t_i = 0

        elif event.type == pygame.MOUSEBUTTONDOWN:

            # ignore if user clicks the UI
            if dropdown.relative_rect.collidepoint(event.pos):
                continue

            if event.pos == None:
                continue

            if control_objective == 'point':
                target_x = event.pos[0]
                target_y = event.pos[1]

                def target_traj(t):
                    return target_x/SCALE, (screen.get_height() - target_y)/SCALE 
                
                pygame.draw.circle(screen, "red", (target_x, target_y), 5)
                z = rerun(z[:,t_i], target_traj)
                t_i = 0
            elif control_objective == 'trajectory':
                traj_x.clear()
                traj_y.clear()
                drawing = True
                drawing_start_time = pygame.time.get_ticks()
                

        if control_objective == 'trajectory' and pygame.mouse.get_pressed()[0]:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            traj_x.append(mouse_x)
            traj_y.append(mouse_y)


    # fill the screen with background to wipe away anything from last frame
    screen.blit(bg_image, (0, 0))

    # redraw trajectory or target point
    if control_objective == 'trajectory':
        for idx, (tx, ty) in enumerate(zip(traj_x, traj_y)):
            pygame.draw.circle(screen, (min(255, idx),0,0), (tx, ty), 5)
    elif control_objective == 'point':
        pygame.draw.circle(screen, "red", (target_x, target_y), 5)

    # create and rotate drone surface
    if(t_i < len(t)):
        drone_surface = pygame.transform.rotate(drone_img, theta[t_i] * 180 / np.pi)
        drone_surface_rect = drone_surface.get_rect(center=(SCALE * x[t_i], screen.get_height() - SCALE * y[t_i]))

    # draw drone
    screen.blit(drone_surface, drone_surface_rect.topleft)

    screen_rect = screen.get_rect()
    if not screen_rect.contains(drone_surface_rect):
        collision_text = big_font.render("Oh no! You crashed!", True, "white")
        text_rect = collision_text.get_rect(center=(resolution[0] // 2, 80))
        screen.blit(collision_text, text_rect)
        collision_text = big_font.render("Press [SPACEBAR] to restart!", True, "white")
        text_rect = collision_text.get_rect(center=(resolution[0] // 2, 120))
        screen.blit(collision_text, text_rect)

    if t_i < len(t):
        text_time = normal_font.render(f"t={round(t[t_i],2)}s", True, "white")
        screen.blit(text_time, (250, 15))

    keys = pygame.key.get_pressed()
    if keys[pygame.K_SPACE]:
        t_i = 0
        restart()
    if keys[pygame.K_RETURN]:
        if control_objective == 'point':
            def target_traj(t):
                return target_x/SCALE, (screen.get_height() - target_y)/SCALE 
        text_plot = normal_font.render(f"Please close the figure to continue the simulation.", True, "yellow")
        screen.blit(text_plot, (350, 15))
        # plot results
        plot_results(t, x, y, theta, target_traj)

    text_plot = normal_font.render(f"Press [RETURN] to view plots", True, "white")
    screen.blit(text_plot, (350, 15))

    manager.draw_ui(screen)

    pygame.display.flip()


pygame.quit()