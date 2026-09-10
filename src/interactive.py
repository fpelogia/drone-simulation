import pygame
import pygame_gui
from scipy.integrate import solve_ivp
import numpy as np
from controllers.controller import ControllerFSF
from dynamics import drone_dynamics

# pygame setup
pygame.init()
resolution = (1280, 720)
screen = pygame.display.set_mode(resolution)

bg_image = pygame.image.load("misc/background.jpg").convert()
drone_img = pygame.image.load("misc/drone.jpg").convert()
bg_image = pygame.transform.scale(bg_image, (resolution[0], resolution[1]))


manager = pygame_gui.UIManager(resolution)
clock = pygame.time.Clock()
running = True
dt = 0

t_i = 0
SCALE = 40
drawing = False

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
t = np.linspace(t_start, t_end, 100)

z = [0,0,0,0,0,0]
x, y, theta, x_dot, y_dot, theta_dot = z

def restart():
    global z
    z = rerun([0,0,0,0,0,0], lambda t : (10,10))

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
my_font = pygame.font.Font(None, 32)

restart()

delay = 0

traj_x = []
traj_y = []

while running:

    # limits FPS to 60
    # dt is delta time in seconds since last frame, used for framerate-
    # independent physics.
    dt = clock.tick(60) / 1000

    manager.update(dt)


    delay += 1
    if(delay > 5 and t[t_i] < t_end):
        t_i += 1
        delay = 0
    # poll for events
    # pygame.QUIT event means the user clicked X to close your window
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
                x_pts = np.array(traj_x)
                y_pts = np.array(traj_y)
                t_draw = np.linspace(t_start, t_end, len(x_pts))

                def target_traj(current_t):
                    # interpolate recorded trajectory
                    clamped_t = np.clip(current_t, t_start, t_end)
                    x_target = np.interp(clamped_t, t_draw, x_pts) / SCALE
                    y_target = (screen.get_height() - np.interp(clamped_t, t_draw, y_pts)) / SCALE
                    return x_target, y_target

                z = rerun(z[:,-1], target_traj)
                t_i = 0

        elif event.type == pygame.MOUSEBUTTONDOWN:
            print('MOUSEEEEEEE')

            if control_objective == 'point':
                def target_traj(t):
                    return event.pos[0]/SCALE, (screen.get_height() - event.pos[1])/SCALE 
                z = rerun(z[:,-1], target_traj)
                t_i = 0
            elif control_objective == 'trajectory':
                traj_x.clear()
                traj_y.clear()
                drawing = True
                

        if control_objective == 'trajectory' and pygame.mouse.get_pressed()[0]:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            traj_x.append(mouse_x)
            traj_y.append(mouse_y)

    # fill the screen with a color to wipe away anything from last frame
    #screen.fill("black")
    screen.blit(bg_image, (0, 0))

    if control_objective == 'trajectory':
        for (tx, ty) in zip(traj_x, traj_y):
            pygame.draw.circle(screen, "gray", (tx, ty), 2)

    # create drone surface
    #drone_surface = pygame.Surface((SCALE*L, 10), pygame.SRCALPHA)
    #drone_surface.fill((255, 0, 0))  # Red rectangle


    if(t_i < len(t)):
        drone_surface = pygame.transform.rotate(drone_surface, theta[t_i]*180/np.pi)
        drone_surface_rect = drone_surface.get_rect(center=(SCALE*x[t_i], screen.get_height() - SCALE*y[t_i]))

    # draw drone
    screen.blit(drone_surface, drone_surface_rect.topleft)

    screen_rect = screen.get_rect()
    if not screen_rect.contains(drone_surface_rect):
        collision_text = my_font.render("COLLIDED!", True, "yellow")
        text_rect = collision_text.get_rect(center=(resolution[0] // 2, 80))
        screen.blit(collision_text, text_rect)

    text_time = my_font.render(f"t={round(t[t_i],2)}s", True, "green")
    screen.blit(text_time, (300, 15))

    keys = pygame.key.get_pressed()
    if keys[pygame.K_SPACE]:
        t_i = 0
        restart()

    manager.draw_ui(screen)

    # flip() the display to put your work on screen
    pygame.display.flip()


pygame.quit()