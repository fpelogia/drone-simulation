import pygame
import pygame_gui
from scipy.integrate import RK45
import numpy as np
from controllers.controller import ControllerFSF
from dynamics import drone_dynamics
from plots import plot_results


# pygame setup
pygame.init()
resolution = (1280, 720)
screen = pygame.display.set_mode(resolution)
manager = pygame_gui.UIManager(resolution)
pygame.font.init()

normal_font = pygame.font.Font(None, 32)
big_font = pygame.font.Font(None, 50)

dropdown = pygame_gui.elements.UIDropDownMenu(
    options_list=[
        'Target Point (Click)',
        'Trajectory (Draw)'
    ],
    starting_option="Target Point (Click)",
    relative_rect=pygame.Rect(10, 10, 200, 30),
    manager=manager,
)

text_plot = normal_font.render("Press [RETURN] to view plots", True, "white")

# load assets
bg_image = pygame.image.load("misc/background.jpg").convert()
drone_img = pygame.image.load("misc/drone.png").convert_alpha()
bg_image = pygame.transform.scale(bg_image, resolution)

# simulation and rendering timing
clock = pygame.time.Clock()
dt_frame = 0

SCALE = 40
PHYSICS_DT = 1 / 120
t_start = 0
t_end = np.inf

sim_time = 0.0
accumulator = 0.0
running = True

# control state
drawing = False
drawing_start_time = 0
target_x = 0
target_y = 0
plot_start_index = 0
control_objective = 'point'

def change_control_objective(selected_value):
    global control_objective

    print(f"Control objective changed: {selected_value}")

    if selected_value == 'Target Point (Click)':
        control_objective = 'point'
    elif selected_value == 'Trajectory (Draw)':
        control_objective = 'trajectory'

    controller.target_fn = lambda t: (15, 10)
    restart()

# system parameters (2D Drone)
m = 2 # mass (kg)
L = 2 # length (m)
g = 9.81 # gravity (m/s^2)
I = (1/12) * m * L**2 # moment of inertia (kg*m^2)

params = {"m": m, "L": L, "g": g, "I": I}

# rescale drone image
drone_width = int(SCALE * L)
drone_height = int(
    drone_width * (drone_img.get_height() / drone_img.get_width())
) # Mantém a proporção da imagem

drone_img = pygame.transform.smoothscale(
    drone_img,
    (drone_width, drone_height)
)

# controller and trajectory state
z = np.zeros(6)

controller = ControllerFSF(
    type='lqr',
    target_fn=lambda t: (15, 10)
)

traj_x = []
traj_y = []

t_history = []
x_history = []
y_history = []
theta_history = []

def handle_events():
    global running, drawing, drawing_start_time
    global target_x, target_y, plot_start_index

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

        elif event.type == pygame.KEYDOWN and event.key == pygame.K_RETURN:
            if len(t_history) > 1:
                plot_results(
                    np.array(t_history[plot_start_index:]),
                    np.array(x_history[plot_start_index:]),
                    np.array(y_history[plot_start_index:]),
                    np.array(theta_history[plot_start_index:]),
                    controller.target_fn
                )

        elif event.type == pygame.MOUSEBUTTONUP and drawing:
            drawing = False

            if len(traj_x) > 1:
                drawing_duration = max(
                    1.0,
                    (pygame.time.get_ticks() - drawing_start_time) / 1000.0
                )

                effective_duration = max(
                    3.0,
                    drawing_duration * 2.0
                ) # Dá mais tempo para o drone acompanhar

                x_pts = np.array(traj_x)
                y_pts = np.array(traj_y)

                trajectory_start_time = sim_time
                trajectory_end_time = (
                    trajectory_start_time + effective_duration
                )

                plot_start_index = len(t_history)

                t_draw = np.linspace(
                    trajectory_start_time,
                    trajectory_end_time,
                    len(x_pts)
                )

                def target_traj(current_t):
                    clamped_t = np.clip(
                        current_t,
                        trajectory_start_time,
                        trajectory_end_time
                    )

                    x_target = np.interp(
                        clamped_t,
                        t_draw,
                        x_pts
                    ) / SCALE

                    y_target = (
                        screen.get_height() -
                        np.interp(clamped_t, t_draw, y_pts)
                    ) / SCALE

                    return x_target, y_target

                controller.target_fn = target_traj

        elif event.type == pygame.MOUSEBUTTONDOWN:

            # ignore if user clicks the UI
            if dropdown.relative_rect.collidepoint(event.pos):
                continue

            if event.pos == None:
                continue

            if control_objective == 'point':
                target_x = event.pos[0]
                target_y = event.pos[1]
                plot_start_index = len(t_history)

                def target_traj(t):
                    return (
                        target_x / SCALE,
                        (screen.get_height() - target_y) / SCALE
                    )

                controller.target_fn = target_traj

            elif control_objective == 'trajectory':
                traj_x.clear()
                traj_y.clear()

                plot_start_index = len(t_history)
                drawing = True
                drawing_start_time = pygame.time.get_ticks()

        if control_objective == 'trajectory' and pygame.mouse.get_pressed()[0]:
            mouse_x, mouse_y = pygame.mouse.get_pos()
            traj_x.append(mouse_x)
            traj_y.append(mouse_y)

    # Keyboard events
    keys = pygame.key.get_pressed()

    if keys[pygame.K_SPACE]:
        restart()


# Restart simulation and reset solver
def restart():
    global solver, z, sim_time, accumulator
    global t_history, x_history, y_history, theta_history
    global plot_start_index

    z = np.array([15., 0., 0., 0., 0., 0.])
    sim_time = 0.0
    accumulator = 0.0
    plot_start_index = 0

    t_history = [sim_time]
    x_history = [z[0]]
    y_history = [z[1]]
    theta_history = [z[2]]

    solver = RK45(
        fun=lambda t, z: drone_dynamics(t, z, params, controller),
        t0=sim_time,
        y0=z,
        t_bound=t_end,
        rtol=1e-3,
        atol=1e-6,
        max_step=PHYSICS_DT,
    )

# first time
restart()

while running:

    dt_frame = clock.tick(60) / 1000 # 60 fps

    manager.update(dt_frame)

    accumulator += dt_frame

    while accumulator >= PHYSICS_DT and solver.status == "running":
        solver.step()

        z = solver.y.copy()
        sim_time = solver.t

        t_history.append(sim_time)
        x_history.append(z[0])
        y_history.append(z[1])
        theta_history.append(z[2])

        accumulator -= PHYSICS_DT

    if solver.status == "failed":
        print("Solver failed:", solver.status)
        running = False

    # Handle Mouse, Keyboard, UI events
    handle_events()

    x, y, theta = z[:3]

    # Clear screen
    screen.blit(bg_image, (0, 0))

    # redraw trajectory or target point
    if control_objective == 'trajectory':
        for idx, (tx, ty) in enumerate(zip(traj_x, traj_y)):
            pygame.draw.circle(
                screen,
                (min(255, idx), 0, 0),
                (tx, ty),
                5
            )

    elif control_objective == 'point':
        pygame.draw.circle(
            screen,
            "red",
            (target_x, target_y),
            5
        )

    # create and rotate drone surface
    drone_surface = pygame.transform.rotate(
        drone_img,
        theta * 180 / np.pi
    )

    drone_surface_rect = drone_surface.get_rect(
        center=(
            SCALE * x,
            screen.get_height() - SCALE * y
        )
    )

    # draw drone
    screen.blit(drone_surface, drone_surface_rect.topleft)

    screen_rect = screen.get_rect()

    if not screen_rect.contains(drone_surface_rect):
        collision_text = big_font.render(
            "Oh no! You crashed!",
            True,
            "white"
        )

        text_rect = collision_text.get_rect(
            center=(resolution[0] // 2, 80)
        )

        screen.blit(collision_text, text_rect)

        collision_text = big_font.render(
            "Press [SPACEBAR] to restart!",
            True,
            "white"
        )

        text_rect = collision_text.get_rect(
            center=(resolution[0] // 2, 120)
        )

        screen.blit(collision_text, text_rect)

    text_time = normal_font.render(
        f"t={round(sim_time, 2)}s",
        True,
        "white"
    )

    screen.blit(text_time, (250, 15))

    if pygame.key.get_pressed()[pygame.K_RETURN]:
        text_plot = normal_font.render(
            "Please close the figure to continue the simulation.",
            True,
            "yellow"
        )
    else:
        text_plot = normal_font.render(
            "Press [RETURN] to view plots",
            True,
            "white"
        )

    screen.blit(text_plot, (350, 15))

    manager.draw_ui(screen)

    pygame.display.flip()


pygame.quit()