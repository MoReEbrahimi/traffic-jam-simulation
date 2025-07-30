import pygame
import numpy as np
import sys

pygame.init()

WIDTH, HEIGHT = 1200, 800
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Traffic Simulation with Adaptive Charts")
clock = pygame.time.Clock()

WHITE = (255, 255, 255)
BLACK = (0, 0, 0)
GRAY = (180, 180, 180)
RED = (255, 50, 50)
ORANGE = (255, 165, 0)
BLUE = (30, 144, 255)

FONT = pygame.font.SysFont(None, 22)

# Small button
class Button:
    def __init__(self, x, y, w, h, label, callback):
        self.rect = pygame.Rect(x, y, w, h)
        self.label = label
        self.callback = callback

    def draw(self, surface):
        pygame.draw.rect(surface, BLACK, self.rect, border_radius=3)
        text = FONT.render(self.label, True, WHITE)
        surface.blit(text, (self.rect.x + 4, self.rect.y + 2))

    def handle_event(self, event):
        if event.type == pygame.MOUSEBUTTONDOWN and self.rect.collidepoint(event.pos):
            self.callback()

# Slider with +/- buttons
class Slider:
    def __init__(self, x, y, min_val, max_val, val, name, step=1):
        self.x, self.y = x, y
        self.min, self.max = min_val, max_val
        self.val = val
        self.name = name
        self.step = step
        self.width = 150
        self.dragging = False
        self.buttons = [
            Button(self.x - 25, self.y + 2, 20, 20, "-", self.decrease),
            Button(self.x + self.width + 5, self.y + 2, 20, 20, "+", self.increase)
        ]

    def draw(self, surface):
        pygame.draw.rect(surface, GRAY, (self.x, self.y + 10, self.width, 4))
        knob_x = self.x + ((self.val - self.min) / (self.max - self.min)) * self.width
        pygame.draw.circle(surface, RED if self.dragging else BLACK, (int(knob_x), self.y + 12), 8)
        label = FONT.render(f"{self.name}: {round(self.val, 2)}", True, BLACK)
        surface.blit(label, (self.x, self.y - 15))
        for btn in self.buttons:
            btn.draw(surface)

    def update(self, mouse_pos):
        rel_x = mouse_pos[0] - self.x
        percent = np.clip(rel_x / self.width, 0, 1)
        self.val = self.min + percent * (self.max - self.min)
        self.val = round(self.val / self.step) * self.step

    def handle_event(self, event):
        for btn in self.buttons:
            btn.handle_event(event)
        if event.type == pygame.MOUSEBUTTONDOWN and pygame.Rect(self.x, self.y, self.width, 20).collidepoint(event.pos):
            self.dragging = True
        elif event.type == pygame.MOUSEBUTTONUP:
            self.dragging = False
        elif event.type == pygame.MOUSEMOTION and self.dragging:
            self.update(event.pos)

    def increase(self):
        self.val = min(self.max, self.val + self.step)

    def decrease(self):
        self.val = max(self.min, self.val - self.step)

# Dynamic Y-axis max calculato
def get_max(data, min_val=1):
    return max(min_val, max(data) * 1.1) if data else min_val

# Draw chart
def draw_chart(surface, data_list, color, title, x, y, width, height, y_max):
    pygame.draw.rect(surface, BLACK, (x, y, width, height), 1)
    steps = 5
    for i in range(steps + 1):
        y_val = i * y_max / steps
        y_pos = y + height - (i * height / steps)
        pygame.draw.line(surface, GRAY, (x, y_pos), (x + width, y_pos), 1)
        label = FONT.render(f"{y_val:.1f}", True, BLACK)
        surface.blit(label, (x - 35, y_pos - 8))
    title_surf = FONT.render(title, True, BLACK)
    surface.blit(title_surf, (x + width // 2 - 40, y - 20))
    if len(data_list) > 1:
        for i in range(1, len(data_list)):
            x1 = x + (i - 1) * (width / len(data_list))
            x2 = x + i * (width / len(data_list))
            y1 = y + height - (data_list[i - 1] / y_max) * height
            y2 = y + height - (data_list[i] / y_max) * height
            pygame.draw.line(surface, color, (x1, y1), (x2, y2), 2)

# Traffic simulation functions
def init_sim(N_CARS, ROAD_LENGTH):
    positions = np.linspace(0, ROAD_LENGTH, N_CARS, endpoint=False)
    velocities = np.full(N_CARS, v0_slider.val) + np.random.normal(0, 2.0, N_CARS)
    velocities[0] -= 5
    return positions, velocities

def idm_acc(s, v, dv, a_max, b, v0, T, s0):
    s_star = s0 + v * T + (v * dv) / (2 * np.sqrt(a_max * b))
    acc = a_max * (1 - (v / v0)**4 - (s_star / max(s, 0.1))**2)
    return acc

def update_sim(positions, velocities, dt, ROAD_LENGTH, a_max, b, v0, T, s0, N_CARS):
    new_pos = positions.copy()
    new_vel = velocities.copy()
    colors = [BLUE] * N_CARS
    red_count = 0
    orange_count = 0

    for i in range(N_CARS):
        j = (i + 1) % N_CARS
        s = (positions[j] - positions[i]) % ROAD_LENGTH - 5
        dv = velocities[i] - velocities[j]
        acc = idm_acc(s, velocities[i], dv, a_max, b, v0, T, s0)

        new_vel[i] += acc * dt
        new_vel[i] = max(0, min(new_vel[i], v0))
        new_pos[i] = (positions[i] + new_vel[i] * dt) % ROAD_LENGTH

        if acc < -2:
            colors[i] = RED
            red_count += 1
        elif s < 8:
            colors[i] = ORANGE
            orange_count += 1

    mean_speed = np.mean(new_vel)
    return new_pos, new_vel, colors, mean_speed, red_count, orange_count

# Sliders
sliders = [
    Slider(30, 30, 20, 150, 50, "N_CARS", 1),
    Slider(30, 70, 200, 2000, 500, "ROAD_LEN", 10),
    Slider(30, 110, 5, 40, 10, "v₀", 0.5),
    Slider(30, 150, 0.1, 3.0, 0.4, "a_max", 0.1),
    Slider(30, 190, 0.1, 5.0, 3.4, "b", 0.1),
    Slider(30, 230, 0.5, 3.0, 0.7, "T", 0.1),
    Slider(30, 270, 0.0, 10.0, 2.1, "s₀", 0.1),
]
ncar_slider, road_slider, v0_slider, a_slider, b_slider, T_slider, s0_slider = sliders

# Main variables
positions, velocities = init_sim(int(ncar_slider.val), road_slider.val)
dt = 0.2
chart_data = []

# Main loop
running = True
while running:
    clock.tick(60)
    screen.fill(WHITE)

    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        for slider in sliders:
            slider.handle_event(event)

    if int(ncar_slider.val) != len(positions):
        positions, velocities = init_sim(int(ncar_slider.val), road_slider.val)

    positions, velocities, colors, mean_speed, red_ct, orange_ct = update_sim(
        positions, velocities, dt,
        road_slider.val,
        a_slider.val,
        b_slider.val,
        v0_slider.val,
        T_slider.val,
        s0_slider.val,
        int(ncar_slider.val)
    )

    chart_data.append((mean_speed, red_ct, orange_ct))
    if len(chart_data) > 300:
        chart_data.pop(0)

    # Draw cars on circle
    radius = 250
    cx, cy = WIDTH // 2 + 200, HEIGHT // 2 - 100
    for i, pos in enumerate(positions):
        theta = (pos / road_slider.val) * 2 * np.pi
        x = cx + radius * np.cos(theta)
        y = cy + radius * np.sin(theta)
        pygame.draw.circle(screen, colors[i], (int(x), int(y)), 5)

    for slider in sliders:
        slider.draw(screen)

     # Display numerical outputs
    stats = [
        f"Mean Speed: {mean_speed:.2f}",
        f"Red Cars: {red_ct}",
        f"Orange Cars: {orange_ct}"
    ]
    for i, line in enumerate(stats):
        txt = FONT.render(line, True, BLACK)
        screen.blit(txt, (30, 320 + i * 25))

    # Draw charts
    mean_speeds = [d[0] for d in chart_data]
    reds = [d[1] for d in chart_data]
    oranges = [d[2] for d in chart_data]

    ymax_speed = get_max(mean_speeds)
    ymax_red = get_max(reds)
    ymax_orange = get_max(oranges)

    chart_x = 80
    draw_chart(screen, mean_speeds, BLUE, "Mean Speed", chart_x, 420, 300, 100, ymax_speed)
    draw_chart(screen, reds, RED, "Red Cars", chart_x, 540, 300, 100, ymax_red)
    draw_chart(screen, oranges, ORANGE, "Orange Cars", chart_x, 660, 300, 100, ymax_orange)

    pygame.display.flip()

pygame.quit()
sys.exit()
