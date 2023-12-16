import pygame
import sys
import lcm
import sys
sys.path.append('/usr/lib/python3.9/site-packages/')
from mbot_lcm_msgs.twist2D_t import twist2D_t
from mbot_lcm_msgs.pose2D_t import pose2D_t

address = "udpm://239.255.76.67:7667?ttl=1"

lc = lcm.LCM(address)

# Initialize Pygame
pygame.init()

# Constants
SCREEN_WIDTH = 600
SCREEN_HEIGHT = 200
FPS = 60

# Colors
WHITE = (255, 255, 255)
BLACK = (0, 0, 0)

# PID gains
pid_gains = {
    'Inner': {'P': 5.3, 'I': 0.3, 'D': 0.121},
    'Outer': {'P': 0.3, 'I': 0.0, 'D': 3.2},
    'Steering': {'P': 0.0, 'I': 0.0, 'D': 0.0}
}

# Selected gain
selected_controller = 'Inner'
selected_gain = 'P'

# Function to draw text on the screen
def draw_text(surface, text, size, color, x, y):
    font = pygame.font.Font(None, size)
    text_surface = font.render(text, True, color)
    text_rect = text_surface.get_rect(topleft=(x, y))
    surface.blit(text_surface, text_rect)

# Function to draw the PID gains on the screen
def draw_pid_values(surface):
    col_width = SCREEN_WIDTH // 4

    for i, controller in enumerate(pid_gains.keys()):
        x = i * col_width + col_width // 2
        y = 50

        draw_text(surface, f'{controller} Controller', 24, BLACK, x, y)
        draw_text(surface, f'P: {pid_gains[controller]["P"]:.6f}', 24, BLACK, x, y + 30)
        draw_text(surface, f'I: {pid_gains[controller]["I"]:.6f}', 24, BLACK, x, y + 50)
        draw_text(surface, f'D: {pid_gains[controller]["D"]:.6f}', 24, BLACK, x, y + 70)

    # Draw selected gain indicator
    x = col_width // 2 - 10 + col_width * (['Inner', 'Outer', 'Steering'].index(selected_controller))
    y = 80 + 20 * (['P', 'I', 'D'].index(selected_gain))
    draw_text(surface, '>', 24, BLACK, x, y)

# Main function
def main():
    # Set up the screen
    screen = pygame.display.set_mode((SCREEN_WIDTH, SCREEN_HEIGHT))
    pygame.display.set_caption('PID Controller GUI')
    
    clock = pygame.time.Clock()

    global selected_controller 
    global selected_gain

    shift_left = {
        "Inner":"Steering",
        "Outer":"Inner",
        "Steering":"Outer"
    }

    shift_right = {
        "Inner":"Outer",
        "Outer":"Steering",
        "Steering":"Inner"
    }

    shift_down = {
        "P":"I",
        "I":"D",
        "D":"P"
    }

    shift_up = {
        "P":"D",
        "I":"P",
        "D":"I"
    }

    while True:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                pygame.quit()
                sys.exit()
            elif event.type == pygame.KEYDOWN:
                if event.key == pygame.K_UP:
                    selected_gain = shift_up[selected_gain]
                elif event.key == pygame.K_DOWN:
                    selected_gain = shift_down[selected_gain]
                elif event.key == pygame.K_LEFT:
                    selected_controller = shift_left[selected_controller]
                elif event.key == pygame.K_RIGHT:
                    selected_controller = shift_right[selected_controller]
                elif event.key == pygame.K_KP_MULTIPLY:
                    pid_gains[selected_controller][selected_gain] *= 1.1
                elif event.key == pygame.K_KP_DIVIDE:
                    pid_gains[selected_controller][selected_gain] /= 1.1
                elif event.key == pygame.K_KP_PLUS:
                    pid_gains[selected_controller][selected_gain] += .1
                elif event.key == pygame.K_KP_MINUS:
                    pid_gains[selected_controller][selected_gain] -= .1
                elif event.key == pygame.K_KP0:
                    pid_gains[selected_controller][selected_gain] = 0
                lcm_message = twist2D_t()
                lcm_message.vx = pid_gains["Inner"]["P"]
                lcm_message.vy = pid_gains["Inner"]["I"]
                lcm_message.wz = pid_gains["Inner"]["D"]
                lc.publish("MBOT_VEL_CMD", lcm_message.encode())
                lcm_message = pose2D_t()
                lcm_message.x = pid_gains["Outer"]["P"]
                lcm_message.y = pid_gains["Outer"]["I"]
                lcm_message.theta = pid_gains["Outer"]["D"]
                lc.publish("MBOT_ODOMETRY_RESET", lcm_message.encode())

        # Fill the background
        screen.fill(WHITE)

        # Draw PID values
        draw_pid_values(screen)

        # Update the display
        pygame.display.flip()

        # Cap the frame rate
        clock.tick(FPS)

if __name__ == "__main__":
    main()


