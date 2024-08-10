import pygame
import RPi.GPIO as GPIO
import time

# Initialize Pygame
pygame.init()

# Set up GPIO
RL1 = 2  
RL2 = 3
RL3 = 4
GPIO.setmode(GPIO.BCM)
GPIO.setup(RL1, GPIO.OUT)
GPIO.setup(RL2, GPIO.OUT)
GPIO.setup(RL3, GPIO.OUT)

# Set up colors
WHITE = (255, 255, 255)
GREEN = (0, 255, 0)
RED = (255, 0, 0)

# Set up the screen
WIDTH, HEIGHT = 400, 300
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption("Power Control")

# Set up fonts
font = pygame.font.Font(None, 36)

# Function to draw a button
def draw_button(text, rect, color):
    pygame.draw.rect(screen, color, rect)
    text_surf = font.render(text, True, WHITE)
    text_rect = text_surf.get_rect(center=rect.center)
    screen.blit(text_surf, text_rect)

# Main loop
running = True
led_state = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
        elif event.type == pygame.MOUSEBUTTONDOWN:
            if button_rect.collidepoint(event.pos):
                led_state = not led_state
                GPIO.output(RL1, led_state)
                
                if led_state == False:
                    time.sleep(1)  # Sleep for 1 second
                    GPIO.output(RL2, led_state)
                    time.sleep(1)  # Sleep for 1 second
                    GPIO.output(RL3, led_state)
                else:
                    GPIO.output(RL2, led_state)
                    GPIO.output(RL3, led_state)

    screen.fill(WHITE)
    
    # Draw button
    button_text = "Turn Off" if not led_state else "Turn On"
    button_color = RED if not led_state else GREEN
    button_rect = pygame.Rect(WIDTH // 2 - 50, HEIGHT // 2 - 25, 100, 50)
    draw_button(button_text, button_rect, button_color)
    
    pygame.display.flip()

# Clean up
GPIO.cleanup()
pygame.quit()
