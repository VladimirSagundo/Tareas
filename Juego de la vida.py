import pygame
import numpy as np

pygame.init()
width, height = 600, 600
cell_size = 20
cols, rows = width // cell_size, height // cell_size

screen = pygame.display.set_mode((width, height))
pygame.display.set_caption("Juego de la Vida de Conway")

BLACK = (0, 0, 0)
WHITE = (255, 255, 255)

def initialize_grid(rows, cols):
    """Crea una grilla con todas las células muertas."""
    return np.zeros((rows, cols), dtype=int)

def update_grid(grid):
    """Aplica las reglas del Juego de la Vida de Conway para actualizar la grilla."""
    new_grid = np.copy(grid)
    for row in range(rows):
        for col in range(cols):
            live_neighbors = np.sum(grid[row-1:row+2, col-1:col+2]) - grid[row, col]
            if grid[row, col] == 1:
                if live_neighbors < 2 or live_neighbors > 3:
                    new_grid[row, col] = 0
            else:
                if live_neighbors == 3:
                    new_grid[row, col] = 1
    return new_grid

def draw_grid(screen, grid):
    """Dibuja la grilla en la pantalla."""
    for row in range(rows):
        for col in range(cols):
            color = WHITE if grid[row, col] == 1 else BLACK
            pygame.draw.rect(screen, color, (col * cell_size, row * cell_size, cell_size - 1, cell_size - 1))

def main():
    grid = initialize_grid(rows, cols)
    running = True
    selecting = True

    while running:
        for event in pygame.event.get():
            if event.type == pygame.QUIT:
                running = False
            if event.type == pygame.MOUSEBUTTONDOWN and selecting:
                x, y = event.pos
                col, row = x // cell_size, y // cell_size
                grid[row, col] = 1 - grid[row, col]
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    selecting = not selecting

        screen.fill(BLACK)
        draw_grid(screen, grid)
        pygame.display.flip()

        if not selecting:
            grid = update_grid(grid)
            pygame.time.delay(100)

    pygame.quit()

if __name__ == "__main__":
    main()