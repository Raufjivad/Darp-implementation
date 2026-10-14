"""
import sys
import pygame 

pygame.init()
screen = pygame.display.set_mode((600, 600))

BLK = (0, 0, 0) 

x, y = 0, 200 
width, height= 200, 200

rect = pygame.Rect(x,y, width, height)

ROWS = 10
COLS = 10
RECT_WIDTH = 40 
RECT_HEIGHT = 40

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    screen.fill((255, 255, 255)) # Clear screen with white
    pygame.draw.rect(screen, BLK, rect) # Draw the rectangle
    pygame.display.flip()

pygame.quit()
"""
#prefixes   
import pygame
from collections import deque

pygame.init()
screen = pygame.display.set_mode((600, 600))
pygame.display.set_caption("Pygame Grid")

WHITE = (255, 255, 255)
BLUE = (0, 102, 204)
BLACK = (0, 0, 0)
GRAY = (200, 200, 200)
PURPLE = (128, 0 ,128)
#--------------------------------------------------------------------------------
# 1. Define Grid Settings
ROWS = 10          # Number of rows
COLS = 10          # Number of columns
RECT_WIDTH = 50   # Width of each rectangle
RECT_HEIGHT = 50  # Height of each rectangle
GAP = 0          # Space between rectangles

grid = [[0 for _ in range(COLS)] for _ in range(ROWS)]

#grid_env = create_grid_env(ROWS, COLS, pos, obs)

# A list of obstacle positions
obs = [
    (3, 2),
    (3, 3),
    (3, 4),
    (4, 3),
    (4, 2),
    (4, 4)
]
for row, col in obs:
    grid[row][col] = -1
print(grid)
#--------------------------------------------------------------------------------
#grid[0][0] = 1
#grid[8][2] = 2
#grid[7][8] = 3
"""Position of Robots"""
pos = [
    (0,0),
    (8,2),
    (7,8)
]

for amount, (row, col) in enumerate(pos, start=1):
    grid[row][col] = amount

#--------------------------------------------------------------------------------
# Make availabe list
#fgrid = obs/grid
#print(fgrid)
#def create_grid_env(rows: int, cols: int, robots: Sequence[Cell], obstacles: Sequence[Cell]) -> Grid:
#    grid_env = [[FREE for _ in range(cols)] for _ in range(rows)]

 #   for row, col in obstacles:
 #       grid_env[row][col] = OBSTACLE

  #  for robot_id, (row, col) in enumerate(robots):
   #     grid_env[row][col] = robot_id

#ROWS, COLS = len(grid), len(grid[0])

# Offset to center the entire grid on the screen (optional)
START_X = 50
START_Y = 50

def euclidean_distance(first: Cell, second: Cell) -> float:
    return sqrt((first[0] - second[0]) ** 2 + (first[1] - second[1]) ** 2)

running = True
while running:
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            running = False
            
    screen.fill(WHITE) # Background color

    # 2. Loop through Rows and Columns to Draw the Grid
    for row in range(ROWS):
        for col in range(COLS):
            # Calculate the X and Y coordinates for the current rectangle
            x = START_X + col * (RECT_WIDTH + GAP)
            y = START_Y + row * (RECT_HEIGHT + GAP)
            
            # Create and draw the rectangle
            rect = pygame.Rect(x, y, RECT_WIDTH, RECT_HEIGHT)
            pygame.draw.rect(screen, BLUE, rect)

            if grid[row][col] == -1:
                current_color = BLACK
            elif grid[row][col] > 0:
                current_color = PURPLE
            else:
                current_color =BLUE


            
            # Optional: Draw a border around each box to make them pop
            pygame.draw.rect(screen, current_color, rect)
            pygame.draw.rect(screen, BLACK, rect, 1)


    pygame.display.flip()

pygame.quit()

