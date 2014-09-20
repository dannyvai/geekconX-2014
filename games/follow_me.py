import os, sys
import pygame
from pygame.locals import *

import random
import time

screen_size_x = 640
screen_size_y = 480

random.seed(time.time())

pygame.init()
screen = pygame.display.set_mode((screen_size_x ,screen_size_y))
pygame.display.set_caption('Monkey Fever')
pygame.mouse.set_visible(0)

background = pygame.Surface(screen.get_size())
background = background.convert()
background.fill((250, 250, 250))


screen.blit(background, (0, 0))
pygame.display.flip()

clock = pygame.time.Clock()


def create_random_path():
	start_pos_x = 0
	start_pos_y = 0
	path_x = [0]
	path_y = [0]
	while start_pos_x < screen_size_x and start_pos_y <  screen_size_y :
		rand_x = int(((random.random()*screen_size_x)%100 +60 + start_pos_x)) 
		rand_y = int(((random.random()*screen_size_y)%100 + 30 + start_pos_y) )
		if rand_x >  screen_size_x :
			rand_x = screen_size_x
		if rand_y >  screen_size_y :
			rand_y = screen_size_y
		
		path_x.append(rand_x)
		path_y.append(rand_y)
		start_pox_x = rand_x
		start_pos_y = rand_y
		print start_pox_x,start_pos_y 
	return path_x,path_y

def draw_path(path_x,path_y):
		for i in range(1,len(path_x)):
			pygame.draw.line(screen, (  0, 255,   0), [path_x[i-1],path_y[i-1]], [path_x[i],path_y[i]], 10)
		
path_x,path_y = create_random_path()
print path_x
print path_y


while 1:
    clock.tick(60)
    draw_path(path_x,path_y)
    pygame.display.flip()