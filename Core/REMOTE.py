#!usr/bin/python3

'''

Drive motors python test script
Made by Alec

'''

import time
import pygame
import motor_controller as motors

#Create our device objects
motors.init()
pygame.init()
display_width = 400
display_height = 200
screen = pygame.display.set_mode((display_width,display_height))
pygame.display.set_caption('ASBOT REMOTE')
pygame.mouse.set_visible(1)

black = (0,0,0)
white = (255,255,255)
		
def text_objects(text, font):
	textSurface = font.render(text, True, white)
	return textSurface, textSurface.get_rect()
	
def text_objects2(text, font):
	textSurface = font.render(text, True, black)
	return textSurface, textSurface.get_rect()
		
def message_display(text):
	screen.fill(black)
	largeText2 = pygame.font.Font('freesansbold.ttf',20)
	TextSurf2, TextRect2 = text_objects(text, largeText2)
	TextRect2.center = ((display_width/2), ((display_height/2)-25))
	screen.blit(TextSurf2, TextRect2)
	
	pygame.display.update()
	
def message_display_lower(text):
	pygame.draw.rect(screen,black, (0,125, 400, 75))
	largeText = pygame.font.Font('freesansbold.ttf',15)
	TextSurf, TextRect = text_objects(text, largeText)
	TextRect.center = ((display_width/2), ((display_height/2)+35))
	screen.blit(TextSurf, TextRect)
	pygame.display.update()

		
message_display("W,A,S,D To move")
		
while True:
	ticksA = motors.get_ticksA()
	ticksB = motors.get_ticksB()
	distances = motors.get_distance(ticksA,ticksB)
	encoderA_meters = distances.return_distanceA()
	encoderB_meters = distances.return_distanceB()
	#encoderA_meters = mA.get_ticks()
	#encoderB_meters = mB.get_ticks()
	for event in pygame.event.get():
		if event.type == pygame.KEYDOWN:
			if event.key == pygame.K_w:
				motors.driveMotors(100,100)
				message_display("Driving Foward")
			if event.key == pygame.K_s:
				motors.driveMotors(-100,-100)
				message_display("Driving Backward")
			if event.key == pygame.K_a:
				motors.driveMotors(100,-100)
				message_display("Turning Left")
			if event.key == pygame.K_d:
				motors.driveMotors(-100,100)
				message_display("Turning Right")
		if event.type == pygame.KEYUP:
			motors.driveMotors(0,0)
			message_display("Stopped")
	
	message_display_lower("Motor A, MotorB:  %s,  %s" % (round(encoderA_meters, 2), round(encoderB_meters, 2)))
	time.sleep(0.1)
	
