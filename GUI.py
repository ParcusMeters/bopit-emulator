import pygame
import sys

import serial
import struct


class DisplayText:
    def __init__(self, screen):
        self.instruction = "Press Right Button to Start!!!!!"
        self.window = screen
        self.window.fill(baby_blue)
        self.score = 0
        self.highscore = 0

        self.font = pygame.font.Font(None, 36)

    def set_score(self, score):
        self.score = score

    def update_score(self):
        self.score += 1
        if self.score > self.highscore:
            self.highscore = self.score

    def set_instruction(self, instruction):
        self.instruction = instruction

    def game_over(self):
        self.instuction = "Game Over"

    def draw(self):
        self.window.fill(baby_blue)
        # Draw score
        score_text = self.font.render("Score: " + str(self.score), True, BLACK)
        self.window.blit(score_text, (10, 10))

        # Draw high score
        highscore_text = self.font.render(
            "Highscore: " + str(self.highscore), True, BLACK
        )
        self.window.blit(
            highscore_text, (window_width - highscore_text.get_width() - 10, 10)
        )

        # Draw instruction
        instruction_text = self.font.render(str(self.instruction), True, BLACK)
        instruction_x = (window_width - instruction_text.get_width()) // 2
        instruction_y = (window_height - instruction_text.get_height()) // 2
        self.window.blit(instruction_text, (instruction_x, instruction_y))


# Initialize Pygame
pygame.init()
pygame.mixer.music.load("./Gameplay_Background.mp3")
pygame.mixer.music.play(-1)

window_width, window_height = 800, 600
window = pygame.display.set_mode((window_width, window_height))
pygame.display.set_caption("Bop It Emulator")

# Define colors
BLACK = (0, 0, 0)
WHITE = (255, 255, 255)
baby_blue = (122, 217, 255)


# Create an instance of DisplayText
display = DisplayText(window)

# Simulate a change in the score that triggers the GUI update


# Game loop
while True:
    # Handle events
    for event in pygame.event.get():
        if event.type == pygame.QUIT:
            pygame.quit()
            sys.exit()

    # Fill the screen with a white color

    # Draw the display text
    display.draw()

    # Update the display
    pygame.display.flip()

    with serial.Serial("/dev/cu.usbmodemSDA41E9CE611", 115200) as ser:
        # print(ser.readline()) #This should read to the first '\n' character
        #                       #If you don't see anything on the output
        #                       #it could be because the board did not send a line
        #                       #try pressing the reset button to resend the string
        while True:
            # Read one character
            some_bytes = ser.read()

            # Returns a list (of length 1) of decoded python types
            decoded_bytes = struct.unpack("B", some_bytes)
            print(decoded_bytes)
            # whatever instruction we need
            # 48 is char 0
            if decoded_bytes[0] == 0:
                display.set_instruction("RIGHT PRESS")
                display.update_score()
                break

                # some gui stuff if we want
            elif decoded_bytes[0] == 1:
                display.set_instruction("COVER IT")
                display.update_score()
                break

            elif decoded_bytes[0] == 2:
                display.set_instruction("SHAKE IT")
                display.update_score()
                break

            elif decoded_bytes[0] == 9:
                display.set_instruction("GAME OVER: PRESS RIGHT BUTTON TO PLAY AGAIN")
                display.set_score(0)
                break

            elif decoded_bytes[0] == 8:
                display.set_instruction("PRESS RIGHT BUTTON TO START")
                # display.update_score()
                break
            elif decoded_bytes[0] == 3:
                display.set_instruction("LEFT PRESS")
                # display.update_score()
                break
