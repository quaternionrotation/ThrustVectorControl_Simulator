import math
import pygame
import numpy
import datetime
import random

pygame.init()
WIDTH, HEIGHT = 960, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('TVC Sim')
clock = pygame.time.Clock()
pygame.font.init()
font = pygame.font.SysFont('Arial', 20)

# ------------------------------------------- CONFIG
g = 9.81
dt = 1/60
scaleFactor = 6
MOI = 0.041 # for rads!
InitialThrust = 20 # newtons
initialTVCAngle = 0
initialAngle = 60
COMMotorLength = 0.5 # metres
P = 0.9
I = 0.01
D = 0.12
actuationSpeed = 920 # in deg/s
vehicleMass = 0.9 # Kg
doLateral = True
maxGimbal = 10
InitialSetPoint = 0
state = 1
sK = 0.4
sD = 2

# -------------------------------------------

isA = False
isD = False

time = 0

rocketImg = pygame.image.load('rokit.png')
rocketWidth = rocketImg.get_width()
rocketHeight = rocketImg.get_height()

flameImg = pygame.image.load("flame.png").convert_alpha()
flameWidth = flameImg.get_width()
flameHeight = flameImg.get_height()

class Vehicle:

    def __init__ (self, x, y, vx, vy, mass, a, w, angle, tvcangle, thrust):
            self.x = x
            self.y = y
            self.vx = vx
            self.vy = vy
            self.mass = mass
            self.a = a
            self.w = w
            self.angle = angle
            self.tvcangle = tvcangle
            self.integral = 0
            self.thrust = thrust
            self.trail = []
            self.setPoint = InitialSetPoint
            self.prevx = 0

    def do_pid(self):
          
          deviation = self.angle - self.setPoint
          self.integral += deviation * dt
          tvctarget = P * deviation + D * self.w * 57.2958 + I * self.integral 
          maxactuation = actuationSpeed * dt
          self.tvcangle += numpy.clip(tvctarget - self.tvcangle, -maxactuation, maxactuation) # Accounts for actuator delay
          self.tvcangle = numpy.clip(self.tvcangle, -maxGimbal, maxGimbal)

          
          #derivative = (self.prevx - self.prevx) / dt
          self.setPoint = sK * self.x + sD * self.vx
          prevx = self.x

          #if self.y < -70:
                #self.thrust = 0

    #def state_machine(self):
          #if state == 1:
                

    def update_position(self):
            
            # Lateral 
            if doLateral:
                motorangle = self.tvcangle + self.angle
                fx = fy = 0
                fx -= self.thrust * math.sin(0.0174533 * motorangle)
                fy -= self.thrust * math.cos(0.0174533 * motorangle)
                fy += g * self.mass # Gravity
                ax = fx / self.mass
                ay = fy /  self.mass
                self.vx += ax * dt 
                self.vy += ay * dt
                self.x += self.vx * dt
                self.y += self.vy * dt
            else:
                  self.x = 0
                  self.y = 0

            # Rotation
            self.a = COMMotorLength * self.thrust * math.sin(self.tvcangle * 0.0174533) / MOI
            self.w -= self.a * dt 
            self.angle += self.w * dt * 57.2958 # Rads

            self.trail.append((int(self.x * scaleFactor + WIDTH / 2), int(self.y * scaleFactor + HEIGHT / 2)))
            #if len(self.trail) > 200:
                  #self.trail.pop(0)

    def draw(self, screen):
          rotatedRocket = pygame.transform.rotate(rocketImg, self.angle)
          rotatedRocketRect = rotatedRocket.get_rect(center=(self.x * scaleFactor + WIDTH //2, self.y * scaleFactor + HEIGHT //2))
          
          if len(self.trail) > 1:
            pygame.draw.lines(screen, (255, 50, 50), False, self.trail, 1)

          # Nozzle Position
          nozzlex = rocketHeight/2 * math.sin(self.angle * 0.0174533)
          nozzley = rocketHeight/2 * math.cos(self.angle * 0.0174533)

          rotatedFlame = pygame.transform.rotate(flameImg, self.angle + self.tvcangle)
          rotatedFlameRect = rotatedFlame.get_rect(center=(self.x * scaleFactor + WIDTH //2 + nozzlex, self.y * scaleFactor+ HEIGHT //2 + nozzley))
          
          if self.thrust > 0:
            screen.blit(rotatedFlame, rotatedFlameRect.topleft) # Flame first so rocket covers it
          screen.blit(rotatedRocket, rotatedRocketRect.topleft)

          title = font.render('TVC SIM', False, (255, 255, 255))
          screen.blit(title, (10,10))
          title = font.render('Time: ' + str(time), False, (255, 255, 255))
          screen.blit(title, (10,40))
          title = font.render('Height: ' + str(-self.y), False, (255, 255, 255))
          screen.blit(title, (10,70))
          title = font.render('Speed: ' + str(math.sqrt(self.vx ** 2 + self.vy **2)), False, (255, 255, 255))
          screen.blit(title, (10,100))
          title = font.render('Angle: ' + str(self.angle), False, (255, 255, 255))
          screen.blit(title, (10,130))
          title = font.render('Gimbal Angle: ' + str(self.tvcangle), False, (255, 255, 255))
          screen.blit(title, (10,160))
          title = font.render('Thrust: ' + str(self.thrust), False, (255, 255, 255))
          screen.blit(title, (10,190))
          title = font.render('LatDist: ' + str(self.x), False, (255, 255, 255))
          screen.blit(title, (10,220))
          title = font.render('Setpoint: ' + str(self.setPoint), False, (255, 255, 255))
          screen.blit(title, (10,250))

vehicles = [
    Vehicle(0, 50, 0, 0, vehicleMass, 0, 0, initialAngle, initialTVCAngle, InitialThrust)
]

running = True
while running:
      for event in pygame.event.get():
            if event.type == pygame.QUIT:
                  running = False
            if event.type == pygame.KEYDOWN:
                  if event.key == pygame.K_r:
                        for vehicle in vehicles:
                              vehicle.x = 0
                              vehicle.y = 50
                              vehicle.thrust = InitialThrust
                              vehicle.tvcangle = initialTVCAngle
                              vehicle.angle = random.randint(-160, 160)
                              vehicle.vx = random.randint(-50, 50)
                              vehicle.vy = random.randint(-6, 6)
                              vehicle.w = random.randint(-2, 2)
                              vehicle.a = 0
                              vehicle.integral = 0
                              vehicle.trail.clear()
                              vehicle.setPoint = InitialSetPoint
                              vehicle.prevx = 0

      screen.fill((0,0,20))

      for vehicle in vehicles:
            #vehicle.state_machine()
            vehicle.do_pid()
            vehicle.update_position()
            vehicle.draw(screen)

      time += dt
      pygame.display.flip()
      clock.tick(60)

pygame.quit()