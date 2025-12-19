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
scaleFactor = 0.125
MOI = 2e6 # for rads!
InitialThrust = 0 # newtons
initialTVCAngle = 0
initialAngle = -38
COMMotorLength = 0.5 # metres
P = 1.5
I = 0.05
D = 2
actuationSpeed = 920 # in deg/s
vehicleMass = 45000 # Kg
dryMass = 36000 # Kg
doLateral = True
maxGimbal = 15
InitialSetPoint = 0
sK = 0.006#0.01
sD = 0.5#0.3
startHeight = 5000
padLevel = -400
initialYSpeed = 150
initialXSpeed = -150
initialX = 3000
initialY = 0
maxThrust = 1000000
minThrust = 340000
maxBurnRate = 306 # kg/s at max throttle

# -------------------------------------------

isA = False
isD = False

time = 0

rocketImg = pygame.image.load('falcon.png')
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
            self.state = 1
            self.heightabovepad = startHeight - padLevel
            self.startupAlt = 0
            self.startupSpd = 0
            self.touchspeed = 0

    def do_pid(self):
          
          deviation = self.angle - self.setPoint
          if self.thrust > 0:
            self.integral += deviation * dt
          tvctarget = P * deviation + D * self.w * 57.2958 + I * self.integral 
          maxactuation = actuationSpeed * dt
          self.tvcangle += numpy.clip(tvctarget - self.tvcangle, -maxactuation, maxactuation) # Accounts for actuator delay
          self.tvcangle = numpy.clip(self.tvcangle, -maxGimbal, maxGimbal)

          #derivative = (self.prevx - self.prevx) / dt
          self.setPoint = numpy.clip(sK * self.x + sD * self.vx, -30, 30)
          if self.thrust == maxThrust:
            self.setPoint = 0

          #if self.y < -70:
                #self.thrust = 0

    def state_machine(self):
          if self.state == 0:
               self.state = 1
          if self.state == 1:
                self.thrust = InitialThrust
                if self.vy > 0:
                  landing_dist = abs(((self.vy) ** 2 )/(2 * (maxThrust * 0.7 * (self.vy / math.sqrt(self.vx **2 + self.vy **2)) ) / ((self.mass + dryMass) / 2) - g))
                  print(landing_dist)
                  if self.heightabovepad < landing_dist and self.heightabovepad > 10:
                       self.state = 2
                       self.startupAlt = self.heightabovepad
                       self.startupSpd = math.sqrt(self.vx ** 2 + self.vy ** 2)
          if self.state == 2:
                self.thrust = numpy.clip((((self.vy ** 2) * self.mass) / (2 * self.heightabovepad)) / math.cos(0.0174533 * (self.tvcangle + self.angle)) + g * self.mass, minThrust, maxThrust) 
                # Burn Rate
                self.mass -= (self.thrust / maxThrust) * maxBurnRate * dt
                if self.heightabovepad <= 0:
                     self.state = 3
                     self.touchspeed = self.vy
          if self.state == 3:
               self.thrust = 0
    def update_position(self):
            
            self.heightabovepad = - padLevel - self.y 

            # Lateral 
            if doLateral:
                motorangle = self.tvcangle - self.angle
                fx = fy = 0
                fx += self.thrust * math.sin(0.0174533 * motorangle)
                fy -= self.thrust * math.cos(0.0174533 * motorangle)
                fy += g * self.mass # Gravity
                ax = fx / self.mass
                ay = fy / self.mass
                self.vx += ax * dt 
                self.vy += ay * dt

            # Collision with ground
                if self.state == 3:
                     self.vy = 0
                     self.vx = 0

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
          nozzlex = (rocketHeight/2 -5) * math.sin(self.angle * 0.0174533)
          nozzley = (rocketHeight/2 -5) * math.cos(self.angle * 0.0174533)

          rotatedFlame = pygame.transform.rotate(flameImg, self.angle + self.tvcangle)
          rotatedFlameRect = rotatedFlame.get_rect(center=(self.x * scaleFactor + WIDTH //2 + nozzlex, self.y * scaleFactor+ HEIGHT //2 + nozzley))
          
          if self.thrust > 0:
            screen.blit(rotatedFlame, rotatedFlameRect.topleft) # Flame first so rocket covers it
          screen.blit(rotatedRocket, rotatedRocketRect.topleft)

          pygame.draw.line(screen, (10, 255, 10), (0, HEIGHT//2 + (-padLevel + 60 / scaleFactor) * scaleFactor), (WIDTH, HEIGHT//2 + (-padLevel + 60 / scaleFactor )* scaleFactor))

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
          title = font.render('Pad Height: ' + str(self.heightabovepad), False, (255, 255, 255))
          screen.blit(title, (10,280))
          title = font.render('Scale: ' + str(scaleFactor), False, (255, 255, 255))
          screen.blit(title, (10,310))
          title = font.render('State: ' + str(self.state), False, (255, 255, 255))
          screen.blit(title, (10,340))
          title = font.render('Mass: ' + str(self.mass), False, (255, 255, 255))
          screen.blit(title, (10,370))
          title = font.render('BurnAlt: ' + str(self.startupAlt), False, (255, 255, 255))
          screen.blit(title, (10,400))
          title = font.render('BurnSpd: ' + str(self.startupSpd), False, (255, 255, 255))
          screen.blit(title, (10,430))
          title = font.render('TDownVSpd: ' + str(self.touchspeed), False, (255, 255, 255))
          screen.blit(title, (10,460))

vehicles = [
    Vehicle(initialX, -startHeight, initialXSpeed, initialYSpeed, vehicleMass, 0, 0, initialAngle, initialTVCAngle, InitialThrust)
]

running = True
while running:
      for event in pygame.event.get():
            if event.type == pygame.QUIT:
                  running = False
            if event.type == pygame.KEYDOWN:
                  if event.key == pygame.K_r:
                        for vehicle in vehicles:
                              vehicle.x = random.randint(initialX - 3000, initialX + 1000)
                              vehicle.y = -startHeight
                              vehicle.thrust = InitialThrust
                              vehicle.tvcangle = initialTVCAngle
                              vehicle.angle = random.randint(-40, 40)
                              vehicle.vx = random.randint(initialXSpeed - 30, initialXSpeed + 30)
                              vehicle.vy = initialYSpeed
                              vehicle.w = random.randint(0, 0)
                              vehicle.a = 0
                              vehicle.integral = 0
                              vehicle.trail = []
                              vehicle.setPoint = InitialSetPoint
                              vehicle.prevx = 0
                              vehicle.vy = initialYSpeed
                              vehicle.state = 0
                              vehicle.mass = vehicleMass
                  if event.key == pygame.K_z:
                       scaleFactor /= 2
                       for vehicle in vehicles:
                            vehicle.trail = []
                  if event.key == pygame.K_x:
                       scaleFactor *= 2
                       for vehicle in vehicles:
                            vehicle.trail = []

      screen.fill((0,0,20))

      for vehicle in vehicles:
            vehicle.state_machine()
            vehicle.do_pid()
            vehicle.update_position()
            vehicle.draw(screen)

      time += dt
      pygame.display.flip()
      clock.tick(60)

pygame.quit()