import math
import pygame
import numpy
import random
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import pandas as pd

pygame.init()
WIDTH, HEIGHT = 960, 1000
screen = pygame.display.set_mode((WIDTH, HEIGHT))
pygame.display.set_caption('TVC Sim')
clock = pygame.time.Clock()
pygame.font.init()
font = pygame.font.SysFont('Arial', 20)

# ------------------------------------------- CONFIG
doVisualisation = True
thrustCurve = 'TSP_E12.csv'#'TSP_E12.csv' # CSV file with time (s) and thrust (N) columns
simLength = 7 # seconds
loopRate = 200 # Hz
g = 9.81
scaleFactor = 14 # pixels per metre
MOI = 0.0343 # Moment of Inertia Kg m^2, measured
initialHeight = -10 # metres
InitialThrust = 0 # newtons
initialTVCAngle = 0
initialAngle = 20 # degrees
COMMotorLength = 0.265 # metres
actuationSpeed = 107 # in deg/s
vehicleMass = 0.826 # Kg
doLateral = True
maxGimbal = 8
InitialSetPoint = 0 # degrees
state = 1
sK = 0
sD = 0
detectDelay = 0 # Simulated delay in launch detect in seconds

# -------------------------------------------

isA = False
isD = False

time = 0
dt = 0

rocketImg = pygame.image.load('rokit.png')
rocketWidth = rocketImg.get_width()
rocketHeight = rocketImg.get_height()

flameImg = pygame.image.load("flame.png").convert_alpha()
flameWidth = flameImg.get_width()
flameHeight = flameImg.get_height()

curve = pd.read_csv(thrustCurve) # Import thrust curve

class Vehicle:

    def __init__ (self, x, y, vx, vy, mass, a, w, angle, tvcangle, thrust, _actuationSpeed, loopRate, p, i, d):
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
            self.p = p
            self.i = i
            self.d = d
            self.actuationSpeed = _actuationSpeed
            self.loopRate = loopRate
            self.lastPIDtime = 0
            self.tvctarget = 0
            self.ax = 0
            self.ay = 0
            self.launchDetected = False
            self.measuredThrust = InitialThrust
            self.launchDetectTime = 0

            # For Graphing
            self.angles = []
            self.alts = []
            self.times = [] 
            self.thrusts = [] 
            self.tvcangles = [] 

    def do_pid(self):
          pid_dt = time - self.lastPIDtime
          if (self.thrust/vehicleMass - g) >= 4 and not self.launchDetected:
            self.launchDetected = True
            self.integral = 0
            self.launchDetectTime = time + detectDelay # Account for delay
          if self.launchDetected and time >= self.launchDetectTime: # Only run PID after launch detected and delay has passed
            if pid_dt >= 1 / self.loopRate:
                  deviation = self.angle - self.setPoint
                  self.integral += deviation * pid_dt
                  targettorque = self.p * deviation + self.d * self.w * 57.2958 + self.i * self.integral # Target Torque
                  if self.measuredThrust != 0:
                        self.tvctarget = numpy.clip(math.degrees(math.asin(numpy.clip(targettorque / (COMMotorLength * self.measuredThrust), -1, 1))), -maxGimbal, maxGimbal)
                  else:
                        self.tvctarget = 0
                  self.lastPIDtime = time
          maxactuation = self.actuationSpeed * dt
          self.tvcangle += numpy.clip(self.tvctarget - self.tvcangle, -maxactuation, maxactuation) # Accounts for actuator delay
          self.tvcangle = numpy.clip(self.tvcangle, -maxGimbal, maxGimbal)

          #derivative = (self.prevx - self.prevx) / dt

          #if self.y < -70:
                #self.thrust = 0

    #def state_machine(self):
          #if state == 1:
                
    def actualThrust(self):
      thrustval = 0
      for i, row in curve.iterrows():
            if row['time'] >= time:
                  thrustval = row['thrust']
                  break
      self.thrust = thrustval

    def measurethrust(self):
            timeintoBurn = time - self.launchDetectTime - 0*detectDelay + 0.260

            if self.launchDetected and time > self.launchDetectTime:
                  if timeintoBurn < 0.535:
                        self.measuredThrust = 88.364*timeintoBurn - 18.975
                  elif timeintoBurn < 0.9:
                        self.measuredThrust = -45.479*timeintoBurn + 52.631
                  elif timeintoBurn < 3.5:
                        self.measuredThrust = 12
                  else:
                        self.measuredThrust = 0

                  if self.measuredThrust < 0:
                        self.measuredThrust = 0
                  
            self.thrusts.append(self.thrust)

    def update_position(self):
            

            # Lateral 
            if doLateral:
                motorangle = self.tvcangle + self.angle
                fx = fy = 0
                fx -= self.thrust * math.sin(0.0174533 * motorangle)
                fy -= self.thrust * math.cos(0.0174533 * motorangle)
                if (self.y <= -initialHeight):
                  fy += g * self.mass # Gravity 
                elif self.vy > 0:
                  self.vy = 0
                self.ax = fx / self.mass
                self.ay = fy /  self.mass
                self.vx += self.ax * dt 
                self.vy += self.ay * dt
                self.x += self.vx * dt
                self.y += self.vy * dt
            else:
                  self.x = 0
                  self.y = 0

            # Rotation
            self.a = (COMMotorLength * self.thrust * math.sin(self.tvcangle * 0.0174533)) / MOI 
            self.w -= self.a * dt 
            self.angle += self.w * dt * 57.2958 # Rads

            self.angles.append(self.angle)
            self.tvcangles.append(self.tvcangle)
            self.alts.append(-initialHeight-self.y)
            self.times.append(time)

            self.trail.append((int(self.x * scaleFactor + WIDTH / 2), int(self.y * scaleFactor + HEIGHT / 2)))
            #if len(self.trail) > 200:
                  #self.trail.pop(0)

            if self.angle > 180:
                  self.angle -= 360
            elif self.angle < -180:
                  self.angle += 360

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

          title = font.render('TVC SIM', False, (0,0,0))
          screen.blit(title, (10,10))
          title = font.render('Time: ' + str(time), False, (0,0,0))
          screen.blit(title, (10,40))
          title = font.render('Height: ' + str(-initialHeight-self.y), False, (0,0,0))
          screen.blit(title, (10,70))
          title = font.render('Speed: ' + str(math.sqrt(self.vx ** 2 + self.vy **2)), False, (0,0,0))
          screen.blit(title, (10,100))
          title = font.render('Angle: ' + str(self.angle), False, (0,0,0))
          screen.blit(title, (10,130))
          title = font.render('Gimbal Angle: ' + str(self.tvcangle), False, (0,0,0))
          screen.blit(title, (10,160))
          title = font.render('Thrust: ' + str(self.thrust), False, (0,0,0))
          screen.blit(title, (10,190))
          title = font.render('LatDist: ' + str(self.x), False, (0,0,0))
          screen.blit(title, (10,220))
          title = font.render('Setpoint: ' + str(self.setPoint), False, (0,0,0))
          screen.blit(title, (10,250))
          title = font.render('Launch Detect Time: ' + str(self.launchDetectTime), False, (0,0,0))
          screen.blit(title, (10,280))

##############################################################

vehicles = [
      Vehicle(0, -initialHeight, 0, 0, vehicleMass, 0, 0, initialAngle, initialTVCAngle, InitialThrust, actuationSpeed, loopRate, 0.02, 0.007, 0.007),
]

##############################################################

plt.ion()
fig, axs = plt.subplots(2, 2, figsize=(12, 10))  # Define figure and subplots
lines = []
lines2 = []
lines3 = []
lines4 = []

for i, vehicle in enumerate(vehicles):
    legend = str(vehicle.p) + ', ' + str(vehicle.i) + ', ' + str(vehicle.d) + ' (' + str(vehicle.actuationSpeed) + 'deg/s ' + str(vehicle.loopRate) + 'Hz)'
    angleline, = axs[0, 0].plot(vehicle.times, vehicle.angles, label=legend)
    lines.append(angleline)
    thrustline, = axs[1, 0].plot(vehicle.times, vehicle.thrusts, label=legend)
    lines2.append(thrustline)
    altline, = axs[0, 1].plot(vehicle.times, vehicle.alts, label=legend)
    lines3.append(altline)
    tvcline, = axs[1, 1].plot(vehicle.times, vehicle.tvcangles, label=legend)
    lines4.append(tvcline)

axs[0, 0].set_ylabel('Angle / degrees')
axs[1, 0].set_ylabel('Thrust / N')
axs[0, 1].set_ylabel('Altitude / m')
axs[1, 1].set_ylabel('TVC Angle / degrees')
axs[0, 0].set_xlabel('Time / s')
axs[0, 1].set_xlabel('Time / s')
axs[1, 0].set_xlabel('Time / s')
axs[1, 1].set_xlabel('Time / s')

running = True
while running:
      for event in pygame.event.get():
            if event.type == pygame.QUIT:
                  running = False
            if event.type == pygame.KEYDOWN:
                  if event.key == pygame.K_r:
                        for vehicle in vehicles:
                              time = 0
                              dt = 0
                              vehicle.x = 0
                              vehicle.y = -initialHeight
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
                              vehicle.angles.clear()
                              vehicle.times.clear()
                              vehicle.alts.clear()
                              vehicle.thrusts.clear()
                              vehicle.tvcangles.clear()

      screen.fill((255,255,255))
      # Vertical lines
      for x in range(0, WIDTH, 10 * scaleFactor):
            pygame.draw.line(screen, (200, 200, 200), (x, 0), (x, HEIGHT), 1)

    # Horizontal lines
      for y in range(0, HEIGHT, 10 * scaleFactor):
            pygame.draw.line(screen, (200, 200, 200), (0, y), (WIDTH, y), 1)
    # Floor line
      pygame.draw.line(screen, (0, 0, 255), (0, HEIGHT / 2 - initialHeight * scaleFactor), (WIDTH, HEIGHT / 2 - initialHeight * scaleFactor), 3)

      for i, vehicle in enumerate(vehicles):
            #vehicle.state_machine()
            vehicle.actualThrust()
            vehicle.measurethrust()
            vehicle.do_pid()
            vehicle.update_position()
            vehicle.draw(screen)
            lines[i].set_data(vehicle.times, vehicle.angles)
            lines2[i].set_data(vehicle.times, vehicle.thrusts)
            lines3[i].set_data(vehicle.times, vehicle.alts)
            lines4[i].set_data(vehicle.times, vehicle.tvcangles)

      time += dt
      if time > simLength:
            for row in axs:
                  for ax in row:
                        ax.relim()
                        ax.autoscale_view()
                        ax.legend()
                        ax.grid()

            plt.ioff()
            plt.show()

      if doVisualisation:  
            dt = 0.002
            pygame.display.flip()
            
      else:
            dt = 0.002