import random
import numpy as np
import pygame.draw

# from Target import Target
from Utilities import Utilities as U


class Environment:
    # Analogue of ThermalMap in MATLAB
    # Class handles target locations and forbidden zones in the map.
    def __init__(self, sl):
        self.sl = sl
        self.home = Home(sl)
        self.numTargets = sl.numTargets
        self.width = sl.mapWidth * sl.ppm
        self.height = sl.mapHeight * sl.ppm
        # Generate coords for all targets. Targets are either high or low quality, and their arrangement must be as
        # rotationally symmetric as possible.
        td = sl.targetDistance
        tr = sl.targetRadius
        hd = sl.hazardDistance
        hr = sl.hazardRadius
        q = [sl.targetHQ for _ in range(sl.numTargets)]
        hq = round((1 - sl.propQualityTargets) * sl.numTargets)
        q[:hq] = [sl.targetLQ] * hq
        # Hand-picked coordinates for when sl.numTargets is either 8 or 4.
        if sl.numTargets == 8:
            self.targetCoords = [
                [15 + td * (0.5 ** 0.5), 15 - td * (0.5 ** 0.5), tr, q[0]],
                [15 - td * (0.5 ** 0.5), 15 + td * (0.5 ** 0.5), tr, q[1]],
                [15 - td * (0.5 ** 0.5), 15 - td * (0.5 ** 0.5), tr, q[2]],
                [15 + td * (0.5 ** 0.5), 15 + td * (0.5 ** 0.5), tr, q[3]],
                [15 + td, 15, tr, q[4]],
                [15 - td, 15, tr, q[5]],
                [15, 15 - td, tr, q[6]],
                [15, 15 + td, tr, q[7]]
            ]
            self.hazardCoords = [
                [15, 15 - hd, hr],
                [15, 15 + hd, hr],
                [15 + hd, 15, hr],
                [15 - hd, 15, hr],
                [15 - hd * (0.5 ** 0.5), 15 - hd * (0.5 ** 0.5), hr],
                [15 + hd * (0.5 ** 0.5), 15 + hd * (0.5 ** 0.5), hr],
                [15 + hd * (0.5 ** 0.5), 15 - hd * (0.5 ** 0.5), hr],
                [15 - hd * (0.5 ** 0.5), 15 + hd * (0.5 ** 0.5), hr]
            ]
        elif sl.numTargets == 4:
            self.targetCoords = [
                [15 + td * (0.5 ** 0.5), 15 - td * (0.5 ** 0.5), tr, q[0]],
                [15 - td * (0.5 ** 0.5), 15 + td * (0.5 ** 0.5), tr, q[1]],
                [15 - td * (0.5 ** 0.5), 15 - td * (0.5 ** 0.5), tr, q[2]],
                [15 + td * (0.5 ** 0.5), 15 + td * (0.5 ** 0.5), tr, q[3]]
            ]
            self.hazardCoords = [
                [15 + hd * (0.5 ** 0.5), 15 + hd * (0.5 ** 0.5), hr],
                [15 - hd * (0.5 ** 0.5), 15 - hd * (0.5 ** 0.5), hr],
                [15 - hd * (0.5 ** 0.5), 15 + hd * (0.5 ** 0.5), hr],
                [15 + hd * (0.5 ** 0.5), 15 - hd * (0.5 ** 0.5), hr]
            ]
        else:
            self.targetCoords = []
            self.hazardCoords = []

        # Trim hazardCoords depending on sl specs
        if sl.hazardType == "None":
            self.hazardCoords = []
            self.numHazards = 0
        elif sl.hazardType == "Partial":
            self.hazardCoords = self.hazardCoords[:self.numTargets - hq]
            self.numHazards = self.numTargets - hq
        else:
            self.numHazards = self.numTargets

        # Premake all hazards and targets
        # self.numKnownTargets = min(len(self.targetCoords), sl.numTargets)
        self.targets = [Target(sl, coord, False, True) for coord in self.targetCoords]
        # self.numKnownHazards = min(len(self.hazardCoords), self.numHazards)
        self.hazards = [Hazard(sl, coord, False) for coord in self.hazardCoords]

        # Random placement of hazards and targets temporarily disabled
        # # Force-spawn the known hazards and targets
        # for i in range(self.numKnownTargets):
        #     self.targets[i] = Target(sl, i, False, True)
        #
        # for i in range(self.numKnownHazards):
        #     self.hazards[i] = Hazard(sl, i, False)
        # Attempt to spawn all targets or until tries run out.
        # triesLeft = sl.targetSpawnAttempts
        # for i in range(sl.numTargets):
        #     if i <= self.numKnownTargets:
        #         self.targets[i] = Target(sl, i, False, True)
        #     checkAgain = True
        #     while triesLeft > 0 and checkAgain:
        #         checkAgain = False
        #         # Check if target i is too close to any previous targets.
        #         for j in range(i - 1):
        #             dist = U.isNear(self.targets[i], self.targets[j], sl.targetSpacing)
        #             if not dist is np.nan:
        #                 # print(f"target {i} is too close to {j}, dist = {dist:.2f}")
        #                 checkAgain = True
        #                 break
        #
        #         # now check if targets are too close to hazard boundaries
        #         for k in range(sl.numHazards):
        #             dist = U.isNear(self.targets[i], self.hazards[k], sl.targetSpacing + self.hazards[k].radius)
        #             if not dist is np.nan:
        #                 # print(f"target {i} is too close to hazard {k}, dist = {dist:.2f}")
        #                 checkAgain = True
        #                 break
        #
        #         # Check if the Check passed, respawn if failed.
        #         if checkAgain:
        #             # Check failed, respawn
        #             triesLeft = triesLeft - 1
        #             # print(f"There are {triesLeft} remaining attempts")
        #             self.targets[i] = Target(sl, i, True, True)
        #             # print(f"pos: {self.targets[i].position[0]:.2f}, {self.targets[i].position[1]:.2f}")
        #         else:
        #             # Check passed, continue
        #             # print(f"Check Passed: Target 1 at "
        #             #       f"{self.targets[i].position[0]:.2f}, {self.targets[i].position[1]:.2f}")
        #             break
        #
        #     if checkAgain:
        #         print(f"Only able to create {i - 1}")
        #         self.numTargets = i - 1
        #         break

    def renderEnvironment(self, sl, screen, step):
        # Draw Home
        if sl.homeExists:
            home = self.home
            pygame.draw.circle(
                screen,
                sl.homeColor,
                (
                    sl.ppm * home.position[0],
                    sl.ppm * home.position[1]
                ),
                sl.ppm * home.radius,
                3
            )

        # Draw Hazards
        for i in range(self.numHazards):
            currentHazard = self.hazards[i]
            pygame.draw.circle(
                screen,
                sl.hazardColor,
                (
                    sl.ppm * currentHazard.position[0],
                    sl.ppm * currentHazard.position[1]
                ),
                sl.ppm * currentHazard.radius,
                5
            )
            # hazard ID
            if sl.renderFlavor:
                font = pygame.font.SysFont('Arial', 16)
                text_ID = font.render(f"{i}", True, (255, 255, 255))
                screen.blit(text_ID, (sl.ppm * currentHazard.position[0], sl.ppm * currentHazard.position[1]))

        # Draw Targets
        for i in range(self.numTargets):
            currentTarget = self.targets[i]
            pygame.draw.circle(
                screen,
                currentTarget.color,
                (
                    sl.ppm * currentTarget.position[0],
                    sl.ppm * currentTarget.position[1]
                ),
                sl.ppm * currentTarget.radius,
                3
            )
            # target ID
            if sl.renderFlavor:
                font = pygame.font.SysFont('Arial', 16)
                text_ID = font.render(f"{i}", True, (255, 255, 255))
                screen.blit(text_ID, (sl.ppm * currentTarget.position[0], sl.ppm * currentTarget.position[1]))
                text_Quality = font.render(f"{int(100 * currentTarget.quality)}%", True, (255, 255, 255))
                screen.blit(text_Quality, (sl.ppm * currentTarget.position[0], sl.ppm * currentTarget.position[1] + 10))

        # Draw Time
        total_seconds = sl.dt * step
        current_second = int(total_seconds % 60)
        total_minutes = np.floor(total_seconds / 60)
        current_minute = int(total_minutes % 60)
        total_hours = int(np.floor(total_minutes / 60))
        timeString = str(total_hours) + ":" + str(current_minute).zfill(2) + ":" + str(current_second).zfill(2)
        font = pygame.font.SysFont('Arial', 30)
        text_ID = font.render(f"{timeString}", True, (255, 255, 255))
        screen.blit(text_ID, (self.width - 100, 0))

        return

        # targets are static, so no step()
        # target positions already handled, so no adjustThermalPositions

class Home:
    def __init__(self, sl):
        self.sl = sl
        self.position = [0, 0, 0]
        self.position[0] = sl.homePosition[0]
        self.position[1] = sl.homePosition[1]
        self.radius = sl.homePosition[2]

class Hazard:
    def __init__(self, sl, coord, randomly):
        self.sl = sl
        self.position = [0, 0, 0]
        self.radius = 0
        if not randomly:
            self.position[0] = coord[0]
            self.position[1] = coord[1]
            self.radius = coord[2]
        else:
            self.position[0] = random.uniform(0, sl.mapWidth)
            self.position[1] = random.uniform(0, sl.mapHeight)
            self.radius = random.uniform(sl.hazardSizeRange[0], sl.hazardSizeRange[1])

class Target:
    def __init__(self, sl, coord, randomly, active):
        self.radius = 5
        self.position = [0, 0, 0]
        if not randomly:
            self.position[0] = coord[0]
            self.position[1] = coord[1]
            self.radius = coord[2]
            self.quality = coord[3]
        else:
            self.position[0] = random.uniform(sl.targetSpawnX[0], sl.targetSpawnX[1])
            self.position[1] = random.uniform(sl.targetSpawnY[0], sl.targetSpawnY[1])
            self.radius = 5
            self.quality = random.uniform(sl.targetQualityRange[0], sl.targetQualityRange[1])
        self.position[2] = 0  # targets are on the ground
        self.active = active
        # self.ID = index
        self.sl = sl
        self.color = (0, int(255 * self.quality), 0)
        self.numAgents = 0
