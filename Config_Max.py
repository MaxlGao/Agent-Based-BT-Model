import numpy as np

class SimLaw():
    def __init__(self):
        # storing all the simlaws. Everything else in this page represent default values.
        # Only the one true SL shall have this cursed variable filled out.
        self.SLs = None

        # Parameters that end up getting varied by Multilaw
        self.rngSeed = 46
        self.numAgents = 30
        self.FOV = 2.09439
        self.numTargets = 4
        self.hazardType = "None"
        self.propQualityTargets = 0.5
        self.lifespan = -1
        self.visualRange = 2
        self.maxSpeed = 0.2

        # Common Switches to Flip:
        # Is usage of multiLaw allowed? If so, masterScript will make use of parallel computing and forbid rendering.
        self.multiLaw = False
        # Provided that rendering is allowed, should supplemental elements be displayed?
        self.renderFlavor = False
        # Should progress be printed every 100 simulation steps?
        self.printProgress = False
        if self.multiLaw:
            self.render = False
            self.renderFlavor = False
        else:
            self.render = True

        # Other General Parameters
        self.totalTime = -100
        self.dt = 1
        self.numSims = 1
        self.saveData = True
        self.debugEveryStep = False
        self.debugEveryBypass = False
        self.debugStateChanges = False
        self.printTree = False
        self.frameSkip = 1  # visual only
        self.playbackSpeed = 1
        self.simNumber = 0
        self.varValue = None

        # Environment Parameters
        self.mapWidth = 30
        self.mapHeight = 30
        self.mapCeiling = 10

        # Agent Parameters
        self.spawnX = [12, 18]
        self.spawnY = [12, 18]
        self.agentSize = 0.3
        # self.neighborDelay = 0.5
        # self.neighborFrameSkip = self.neighborDelay / self.dt
        # self.maxNeighbors = 4
        self.agentColor = (0, 0, 255)

        # Behavior Tree Parameters
        self.neighborClearance = 0.8
        self.hazardClearance = 0.5
        self.targetClearance = 1.0
        self.homeClearance = 8
        # self.smoothnessDegree = 0.9
        self.maxRotSpeed = 0.31416
        self.t_U_E = 1000  # Normal 1000
        self.t_R_Q_a = 25  # base value, normally 25
        self.t_R_Q_b = 375  # added time based on quality, normally 375
        self.t_Q_UR_a = 200  # base value, normally 200
        self.t_Q_UR_b = 2  # added time based on local agents, normally 2
        self.agentThreshold = 3
        self.memoryLimit = 20
        self.qualityThreshold = 0.2
        self.recruitProb = 0.05
        self.UColor = (127, 127, 127)  # Grey
        self.EColor = (127, 127, 255)  # Blue
        self.AColor = (255, 255, 255)  # White
        self.RColor = (255,   0, 255)  # Pink
        self.SColor = (255, 255,   0)  # Yellow
        self.DColor = (127, 255, 127)  # Green
        self.XColor = (127,   0,   0)  # Deep Red

        # Home Properties (x, y, r)
        self.homeExists = True
        self.homePosition = [15, 15, 4]
        self.homeColor = (127, 127, 127)

        # Target Parameters (x, y, r, q)
        self.targetDistance = 12
        self.targetRadius = 2
        self.targetHQ = 0.75
        self.targetLQ = 0.25
        # self.targetQualityRange = [0, 1]
        # self.targetSpawnX = [5, 45]
        # self.targetSpawnY = [5, 25]
        # self.targetSpacing = 1
        # self.targetSpawnAttempts = 40
        self.targetColor = (0, 255, 0)

        # Hazard Properties (x, y, r)
        self.hazardDistance = 7.5
        self.hazardRadius = 2
        # self.hazardSizeRange = [2, 7]
        # self.hazardsBlockComms = False  # really hard to implement optimally
        self.hazardColor = (255, 0, 0)

        # Visuals
        # Accepted values are "cross" and "triangle"
        self.agentAppearance = "triangle"
        # pixelsPerMeter controls scaling of pygame display
        self.pixelsPerMeter = 20
        self.ppm = self.pixelsPerMeter  # shortened name, keeping old for compatibility
        self.seeAgentColor = (0, 255, 255)
        self.seeTargetColor = (127, 255, 127)
        self.committedTargetColor = (255, 255, 255)
        self.seeHazardColor = (255, 127, 127)

