class MultiLaw():
    def __init__(self):
        # Small Case
        self.FOV = [2.09439, 6.28318]
        self.visualRange = [1, 2]
        self.maxSpeed = [0.1, 0.2]
        self.lifespan = [300, 600, 1200, -1]
        self.numAgents = [20, 30, 100, 200]
        self.propQualityTargets = [0.25, 0.5, 0.75]
        self.numTargets = [4, 8]
        self.hazardType = ["None", "Partial", "Full"]
        self.rngSeed = [2, 3, 4, 5]

        # # Multivariable Parameters
        # self.FOV = [2.09439, 6.28318]
        # self.visualRange = [1, 2]
        # self.maxSpeed = [0.1, 0.2]
        # self.lifespan = [300, 600, 1200, -1]
        # self.numAgents = [12, 25, 50, 100, 200]
        # self.propQualityTargets = [0.25, 0.5, 0.75]
        # self.numTargets = [4, 8]
        # self.hazardType = ["None", "Partial", "Full"]
        # self.RNGSeed = [1, 2, 3, 4, 5]
