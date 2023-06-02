import numpy as np
import pygame
import random
import py_trees
from AgentControllerBT import BT


class Swarm:
    # Modeled after MATLAB class of the same name
    def __init__(self, sl, environment):
        # Merge Generation function with __init__
        super().__init__()
        self.agents = [Agent(sl) for _ in range(sl.numAgents)]
        self.simlaw = sl
        for i in range(sl.numAgents):
            self.agents[i] = Agent(sl)
            self.agents[i].sl = sl
            self.agents[i].position[0] = random.uniform(sl.spawnX[0], sl.spawnX[1])
            self.agents[i].position[1] = random.uniform(sl.spawnY[0], sl.spawnY[1])
            self.agents[i].position[2] = 0
            self.agents[i].heading = random.uniform(0, 2 * np.pi)
            self.agents[i].velocity[0] = random.uniform(1, 2)
            self.agents[i].velocity[1] = random.uniform(1, 2)
            self.agents[i].velocity[2] = random.uniform(1, 2)
            self.agents[i].ID = i
            self.agents[i].home = environment.home
        self.findItemFunc = None
        self.thisAgent = 1
        self.environment = environment
        self.targets = environment.targets
        self.dictionary = {'status': py_trees.common.Status.INVALID}
        self.BTs = [BT(agent, self, sl, self.dictionary) for agent in self.agents]
        self.simulation_running = True
        self.simulation_done = False
        self.BT_States = [None] * sl.numAgents

    # saveAgentData later
    def stepSimulation(self, sl, step):
        for i in range(sl.numAgents):
            if not (self.agents[i].state == "Done" or self.agents[i].state == "Dead"):
                # Tick Agent BT
                self.BTs[i].tick()
                # Print tree status in html file for only agent 0
                if sl.printTree and i == 0:
                    # print(py_trees.display.ascii_tree(self.BTs[i].root, show_status=True))
                    f = open('BT.html', 'w')
                    f.write('<html><head><title>Behavior Tree</title><body>')
                    f.write(py_trees.display.xhtml_tree(self.BTs[i].root, show_status=True))
                    f.write("</body></html>")
                    f.close()
                # Roll for agent death. Agent must be in motion.
                if self.agents[i].state != "Uncommitted" and \
                        sl.lifespan > 0 and random.random() < 1 - np.exp(-sl.dt / sl.lifespan):
                    self.agents[i].state = "Dead"
                    self.agents[i].committedTarget = None
                    self.agents[i].atTarget = False

        self.simulation_done = all(agent.state == "Done" or agent.state == "Dead" for agent in self.agents)
        return

    def renderAgents(self, screen):
        sl = self.simlaw
        # Drawing lines and shapes specific to Agents
        for i in range(sl.numAgents):
            currentAgent = self.agents[i]
            # Setting Color
            State = currentAgent.state
            color = (0,0,0)
            if State == "Uncommitted":
                color = sl.UColor
            elif State == "Exploring":
                color = sl.EColor
            elif State == "Assessing":
                color = sl.AColor
            elif State == "Recruiting":
                color = sl.RColor
            elif State == "Surveying":
                color = sl.SColor
            elif State == "Done":
                color = sl.DColor
            elif State == "Dead":
                color = sl.XColor
            currentAgent.color = color

            # Agent Text
            if sl.renderFlavor:
                numSeconds = 0
                if State == "Uncommitted":
                    numSeconds = sl.t_U_E
                elif State == "Recruiting":
                    numSeconds = sl.t_R_Q_a + sl.t_R_Q_b * currentAgent.qualityOfCommitted
                elif State == "Surveying":
                    numSeconds = sl.t_Q_UR_a + sl.t_Q_UR_a * len(currentAgent.agentsAtTarget)
                font = pygame.font.SysFont('Arial', 16)

                str_ID = str(currentAgent.ID).zfill(2)
                # str_per = str(int(currentAgent.qualityOfCommitted*100)).zfill(2)
                if State == "Done":
                    string_Combined = f"{str_ID}"
                else:
                    string_Combined = f"{str_ID}[{currentAgent.state[0]}]"
                text_Combined = font.render(string_Combined, True, color)
                screen.blit(text_Combined, (sl.ppm * currentAgent.position[0], sl.ppm * currentAgent.position[1]))
                string_Time = "{:.{}f}".format(currentAgent.timeInState, 1)
                text_Time = font.render(string_Time, True, color)
                screen.blit(text_Time, (sl.ppm * currentAgent.position[0], sl.ppm * currentAgent.position[1] + 10))

            # Vision Range
            if currentAgent.state != "Done" and sl.renderFlavor:
                startAngle = currentAgent.heading - sl.FOV / 2
                endAngle = currentAgent.heading + sl.FOV / 2
                rect = [
                    sl.pixelsPerMeter * (currentAgent.position[0] - sl.visualRange),
                    sl.pixelsPerMeter * (currentAgent.position[1] - sl.visualRange),
                    sl.pixelsPerMeter * sl.visualRange * 2,
                    sl.pixelsPerMeter * sl.visualRange * 2
                ]
                pygame.draw.arc(
                    screen,
                    color,
                    rect,
                    startAngle,
                    endAngle,
                    1
                )
                startAnglePos = (
                    sl.ppm * (currentAgent.position[0] + np.cos(startAngle) * sl.visualRange),
                    sl.ppm * (currentAgent.position[1] - np.sin(startAngle) * sl.visualRange)
                )
                endAnglePos = (
                    sl.ppm * (currentAgent.position[0] + np.cos(endAngle) * sl.visualRange),
                    sl.ppm * (currentAgent.position[1] - np.sin(endAngle) * sl.visualRange)
                )
                pygame.draw.lines(
                    screen,
                    color,
                    False,
                    [
                        startAnglePos,
                        (sl.ppm * currentAgent.position[0], sl.ppm * currentAgent.position[1]),
                        endAnglePos
                    ],
                    1
                )

            # Connections to Neighbors
            if not currentAgent.neighbors is None and currentAgent.state != "Done" and sl.renderFlavor:
                localAgents = currentAgent.neighbors
                numLocalAgents = len(localAgents)
                for j in range(numLocalAgents):
                    pygame.draw.lines(
                        screen,
                        sl.seeAgentColor,
                        False,
                        [
                            (sl.ppm * currentAgent.position[0], sl.ppm * currentAgent.position[1]),
                            (sl.ppm * localAgents[j].position[0], sl.ppm * localAgents[j].position[1])
                        ],
                        1
                    )

            # Committed Target
            if not currentAgent.committedTarget is None and currentAgent.state != "Done":
                committedTarget = currentAgent.committedTarget
                if currentAgent.atTarget:
                    thc = 3
                else:
                    thc = 1
                color = sl.committedTargetColor
                pygame.draw.lines(
                    screen,
                    color,
                    False,
                    [
                        (sl.ppm * currentAgent.position[0], sl.ppm * currentAgent.position[1]),
                        (sl.ppm * committedTarget.position[0], sl.ppm * committedTarget.position[1])
                    ],
                    thc
                )

            # Connections to Hazards
            if not currentAgent.knownHazards is None and currentAgent.state != "Done" and sl.renderFlavor:
                localHazards = currentAgent.knownHazards
                numLocalHazards = len(localHazards)
                for j in range(numLocalHazards):
                    pygame.draw.lines(
                        screen,
                        sl.seeHazardColor,
                        False,
                        [
                            (sl.ppm * currentAgent.position[0], sl.ppm * currentAgent.position[1]),
                            (sl.ppm * localHazards[j].position[0], sl.ppm * localHazards[j].position[1])
                        ],
                        1
                    )

            self.agents[i].render(sl, screen)

    def renderAll(self, sl, screen, clock, step, video_writer):
        # Always check for keypress first

        for event in pygame.event.get():
            if event.type == pygame.KEYDOWN:
                if event.key == pygame.K_SPACE:
                    self.simulation_running = False
                elif event.key == pygame.K_ESCAPE:
                    video_writer.release()
                    pygame.quit()
            elif event.type == pygame.WINDOWCLOSE:
                video_writer.release()
                pygame.quit()

        screen.fill((0, 0, 0))
        self.environment.renderEnvironment(sl, screen, step)
        # self.targets.renderTargets(sl, screen)
        self.renderAgents(screen)

        pygame.display.flip()
        clock.tick(60)


class Agent():
    # Modeled after Agent in MATLAB. Only contains render function.
    def __init__(self, sl):
        super().__init__()
        self.sl = sl
        self.position = np.array([0.0, 0.0, 0.0])
        self.heading = 0.0
        self.velocity = np.array([0.0, 0.0, 0.0])
        self.speed = sl.maxSpeed

        # State can be Uncommitted, Assessing, Recruiting, Surveying, or Done.
        self.state = "Uncommitted"
        self.timeInState = 0

        self.waypoints = []
        self.radius = sl.agentSize
        self.color = sl.agentColor
        self.ID = 0

        self.home = None
        self.knownTargets = []
        self.qualityOfKnowns = []
        self.committedTarget = None
        self.qualityOfCommitted = 0
        self.atTarget = False
        self.agentsAtTarget = []
        self.avoidedTarget = None
        self.recruitedTo = None
        self.attemptedToRecruit = []

        self.neighbors = []
        self.knownHazards = []
        self.threateningHazard = None
        self.threateningNeighbor = None
        self.boundError = None


    def render(self, sl, screen):
        sz = self.radius
        pos = self.position
        head = self.heading
        if sl.agentAppearance == "cross":
            c = np.multiply(sz, [[1, 1], [-1, -1], [0, 0], [1, -1], [-1, 1]])
            c = \
                [
                    [c[0][0] * np.cos(head) - c[0][1] * np.sin(head), -c[0][0] * np.sin(head) - c[0][1] * np.cos(head)],
                    [c[1][0] * np.cos(head) - c[1][1] * np.sin(head), -c[1][0] * np.sin(head) - c[1][1] * np.cos(head)],
                    [c[2][0] * np.cos(head) - c[2][1] * np.sin(head), -c[2][0] * np.sin(head) - c[2][1] * np.cos(head)],
                    [c[3][0] * np.cos(head) - c[3][1] * np.sin(head), -c[3][0] * np.sin(head) - c[3][1] * np.cos(head)],
                    [c[4][0] * np.cos(head) - c[4][1] * np.sin(head), -c[4][0] * np.sin(head) - c[4][1] * np.cos(head)]
                ]
            cross = \
                [
                    (sl.ppm * (pos[0] + c[0][0]), sl.ppm * (pos[1] + c[0][1])),
                    (sl.ppm * (pos[0] + c[1][0]), sl.ppm * (pos[1] + c[1][1])),
                    (sl.ppm * (pos[0] + c[2][0]), sl.ppm * (pos[1] + c[2][1])),
                    (sl.ppm * (pos[0] + c[3][0]), sl.ppm * (pos[1] + c[3][1])),
                    (sl.ppm * (pos[0] + c[4][0]), sl.ppm * (pos[1] + c[4][1]))
                ]
            pygame.draw.lines(screen, self.color, False, cross, 3)
        elif sl.agentAppearance == "triangle":
            t = np.multiply(sz, [[1, 0], [-1, -0.8], [-1, 0.8]])
            t = \
                [
                    [t[0][0] * np.cos(head) - t[0][1] * np.sin(head), -t[0][0] * np.sin(head) - t[0][1] * np.cos(head)],
                    [t[1][0] * np.cos(head) - t[1][1] * np.sin(head), -t[1][0] * np.sin(head) - t[1][1] * np.cos(head)],
                    [t[2][0] * np.cos(head) - t[2][1] * np.sin(head), -t[2][0] * np.sin(head) - t[2][1] * np.cos(head)]
                ]
            triangle = \
                [
                    (sl.ppm * (pos[0] + t[0][0]), sl.ppm * (pos[1] + t[0][1])),
                    (sl.ppm * (pos[0] + t[1][0]), sl.ppm * (pos[1] + t[1][1])),
                    (sl.ppm * (pos[0] + t[2][0]), sl.ppm * (pos[1] + t[2][1])),
                    (sl.ppm * (pos[0] + t[0][0]), sl.ppm * (pos[1] + t[0][1]))
                ]
            pygame.draw.lines(screen, self.color, False, triangle, 3)
