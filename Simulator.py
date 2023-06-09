import random

import cv2
import pygame
import time
from Environment import Environment
from Swarm import Swarm
import numpy as np


def run_simulations(sl):
    random.seed(sl.rngSeed)
    startTime = time.time()
    n = sl.simNumber
    print(f"Starting Simulation {n}")
    env = Environment(sl)
    swarm = Swarm(sl, env)
    screen = None
    clock = None
    oneMore = False

    # Pre sim render
    if sl.render:
        pygame.init()
        screen = pygame.display.set_mode((env.width, env.height))
        pygame.display.set_caption("Particle Simulation")
        clock = pygame.time.Clock()
        simulation_running = False

        y = str(time.gmtime().tm_year)
        M = str(time.gmtime().tm_mon).zfill(2)
        d = str(time.gmtime().tm_mday).zfill(2)
        h = str(time.gmtime().tm_hour).zfill(2)
        m = str(time.gmtime().tm_min).zfill(2)
        s = str(time.gmtime().tm_sec).zfill(2)
        video_filename = "Simulation_" + y + M + d + '_' + h + m + s + ".mp4"
        video_fps = 30
        video_size = (env.width, env.height)
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        video_writer = cv2.VideoWriter(video_filename, fourcc, video_fps, video_size, isColor=True)

    # Mode 1: Run until Done. Dangerous for low populations.
    if sl.totalTime <= 0:
        step = 1
        while not swarm.simulation_done:
            # Handle keypress inputs
            while not swarm.simulation_running and not oneMore:
                if sl.render:
                    for event in pygame.event.get():
                        if event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_SPACE:
                                swarm.simulation_running = True
                            if event.key == pygame.K_RIGHT:
                                oneMore = True
                            elif event.key == pygame.K_ESCAPE:
                                video_writer.release()
                                pygame.quit()
                        elif event.type == pygame.WINDOWCLOSE:
                            video_writer.release()
                            pygame.quit()

            # Step Simulation
            swarm.stepSimulation(sl, step)

            # Render?
            if sl.render and (step % sl.frameSkip == 0 or step == 1):
                swarm.renderAll(sl, screen, clock, step, video_writer)
                frame = pygame.surfarray.array3d(screen)
                frame = cv2.cvtColor(frame.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
                video_writer.write(frame)

            # Report progress
            if step % 100 == 0 and sl.printProgress:
                numDone = 0
                for agent in swarm.agents:
                    if agent.state == "Done":
                        numDone = numDone + 1
                targetList = []
                for target in swarm.environment.targets:
                    targetList.append(target.numAgents)
                print(f"{step:4.0f} steps through Run {n:3.0f}, {numDone:3.0f} are done, "
                      f"target pop: {targetList}")
            if oneMore:
                oneMore = False
            step += 1
    # Mode 2: Run number of steps. Cuts off if time limit reached.
    else:
        steps = int(sl.totalTime / sl.dt)
        for step in range(steps):
            # cut if done
            if swarm.simulation_done:
                break

            # Handle keypress inputs
            while not swarm.simulation_running and not oneMore:
                if sl.render:
                    for event in pygame.event.get():
                        if event.type == pygame.KEYDOWN:
                            if event.key == pygame.K_SPACE:
                                swarm.simulation_running = True
                            if event.key == pygame.K_RIGHT:
                                oneMore = True
                            elif event.key == pygame.K_ESCAPE:
                                video_writer.release()
                                pygame.quit()
                        elif event.type == pygame.WINDOWCLOSE:
                            video_writer.release()
                            pygame.quit()

            # Step Simulation
            swarm.stepSimulation(sl, step)

            # Render?
            if sl.render and (step % sl.frameSkip == 0 or step == 1):
                swarm.renderAll(sl, screen, clock, step, video_writer)
                frame = pygame.surfarray.array3d(screen)
                frame = cv2.cvtColor(frame.swapaxes(0, 1), cv2.COLOR_RGB2BGR)
                video_writer.write(frame)

            # Report progress
            if step % (steps // 100) == 0:
                numDone = 0
                for agent in swarm.agents:
                    if agent.state == "Done":
                        numDone = numDone + 1
                targetList = []
                for target in swarm.environment.targets:
                    targetList.append(target.numAgents)
                print(f"{100 * step / steps:4.1f}% through Run {n:3.0f}, {numDone:3.0f} are done, "
                      f"target pop: {targetList}")
                continue

            if oneMore:
                oneMore = False

    # Post sim render
    if sl.render:
        video_writer.release()
        pygame.quit()
    endTime = time.time()
    print(f"Done with Run {n:3.0f}, took {endTime-startTime:3.2f}s")
    if sl.saveData:
        outputData = [n]
        if sl.multiLaw:
            for value in sl.varValue:
                outputData.append(value)
        outputData.append(step)
        for target in swarm.environment.targets:
            outputData.append(target.numAgents)
        return outputData
