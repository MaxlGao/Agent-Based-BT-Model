import numpy as np


class Utilities:

    def __init__(self, simlaw):
        self.sl = simlaw
        # utilities here

    @staticmethod
    def isNear(a1, a2, threshold=np.nan):
        # handles structs and arrays
        if isinstance(a1, list) or isinstance(a1, np.ndarray):
            a1x = a1[0]
            a1y = a1[1]
            a1z = a1[2]
        else:
            a1x = a1.position[0]
            a1y = a1.position[1]
            a1z = a1.position[2]
        if isinstance(a2, list) or isinstance(a2, np.ndarray):
            a2x = a2[0]
            a2y = a2[1]
            a2z = a2[2]
        else:
            a2x = a2.position[0]
            a2y = a2.position[1]
            a2z = a2.position[2]
        # Returns nan if too far, returns actual distance if close enough
        # A NaN Threshold will tell isNear to ignore the threshold.
        verdict = np.nan
        if not np.isnan(threshold):
            if abs(a1x - a2x) > threshold:
                return verdict
            elif abs(a1y - a2y) > threshold:
                return verdict
            elif abs(a1z - a2z) > threshold:
                return verdict
            elif a1x == a2x and a1y == a2y:
                return verdict

        dist = np.linalg.norm([a1x - a2x,
                               a1y - a2y,
                               a1z - a2z])
        if np.isnan(threshold) or dist <= threshold:
            verdict = dist

        return verdict

    @staticmethod
    def posToPixels(simlaw, position):
        pixels = int(position * simlaw.pixelsPerMeter)
        return pixels

    @staticmethod
    def findItem(swarm, agentNum, sl, itemType):
        # itemType can be Agent or Target.
        currentAgent = swarm.agents[agentNum]
        env = swarm.environment
        numLocalAgents = 0
        localAgentIndices = -1 * np.ones(sl.numAgents)
        numLocalTargets = 0
        localTargetIndices = -1 * np.ones(env.numTargets)
        numLocalHazards = 0
        localHazardIndices = -1 * np.ones(env.numTargets)
        if itemType == "Agents":
            for j in range(sl.numAgents):
                if j == agentNum:
                    continue

                otherAgent = swarm.agents[j]
                dist = Utilities.isNear(currentAgent, otherAgent, sl.visualRange)
                if dist is np.nan:
                    continue

                diff = otherAgent.position - currentAgent.position
                dotProduct = (
                    diff[0] * np.cos(currentAgent.heading) -
                    diff[1] * np.sin(currentAgent.heading)
                ) / dist
                dotProduct = max(-1, min(1, dotProduct))
                angle = np.arccos(dotProduct)
                if angle > sl.FOV / 2:
                    continue
                # print(f"angle between {agentNum}, {j}: {angle:.4f}. Heading of {agentNum}: {currentAgent.heading:.4f}")
                # print(f"{agentNum} Acquired: {j}")
                localAgentIndices[numLocalAgents] = j
                numLocalAgents = numLocalAgents + 1

            localAgents = [swarm.agents[0] for _ in range(numLocalAgents)]
            for k in range(numLocalAgents):
                localAgentIndex = localAgentIndices[k]
                localAgents[k] = swarm.agents[int(localAgentIndex)]
            return localAgents

        elif itemType == "Targets":
            for j in range(env.numTargets):
                otherTarget = env.targets[j]
                dist = Utilities.isNear(currentAgent, otherTarget, sl.visualRange + otherTarget.radius)
                if dist is np.nan:
                    continue
                # print(f"distance between Agent {agentNum} and target {j} is {dist}")

                diff = otherTarget.position - currentAgent.position
                dotProduct = (
                    diff[0] * np.cos(currentAgent.heading) -
                    diff[1] * np.sin(currentAgent.heading)
                ) / dist
                dotProduct = max(-1, min(1, dotProduct))
                angle = np.arccos(dotProduct)
                if angle > sl.FOV / 2:
                    continue
                # print(f"angle between Agent {agentNum}, Target {j}: "
                #       f"{angle:.2f}. Heading of Agent {agentNum}: {currentAgent.heading:.2f}")
                # print(f"Agent {agentNum} Acquired Target {j}")
                localTargetIndices[numLocalTargets] = j
                numLocalTargets = numLocalTargets + 1
                # print(f"Agent {agentNum} sees {numLocalTargets} targets")

            localTargets = [env.targets[0] for _ in range(numLocalTargets)]
            for k in range(numLocalTargets):
                localTargetIndex = localTargetIndices[k]
                localTargets[k] = env.targets[int(localTargetIndex)]
            return localTargets

        elif itemType == "Hazards":
            for j in range(env.numHazards):
                otherHazard = env.hazards[j]
                dist = Utilities.isNear(currentAgent, otherHazard, sl.visualRange + otherHazard.radius)
                if dist is np.nan:
                    continue

                diff = otherHazard.position - currentAgent.position
                angle = np.arccos(
                    (
                            diff[0] * np.cos(currentAgent.heading) -
                            diff[1] * np.sin(currentAgent.heading)
                            # second component is negative because pygame flips y axis
                    ) / dist
                )
                if angle > sl.FOV / 2:
                    continue
                localHazardIndices[numLocalHazards] = j
                numLocalHazards = numLocalHazards + 1

            localHazards = [env.hazards[0] for _ in range(numLocalHazards)]
            for k in range(numLocalHazards):
                localHazardIndex = localHazardIndices[k]
                localHazards[k] = env.hazards[int(localHazardIndex)]
            return localHazards