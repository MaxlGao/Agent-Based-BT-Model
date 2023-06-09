import py_trees
from py_trees.common import Status
from py_trees.blackboard import Blackboard
from py_trees.decorators import Inverter, Retry, Repeat
from py_trees.common import ParallelPolicy
import warnings
import numpy as np
import random
from Utilities import Utilities as U


# Waypoint Management
class GenerateRandomWP(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl, target):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.targetCode = target
        self.target = None
        self.normalDistance = 0
        self.blackboard = Blackboard

    def update(self):
        # print("generating wp")
        # Generate Angle
        angleChange = random.uniform(-1, 1) * self.sl.maxRotSpeed
        # Redefine target and adjust atTarget
        if self.targetCode == "None":
            self.currentAgent.atTarget = False
            self.target = None
        elif self.targetCode == "Home":
            self.currentAgent.atTarget = False
            self.target = self.currentAgent.home
            self.normalDistance = self.sl.homeClearance
        elif self.targetCode == "Target":
            self.target = self.currentAgent.committedTarget
            self.normalDistance = self.sl.targetClearance
            endGoal = self.currentAgent.committedTarget.position
            currentPos = self.currentAgent.position
            vec = endGoal - currentPos
            dist = np.linalg.norm(vec)
            if dist < self.sl.targetClearance or dist < self.currentAgent.committedTarget.radius:
                self.currentAgent.atTarget = True

        # Get Target Angle
        if self.target is not None:
            self.currentAgent.heading = self.currentAgent.heading % (2 * np.pi)
            goalPos = self.target.position
            currentPos = self.currentAgent.position
            vec = goalPos - currentPos
            dist = np.linalg.norm(vec)
            angleToTarget = np.arctan2(-vec[1], vec[0]) - self.currentAgent.heading
            angleToTarget = (np.pi + angleToTarget) % (2 * np.pi) - np.pi
            # bias proposed heading towards target
            # Weighted average depending on square of distance, normalized to normalDistance
            weight = (dist / self.normalDistance) ** 2
            angleChange = (angleChange + weight * angleToTarget) / (1 + weight)
            a = angleChange

        # Apply smoothing
        angleChange = angleChange * self.sl.dt
        self.currentAgent.heading += angleChange
        self.currentAgent.heading = self.currentAgent.heading % (2 * np.pi)
        # (speed does not change right now)
        # Get Position
        x = self.currentAgent.position[0] + np.cos(self.currentAgent.heading) * self.currentAgent.speed * self.sl.dt
        y = self.currentAgent.position[1] - np.sin(self.currentAgent.heading) * self.currentAgent.speed * self.sl.dt
        newPos = [x, y, 0]

        self.currentAgent.waypoints.append(newPos)
        # print(f"Agent {self.currentAgent} has position {newPos}")
        if self.sl.debugEveryStep:
            print(f"SUCCESS: Generate Random WP at ({newPos[0]:.1f},{newPos[1]:.1f}) to {self.targetCode}")
        return Status.SUCCESS


class CheckWP(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def wp_within_forbidden_area(self):
        if self.currentAgent.knownHazards is None:
            self.currentAgent.threateningHazard = None
            return False
        for hazard in self.currentAgent.knownHazards:
            distance = U.isNear(self.currentAgent.waypoints[0], hazard,
                                hazard.radius + self.currentAgent.sl.hazardClearance)
            if not distance is np.nan:
                self.currentAgent.threateningHazard = hazard
                return True
        self.currentAgent.threateningHazard = None
        return False

    def wp_within_neighbor_zone(self):
        if self.currentAgent.neighbors is None:
            self.currentAgent.threateningNeighbor = None
            return False
        smallestDistance = self.currentAgent.sl.neighborClearance
        distance = np.nan
        for otherAgent in self.currentAgent.neighbors:
            # Define Distance, ignore Done and Dead agents
            if otherAgent.state == "Done" or otherAgent.state == "Dead":
                distance = np.nan
            else:
                distance = U.isNear(self.currentAgent.waypoints[0], otherAgent, self.currentAgent.sl.neighborClearance)

            if (not distance is np.nan) and (distance < smallestDistance):
                smallestDistance = distance
                self.currentAgent.threateningNeighbor = otherAgent
        if not distance is np.nan:
            return True
        else:
            self.currentAgent.threateningHazard = None
            return False

    def wp_out_of_bounds(self):
        x = self.currentAgent.waypoints[0][0]
        y = self.currentAgent.waypoints[0][1]
        if x <= 0 + self.sl.hazardClearance:
            self.currentAgent.boundError = "Left"
            return True
        elif x >= self.sl.mapWidth - self.sl.hazardClearance:
            self.currentAgent.boundError = "Right"
            return True
        elif y <= 0 + self.sl.hazardClearance:
            self.currentAgent.boundError = "Top"
            return True
        elif y >= self.sl.mapHeight - self.sl.hazardClearance:
            self.currentAgent.boundError = "Bottom"
            return True
        else:
            self.currentAgent.boundError = None
            return False

    def update(self):
        # print("Checking wp")
        # If the next waypoint is inside known hazards, fail the check
        if self.wp_within_forbidden_area():
            # if self.sl.debugEveryStep:
            #     print("FAILURE: WP in Hazard")
            return Status.FAILURE
        elif self.wp_within_neighbor_zone():
            # if self.sl.debugEveryStep:
            #     print("FAILURE: WP in personal space")
            return Status.FAILURE
        elif self.wp_out_of_bounds():
            return Status.FAILURE
        else:
            # if self.sl.debugEveryStep:
            #     print("SUCCESS: Check WP")
            return Status.SUCCESS


class FixWP(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def update(self):
        if all([self.currentAgent.threateningHazard is None,
               self.currentAgent.threateningNeighbor is None,
               self.currentAgent.boundError is None]):
            if self.sl.debugEveryStep:
                print("SUCCESS: No problem with  WP")
            return Status.SUCCESS
        # Priority is Bounds -> Hazards -> neighbors
        elif self.currentAgent.boundError is not None:
            boundError = self.currentAgent.boundError
            waypoint = np.array(self.currentAgent.waypoints[0])
            pos = self.currentAgent.position
            if boundError == "Top":
                unitVector = np.array([0.0, 1.0, 0.0])
            elif boundError == "Bottom":
                unitVector = np.array([0.0, -1.0, 0.0])
            elif boundError == "Left":
                unitVector = np.array([1.0, 0.0, 0.0])
            else:
                unitVector = np.array([-1.0, 0.0, 0.0])
            newWP = waypoint + unitVector * self.currentAgent.speed * self.sl.dt
            self.currentAgent.waypoints[0] = newWP

            # also direct the agent away from wall
            unitVecToWall = -unitVector
            angleToHazard = np.arccos(
                (
                        unitVecToWall[0] * np.cos(self.currentAgent.heading) -
                        unitVecToWall[1] * np.sin(self.currentAgent.heading)
                )
            )
            # direct away.
            self.currentAgent.heading = self.currentAgent.heading + \
                                        angleToHazard / np.abs(angleToHazard) * self.sl.maxRotSpeed * self.sl.dt
            if self.sl.debugEveryStep:
                print("SUCCESS: Fixed WP")
            return Status.SUCCESS

        elif self.currentAgent.threateningHazard is not None:
            threat = self.currentAgent.threateningHazard
            waypoint = np.array(self.currentAgent.waypoints[0])
            pos = self.currentAgent.position
            # bump the waypoint directly away from the hazard. the vector goes from the threat to the waypoint
            vector = waypoint - threat.position
            dist = np.linalg.norm(vector)
            unitVector = vector / dist
            newWP = waypoint + unitVector * self.currentAgent.speed * self.sl.dt
            self.currentAgent.waypoints[0] = newWP

            # also direct the agent away from the hazard
            vecToHazard = threat.position - pos
            distToHazard = np.linalg.norm(vecToHazard)
            angleToHazard = np.arccos(
                (
                        vecToHazard[0] * np.cos(self.currentAgent.heading) -
                        vecToHazard[1] * np.sin(self.currentAgent.heading)
                ) / distToHazard
            )
            # direct away.
            self.currentAgent.heading = self.currentAgent.heading + \
                                        angleToHazard / np.abs(angleToHazard) * self.sl.maxRotSpeed * self.sl.dt
            if self.sl.debugEveryStep:
                print("SUCCESS: Fixed WP")
            return Status.SUCCESS

        elif self.currentAgent.threateningNeighbor is not None:
            threat = self.currentAgent.threateningNeighbor
            waypoint = np.array(self.currentAgent.waypoints[0])
            pos = self.currentAgent.position
            # bump the waypoint directly away from the hazard.
            vector = waypoint - threat.position
            dist = np.linalg.norm(vector)
            unitVector = vector / dist
            newWP = waypoint + unitVector * self.currentAgent.speed * self.sl.dt
            self.currentAgent.waypoints[0] = newWP

            # also direct the agent away from the neighbor
            vecToNeighbor = threat.position - pos
            # distToNeighbor = np.linalg.norm(vecToNeighbor)
            angleToNeighbor = np.arctan2(-vecToNeighbor[1], vecToNeighbor[0]) - self.currentAgent.heading
            angleToNeighbor = (np.pi + angleToNeighbor) % (2 * np.pi) - np.pi
            # angleToNeighbor2 = np.arccos(
            #     (
            #             vecToNeighbor[0] * np.cos(self.currentAgent.heading) -
            #             vecToNeighbor[1] * np.sin(self.currentAgent.heading)
            #     ) / distToNeighbor
            # )
            # direct away.
            warnings.simplefilter('error')
            self.currentAgent.heading = self.currentAgent.heading - \
                                        angleToNeighbor / np.abs(angleToNeighbor) * self.sl.maxRotSpeed * self.sl.dt
            return Status.SUCCESS


class CommitNextWP(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def update(self):
        # apparently equivalent to == []
        if not self.currentAgent.waypoints:
            if self.sl.debugEveryStep:
                print("FAILURE: Commit WP")
            return Status.FAILURE
        self.currentAgent.position = self.currentAgent.waypoints[0]
        self.currentAgent.position = np.array(self.currentAgent.position)
        self.currentAgent.waypoints.pop(0)

        # if self.sl.debugEveryStep:
        #     print(f"SUCCESS: Commit WP ({self.currentAgent.position[0]:.1f}, {self.currentAgent.position[1]:.1f})")
        return Status.SUCCESS


class GenerateWPToTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def update(self):
        if self.currentAgent.state != "Assessing":
            if self.sl.debugEveryStep:
                print("FAILURE: Not supposed to go to target")
            return Status.FAILURE
        endGoal = self.currentAgent.committedTarget.position
        currentPos = self.currentAgent.position
        vec = endGoal - currentPos
        dist = np.linalg.norm(vec)
        if dist < self.sl.targetClearance or dist < self.currentAgent.committedTarget.radius:
            self.currentAgent.atTarget = True
            if self.sl.debugEveryStep:
                print("FAILURE: Already at Target")
            return Status.FAILURE
        unitVec = vec / dist
        angle = np.arctan2(-vec[1], vec[0])
        self.currentAgent.heading = angle
        newPos = currentPos + unitVec * self.currentAgent.speed * self.sl.dt
        self.currentAgent.waypoints.append(newPos)
        if self.sl.debugEveryStep:
            print("SUCCESS: Generate WP to Target")
        return Status.SUCCESS


# Internal Checks
class RollTime(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl, type, reduceQuality):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.type = type
        self.reduceQuality = reduceQuality
        self.blackboard = Blackboard

    def update(self):
        State = self.currentAgent.state
        q = self.currentAgent.qualityOfCommitted
        # if State == "Uncommitted" and self.type == "U to E":
        #     numSeconds = self.sl.t_U_E
        # elif State == "Recruiting" and self.type == "R to Q":
        #     numSeconds = self.sl.t_R_Q_a + self.sl.t_R_Q_b * self.currentAgent.qualityOfCommitted
        # elif State == "Surveying" and self.type == "Q to UR":
        #     numSeconds = self.sl.t_Q_UR_a + self.sl.t_Q_UR_b * len(self.currentAgent.agentsAtTarget)
        # else:
        #     if self.sl.debugEveryBypass:
        #         print(f"BYPASSED: {self.type} Not Rolled, agent still {self.currentAgent.state}")
        #     return Status.SUCCESS

        if State == "Uncommitted" and self.type == "U to E":
            numSeconds = self.sl.t_U_E
        elif State == "Recruiting" and self.type == "R to Q":
            numSeconds = self.sl.t_R_Q_a + self.sl.t_R_Q_b * self.currentAgent.qualityOfCommitted
        elif State == "Surveying" and self.type == "Q to UR":
            numSeconds = self.sl.t_Q_UR_a + self.sl.t_Q_UR_b * len(self.currentAgent.agentsAtTarget)
        else:
            if self.sl.debugEveryBypass:
                print(f"BYPASSED: {self.type} Not Rolled, agent still {self.currentAgent.state}")
            return Status.SUCCESS

        p = 1 - np.exp(-self.sl.dt / numSeconds)
        # Edge case where surveyors may flip to recruit before even getting to their target
        if State == "Surveying" and not self.currentAgent.atTarget:
            p = 0

        roll = random.random()
        if roll < p:
            # The roll succeeded, and the agent will change state
            if State == "Uncommitted" and self.type == "U to E":
                self.currentAgent.state = "Exploring"
            elif State == "Recruiting" and self.type == "R to Q":
                self.currentAgent.state = "Surveying"
            elif State == "Surveying" and self.type == "Q to UR":
                self.currentAgent.state = "Recruiting"
                self.currentAgent.attemptedToRecruit = []
                self.currentAgent.qualityOfCommitted = self.currentAgent.qualityOfCommitted / 2
                if self.sl.debugStateChanges:
                    print(f"NOTICE: Agent {self.currentAgent.ID} reports drop in quality from "
                          f"{self.currentAgent.qualityOfCommitted * 200:.0f}% to "
                          f"{self.currentAgent.qualityOfCommitted * 100:.0f}%")

            if self.sl.debugStateChanges:
                print(f"SUCCESS: {self.type} Roll Passed (p={p*100:.2f}%, t={numSeconds:.1f}s),"
                      f" agent waited for {self.currentAgent.timeInState:.1f}s and is now {self.currentAgent.state}")
            self.currentAgent.timeInState = 0
            return Status.SUCCESS
        else:
            # The agent will retain its current state.
            self.currentAgent.timeInState += self.sl.dt
            if self.sl.debugEveryStep:
                print(f"RUNNING: {self.type} Roll Failed (p={p*100:.2f}%), agent still {self.currentAgent.state}")
            return Status.RUNNING


class IsRecruited(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def update(self):
        if (self.currentAgent.recruitedTo is not None) and \
                (self.currentAgent.recruitedTo is not self.currentAgent.avoidedTarget):
            self.currentAgent.committedTarget = self.currentAgent.recruitedTo
            self.currentAgent.state = "Assessing"
            if self.sl.debugEveryStep:
                print("SUCCESS: Recruited")
            return Status.SUCCESS
        else:
            self.currentAgent.committedTarget = None
            self.currentAgent.agentsAtTarget = []
            self.currentAgent.recruitedTo = None
            if self.sl.debugEveryStep:
                print("FAILURE: Not Recruited")
            return Status.FAILURE


class CheckForTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl, swarm):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.swarm = swarm
        self.blackboard = Blackboard

    def update(self):
        localTargets = U.findItem(self.swarm, self.currentAgent.ID, self.sl, "Targets")
        self.currentAgent.knownTargets = localTargets

        if not localTargets:
            if self.sl.debugEveryStep:
                print("RUNNING: No Target Sighted")
            return Status.RUNNING
        elif len(localTargets) == 1 and localTargets[0] is self.currentAgent.avoidedTarget:
            if self.sl.debugEveryStep:
                print("RUNNING: Bad Target")
            return Status.RUNNING
        elif len(localTargets) == 1:
            self.currentAgent.committedTarget = localTargets[0]
            self.currentAgent.qualityOfCommitted = self.currentAgent.committedTarget.quality
            self.currentAgent.state = "Assessing"
            if self.sl.debugEveryStep:
                print(f"SUCCESS: Agent {self.currentAgent.ID} has found target {self.currentAgent.committedTarget.ID}, "
                      f"quality {self.currentAgent.qualityOfCommitted}")
            return Status.SUCCESS
        else:
            self.currentAgent.committedTarget = localTargets[1]
            self.currentAgent.qualityOfCommitted = self.currentAgent.committedTarget.quality
            self.currentAgent.state = "Assessing"
            if self.sl.debugEveryStep:
                print(f"SUCCESS: Agent {self.currentAgent.ID} has found target {self.currentAgent.committedTarget.ID}, "
                      f"quality {self.currentAgent.qualityOfCommitted}")
            return Status.SUCCESS


class CheckTargetQuality(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl, memory):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.memory = memory
        self.blackboard = Blackboard

    def update(self):
        q = self.currentAgent.qualityOfCommitted
        threshold = self.sl.qualityThreshold
        mem = self.memory

        # First case assumes visiting for the first time, which requires visitor to be Assessing
        if not mem and self.currentAgent.state == "Assessing":
            q = self.currentAgent.committedTarget.quality
            if q > threshold:
                self.currentAgent.state = "Recruiting"
                self.currentAgent.attemptedToRecruit = []
                if self.sl.debugEveryStep:
                    print("SUCCESS: Real Quality")
                return Status.SUCCESS
            else:
                self.currentAgent.avoidedTarget = self.currentAgent.committedTarget
                self.currentAgent.committedTarget = None
                self.currentAgent.agentsAtTarget = []
                if self.sl.debugEveryStep:
                    print("FAILURE: Real Quality")
                return Status.FAILURE
        # Second case assumes recalling nest quality from memory, and not being around.
        # threshold is cut in half to counter the pre-halving of the Q to UR roll
        elif mem:
            if q > threshold / 2:
                if self.sl.debugEveryStep:
                    print("SUCCESS: Remembered Quality")
                return Status.SUCCESS
            else:
                self.currentAgent.avoidedTarget = self.currentAgent.committedTarget
                self.currentAgent.committedTarget = None
                self.currentAgent.agentsAtTarget = []
                if self.sl.debugStateChanges:
                    print(f"FAILURE: Agent {self.currentAgent.ID} has lost faith in its target.")
                return Status.FAILURE
        # Third case are all the leftovers, and may bypass this block without effect
        else:
            if self.sl.debugEveryBypass:
                print("BYPASSED: Quality")
            return Status.SUCCESS


class CheckVerdict(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def update(self):
        agentsAtTarget = len(self.currentAgent.agentsAtTarget)
        if agentsAtTarget >= self.sl.agentThreshold:
            self.currentAgent.state = "Done"
            self.currentAgent.committedTarget.numAgents += 1
            if self.sl.debugEveryStep:
                print("SUCCESS: Verdict is Agent Threshold")
            return Status.SUCCESS
        else:
            self.currentAgent.state = "Uncommitted"
            self.currentAgent.committedTarget = None
            self.currentAgent.qualityOfCommitted = 0
            self.currentAgent.agentsAtTarget = []
            if self.sl.debugEveryStep:
                print("FAILURE: Verdict is Quality")
            return Status.FAILURE


class DetectObject(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl, swarm):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.swarm = swarm
        self.blackboard = Blackboard

    def update(self):
        localAgents = U.findItem(self.swarm, self.currentAgent.ID, self.sl, "Agents")
        self.currentAgent.neighbors = localAgents
        localTargets = U.findItem(self.swarm, self.currentAgent.ID, self.sl, "Targets")
        self.currentAgent.knownTargets = localTargets
        localHazards = U.findItem(self.swarm, self.currentAgent.ID, self.sl, "Hazards")
        self.currentAgent.knownHazards = localHazards
        if self.sl.debugEveryStep:
            print("RUNNING: Finding Objects")
        return Status.RUNNING


class RecruitOthers(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl, swarm):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.swarm = swarm
        self.blackboard = Blackboard

    def update(self):
        # Get out of here if not in Recruiting mode. Best to set to Success to end it early
        if self.currentAgent.state != "Recruiting":
            if self.sl.debugEveryBypass:
                print("BYPASSED: Not Recruiting")
            return Status.SUCCESS

        # Build a list of agent candidates. Agents can be dead or done; it doesn't matter.
        # Agents need to consent to being recruited. Dead agents can't consent.
        candidates = []
        # Add agents that currentAgent can see, that are not recruited to anything.
        for agent in self.currentAgent.neighbors:
            if agent.recruitedTo is None:
                candidates.append(agent)
        # Then add agents that can see currentAgent
        for agent in self.swarm.agents:
            if agent.recruitedTo is None and self.currentAgent in agent.neighbors:
                candidates.append(agent)

        # There might be duplicates, but those will be dealt with later.
        # Iterate through all candidates
        for agent in candidates:
            if agent not in self.currentAgent.attemptedToRecruit:
                # Add the agent to the recruitment list regardless of whether the recruitment was successful.
                self.currentAgent.attemptedToRecruit.append(agent)
                if random.random() < self.sl.recruitProb:
                    agent.recruitedTo = self.currentAgent.committedTarget

        if self.sl.debugEveryStep:
            print("RUNNING: Recruiting...")
        return Status.RUNNING


class CheckAgentsAtTarget(py_trees.behaviour.Behaviour):
    def __init__(self, name, currentAgent, sl):
        super().__init__(name=name)
        self.currentAgent = currentAgent
        self.sl = sl
        self.blackboard = Blackboard

    def update(self):
        # print(self.currentAgent.agentsAtTarget)
        # First modify the entries already present
        if not self.currentAgent.agentsAtTarget == []:
            for n in reversed(range(len(self.currentAgent.agentsAtTarget))):
                if self.currentAgent.agentsAtTarget[n][1] >= self.sl.memoryLimit:
                    self.currentAgent.agentsAtTarget.pop(n)

        # Then tick up the surviving entries
        if not self.currentAgent.agentsAtTarget == []:
            for n in range(len(self.currentAgent.agentsAtTarget)):
                self.currentAgent.agentsAtTarget[n][1] = self.currentAgent.agentsAtTarget[n][1] + self.sl.dt

        # Then add more entries if the new agents are not already in the list
        # If the agents are in the list, reset their respective times to zero.
        for localAgent in self.currentAgent.neighbors:
            # Locals need to be already at their target, and not already represented in the list.
            match = False
            for agent in self.currentAgent.agentsAtTarget:
                if agent[0] == localAgent:
                    agent[1] = 0
                    match = True
                    break
            if not match and localAgent.atTarget:
                self.currentAgent.agentsAtTarget.append([localAgent, 0])

        # print(self.currentAgent.agentsAtTarget)
        # print("\n")

        if len(self.currentAgent.agentsAtTarget) >= self.sl.agentThreshold and self.currentAgent.atTarget:
            if self.sl.debugEveryStep:
                print("SUCCESS: Agent Threshold met")
            return Status.SUCCESS
        else:
            if self.sl.debugEveryStep:
                print("SUCCESS: Agent Threshold not met")
            return Status.RUNNING


class BT(py_trees.trees.BehaviourTree):
    def __init__(self, currentAgent, swarm, sl, status_dict):
        if True:
            # Define all the leaves
            detect_object = DetectObject("Passive Detection", currentAgent, sl, swarm)
            grwp_1 = GenerateRandomWP("Generate Random WP 1", currentAgent, sl, "None")
            grwp_2 = GenerateRandomWP("Generate Random WP 2", currentAgent, sl, "Home")
            grwp_3 = GenerateRandomWP("Generate Random WP 3", currentAgent, sl, "Target")
            gwp_to_target = GenerateWPToTarget("Generate Direct WP to Target", currentAgent, sl)
            check_wp_1 = CheckWP("Check WP 1", currentAgent, sl)
            check_wp_2 = CheckWP("Check WP 2", currentAgent, sl)
            check_wp_3 = CheckWP("Check WP 3", currentAgent, sl)
            check_wp_4 = CheckWP("Check WP 4", currentAgent, sl)
            fix_wp_1 = FixWP("Fix WP 1", currentAgent, sl)
            fix_wp_2 = FixWP("Fix WP 2", currentAgent, sl)
            fix_wp_3 = FixWP("Fix WP 3", currentAgent, sl)
            fix_wp_4 = FixWP("Fix WP 4", currentAgent, sl)
            commit_wp_1 = CommitNextWP("Commit to WP 1", currentAgent, sl)
            commit_wp_2 = CommitNextWP("Commit to WP 2", currentAgent, sl)
            commit_wp_3 = CommitNextWP("Commit to WP 3", currentAgent, sl)
            commit_wp_4 = CommitNextWP("Commit to WP 4", currentAgent, sl)
            roll_UtoE = RollTime("Roll U to E", currentAgent, sl, "U to E", False)
            roll_RtoQ = RollTime("Roll R to Q", currentAgent, sl, "R to Q", False)
            roll_QtoUR = RollTime("Roll Q to UR", currentAgent, sl, "Q to UR", True)
            is_recruited = IsRecruited("Check if Recruited", currentAgent, sl)
            check_for_target = CheckForTarget("Check for Target", currentAgent, sl, swarm)
            qc_Real = CheckTargetQuality("Check Target Quality", currentAgent, sl, False)
            qc_Memory = CheckTargetQuality("Remember Target Quality", currentAgent, sl, True)
            recruit_others = RecruitOthers("Recruit Others", currentAgent, sl, swarm)
            check_neighbors = CheckAgentsAtTarget("Find Agents at Target", currentAgent, sl)
            verdict = CheckVerdict("Determine Verdict", currentAgent, sl)

            # Define Verify WP four times
            verify_wp_1 = py_trees.composites.Selector(name="[S] Verify WP 1", memory=True)
            verify_wp_1.add_children([check_wp_1, fix_wp_1])
            verify_wp_2 = py_trees.composites.Selector(name="[S] Verify WP 2", memory=True)
            verify_wp_2.add_children([check_wp_2, fix_wp_2])
            verify_wp_3 = py_trees.composites.Selector(name="[S] Verify WP 3", memory=True)
            verify_wp_3.add_children([check_wp_3, fix_wp_3])
            verify_wp_4 = py_trees.composites.Selector(name="[S] Verify WP 4", memory=True)
            verify_wp_4.add_children([check_wp_4, fix_wp_4])

            # Define MRS for three cases, then go to target
            mrs_none = py_trees.composites.Sequence(name="[Q] Aimless Random Walk", memory=True)
            mrs_none.add_children([grwp_1, Retry("[D] Until Success: Verify 1", verify_wp_1, 10000), commit_wp_1])

            mrs_home = py_trees.composites.Sequence(name="[Q] Random Walk to Home", memory=True)
            mrs_home.add_children([grwp_2, Retry("[D] Until Success: Verify 2", verify_wp_2, 10000), commit_wp_2])

            mrs_target = py_trees.composites.Sequence(name="[Q] Random Walk to Target", memory=True)
            mrs_target.add_children([grwp_3, Retry("[D] Until Success: Verify 3", verify_wp_3, 10000), commit_wp_3])

            go_to_target = py_trees.composites.Sequence(name="[Q] Go To Target", memory=True)
            go_to_target.add_children([gwp_to_target, Retry("[D] Until Success: Verify 4", verify_wp_4, 10000), commit_wp_4])

            # Define sections for Pre-Commit Sequence
            find_target = py_trees.composites.Parallel(name="[P] Find Target", policy=ParallelPolicy.SuccessOnOne())
            find_target.add_children([Repeat("[D] Never Stop: MRS None", mrs_none, -1), check_for_target])

            explore = py_trees.composites.Sequence(name="[Q] Explore", memory=True)
            explore.add_children([roll_UtoE, find_target])

            pre_target = py_trees.composites.Parallel(name="[P] Pre-Target Subphase", policy=ParallelPolicy.SuccessOnOne())
            pre_target.add_children([Retry("[D] Until Success: is recruited", is_recruited, 10000), explore])

            assess = py_trees.composites.Sequence(name="[Q] Assess", memory=True)
            assess.add_children([Inverter("[D] Inv: GT Target", go_to_target), qc_Real])

            pre_commit_phase = py_trees.composites.Sequence(name="[Q] Pre-Commit Phase", memory=True)
            pre_commit_phase.add_children([pre_target, assess])

            # Define sections for Commit Sequence
            recruit = py_trees.composites.Parallel(name="[P] Recruit Others", policy=ParallelPolicy.SuccessOnOne())
            recruit.add_children([roll_RtoQ, recruit_others, Repeat("[D] Never Stop: MRS Home", mrs_home, -1)])

            survey_target = py_trees.composites.Parallel(name="[P] Survey Population", policy=ParallelPolicy.SuccessOnOne())
            survey_target.add_children([Inverter("[D] Inv: QtoUR", roll_QtoUR),
                                        Repeat("[D] Never Stop: MRS Target", mrs_target, -1), check_neighbors])

            bounce_until_verdict = py_trees.composites.Selector(name="[S] Bounce", memory=True)
            bounce_until_verdict.add_children([Inverter("[D] Inv: QC Memory", qc_Memory),
                                               Inverter("[D] Inv: Recruit", recruit), survey_target])

            commit_phase = py_trees.composites.Sequence(name="[Q] Commit Phase", memory=True)
            commit_phase.add_children([Retry("[D] Until Success: Bounce", bounce_until_verdict, 10000), verdict])

            # Conglomerate
            roam = py_trees.composites.Sequence(name="[Q] Roam", memory=True)
            roam.add_children([pre_commit_phase, commit_phase])

            root = py_trees.composites.Parallel(name="[P] Act", policy=ParallelPolicy.SuccessOnOne())
            root.add_children([detect_object, Retry("[D] Until Success: Roam", roam, 10000)])


        self.status_dict = status_dict
        # print(py_trees.display.ascii_tree(root, show_status=True))
        super().__init__(root)

    def tick(self):
        super().tick()
        root = self.root
        self.get_tree_status(root)
        pass

    def get_tree_status(self, root_node):
        if not root_node.children:
            self.status_dict[root_node.name] = str(root_node.status) + ", message " + root_node.feedback_message
            return
        else:
            self.status_dict[root_node.name] = str(root_node.status) + ", message " + root_node.feedback_message
            for child in root_node.children:
                self.get_tree_status(child)
            return